"""
Minimal MAVLink FTP client.

ArduPilot already implements the FTP SERVER side (confirmed working via
MAVProxy's `ftp put`/`ftp get` commands). This module only implements the
CLIENT side -- PUT and GET -- talking to that existing server. No server
logic needed here at all.

Protocol reference: https://mavlink.io/en/services/ftp.html

The FTP packet lives inside the 251-byte payload of a single
FILE_TRANSFER_PROTOCOL message:

    seq_number     uint16   (2 bytes)
    session        uint8    (1 byte)
    opcode         uint8    (1 byte)
    size           uint8    (1 byte)   -- bytes of `data` actually used
    req_opcode     uint8    (1 byte)   -- opcode this Ack/Nak responds to
    burst_complete uint8    (1 byte)
    padding        uint8    (1 byte)
    offset         uint32   (4 bytes)
    data           239 bytes
    ------------------------------
    total: 12 + 239 = 251 bytes
"""

import struct
import time
from typing import Optional

from pymavlink import mavutil


# Opcodes
OP_NONE             = 0
OP_TERMINATE_SESSION = 1
OP_RESET_SESSIONS   = 2
OP_LIST_DIRECTORY   = 3
OP_OPEN_FILE_RO     = 4
OP_READ_FILE        = 5
OP_CREATE_FILE      = 6
OP_WRITE_FILE       = 7
OP_REMOVE_FILE      = 8
OP_CREATE_DIRECTORY = 9
OP_REMOVE_DIRECTORY = 10
OP_OPEN_FILE_WO     = 11
OP_TRUNCATE_FILE    = 12
OP_RENAME           = 13
OP_CALC_FILE_CRC32  = 14
OP_BURST_READ_FILE  = 15
OP_ACK              = 128
OP_NAK              = 129

_HEADER_FMT  = "<HBBBBBBI"          # seq, session, opcode, size, req_opcode, burst, pad, offset
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)   # 12 bytes
_DATA_SIZE   = 239
_PACKET_SIZE = _HEADER_SIZE + _DATA_SIZE      # 251 bytes, matches the message payload


class FtpError(Exception):
    pass


def _pack_packet(seq, session, opcode, size, req_opcode, offset, data: bytes) -> bytes:
    data = data[:_DATA_SIZE].ljust(_DATA_SIZE, b"\x00")
    header = struct.pack(
        _HEADER_FMT, seq, session, opcode, size, req_opcode, 0, 0, offset
    )
    return header + data


def _unpack_packet(payload: bytes):
    seq, session, opcode, size, req_opcode, burst, pad, offset = struct.unpack(
        _HEADER_FMT, payload[:_HEADER_SIZE]
    )
    data = payload[_HEADER_SIZE:_HEADER_SIZE + size]
    return seq, session, opcode, size, req_opcode, offset, data


class FtpClient:
    """
    Minimal MAVLink FTP client for a single connection. Talks to whichever
    (target_system, target_component) you pass to put_file()/get_file() --
    in our case, always ArduPilot's autopilot component.
    """

    def __init__(self, connection, target_system: int, target_component: int = 1):
        self._conn    = connection
        self._sysid   = target_system
        self._compid  = target_component
        self._seq     = 0

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) % 65536
        return self._seq

    def _send(self, opcode, session=0, size=0, req_opcode=0, offset=0, data=b"") -> int:
        seq = self._next_seq()
        packet = _pack_packet(seq, session, opcode, size, req_opcode, offset, data)
        self._conn.mav.file_transfer_protocol_send(
            0, self._sysid, self._compid, packet
        )
        return seq

    def _wait_response(self, expected_seq, timeout=3.0):
        """Waits for a FILE_TRANSFER_PROTOCOL reply matching our target and
        a higher-or-equal seq (ArduPilot echoes seq+1 conventions vary, so
        we just take the next FTP message from the right system)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self._conn.recv_match(type="FILE_TRANSFER_PROTOCOL", blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_srcSystem() != self._sysid:
                continue
            payload = bytes(msg.payload)
            return _unpack_packet(payload)
        return None

    # -------------------------------------------------------------------
    # PUT -- upload a local file to a remote path on the autopilot
    # -------------------------------------------------------------------

    def put_file(self, local_path: str, remote_path: str, timeout: float = 10.0) -> bool:
        with open(local_path, "rb") as f:
            file_bytes = f.read()

        # 1. CreateFile -- creates (or truncates) the remote file, opens for write
        path_bytes = remote_path.encode("utf-8")
        seq = self._send(OP_CREATE_FILE, size=len(path_bytes), data=path_bytes)
        resp = self._wait_response(seq, timeout)
        if resp is None:
            raise FtpError("No response to CreateFile for {0}".format(remote_path))
        _, session, opcode, size, req_opcode, offset, data = resp
        if opcode == OP_NAK:
            raise FtpError("CreateFile NAK, error code={0}".format(data[0] if data else "?"))
        if opcode != OP_ACK:
            raise FtpError("Unexpected response to CreateFile: opcode={0}".format(opcode))

        # 2. WriteFile -- send the file in 239-byte chunks
        written = 0
        while written < len(file_bytes):
            chunk = file_bytes[written:written + _DATA_SIZE]
            seq = self._send(
                OP_WRITE_FILE, session=session, size=len(chunk),
                offset=written, data=chunk,
            )
            resp = self._wait_response(seq, timeout)
            if resp is None:
                raise FtpError("No response to WriteFile at offset {0}".format(written))
            _, _, opcode, _, _, _, data = resp
            if opcode == OP_NAK:
                raise FtpError("WriteFile NAK at offset {0}, error={1}".format(
                    written, data[0] if data else "?"))
            if opcode != OP_ACK:
                raise FtpError("Unexpected response to WriteFile: opcode={0}".format(opcode))
            written += len(chunk)

        # 3. TerminateSession
        seq = self._send(OP_TERMINATE_SESSION, session=session)
        self._wait_response(seq, timeout=2.0)   # best effort, don't fail the PUT over this

        return True

    # -------------------------------------------------------------------
    # GET -- download a remote file from the autopilot to a local path
    # -------------------------------------------------------------------

    def get_file(self, remote_path: str, local_path: str, timeout: float = 10.0) -> bool:
        # 1. OpenFileRO -- opens the remote file for reading, returns file size
        path_bytes = remote_path.encode("utf-8")
        seq = self._send(OP_OPEN_FILE_RO, size=len(path_bytes), data=path_bytes)
        resp = self._wait_response(seq, timeout)
        if resp is None:
            raise FtpError("No response to OpenFileRO for {0}".format(remote_path))
        _, session, opcode, size, req_opcode, offset, data = resp
        if opcode == OP_NAK:
            raise FtpError("OpenFileRO NAK, error code={0}".format(data[0] if data else "?"))
        if opcode != OP_ACK:
            raise FtpError("Unexpected response to OpenFileRO: opcode={0}".format(opcode))

        file_size = struct.unpack("<I", data[:4])[0] if len(data) >= 4 else None

        # 2. ReadFile -- pull the file in 239-byte chunks until done
        collected = b""
        read_offset = 0
        while file_size is None or read_offset < file_size:
            seq = self._send(
                OP_READ_FILE, session=session,
                size=_DATA_SIZE, offset=read_offset,
            )
            resp = self._wait_response(seq, timeout)
            if resp is None:
                raise FtpError("No response to ReadFile at offset {0}".format(read_offset))
            _, _, opcode, resp_size, _, _, data = resp
            if opcode == OP_NAK:
                # End of file is signalled via NAK with a specific error code
                # in most implementations -- treat any NAK here as EOF/done.
                break
            if opcode != OP_ACK:
                raise FtpError("Unexpected response to ReadFile: opcode={0}".format(opcode))
            if resp_size == 0:
                break
            collected += data[:resp_size]
            read_offset += resp_size
            if resp_size < _DATA_SIZE:
                break   # short read -- last chunk

        with open(local_path, "wb") as f:
            f.write(collected)

        # 3. TerminateSession
        seq = self._send(OP_TERMINATE_SESSION, session=session)
        self._wait_response(seq, timeout=2.0)

        return True
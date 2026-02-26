import time
import struct
from typing import Optional, Any, List
from pymavlink import mavutil
import os 
import threading

# Import infrastructure components
from wrapper import constants
from wrapper.interaction_journal import InteractionJournal
from wrapper import replay_buffer
from wrapper.replay_buffer import ReplayBuffer
from wrapper.ipc_sem import IPCClient


# Proxy class for intercepting outgoing commands (e.g., master.mav.command_long_send)
class _MavSenderProxy:
   
    def __init__(self, real_mav, wrapper_ref):
        self._real_mav = real_mav
        self._wrapper = wrapper_ref

    # Generalized execution flow for output operations
    def _execute_send_flow(self, api_id: int, summary: str, method_name: str, *args, **kwargs):
        
        # Acquire checkpoint lock
        self._wrapper._checkpoint_lock.acquire()

        # Replay mode
        if self._wrapper.is_replay_mode:
            msg = self._wrapper._handle_replay_send(summary)
            
            if msg is not None:
                self._wrapper._wrapper_log(
                    direction="SEND",
                    api=method_name,
                    outcome="SUPPRESSED",
                    summary=summary,
                    extra="replay_mask"
                )
                return 
            

        # Check if need to go to replay mode
        if self._wrapper._poll_restore_status():
            return self._execute_send_flow(api_id, summary, method_name, *args, **kwargs)

        # Live mode
        
        # Log the interaction
        self._wrapper._log_send_interaction(api_id, summary)
        self._wrapper._wrapper_log(
            direction="SEND",
            api=method_name,
            outcome="TX",
            summary=summary
        )

        # Call underlying pymavlink method
        real_method = getattr(self._real_mav, method_name)
        real_method(*args, **kwargs)

        self._wrapper._checkpoint_lock.release()


    '''Intercepted primitives'''
    def command_long_send(self, target_system, target_component, command, confirmation, p1, p2, p3, p4, p5, p6, p7):
        args_summary = f"CMD_LONG(sys={target_system}, cmd={command}, p1={p1})"
        self._execute_send_flow(
            constants.COMMAND_LONG_SEND_ID, args_summary, "command_long_send",
            target_system, target_component, command, confirmation, p1, p2, p3, p4, p5, p6, p7
        )
    
    def command_int_send(self, target_system, target_component, frame, command, current, autocontinue, p1, p2, p3, p4, x, y, z):
        args_summary = f"CMD_INT(sys={target_system}, cmd={command}, frame={frame})"
        self._execute_send_flow(
            constants.COMMAND_INT_SEND_ID, args_summary, "command_int_send",
            target_system, target_component, frame, command, current, autocontinue, p1, p2, p3, p4, x, y, z
        )

    def set_mode_send(self, target_system, custom_mode, base_mode):
        args_summary = f"SET_MODE(sys={target_system}, base={base_mode}, custom={custom_mode})"
        self._execute_send_flow(
            constants.SET_MODE_SEND_ID, args_summary, "set_mode_send",
            target_system, custom_mode, base_mode
        )

    def param_set_send(self, target_system, target_component, param_id, param_value, param_type):
        p_id_str = param_id.decode('utf-8') if isinstance(param_id, bytes) else str(param_id)
        args_summary = f"PARAM_SET(sys={target_system}, id={p_id_str}, val={param_value})"
        self._execute_send_flow(
            constants.PARAM_SET_SEND_ID, args_summary, "param_set_send",
            target_system, target_component, param_id, param_value, param_type
        )
    
    def __getattr__(self, name):
        # Pass through all other attributes to the real mav object
        return getattr(self._real_mav, name)

# Wrapepr component class
class MavlinkWrapper:

    def __init__(self, connection: Any, recv_path: str, send_path: str, sync_period: float = 0, wrapper_log: bool = False):

        self._conn = connection
        self.sync_period = sync_period
        is_strict = (self.sync_period == 0)
        self.log = wrapper_log
        # Init log journals
        self._recv_journal = InteractionJournal(recv_path, is_strict)
        self._send_journal = InteractionJournal(send_path, is_strict)
        
        # Replay mode buffers
        self._recv_buffer: Optional[ReplayBuffer] = None
        self._send_buffer: Optional[ReplayBuffer] = None
        self._is_replay_mode: bool = False

        # Setup mav proxy onject for outgoing commands
        self._mav_proxy = _MavSenderProxy(connection.mav, self)
        
        # Init lock
        self._checkpoint_lock = IPCClient()

        # Initial state check for replay mode
        self._poll_restore_status()

        #Set up periodic fsync if needed
        self._sync_thread = None
        if not is_strict:
            self._stop_sync = threading.Event()
            self._sync_thread = threading.Thread(
                target=self._periodic_fsync, 
                args=(self.sync_period,), # Sync every 1.0 seconds
                daemon=True
            )
            self._sync_thread.start()

        if self.log:
            BASE_DIR = os.environ.get("CHECKPOINT_BASEDIR", "/mnt/checkpoints")
            LOG_DIR = f"{BASE_DIR}/wrapper_logs"
            
            os.makedirs(LOG_DIR, exist_ok=True)
            timestamp = int(time.time())
            self._evidence_path = f"{LOG_DIR}/wrapper_log.csv"

    '''Intercepted Primitives'''
    
    # Mav forwards all the send commands to mav proxy object (e.g. master.mav.command_long_send())
    @property
    def mav(self):
    
        return self._mav_proxy
    
    def wait_heartbeat(self):
        msg = self._execute_receive_flow(
            constants.WAIT_HEARTBEAT_ID,
            "wait_heartbeat"
        )

        # * If this is a replay mode answer
        if self._is_replay_mode:
            self._conn.wait_heartbeat()

        return msg
    
    def recv_match(
        self,
        condition=None,
        type=None,
        blocking=False,
        timeout=None,
        **kwargs, 
    ):
        return self._execute_receive_flow(
            constants.RECV_MATCH_ID,
            "recv_match",
            condition=condition,
            type=type,
            blocking=blocking,
            timeout=timeout,
            **kwargs
        )

    def recv_msg(self):
        """Intercepts recv_msg using the generic flow."""
        return self._execute_receive_flow(
            constants.RECV_MSG_ID,
            "recv_msg"
        )

    def close(self):
        
        if self._sync_thread != None:
            self._stop_sync.set()
            self._sync_thread.join()

        self._recv_journal.close()
        self._send_journal.close()
        # todo create a flag file
        try:
            self._conn.close()
        except Exception:
            pass

        # Create mission complete flag
        checkpoint_dir = os.environ.get("CHECKPOINT_BASEDIR", "/mnt/checkpoints")
        flag_path = os.path.join(f"{checkpoint_dir}/mission_logs", "mission_completed.flag")
        
        try:
            with open(flag_path, 'w') as f:
                f.write(f"Completed at {time.ctime()}")
            # print(f"Flag file created at {flag_path}")
        except Exception as e:
            print(f"[Wrapper] Failed to create flag file: {e}")
    
    @property
    def is_replay_mode(self) -> bool:
        return self._is_replay_mode
    
    # Generalized execution flow for input operations
    def _execute_receive_flow(self, api_id: int, method_name: str, *args, **kwargs):
        
        # Acquire checkpoint lock
        self._checkpoint_lock.acquire()

        # Replay mode
        if self.is_replay_mode:
            
            # Consume next entry from replay buffer
            msg = self._handle_replay_receive()

            # Return msg if the buffer is not exhausted
            if msg is not None:
                self._wrapper_log(
                    direction="RECV",
                    api=method_name,
                    outcome="REPLAY",
                    summary=f"type={msg.get_type()} src={msg.get_srcSystem()}"
                )
                return msg

        # Check if need to go to replay mode
        if self._poll_restore_status():
            # Call again the same primitive
            return self._execute_receive_flow(api_id, method_name, *args, **kwargs)

        # Live Mode execution
        # Call the underlying pymavlink method
        real_method = getattr(self._conn, method_name)
        msg = real_method(*args, **kwargs)
        
        #Log the interaction
        self._log_receive_interaction(msg, api_id)
        if msg is not None:
            self._wrapper_log(
                direction="RECV",
                api=method_name,
                outcome="LIVE",
                summary=f"type={msg.get_type()} src={msg.get_srcSystem()}"
            )

        #Release checkpoint lock
        self._checkpoint_lock.release()
        
        return msg

    # Check if replay is needed
    def _poll_restore_status(self) -> bool:
        recv_needs_restore = self._recv_journal.check_restore_needed()
        send_needs_restore = self._send_journal.check_restore_needed()

        # If no replay is needed reurn False
        if not (recv_needs_restore or send_needs_restore):
            return False

        # Replay is needed
        # Read new bytes
        new_recv_bytes = self._recv_journal.read_restore_chunk()
        new_send_bytes = self._send_journal.read_restore_chunk()

        # Parse bytes to temp buffers
        new_recv_buf = replay_buffer.parse_receive_log(new_recv_bytes)
        new_send_buf = replay_buffer.parse_send_log(new_send_bytes)

        # Merge or Set Buffers
        # If we already have a buffer (rare nested case), we extend it. 
        # Otherwise, we just use the new one.
        self._extend_replay_buffer('recv', new_recv_buf)
        self._extend_replay_buffer('send', new_send_buf)
        
        self._is_replay_mode = True
        print("\n\n[Wrapper] Enter Replay Mode!\n")

        return True

    '''Helper methods'''
    """Helper to merge new entries into the existing replay queue."""
    def _extend_replay_buffer(self, kind: str, new_buf: ReplayBuffer):
        
        # Determine which buffer we are updating
        current_buf = self._recv_buffer if kind == 'recv' else self._send_buffer
        
        if current_buf is None or current_buf.is_exhausted:
            # Simple case: replace the buffer
            if kind == 'recv': 
                self._recv_buffer = new_buf
            else: 
                self._send_buffer = new_buf
        else:
            # Merge case: We have remaining items, append new ones
            # (Requires accessing internal list, assuming ReplayBuffer exposes it or we recreate)
            combined_entries = current_buf._entries[current_buf._index:] + new_buf._entries
            merged_buf = ReplayBuffer(combined_entries)
            
            if kind == 'recv': self._recv_buffer = merged_buf
            else: self._send_buffer = merged_buf

    """Fetches next message from buffer or transitions to Live Mode."""
    def _handle_replay_receive(self):
        
        if self._recv_buffer is None or self._recv_buffer.is_exhausted:
            print("\n[Wrapper] Replay buffer exhausted. Transitioning to LIVE MODE.\n\n")
            self._is_replay_mode = False
            return None # Return None to allow loop to retry in live mode
        
        entry = self._recv_buffer.next_entry()
        return entry.payload

    """Consumes a send entry from buffer."""
    def _handle_replay_send(self, summary):
        
        if self._send_buffer is None or self._send_buffer.is_exhausted:
            # print("[Wrapper] Replay buffer exhausted. Transitioning to LIVE MODE.")
            # self._is_replay_mode = False
            return None
        entry = self._send_buffer.next_entry()
        return entry
    
    """Formats and writes a receive entry to disk."""
    def _log_receive_interaction(self, msg, api_id):
        
        ts_us = int(time.time() * 1e6) 
        
        if msg is None:
            # Header: Type=REC_NONE
            header = struct.pack(constants.RECV_HEADER_FMT, constants.REC_NONE, api_id, 0, ts_us)
            self._recv_journal.append_bytes(header)
        else:
            # Header: Type=REC_MSG
            buf = msg.get_msgbuf()
            header = struct.pack(constants.RECV_HEADER_FMT, constants.REC_MSG, api_id, len(buf), ts_us)
            self._recv_journal.append_bytes(header + buf)

    """Formats and writes a send entry to disk."""
    def _log_send_interaction(self, api_id, summary: str):
        
        ts_us = int(time.time() * 1e6)
        summary_bytes = summary.encode('utf-8')
        
        header = struct.pack(constants.SEND_HEADER_FMT, api_id, len(summary_bytes), ts_us)
        
        self._send_journal.append_bytes(header + summary_bytes)

    # Thread Routine for periodic fsync
    def _periodic_fsync(self, interval: float):
        
        #Create new checkpoint lock so fsync-checkpoint-primitive_execution are sequential/atomic
        checkpoint_lock = IPCClient()
        while not self._stop_sync.is_set():
            try:
                checkpoint_lock.acquire()
                if self._recv_journal._dirty:
                    self._recv_journal.sync_to_disk()
                
                if self._send_journal._dirty:
                    self._send_journal.sync_to_disk()

                checkpoint_lock.release()
                
            except Exception as e:
                print(f"[Wrapper] Sync failed: {e}")
            
            self._stop_sync.wait(interval)

    def _wrapper_log(
        self,
        *,
        direction: str,      # "SEND" | "RECV"
        api: str,            # e.g. "command_int_send", "recv_match"
        outcome: str,        # INTENT | TX | SUPPRESSED | REPLAY | LIVE
        summary: str = "",   # compact semantic signature
        extra: str = ""      # optional free-form context
    ):
        """
        Lightweight, append-only wrapper evidence log.
        Designed for causal ordering and post-mortem reconstruction.
        """
        if not self.log:
            return
        ts_us = int(time.time() * 1e6)
        run_id = os.environ.get("RUN_ID", "NA")
        mode = "REPLAY" if self._is_replay_mode else "LIVE"

        # Stable, parseable line format
        line = (
            f"{run_id},"
            f"{ts_us},"
            f"{mode},"
            f"{direction},"
            f"{api},"
            f"{outcome},"
            f"{summary},"
            f"{extra}\n"
        )

        with open(self._evidence_path, "a") as f:
            f.write(line)
            f.flush()
            os.fsync(f.fileno())

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def __getattr__(self, name):
        # Forward all non-intercepted calls to the real connection
        return getattr(self._conn, name)

# Factory funciton
def mavlink_wrapper_connection(*args, **kwargs) -> MavlinkWrapper:
    
    # Pop custom argument for relaxed fsync period
    sync_period = kwargs.pop('sync_period', 0)
    wrapper_log = kwargs.pop('wrapper_log', False)
    # Init the default PyMAVLink connection
    inner = mavutil.mavlink_connection(*args, **kwargs)

    # Define log paths
    BASE_DIR = os.environ.get("CHECKPOINT_BASEDIR", "/app/log")
    LOG_DIR = f"{BASE_DIR}/logfiles"
    os.makedirs(LOG_DIR, exist_ok=True)
    
    recv_log = f"{LOG_DIR}/recv.bin" 
    send_log = f"{LOG_DIR}/send.bin"

    # Return the wrapped connection object
    return MavlinkWrapper(inner, recv_log, send_log, sync_period=sync_period, wrapper_log = wrapper_log)

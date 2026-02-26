import os, sys
from typing import BinaryIO

class InteractionJournal:

    def __init__(self, file_path: str, fsync_mode: bool):
        self._path = file_path
        # Open in Append Binary mode
        self._file: BinaryIO | None = None
        self._current_inode: int | None = None
        self._open_journal()
        self._dirty = False
        self.strict_mode = fsync_mode

    
    def _open_journal(self):
        if self._file and not self._file.closed:
            self._file.close()
        
        self._file = open(self._path, "ab", buffering=64*1024)
        self._file.seek(0)

        # Track Inode to detect log resets/truncations performed by the Supervisor
        self._current_inode = os.fstat(self._file.fileno()).st_ino

    def sync_to_disk(self):
        try:
            self._file.flush()
            os.fsync(self._file.fileno())
            self._dirty = False
            
        except Exception as e:
            #Catch anything else to prevent crash
            error_line = sys.exc_info()[-1].tb_lineno
            print(f"[Logger] Unexpected error: {e} on line: {error_line}")

    def append_bytes(self, data: bytes) -> None:
        
        try:
            self._file.write(data)
            self._dirty = True

            # Strict mode: fsync per message
            if self.strict_mode:
                self.sync_to_disk()
            
        except Exception as e:
            print(f"[Journal] Write error: {e} at line {sys.exc_info()[-1].tb_lineno}")

    def check_restore_needed(self) -> bool:
        
        try:
            stat_info = os.stat(self._path)
            disk_inode = stat_info.st_ino
            disk_size = stat_info.st_size

            #If the inode changed, the Supervisor reset the logs after a checkpoint
            if disk_inode != self._current_inode:
                # print(f"[Wrapper] Log Rotation Detected! (Old Inode: {self._current_inode} -> New: {disk_inode})")
                self._open_journal()
                return disk_size > 0
            
            #if disk is larger than current pointer, there is new data to replay
            return disk_size > self._file.tell()
            
        except FileNotFoundError:
            return False
        except Exception as e:
            print(f"[Journal] Poll error: {e}")
            return False

    def read_restore_chunk(self) -> bytes:

        try:
            # re-sync file size info
            current_disk_size = os.fstat(self._file.fileno()).st_size
            read_len = current_disk_size - self._file.tell()
            
            if read_len <= 0:
                return b""

            # read the new interaction bytes from the log
            with open(self._path, "rb") as reader:
                reader.seek(self._file.tell())
                delta_bytes = reader.read(read_len)
            
            # Update pointer to the new end of file
            self._file.seek(0, 2)
        except Exception as e:
            print(f"[Journal] Read error: {e}")
        
        return delta_bytes

    def close(self) -> None:
        if self._file and not self._file.closed:
            self.sync_to_disk()
            self._file.close()

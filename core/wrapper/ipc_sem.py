import sysv_ipc
import time

class IPCClient:

    def __init__(self, key: int = 0x1234):
        self.key = key
        self._lock_acquired = False
        self.sem = None

        self._attach()
        
    def _attach(self):
        while True:
            try:
                #Open the sempahore hte supervisor created
                self.sem = sysv_ipc.Semaphore(self.key)
                self._lock_acquired = False
                # print("[IPC] Attached to semaphore.")
                return
            except (sysv_ipc.ExistentialError, sysv_ipc.Error):
                # Supervisor might be initializing or recovering
                time.sleep(0.1)

    def acquire(self):
        if self._lock_acquired:
            return

        while True:
            try:

                self.sem.acquire()
                self._lock_acquired = True
                return
            except sysv_ipc.ExistentialError:
                # Semaphore was deleted/recreated (Supervisor reset/COntainer restart)
                self._attach()
                continue
            except sysv_ipc.Error as e:
                msg = str(e)
                if "Interrupted" in msg or "Signaled" in msg:
                    continue
                if "Identifier removed" in msg or "Invalid argument" in msg:
                    self._attach()
                    continue
                raise e

    def release(self):
        if self._lock_acquired:
            try:
                self.sem.release()
                self._lock_acquired = False
            except sysv_ipc.Error:
                # If the ID is gone, don't hold it anymore
                self._lock_acquired = False

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()
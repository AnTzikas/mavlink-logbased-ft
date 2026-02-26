import socket
import threading
import time
import os
import subprocess
import pathlib
import signal
import sys
import sysv_ipc

# --- Configuration ---
COMMAND = os.environ.get("COMMAND", "python3 /app/surveillance_mission.py")
APP_NAME = os.environ.get("APP_NAME", "surveillance_mission.py")
CHECKPOINT_INTERVAL = int(os.environ.get("CHECKPOINT_INTERVAL", 120))


CHECKPOINT_BASEDIR = os.environ.get("CHECKPOINT_BASEDIR", "/mnt/checkpoints")
CHECKPOINT_DIR = f"{CHECKPOINT_BASEDIR}/checkpoint"
LOGFILES_DIR = f"{CHECKPOINT_BASEDIR}/logfiles"
RECV_LOG_PATH = f"{CHECKPOINT_BASEDIR}/logfiles/recv.bin"
SEND_LOG_PATH = f"{CHECKPOINT_BASEDIR}/logfiles/send.bin"
# ---------------------

def fix_permissions(directory):
    # Retrieve the Host User IDs passed in via 'docker run -e ...'
    uid = int(os.environ.get("PUID", -1))
    gid = int(os.environ.get("PGID", -1))
    
    if uid != -1:
        # Change ownership of the directory and all files inside
        # We use a shell command here because it's faster for many small .img files
        subprocess.run(["chown", "-R", f"{uid}:{gid}", directory], check=True)

class Supervisor:
    def __init__(self):
        self.app_pid = None
        # Create/Attach Semaphore
        try:
            self.sem = sysv_ipc.Semaphore(0x1234, sysv_ipc.IPC_CREAT, initial_value=1)
        except sysv_ipc.ExistentialError:
            self.sem = sysv_ipc.Semaphore(0x1234)

        # Ensure directory structure
        pathlib.Path(CHECKPOINT_DIR).mkdir(parents=True, exist_ok=True)
        pathlib.Path(LOGFILES_DIR).mkdir(parents=True, exist_ok=True)

    def _initialize_mission(self):
        chkpt_path = pathlib.Path(CHECKPOINT_DIR) # Add this line
        if any(chkpt_path.iterdir()):
            # Restore mission program
            print("\n\n\n[Supervisor]: Found existing checkpoint. Restoring...")
            
            restore_cmd = [
                "criu", "restore",
                "-D", CHECKPOINT_DIR,
                "--shell-job",
                "--tcp-close",
                "--ext-unix-sk",
                "--skip-in-flight"
            ]
            
            # Run CRIU restore command
            subprocess.Popen(restore_cmd)
            
            # Try to find process id
            for _ in range(5):
                self.app_pid = self.get_pid_by_name()
                if self.app_pid: return True
                time.sleep(1)

        else:
            # Fresh start
            print(f"[Supervisor]: Starting fresh process: {COMMAND}")

            #Reserve PID space to avoid confilcts at low pids
            for _ in range(300): subprocess.run(["true"], check=False)
            
            child_proc = subprocess.Popen(COMMAND.split())
            
            self.app_pid = child_proc.pid
            return True
        
        return False

    def _perform_checkpoint(self):
        self.sem.acquire()
        print(f"[Supervisor] Snapshotting PID {self.app_pid}...")
        try:
            subprocess.run(["criu", "dump", "-t", str(self.app_pid), "-D", CHECKPOINT_DIR, 
                            "--shell-job", "--leave-running", "--tcp-close", 
                            "--ext-unix-sk", "--skip-in-flight", "-v0", "-o", "dump_criu.log"], check=True)
            
            self.rotate_logs()
            fix_permissions(CHECKPOINT_BASEDIR)
            print("[Supervisor] Snapshot SUCCESS.")
        except Exception as e:
            print(f"[Supervisor] Snapshot FAILED: {e}")
        finally:
            self.sem.release()

    def get_pid_by_name(self):
        try:
            output = subprocess.check_output(["pgrep", "-f", "-o", APP_NAME])
            return int(output.decode('utf-8').strip())
        except:
            return None
    
    def rotate_logs(self):
        for path in [RECV_LOG_PATH, SEND_LOG_PATH]:
            try:
                if os.path.exists(path):
                    # Ensure the file is actually unlinked
                    os.remove(path)
                
                # This creates a fresh file with a NEW Inode
                with open(path, 'wb') as f:
                    os.fsync(f.fileno()) # Ensure the directory entry is committed
            except OSError as e:
                print(f"[Supervisor] Rotation failed for {path}: {e}")
    
    def run(self):
        """Main lifecycle controller."""

        checkpoint_dir = os.environ.get("CHECKPOINT_BASEDIR", "/mnt/checkpoints")
        flag_path = os.path.join(checkpoint_dir, "mission_completed.flag")

        while True:
            if not self._initialize_mission():
                print("[Supervisor] Fatal: Mission failed to start/restore.")
                sys.exit(1)

            print(f"[Supervisor] Monitoring PID {self.app_pid}...")
            
            try:
                while True:
                    # Watchdog & Sleep
                    elapsed = 0
                    while elapsed < CHECKPOINT_INTERVAL:
                        if self.get_pid_by_name() is None:
                            if os.path.exists(flag_path):
                                print("[Supervisor] Mission terminated. Exiting.")
                                return
                            else:
                                os._exit(137)
                        time.sleep(2)
                        elapsed += 2

                    
                    self._perform_checkpoint()

            except KeyboardInterrupt:
                # print("[Supervisor] Shutdown signal received.")
                # self.cleanup()
                # sys.exit(0)
                pass

    def cleanup(self):
        if self.app_pid:
            try: os.kill(self.app_pid, signal.SIGTERM)
            except: pass
        try: self.sem.remove()
        except: pass
        print("[Supervisor] Cleanup complete.")
        
if __name__ == "__main__":
    Supervisor().run()
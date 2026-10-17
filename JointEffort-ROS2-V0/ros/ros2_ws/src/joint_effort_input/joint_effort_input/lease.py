"""Project-local advisory ownership shared with demo/plan/cycle clients."""
import fcntl
from pathlib import Path


class Lease:
    def __init__(self, path):
        self.path = Path(path); self.file = None

    def acquire(self):
        if self.file: return True
        self.path.parent.mkdir(parents=True, exist_ok=True)
        handle = self.path.open('a')
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            handle.close(); return False
        self.file = handle; return True

    def release(self):
        if self.file:
            fcntl.flock(self.file, fcntl.LOCK_UN); self.file.close(); self.file = None

"""Linux process-lifetime locks; acquire before constructing a hardware node.

All cooperating processes must share the lock directory and runtime scope.
Never unlink a lock file: another process may already hold its inode open.
Separate containers need a shared lock mount; this is not a distributed lock.
"""
import hashlib
import os
from pathlib import Path


class OwnershipError(RuntimeError):
    pass


class ProcessOwnership:
    def __init__(self, key, directory=None, scope=None):
        scope = scope or os.environ.get('GRAEY_RUNTIME_SCOPE', 'vehicle')
        directory = directory or os.environ.get('GRAEY_LOCK_DIR', '/tmp/graey-process-locks')
        digest = hashlib.sha256((scope + ':' + key).encode()).hexdigest()
        self.path = Path(directory) / (digest + '.lock')
        self.key, self.file = key, None

    def __enter__(self):
        import fcntl
        self.path.parent.mkdir(parents=True, exist_ok=True)
        handle = self.path.open('a+')
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            handle.close()
            raise OwnershipError(f'{self.key} already owned; duplicate start rejected') from None
        except BaseException:
            handle.close()
            raise
        self.file = handle
        return self

    def __exit__(self, *unused):
        # Closing releases the lock, including after SIGKILL/process exit.
        if self.file is not None:
            self.file.close()
            self.file = None


def executable_pids(name, proc_root='/proc'):
    """Count node executables, excluding ros2 wrappers and shell command strings."""
    found = []
    for entry in Path(proc_root).iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            args = (entry / 'cmdline').read_bytes().decode('utf-8', 'replace').split('\0')
        except OSError:
            continue
        if not args or not args[0]:
            continue
        exe = Path(args[0]).name
        # Python console scripts put their executable path in argv[1].
        candidate = args[1] if exe.startswith('python') and len(args) > 1 else args[0]
        if Path(candidate).name == name:
            found.append(int(entry.name))
    return sorted(found)

"""Crusader Connection/_ChunkWriter copied unchanged; GRAEY package import only."""
import json
import math
import socket
import threading
import time
from collections import deque
from . import gcs_wire

FILE_CHUNK = 256 * 1024
BULK_WAIT_S = 30.0                      # a download gives up when the link frees no room in this
OPS_WORKERS = 4


def clean(obj):
    """JSON-safe: non-finite floats become None, tuples lists."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean(v) for v in obj]
    return obj


def signature(frame):
    """What a frame says, without the ages that change on every tick."""
    return json.dumps(frame, sort_keys=True, default=str,
                      separators=(",", ":")).replace('"age"', "")


def _round(v, n=2):
    return gcs_wire.finite(v, n)


class Connection:
    """One laptop: a reader thread, a sender thread, and two bounded lanes between them.

    The sender drains the control lane (state, results, logs, pictures) before every bulk
    chunk (a download's file frames), so a download delays a control frame by one chunk."""

    QUEUE = 200                             # control frames
    BULK_QUEUE = 4                          # file chunks: 1 MB queued, the producer waits

    def __init__(self, sock, addr, handle, log):
        self.sock, self.addr = sock, addr
        self.min_level = 20                 # INFO: the Logs tab's default
        self.dropped = 0
        self._ctl, self._bulk = deque(), deque()    # (tag, frame): frame type | request id
        self._cv = threading.Condition()    # guards both lanes and _alive
        self._alive = True
        self._handle, self._log = handle, log
        threading.Thread(target=self._send_loop, daemon=True).start()
        threading.Thread(target=self._read_loop, daemon=True).start()

    @property
    def alive(self):
        return self._alive

    def send(self, header, blob=b""):
        """Queue one control frame -> queued? Never blocks: a full lane drops its oldest state
        (a newer one supersedes it), or this frame when it holds none."""
        frame = self._encode(header, blob)
        if frame is None:
            return False
        with self._cv:
            if not self._alive:
                return False
            if len(self._ctl) >= self.QUEUE:
                self.dropped += 1
                if not self._drop_state():
                    return False
            self._ctl.append((header.get("t"), frame))
            self._cv.notify_all()
            return True

    def send_bulk(self, header, blob, wait_s):
        """Queue one file chunk -> queued? Waits up to wait_s for room; False when the link
        closed or the wait ran out."""
        frame = self._encode(header, blob)
        if frame is None:
            return False
        with self._cv:
            if not self._wait(lambda: len(self._bulk) < self.BULK_QUEUE, wait_s):
                return False
            self._bulk.append((header.get("id"), frame))
            self._cv.notify_all()
            return True

    def drain_bulk(self, tag, wait_s):
        """Wait until the sender has taken every queued chunk of one request -> did it? A
        control frame queued after this goes out behind that last chunk, not ahead of it."""
        with self._cv:
            return self._wait(lambda: all(e[0] != tag for e in self._bulk), wait_s)

    def drop_bulk(self, tag):
        """Forget the queued chunks of one request: an aborted download leaves nothing behind."""
        with self._cv:
            keep = [e for e in self._bulk if e[0] != tag]
            self._bulk.clear()
            self._bulk.extend(keep)
            self._cv.notify_all()

    def _encode(self, header, blob):
        try:
            return gcs_wire.encode(clean(header), blob)
        except ValueError as e:
            self._log.error(f"frame {header.get('t')!r} not sent: {e}")
            return None

    def _drop_state(self):
        for i, (tag, _frame) in enumerate(self._ctl):
            if tag == "state":
                del self._ctl[i]
                return True
        return False

    def _wait(self, ready, wait_s):
        """With the lock held: wait for ready() -> True, False if the link closed first."""
        self._cv.wait_for(lambda: not self._alive or ready(), wait_s)
        return self._alive and ready()

    def _next_frame(self):
        """Control first, then ONE bulk chunk; None once closed."""
        with self._cv:
            self._cv.wait_for(lambda: not self._alive or self._ctl or self._bulk)
            if not self._alive:
                return None
            frame = (self._ctl or self._bulk).popleft()[1]
            self._cv.notify_all()           # a bulk slot is free
            return frame

    def _send_loop(self):
        try:
            while self._alive:
                frame = self._next_frame()
                if frame is None:
                    break
                self.sock.sendall(frame)
        except OSError as e:
            self._log.info(f"laptop {self.addr[0]}: send failed ({e})")
        finally:
            self.close()

    def _read_loop(self):
        try:
            while self._alive:
                header, blob = gcs_wire.read_frame(self.sock)
                self._handle(self, header, blob)
        except (ConnectionError, OSError) as e:
            self._log.info(f"laptop {self.addr[0]}: {e}")
        finally:
            self.close()

    def close(self):
        with self._cv:
            if not self._alive:
                return
            self._alive = False
            self._ctl.clear()
            self._bulk.clear()
            self._cv.notify_all()           # the sender, and any download waiting for room
        for fn in (lambda: self.sock.shutdown(socket.SHUT_RDWR), self.sock.close):
            try:
                fn()
            except OSError:
                pass


class _ChunkWriter:
    """A file-like object that sends what is written as file frames, with backpressure.
    Raises OSError once the link closed or stopped taking chunks, and keeps raising: tarfile
    writes again while it unwinds, and must not wait out the timeout a second time."""

    def __init__(self, conn, rid):
        self.conn, self.rid, self.seq, self.sent = conn, rid, 0, 0
        self._buf = bytearray()
        self._failed = False

    @property
    def mb(self):
        return self.sent / 1048576.0

    def write(self, data):
        self._buf += data
        while len(self._buf) >= FILE_CHUNK:
            self._send(bytes(self._buf[:FILE_CHUNK]), last=False)
            del self._buf[:FILE_CHUNK]
        return len(data)

    def flush(self):
        pass

    def finish(self):
        self._send(bytes(self._buf), last=True)

    def _send(self, chunk, last):
        ok = not self._failed and self.conn.send_bulk(
            {"t": "file", "id": self.rid, "seq": self.seq, "last": last}, chunk, BULK_WAIT_S)
        if ok and last:                     # the result must follow the chunk that ends the file
            ok = self.conn.drain_bulk(self.rid, BULK_WAIT_S)
        if not ok:
            self._failed = True
            raise OSError("the laptop closed or stopped reading the download")
        self.seq += 1
        self.sent += len(chunk)



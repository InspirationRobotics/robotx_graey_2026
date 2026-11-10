"""Record the identified DOWN camera locally; never select an arbitrary OAK."""
import argparse
import json
import signal
import time
from pathlib import Path


def main():
    import cv2
    import depthai as dai
    from . import camera
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', required=True)
    parser.add_argument('--mxid', default='14442C10C1B6BED200')
    args = parser.parse_args()
    folder = Path(args.directory)
    folder.mkdir(parents=True, exist_ok=True)
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    info = next((d for d in dai.Device.getAllAvailableDevices() if d.getMxId() == args.mxid), None)
    if info is None:
        raise RuntimeError('DOWN camera unavailable or already owned by another process: '+args.mxid)
    with dai.Device(camera.build_pipeline(fps=15, with_depth=False), info, dai.UsbSpeed.HIGH) as dev:
        queue = dev.getOutputQueue('rgb', 4, False)
        writer = cv2.VideoWriter(str(folder/'bottom.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 15., (camera.W, camera.H))
        if not writer.isOpened():
            raise RuntimeError('Could not open bottom-camera video file')
        try:
            with (folder/'frames.jsonl').open('w') as timestamps:
                count = 0
                while running:
                    packet = queue.tryGet()
                    if packet is None:
                        time.sleep(.01)
                        continue
                    writer.write(packet.getCvFrame())
                    now = time.monotonic()
                    timestamps.write(json.dumps(dict(frame=count, monotonic=now,
                        device_seconds=packet.getTimestamp().total_seconds(), wall=time.time()))+'\n')
                    timestamps.flush()
                    count += 1
                    tmp = folder/'camera-status.tmp'
                    tmp.write_text(json.dumps(dict(frames=count, monotonic=now, mxid=args.mxid)))
                    tmp.replace(folder/'camera-status.json')
        finally:
            writer.release()


if __name__ == '__main__':
    main()

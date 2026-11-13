#!/usr/bin/env python3
"""Per-camera health check: every OAK, by name (DOWN / FRONT), RGB + depth, and its USB speed.

Run in the graey container on the Jetson:
    docker exec -it graey bash -c 'source /opt/ros/humble/setup.bash &&
        cd /root/robotx_ws/src/robotx_graey_2026 && PYTHONPATH=.:$PYTHONPATH python3 tools/cam_test.py'

Only one process may hold a camera. Stop whatever holds them first: the camera tool
(GUI Camera stop, or curl -X POST http://127.0.0.1:8096/camera/stop), a headless dive
recording (api/vision/bottom_recorder.py) or pole_tracker. Exit status 0 = every camera passed.

USB SPEED IS THE POINT OF THIS AFTER A CABLE CHANGE. SUPER means the USB 3 path is real end
to end. HIGH means something in the chain is still USB 2 - the port, a hub or the cable - and
a USB 3 cable in a USB 2 port negotiates HIGH and looks like nothing changed. 640x360 RGB plus
depth is ~92 Mbps per camera at 10 fps, so USB 2 carries it; HIGH costs headroom.

Each camera is tried at SUPER, then at HIGH only if SUPER failed. A cable whose SuperSpeed
pairs are dead still enumerates over the USB 2 wires (the camera is DETECTED with its MxId),
then disappears when depthai flashes firmware and waits for a USB 3 link that never comes:
"X_LINK_DEVICE_NOT_FOUND after booting". Pinned to HIGH it usually works, bandwidth-limited,
which separates "the camera is broken" from "the cable is only wired for USB 2".
"""
import sys
import time

import depthai as dai

from robotx_graey_2026.api.vision import camera

GRAB_S = 5.0                                    # long enough for a stable frame rate


def test(info, speed):
    """One camera at one USB speed: True when both RGB and depth frames arrive."""
    mxid = info.getMxId()
    print(f'\n=== {camera.camera_name(mxid)} {mxid}   (trying {speed.name}) ===')
    try:
        with dai.Device(camera.build_pipeline(fps=15, with_depth=True), info, speed) as dev:
            got = dev.getUsbSpeed().name
            print(f'  USB speed      : {got}{"" if got == "SUPER" else "   <-- NOT USB 3"}')
            try:
                print(f'  sensors        : {sorted(c.name for c in dev.getConnectedCameras())}')
            except Exception:
                pass                            # binding name varies across depthai
            qr, qd = dev.getOutputQueue('rgb', 4, False), dev.getOutputQueue('depth', 4, False)
            rgb = depth = cover = 0
            t0 = time.time()
            while time.time() - t0 < GRAB_S:    # never block: a depthai call in progress can't be interrupted
                rgb += qr.tryGet() is not None
                d = qd.tryGet()
                if d is not None:
                    depth += 1
                    f = d.getFrame()
                    # 0 = no stereo match there. Low coverage on a textured scene = the stereo pair, not RGB.
                    cover = max(cover, int((f > 0).sum() * 100 // f.size))
                time.sleep(0.005)
            ok = rgb > 0 and depth > 0
            print(f'  rgb            : {rgb} frames  ({rgb / GRAB_S:.1f} fps)')
            print(f'  depth          : {depth} frames  ({depth / GRAB_S:.1f} fps)')
            print(f'  depth coverage : {cover}% of pixels matched')
            print(f'  RESULT         : {"PASS" if ok else "FAIL"}')
            return ok
    except Exception as e:
        print(f'  RESULT         : FAIL - {e}')
        return False


def main():
    devs = sorted(dai.Device.getAllAvailableDevices(), key=lambda d: camera.camera_name(d.getMxId()))
    print(f'{len(devs)} OAK device(s) found: '
          + (', '.join(camera.camera_name(d.getMxId()) for d in devs) or 'none'))
    missing = sorted(set(camera.CAMERAS.values()) - {camera.camera_name(d.getMxId()) for d in devs})
    if missing:
        print(f'  not found: {", ".join(missing)} - check cabling, and that nothing else holds the cameras')
    passed = 0
    for d in devs:
        if test(d, dai.UsbSpeed.SUPER):
            passed += 1
        elif test(d, dai.UsbSpeed.HIGH):        # a RESULT, not a recovery: that cable/port has no USB 3
            passed += 1
            print('  ^^ USABLE BUT USB 2 ONLY - replace this cable')
    print(f'\n{passed}/{len(devs)} passed')
    return 0 if devs and passed == len(devs) and not missing else 1


if __name__ == '__main__':
    sys.exit(main())

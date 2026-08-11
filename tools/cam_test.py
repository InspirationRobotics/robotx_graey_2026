#!/usr/bin/env python3
"""Per-camera health check: enumerate every OAK, then stream from each in turn.

    python3 tools/cam_test.py

The rest of the codebase opens dai.Device(pipeline) with no device argument, which
takes whichever camera answers first - fine with one camera, useless for testing two.
This addresses each by MxId so a failure can be pinned to a specific device.

USB SPEED IS THE POINT OF THIS AFTER A CABLE CHANGE. SUPER means the USB 3 path is
real end to end. HIGH means something in the chain is still USB 2 - the port, a hub, or
the cable - and a USB 3 cable into a USB 2 port negotiates HIGH and looks like nothing
changed. At 640x360 RGB plus depth this is about 92 Mbps per camera at 10 fps, so USB 2
does carry it; what HIGH costs is headroom for any resolution increase, and it puts a
streaming device on the shared hub whose transaction translator has reset on us before.

Each device is tried at SUPER and then, only if that failed, at HIGH. A cable whose
SuperSpeed pairs are dead still enumerates over the USB 2 wires, so the device is
DETECTED and reports its MxId, then disappears when depthai flashes firmware and waits
for it on a link that was never there - "X_LINK_DEVICE_NOT_FOUND after booting". Pinning
the speed to HIGH stops depthai attempting that re-enumeration, so a device that fails
outright at SUPER often works, just bandwidth-limited. SUPER then HIGH separates "the
camera is broken" from "the cable is only wired for USB 2".

Only one process may hold a camera, so stop the stack first:
    sudo systemctl stop graey-ros
"""
import time

import depthai as dai

from robotx_graey_2026.api.vision import camera

GRAB_S = 5.0                                    # long enough for a stable frame rate


def test(info, cap):
    mxid = info.getMxId()
    print(f'\n=== {mxid}   (trying {cap.name}) ===')
    try:
        with dai.Device(
                camera.build_pipeline(fps=15, with_depth=True), info, cap) as dev:
            speed = dev.getUsbSpeed().name
            print(f'  USB speed      : {speed}'
                  f'{"" if speed == "SUPER" else "   <-- NOT USB 3"}')
            try:
                print(f'  sensors        : '
                      f'{sorted(c.name for c in dev.getConnectedCameras())}')
            except Exception:
                pass                            # binding name varies across depthai
            qr = dev.getOutputQueue('rgb', 4, False)
            qd = dev.getOutputQueue('depth', 4, False)
            rgb = depth = cover = 0
            t0 = time.time()
            while time.time() - t0 < GRAB_S:
                if qr.tryGet() is not None:
                    rgb += 1
                d = qd.tryGet()
                if d is not None:
                    depth += 1
                    f = d.getFrame()
                    # Zero means "no stereo match here". A low number on a textured
                    # scene points at the stereo pair, not the RGB sensor.
                    cover = max(cover, int((f > 0).sum() * 100 // f.size))
                time.sleep(0.005)               # never block - a depthai call in
            ok = rgb > 0 and depth > 0          # progress cannot be interrupted
            print(f'  rgb            : {rgb} frames  ({rgb / GRAB_S:.1f} fps)')
            print(f'  depth          : {depth} frames  ({depth / GRAB_S:.1f} fps)')
            print(f'  depth coverage : {cover}% of pixels matched')
            print(f'  RESULT         : {"PASS" if ok else "FAIL"}')
            return ok
    except Exception as e:
        print(f'  RESULT         : FAIL - {e}')
        return False


def main():
    devs = dai.Device.getAllAvailableDevices()
    print(f'{len(devs)} OAK device(s) found')
    if not devs:
        print('  none - check cabling, and that graey-ros is stopped')
        return
    passed = 0
    for d in devs:
        # Falling back is a RESULT, not a recovery: passing only at HIGH means the
        # SuperSpeed pairs on that cable or port are dead. Say so loudly.
        if test(d, dai.UsbSpeed.SUPER):
            passed += 1
        elif test(d, dai.UsbSpeed.HIGH):
            passed += 1
            print('  ^^ USABLE BUT USB 2 ONLY - replace this cable')
    print(f'\n{passed}/{len(devs)} passed')


if __name__ == '__main__':
    main()

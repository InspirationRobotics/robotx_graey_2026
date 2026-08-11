#!/bin/bash
# EMI sometimes knocks the Cube off the bus hard enough that it fails SET_ADDRESS and
# never re-enumerates ("device not accepting address ... unable to enumerate").
# udev cannot help - there is no add event to act on. Resetting the hub is the
# software equivalent of replugging, and it brings every device back.
#
# Driven by graey-usb-watchdog.timer. Without that timer nothing ever calls this.
CUBE=/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_24002A000B51303231383439-if00
HUB=1-2.2                                      # fixed internal hub; confirm in dmesg if it moves

[ -e "$CUBE" ] && exit 0                       # nothing to do
sleep 5                                        # a reset or replug looks identical for a few
[ -e "$CUBE" ] && exit 0                       # seconds - never fight a recovery in progress
[ -e "/sys/bus/usb/devices/$HUB" ] || exit 0   # hub itself gone; a reset cannot help

logger -t graey-usb "Cube missing from by-id - resetting hub $HUB"
echo "$HUB" > /sys/bus/usb/drivers/usb/unbind
sleep 2
echo "$HUB" > /sys/bus/usb/drivers/usb/bind

# Graey Jetson host setup (NOT covered by the Dockerfile)

Hardware: Jetson Orin Nano, JetPack 6.2 (L4T R36.4, Ubuntu 22.04). User `graey`.

## 1. Performance + swap
    sudo nvpmodel -m 0
    sudo jetson_clocks
    sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile
    sudo mkswap /swapfile && sudo swapon /swapfile
    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

## 2. Docker access
    sudo usermod -aG docker $USER    # then log out/in
    docker info | grep -i runtime    # must list 'nvidia'

## 3. Tether static IP (wired iface is enP8p1s0; WiFi is wlP1p1s0)
    sudo nmcli con add type ethernet ifname enP8p1s0 con-name tether ip4 192.168.2.2/24
    sudo nmcli con up tether
Laptop side: static 192.168.2.1 / 255.255.255.0. SSH: ssh graey@192.168.2.2

## 4. CH340 driver for the LED Arduino  (NOT in NVIDIA's kernel - must be built)
    sudo apt-get install -y nvidia-l4t-kernel-headers build-essential
    mkdir -p ~/ch341_build && cd ~/ch341_build
    wget https://raw.githubusercontent.com/torvalds/linux/v5.15/drivers/usb/serial/ch341.c
    printf 'obj-m += ch341.o\nall:\n\tmake -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules\n' > Makefile
    make
    sudo cp ch341.ko /lib/modules/$(uname -r)/kernel/drivers/usb/serial/
    sudo depmod -a && echo ch341 | sudo tee -a /etc/modules
REBUILD THIS after any JetPack/kernel update.

## 5. Remove brltty - it hijacks CH340 adapters via usbfs
    sudo apt-get remove -y brltty

## 6. Build image + container
    docker build -t graey:dev ~/robotx_ws/src/robotx_graey_2026/docker
    ~/robotx_ws/src/robotx_graey_2026/scripts/run_container.sh

## Device paths - ALWAYS use by-id, ttyUSB* numbering is NOT stable across reboots
    Cube Orange : /dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_24002A000B51303231383439-if00
    VN-100 IMU  : /dev/serial/by-id/usb-FTDI_USB-RS232_Cable_AV0K9DQE-if00-port0
    CP2102      : /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
    LED Arduino : /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
    DVL A50     : 192.168.2.10 tcp/16171 (also answers on 192.168.194.95)

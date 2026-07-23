#!/bin/bash
# Create the 'graey' container. Environment = image, code = bind-mounted from host.
# These runtime flags are NOT in the image - they must be passed here.
docker run -dit --name graey \
  --runtime nvidia \
  --privileged \
  --net=host \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /dev:/dev \
  -v /home/graey/robotx_ws:/root/robotx_ws \
  --restart unless-stopped \
  graey:dev

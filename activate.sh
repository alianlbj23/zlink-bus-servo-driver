#!/bin/bash

# 1. 統一管理 -v 參數
VOLUME_ARGS="-v $(pwd)/src:/workspaces/src"


echo "Detected OS: $OS, Architecture: $ARCH"

# 設定適當的 Docker 參數
device_options=""

# 檢查設備並加入 --device 參數
if [ -e /dev/usb_front_wheel ]; then
    device_options+=" --device=/dev/usb_front_wheel"
fi
if [ -e /dev/usb_rear_wheel ]; then
    device_options+=" --device=/dev/usb_rear_wheel"
fi
if [ -e /dev/usb_robot_arm ]; then
    device_options+=" --device=/dev/usb_robot_arm"
fi

docker run -it --rm \
        --network compose_my_bridge_network \
        $device_options \
        --env-file .env \
        -v "$(pwd)/src:/workspaces/src" \
        ghcr.io/screamlab/pros_car_docker_image:latest \
        /bin/bash



#!/bin/bash

# 1. 統一管理 -v 參數
VOLUME_ARGS="-v $(pwd)/src:/workspaces/src"


echo "Detected OS: $OS, Architecture: $ARCH"

# 設定適當的 Docker 參數
device_options=""

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



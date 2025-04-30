#!/bin/bash

# 1. 定義掛載點參數 (每個參數分開)
VOLUME_ARGS=(
    "-v" "$(pwd)/src:/workspaces/src"
    "-v" "$(pwd)/setting_script:/workspaces/setting_script"
    # 在這裡新增更多掛載點，例如:
    # "-v" "$(pwd)/data:/workspaces/data"
)

# 2. 定義 Docker 命令的基本參數
DOCKER_RUN_ARGS=(
    "run" "-it" "--rm"
    "--network" "compose_my_bridge_network"
    "--env-file" ".env"
)

# 3. 條件性地添加設備參數
if [ -e /dev/usb_robot_arm ]; then
    DOCKER_RUN_ARGS+=("--device=/dev/usb_robot_arm")
fi

# 4. 合併掛載點參數
DOCKER_RUN_ARGS+=( "${VOLUME_ARGS[@]}" )

# 5. 添加映像名稱和要執行的命令
DOCKER_RUN_ARGS+=(
    "ghcr.io/screamlab/pros_car_docker_image:latest"
    "/bin/bash"
)

# (可選) 打印最終執行的命令，方便調試
# echo "將執行: docker ${DOCKER_RUN_ARGS[@]}"

# 執行 Docker 命令
docker "${DOCKER_RUN_ARGS[@]}"
#!/bin/bash
DOCKER_IMAGE='registry.cn-hangzhou.aliyuncs.com/slam_docker/slam_docker'
DOCKER_IMAGE_TAG='gpu'
DOCKER_VOLUME='ros12-amr-dev-home'

RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color
if [ "$EUID" -ne 0 ]
  then printf "${RED}!!! Please run with sudo !!!${NC}\n"
  exit
fi

# Create volume to be used as home if not created before
if ! (docker volume list | grep -q ${DOCKER_VOLUME}); then
    printf "${YELLOW}Creating ${DOCKER_VOLUME} that does not exist.${NC}\n"
    if docker volume create ${DOCKER_VOLUME} &> /dev/null; then
        printf "${GREEN}${DOCKER_VOLUME} has been created successfully.${NC}\n"
    else
        printf "${RED}!!! Failed to creat docker volume ${DOCKER_VOLUME}. Abort.${NC}\n"
        exit -1
    fi
fi

if [[ $# -lt 2 ]]; then
    echo "[Usage]: ./run_docker.sh PATH_TO_ROS1_WS PATH_TO_ROS2_WS [host]"
    exit -1
else
    printf "${GREEN}$1 is mapped to /slam/ros1_ws in docker.${NC}\n"
    printf "${GREEN}$2 is mapped to /slam/ros2_ws in docker.${NC}\n"
    if [[ $# -eq 3 && $3 == "host" ]]; then
        printf "${YELLOW}Attached to host network.\n"
        NETWORK_SETTING="--privileged --net=host"
    else
        NETWORK_SETTING="-p 2222:2222 --net=host"
    fi
fi

XAUTH=/tmp/.docker.xauth
if [ -d $XAUTH ]; then
    sudo rm -rf $XAUTH
fi
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    touch $XAUTH
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    fi
    chmod a+r $XAUTH
fi

# Allow root connect to xhost
# xhost +si:localuser:root > /dev/null
xhost +

docker run -it --rm \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --env="QT_LOGGING_RULES=*.debug=false;qt.qpa.*=false" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --volume="${DOCKER_VOLUME}:/root" \
    --volume="$1:/slam/ros1_ws" \
    --volume="$2:/slam/ros2_ws" \
    --volume="/dev:/dev" \
    --runtime=nvidia ${NETWORK_SETTING} \
    --workdir="/slam" \
    ${DOCKER_IMAGE}:${DOCKER_IMAGE_TAG} \
    terminator --title="DOCKER ${DOCKER_IMAGE}:${DOCKER_IMAGE_TAG}"


USER_ID="$(id -u)"
XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

VOLUMES="--volume=$(pwd):/catkin_ws/src:rw
         --volume=$XSOCK:$XSOCK:rw
         --volume=$XAUTH:$XAUTH:rw"

IMAGE="ros-test:noetic"

xhost +local:docker
docker run \
    -it --rm \
    $VOLUMES \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --env="USER_ID=$USER_ID" \
    --net=host \
    --privileged \
    $IMAGE 
xhost -local:docker

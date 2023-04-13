DISPLAY=:0
xhost +


docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    -v /home/lwk/grad/cyber_ros_bridge/catkin_ws:/catkin_ws/ \
    --net=host \
    --runtime=nvidia \
    --privileged \
    los/cyber_ros_bridge:latest \
    /bin/bash

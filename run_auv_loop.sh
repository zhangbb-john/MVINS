export VINS_DIR="/home/ubuntu/VINS-Mono-Enhance/src/vins_mono_enhanced"
export OUTPUT_DIR="/home/ubuntu"

#!/bin/bash

# Loop indefinitely
while true; do
    # Start ROS nodes
    echo "Starting ROS nodes..."

    echo $VINS_DIR; cd $VINS_DIR; pwd; cd ../../; source ./devel/setup.bash;roslaunch vins_estimator auv_right.launch|tee ~/estimator.txt&
    sleep 5;
    cd $VINS_DIR; cd ../../; source ./devel/setup.bash;rosrun benchmark_publisher gps_publisher&
    sleep 5;
    cd $VINS_DIR; cd ../../; source ./devel/setup.bash; roslaunch vins_estimator vins_rviz.launch|tee ~/vins_indoor.txt&

    sleep 3400;
    FILENAME=$(date +'%Y-%m-%d_%H-%M-%S').png;
    FULL_PATH="$OUTPUT_DIR/$FILENAME";
    gnome-screenshot -f "$FULL_PATH";
    sleep 10;
    rosnode kill -a;
    ps -ef|grep rqt|grep -v grep| cut -c 9-16|xargs kill -s 9;
    sleep 10;
done


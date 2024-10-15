export VINS_DIR="/workspace/assignments/MVINS/src/MVINS"
export OUTPUT_DIR="/workspace/data/Result"

#!/bin/bash

# Loop indefinitely
while true; do
    # Start ROS nodes
    echo "Starting ROS nodes..."

    echo $VINS_DIR; cd $VINS_DIR; pwd; cd ../../; source ./devel/setup.bash;roslaunch vins_estimator auv_right.launch|tee ~/estimator.txt&
    sleep 5;
    cd $VINS_DIR; cd ../../; source ./devel/setup.bash;rosrun benchmark_publisher gps_publisher&    
    sleep 5;
    gnome-terminal --title="benchmark" -e 'bash -c "    cd $VINS_DIR; cd ../../; source ./devel/setup.bash;rosrun benchmark_publisher benchmark_node;exec bash"'&

    cd $VINS_DIR; cd ../../; source ./devel/setup.bash; roslaunch vins_estimator vins_rviz.launch|tee ~/vins_indoor.txt&
    echo "begin waiting";
    sleep 1800;
    echo "after waiting"
    FILENAME=$(date +'%Y-%m-%d_%H-%M-%S').png;
    FULL_PATH="$OUTPUT_DIR/$FILENAME";
    gnome-screenshot -f "$FULL_PATH";
    sleep 10;
    rosnode kill -a;
    ps -ef|grep rqt|grep -v grep| cut -c 9-16|xargs kill -s 9;
    sleep 10;
done


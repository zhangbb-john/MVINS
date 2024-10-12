export VINS_DIR="/home/ubuntu/VINS-Mono-Enhance/src/vins_mono_enhanced"
export OUTPUT_DIR="/home/ubuntu"

#!/bin/bash

# Loop indefinitely
while true; do
    # Start ROS nodes
    echo "Starting ROS nodes..."

    gnome-terminal --title="estimator" -e 'bash -c "
    cd /home/ubuntu/VINS-Mono-Enhance/src/vins_mono_enhanced; cd ../../; source ./devel/setup.bash;roslaunch vins_estimator mini_auv.launch|tee ~/VINS-Mono-Benchmark/vins.txt "';
    sleep 5;
    gnome-terminal --title="rviz" -e 'bash -c "cd /home/ubuntu/VINS-Mono-Enhance/src/vins_mono_enhanced; cd ../../; source ./devel/setup.bash;roslaunch vins_estimator vins_rviz.launch"';

    gnome-terminal --title="benchmark" -e 'bash -c "cd /home/ubuntu/VINS-Mono-Enhance/src/vins_mono_enhanced; cd ../../; source ./devel/setup.bash;rosrun benchmark_publisher gps_publisher"';

    sleep 2800
    rosnode kill -a;
    sleep 10;
    rosnode kill -a;
    ps -ef|grep roslaunch|grep -v grep| cut -c 9-16|xargs kill -s 9;
    ps -ef|grep rviz|grep -v grep| cut -c 9-16|xargs kill -s 9;

    ps -ef|grep rqt|grep -v grep| cut -c 9-16|xargs kill -s 9;
    sleep 10;
    
done


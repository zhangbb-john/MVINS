
gnome-terminal --title="estimator" -e 'bash -c "
cd /home/ubuntu/VINS-Mono-Benchmark/src/vins-mono-benchmark; cd ../../; source ./devel/setup.bash;roslaunch vins_estimator auv_right.launch|tee ~/VINS-Mono-Benchmark/vins.txt&& read -p PressEnter&& sleep 100;
 exec bash "';
sleep 5;
gnome-terminal --title="rviz" -e 'bash -c "cd /home/ubuntu/VINS-Mono-Benchmark/src/vins-mono-benchmark; cd ../../; source ./devel/setup.bash;roslaunch vins_estimator vins_rviz.launch"';

gnome-terminal --title="benchmark" -e 'bash -c "cd /home/ubuntu/VINS-Mono-Benchmark/src/vins-mono-benchmark; cd ../../; source ./devel/setup.bash;rosrun benchmark_publisher gps_publisher"';


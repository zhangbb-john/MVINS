cd ../../
rm -rf ./devel
rm -rf ./build
catkin_make -DCATKIN_WHITELIST_PACKAGES="auv_nav_msg";
catkin_make -DCATKIN_WHITELIST_PACKAGES="cola2_msgs";
catkin_make -DCATKIN_WHITELIST_PACKAGES="";
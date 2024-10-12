#include <cstdio>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <auv_nav_msg/Gnss.h>
#include <utility/utility.h>
ros::Publisher pub_gps_odom;
Eigen::Vector3d position(-100, -100, 0);
Eigen::Vector3d gps_origin(0, 0, 0);
bool flag_init = false;
void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    position << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z;
}
void gpsCallback(const auv_nav_msg::Gnss msg)
{
    if (msg.latitude < 10) 
    {
        return;
    }
    if ((!flag_init) && (position.x() > -99))
    {
        double lat, lon, x, y;
        Utility::xy2gps(msg.latitude / 180 * M_PI, msg.longitude  / 180 * M_PI, lat, lon, -position.x(), -position.y());
        gps_origin << lat, lon, 0;
        flag_init = true;
    }
    else if (flag_init)
    {
        std::cout << "debug: gps call back" << std::endl;
        double x, y;
        Utility::gps2xy(gps_origin.x(), gps_origin.y(), msg.latitude / 180 * M_PI, msg.longitude  / 180 * M_PI, x, y);
        nav_msgs::Odometry odom_msg;
        odom_msg.header = msg.header;
        odom_msg.header.frame_id = "world";
        odom_msg.child_frame_id = "gps";
        odom_msg.pose.pose.position.x = y;
        odom_msg.pose.pose.position.y = x;
        odom_msg.pose.pose.position.z = 0;
        pub_gps_odom.publish(odom_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_publisher");
    ros::NodeHandle n("~");
    ros::Subscriber sub_odom = n.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 1000, odomCallback);
    ros::Subscriber sub_gps = n.subscribe<auv_nav_msg::Gnss>("/Sensor/Gps", 1000, gpsCallback);
    pub_gps_odom = n.advertise<nav_msgs::Odometry>("gps", 1000);
    ros::spin();
}
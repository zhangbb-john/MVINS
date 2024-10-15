#include <cstdio>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <chrono>
#include <auv_nav_msg/DetectLoop.h>
using namespace std;
using namespace Eigen;
using namespace ceres;
const int SKIP = 300;
Eigen::Vector3d gt_position(-100, -100, -100), est_position(-100, -100, -100);
Eigen::Quaterniond gt_q(1, 0, 0, 0);
Eigen::Quaterniond est_q(1, 0, 0, 0);
vector<Eigen::Vector3d> gt_vec;
vector<double> gt_stamps;

ros::Publisher pub_odom;
ros::Publisher pub_gt_path, pub_sm_path;

nav_msgs::Path new_path;
int init = 0;
Quaterniond baseRgt;
Vector3d baseTgt;
tf::Transform trans;
bool flag_loop = false;
bool flag_align = false;
int last_loop_num = -1;
int count_path = 0;
double est_stamp = -1;
double last_opt_stamp = 0;
std::vector<Vector2d> optimized_poses;  // Start with original points
nav_msgs::Path smooth_path;
#include <iostream>
#include <vector>
#include <cmath>  // For std::abs
#include <algorithm>  // For std::lower_bound

// Function to find the index with the minimum difference
std::pair<int, int> findClosest(const std::vector<double>& nums, double target) {
    int n = nums.size();
    
    // If the set is empty
    if (n == 0) {
        return {-1, -1}; // Return -1 to indicate no valid index
    }

    // Binary search for the closest element
    auto it = std::lower_bound(nums.begin(), nums.end(), target);

    // Check boundary cases
    if (it == nums.begin()) {
        // The closest is the first element
        return {nums[0], 0};
    } 
    else if (it == nums.end()) {
        // The closest is the last element
        return {nums[n - 1], n - 1};
    } 
    else {
        // Find the closest between two neighboring elements
        int idx = std::distance(nums.begin(), it);
        double prev = nums[idx - 1];
        double curr = nums[idx];

        if (std::abs(prev - target) <= std::abs(curr - target)) {
            return {prev, idx - 1};
        } else {
            return {curr, idx};
        }
    }
}



void gt_callback(const nav_msgs::Odometry odom)
{
    if (est_position.x() < -50 || est_stamp < 0)
        return;
    gt_position << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
    Eigen::Matrix3d ned2body = Eigen::Quaterniond(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z).toRotationMatrix();  // New quaternion (w, x, y, z) 
    // Eigen::Matrix3d ned2body = Quaterniond(qw, qx, qy, qz).toRotationMatrix();
    Eigen::Matrix3d body2ahrs;
    body2ahrs << 1, 0, 0,
                0, -1, 0,
                0, 0, -1;
    Eigen::Matrix3d enu2ned;
    enu2ned << 0, 1, 0,
                1, 0, 0,
                0, 0, -1;
    Eigen::Matrix3d enu2ahrs = enu2ned * ned2body * body2ahrs; // bodyframe2ahrs frame ()
    // std::cout << "enu2ahrs is " << std::endl << enu2ahrs << std::endl;
    gt_q = Eigen::Quaterniond(enu2ahrs);

    std::cout << "flag align is " << flag_align << std::endl;
    if (!flag_align)
    {   
        gt_stamps.push_back(odom.header.stamp.toSec());
        gt_vec.push_back(Vector3d(gt_position.x(), gt_position.y(), gt_position.z()));
        auto result = findClosest(gt_stamps, est_stamp);
        if (abs(result.first - est_stamp) > 1.5)
        {
            return;
        }
        Vector3d gt_position_closest = gt_vec[result.second];
        baseRgt = est_q * gt_q.inverse();
        // baseRgt = Eigen::Quaterniond(1, 0, 0, 0);
        baseTgt = est_position - baseRgt * Vector3d(gt_position_closest.x(), gt_position_closest.y(), gt_position_closest.z());//+ Eigen::Vector3d(0, -0.5, 0);
        
        if (init++ >= SKIP)
        {
            std::cout << "est_position is " << est_position.transpose() << std::endl;
            std::cout << "gt position is " << Vector3d(gt_position_closest.x(), gt_position_closest.y(), gt_position_closest.z()).transpose() << std::endl;
            std::cout << "baseTgt is " << baseTgt.transpose() << std::endl;
            gt_stamps.clear();
            gt_vec.clear();
            flag_align = true;
        }
        return;
    }

    nav_msgs::Odometry odometry;
    odometry.header.stamp = odom.header.stamp;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    Vector3d tmp_T = baseTgt + baseRgt * gt_position;
    gt_stamps.push_back(odom.header.stamp.toSec());
    gt_vec.push_back(tmp_T);
    auto result = findClosest(gt_stamps, est_stamp);
    if (abs(result.first - est_stamp) > 1.5)
    {
        return;
    }
    tmp_T = gt_vec[result.second];

    odometry.pose.pose.position.x = tmp_T.x();
    odometry.pose.pose.position.y = tmp_T.y();
    odometry.pose.pose.position.z = tmp_T.z();
    std::cout << "gp position is (" << tmp_T.x() << ", " << tmp_T.y() << ")" << std::endl;
    Quaterniond tmp_R = baseRgt * gt_q;

    odometry.pose.pose.orientation.w = tmp_R.w();
    odometry.pose.pose.orientation.x = tmp_R.x();
    odometry.pose.pose.orientation.y = tmp_R.y();
    odometry.pose.pose.orientation.z = tmp_R.z();

    Vector3d tmp_V = {0, 0, 0};
    odometry.twist.twist.linear.x = tmp_V.x();
    odometry.twist.twist.linear.y = tmp_V.y();
    odometry.twist.twist.linear.z = tmp_V.z();
    // pub_odom.publish(odometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(gt_stamps[result.second]);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = odometry.pose.pose;
    
    new_path.header = odometry.header;
    new_path.poses.push_back(pose_stamped);
    pub_gt_path.publish(new_path); 
    return;
}
// Data fidelity residual
struct DataFidelityResidual {
    DataFidelityResidual(double x_original, double y_original) 
        : x_original_(x_original), y_original_(y_original) {}

    template <typename T>
    bool operator()(const T* const xy, T* residual) const {
        residual[0] = xy[0] - T(x_original_);
        residual[1] = xy[1] - T(y_original_);
        return true;
    }

private:
    double x_original_, y_original_;
};

// Smoothness residual
struct SmoothnessResidual {
    SmoothnessResidual(double weight)
        : weight_(weight){}
    template <typename T>
    bool operator()(const T* const xy1, const T* const xy2, const T* const xy3, T* residual) const {
        // Central finite difference approximation of the second derivative for smoothness
        residual[0] = T(weight_) * (xy3[0] - T(2.0) * xy2[0] + xy1[0]);
        residual[1] = T(weight_) * (xy3[1] - T(2.0) * xy2[1] + xy1[1]);
        return true;
    }
private:
    double weight_;
};
void path_callback(const nav_msgs::Path& path)
{
    count_path++;

    if (path.poses.empty()) 
    {
        ROS_WARN("Received empty path!");
        return;
    }
    if (count_path % 2 > 0)
    {
        std::cout << "waiting" << std::endl;
        return;
    }

    // Get the most recent pose (the last one in the path)
    geometry_msgs::PoseStamped recent_pose = path.poses.back();
    est_stamp = recent_pose.header.stamp.toSec();
    // Extract position
    double x = recent_pose.pose.position.x;
    double y = recent_pose.pose.position.y;
    double z = recent_pose.pose.position.z;
    // Extract quaternion orientation
    double qx = recent_pose.pose.orientation.x;
    double qy = recent_pose.pose.orientation.y;
    double qz = recent_pose.pose.orientation.z;
    double qw = recent_pose.pose.orientation.w;

    est_position << x, y, z;
    est_q = Eigen::Quaterniond(qw, qx, qy, qz);
    if (path.poses.size() < 5)
    {
        std::cout << "path.poses.size() is " << path.poses.size() << ", return " << std::endl;
        return;
    }
    int start_id = 0;
    if (path.poses.size() > 100)
    {
        start_id = path.poses.size() - 100;
        if (est_stamp - last_opt_stamp > 30)
        {
            start_id = 0;
            last_opt_stamp = est_stamp;
            std::cout << "now start to global optimizing" << std::endl;
        }
        else 
        {
            std::cout << "est_stamp - last_opt_stamp is " << est_stamp - last_opt_stamp << std::endl;
        }
    }

    ceres::Problem problem;
    // Add data fidelity constraints
    std::vector<Vector2d> data;
    std::cout << "start_id is " << start_id << "; read path, path.poses[0] is " << path.poses[0].pose.position.x << path.poses[0].pose.position.y << std::endl;
    for (size_t i = start_id; i < path.poses.size(); i = i + 1)
    {
        Vector2d pt(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
        data.push_back(pt);
    }
    std::vector<Vector2d> trajectory(data);  // Start with original points
    if (optimized_poses.size() > 0 && start_id > 0)
    {
        for (size_t i = 0; i < optimized_poses.size() - start_id; i++)
        {
            trajectory[i] = optimized_poses[i + start_id];
        }
    }
    std::cout << "trajectory.size is " << trajectory.size() << "; optimized_poses size is " << optimized_poses.size() << "; path.poses.size() is " << path.poses.size() << std::endl;

    for (size_t i = 0; i < trajectory.size(); ++i) 
    {
        std::cout << "before optimization, trajectory[" << i << "] is " << trajectory[i].transpose() << std::endl;

        // std::cout << i << "th data is " << trajectory[i] << std::endl;
        problem.AddResidualBlock(
            new AutoDiffCostFunction<DataFidelityResidual, 2, 2>(
                new DataFidelityResidual(trajectory[i][0], trajectory[i][1])),
            nullptr,  // No loss function, squared loss is used
            trajectory[i].data());
    }
    problem.SetParameterBlockConstant(trajectory[0].data());
    double weight = 3;
    if (trajectory.size() < 20)
    {
        weight = 0.2;
    }
    // Add smoothness constraints
    for (size_t i = 1; i + 1 < trajectory.size(); ++i) 
    {
        // std::cout << "smoothing " << i << "th data is " << trajectory[i] << std::endl;
        problem.AddResidualBlock(
            new AutoDiffCostFunction<SmoothnessResidual, 2, 2, 2, 2>(
                new SmoothnessResidual(weight)),
            nullptr,  // No loss function, squared loss is used
            trajectory[i - 1].data(), trajectory[i].data(), trajectory[i + 1].data());
    }

    Solver::Options options;
    options.linear_solver_type = DENSE_QR;
    options.max_solver_time_in_seconds = 0.05;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << " before solver" << std::endl;
    ceres::Solve(options, &problem, &summary);
    // End time measurement
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate the duration
    std::chrono::duration<double> duration = end - start;

    // Output the duration in seconds
    std::cout << "Total time cost: " << duration.count() << " seconds" << std::endl << "report";

    cout << summary.BriefReport() << endl;
    cout << "Adjusted trajectory:" << endl;
    // for (const auto& point : trajectory) {
    //     cout << point.transpose() << endl;
    // }
    smooth_path.header = path.header;
    int push_id = path.poses.size() - 50;
    if (push_id < 0 || start_id == 0)
    {
        push_id = 0;
    }

    // std::cout << "path.poses.size() is " << path.poses.size() << "; push_id is "<< push_id << std::endl;
    for (size_t i = push_id; i < path.poses.size(); i++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = path.poses[i].header;
        pose_stamped.pose = path.poses[i].pose;
        pose_stamped.pose.position.x = trajectory[i - start_id][0];
        pose_stamped.pose.position.y = trajectory[i - start_id][1];
        std::cout << i << "th point is (" << pose_stamped.pose.position.x << ", " << pose_stamped.pose.position.y << ")" << std::endl;

        if (i < optimized_poses.size())
        {
            optimized_poses[i] = Vector2d(trajectory[i - start_id][0], trajectory[i - start_id][1]);
            smooth_path.poses[i] = pose_stamped;
        }
        else 
        {
            smooth_path.poses.push_back(pose_stamped);
            optimized_poses.push_back(Vector2d(trajectory[i - start_id][0], trajectory[i - start_id][1]));
        }
    }
    std::cout << "smooth_path.poses.size() is " << smooth_path.poses.size() << std::endl;
    pub_sm_path.publish(smooth_path);
    
    return;
}
void loop_callback(auv_nav_msg::DetectLoop msg)
{

    if (msg.connect_num > last_loop_num)
    {
        last_loop_num = msg.connect_num;
        flag_loop = true;
    }
    else 
    {
        flag_loop = false;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mvis_benchmark_node");
    ros::NodeHandle n("~");
    pub_gt_path = n.advertise<nav_msgs::Path>("gt_path", 1000);
    pub_sm_path = n.advertise<nav_msgs::Path>("estimated_path", 1000);

    ros::Subscriber sub_odom = n.subscribe("/Locater/Odom", 1000, gt_callback);
    ros::Subscriber sub_path = n.subscribe("/pose_graph/pose_graph_path", 1000, path_callback);
    ros::Subscriber sub_detect_loop = n.subscribe("/pose_graph/detect_loop", 1000, loop_callback);
    ros::Rate r(20);
    ros::spin();
}

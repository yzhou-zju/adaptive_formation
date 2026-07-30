#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <relative_loc/AnonymousBearingMeas.h>
#include <mutex>
#include <unordered_map>
#include <random>
#include <thread>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>

using namespace std;
using namespace Eigen;

set<int> lidar_drone_id;
set<int> vision_drone_id;

mutex lidar_odom_mutex;
mutex vision_odom_gt_mutex;
unordered_map<int, nav_msgs::Odometry> lidar_odom;
unordered_map<int, nav_msgs::Odometry> vision_odom_gt;

double bearing_rate;

random_device rd;
mt19937 generator(rd());
normal_distribution<double> small_nor_dist;
normal_distribution<double> large_nor_dist;
bernoulli_distribution ber_dist;

void readYaml(const string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    cv::FileNode drone_id_node = fs["drone_id"];
    for (cv::FileNodeIterator it = drone_id_node.begin(); it != drone_id_node.end(); ++it)
    {
        int id = (int)(*it)["id"];
        bool is_lidar = (int)(*it)["is_lidar"];

        if (is_lidar)
            CHECK((lidar_drone_id.insert(id)).second);
        else
            CHECK((vision_drone_id.insert(id)).second);
    }

    bearing_rate = fs["bearing_rate"];
    double noise_stddev = fs["bearing_noise_stddev"];
    small_nor_dist = normal_distribution<double>(0.0, noise_stddev);
    large_nor_dist = normal_distribution<double>(0.0, 2.5);
    double outlier_ratio = fs["outlier_ratio"];
    ber_dist = bernoulli_distribution(outlier_ratio);
    fs.release();
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    int id = stoi(msg->child_frame_id.substr(6, 10));
    if (vision_drone_id.find(id) != vision_drone_id.end())
        return;
    CHECK(lidar_drone_id.find(id) != lidar_drone_id.end());
    lock_guard<mutex> lock(lidar_odom_mutex);
    lidar_odom[id] = *msg;
}

void odomGtCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    int id = stoi(msg->child_frame_id.substr(6, 10));
    CHECK(vision_drone_id.find(id) != vision_drone_id.end());
    lock_guard<mutex> lock(vision_odom_gt_mutex);
    vision_odom_gt[id] = *msg;
}

void computeBearing(ros::Publisher &bearing_pub)
{
    for (int vision_id : vision_drone_id)
    {
        lock_guard<mutex> lock(vision_odom_gt_mutex);
        if (vision_odom_gt.find(vision_id) == vision_odom_gt.end() || abs((ros::Time::now() - vision_odom_gt[vision_id].header.stamp).toSec()) > 0.3)
            continue;

        const auto &vision_pose = vision_odom_gt[vision_id].pose.pose;
        Vector3d vision_position(vision_pose.position.x, vision_pose.position.y, vision_pose.position.z);
        Quaterniond vision_orientation(vision_pose.orientation.w, vision_pose.orientation.x, vision_pose.orientation.y, vision_pose.orientation.z);

        for (int lidar_id : lidar_drone_id)
        {
            lock_guard<mutex> lock(lidar_odom_mutex);
            if (lidar_odom.find(lidar_id) == lidar_odom.end() || abs((ros::Time::now() - lidar_odom[lidar_id].header.stamp).toSec()) > 0.3)
                continue;

            const auto &lidar_pose = lidar_odom[lidar_id].pose.pose;
            Vector3d lidar_position(lidar_pose.position.x, lidar_pose.position.y, lidar_pose.position.z);

            Vector3d bearing = (vision_orientation.inverse() * (lidar_position - vision_position)).normalized();

            if (ber_dist(generator))
            {
                bearing.x() += large_nor_dist(generator);
                bearing.y() += large_nor_dist(generator);
                bearing.z() += large_nor_dist(generator);
            }
            else
            {
                bearing.x() += small_nor_dist(generator);
                bearing.y() += small_nor_dist(generator);
                bearing.z() += small_nor_dist(generator);
            }
            bearing.normalize();

            geometry_msgs::PointStamped bearing_with_noise;
            bearing_with_noise.header = vision_odom_gt[vision_id].header;
            bearing_with_noise.point.x = bearing.x();
            bearing_with_noise.point.y = bearing.y();
            bearing_with_noise.point.z = bearing.z();

            relative_loc::AnonymousBearingMeas bearing_msg;
            bearing_msg.id = vision_id;
            bearing_msg.anonymous_bearing = bearing_with_noise;
            bearing_pub.publish(bearing_msg);
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bearing_generator");
    ros::NodeHandle nh;

    string yaml_file = argv[1];
    readYaml(yaml_file);

    ros::Subscriber odom_sub = nh.subscribe("/others_odom", 10, odomCallback);
    ros::Subscriber odom_gt_sub = nh.subscribe("/others_odom_gt", 10, odomGtCallback);

    ros::Publisher bearing_pub = nh.advertise<relative_loc::AnonymousBearingMeas>("/bearing_meas", 10);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate rate(bearing_rate);
    while (ros::ok())
    {
        computeBearing(bearing_pub);
        rate.sleep();
    }

    return 0;
}
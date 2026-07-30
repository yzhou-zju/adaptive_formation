#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <unordered_map>
#include <thread>

struct DroneInfo
{
    int id;
    std::string odom_gt_topic;
    geometry_msgs::Point offset;
};

double publish_interval;
std::vector<DroneInfo> readYaml(const std::string &filename)
{
    std::vector<DroneInfo> drone_info_list;
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    publish_interval = fs["odom_gt_publish_interval"];

    cv::FileNode drone_info_node = fs["vision_drone_id_and_odom_gt_topic"];
    for (cv::FileNodeIterator it = drone_info_node.begin(); it != drone_info_node.end(); ++it)
    {
        DroneInfo info;
        info.id = (int)(*it)["id"];
        info.odom_gt_topic = (std::string)(*it)["odom_gt_topic"];
        cv::FileNode offset_node = (*it)["init_pos"];
        info.offset.x = (double)offset_node[0];
        info.offset.y = (double)offset_node[1];
        info.offset.z = (double)offset_node[2];
        drone_info_list.push_back(info);
    }

    fs.release();
    return drone_info_list;
}

ros::Publisher pub;
std::unordered_map<std::string, ros::Time> last_publish_time_map;
std::unordered_map<std::string, int> topic_to_id_map;
std::unordered_map<std::string, bool> topic_to_initialized_map;
std::unordered_map<std::string, geometry_msgs::Point> topic_to_first_odom_pos_map;
std::unordered_map<std::string, geometry_msgs::Point> topic_to_offset_map;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, const std::string &topic)
{
    ros::Time current_time = ros::Time::now();
    if (last_publish_time_map.find(topic) != last_publish_time_map.end() &&
        (current_time - last_publish_time_map[topic]).toSec() < publish_interval)
    {
        return;
    }

    if (!topic_to_initialized_map[topic])
    {
        topic_to_first_odom_pos_map[topic] = msg->pose.pose.position;
        topic_to_initialized_map[topic] = true;
    }

    geometry_msgs::Point first_odom_pos = topic_to_first_odom_pos_map[topic];
    geometry_msgs::Point offset = topic_to_offset_map[topic];

    nav_msgs::Odometry modified_msg = *msg;
    modified_msg.header.stamp = current_time;
    modified_msg.pose.pose.position.x = msg->pose.pose.position.x - first_odom_pos.x + offset.x;
    modified_msg.pose.pose.position.y = msg->pose.pose.position.y - first_odom_pos.y + offset.y;
    modified_msg.pose.pose.position.z = msg->pose.pose.position.z - first_odom_pos.z + offset.z;

    modified_msg.child_frame_id = "drone_" + std::to_string(topic_to_id_map[topic]);
    pub.publish(modified_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    last_publish_time_map[topic] = current_time;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_gt_aggregator");
    ros::NodeHandle nh;

    std::string yaml_file = argv[1];
    std::vector<DroneInfo> drone_info_list = readYaml(yaml_file);

    pub = nh.advertise<nav_msgs::Odometry>("/others_odom_gt", 10);

    std::vector<ros::Subscriber> subs;
    for (const auto &drone_info : drone_info_list)
    {
        topic_to_id_map[drone_info.odom_gt_topic] = drone_info.id;
        topic_to_initialized_map[drone_info.odom_gt_topic] = false;
        topic_to_offset_map[drone_info.odom_gt_topic] = drone_info.offset;
        ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(drone_info.odom_gt_topic, 10, boost::bind(odomCallback, _1, drone_info.odom_gt_topic), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
        subs.push_back(sub);
    }

    ros::spin();
    return 0;
}
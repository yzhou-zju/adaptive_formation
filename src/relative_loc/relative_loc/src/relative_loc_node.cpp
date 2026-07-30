#include <algorithm>
#include <chrono>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "nlink_parser/LinktrackNodeframe3.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include "relative_loc/Drift.h"
#include "relative_loc/DistanceMeas.h"
#include "relative_loc/AnonymousBearingMeas.h"
#include "quadrotor_msgs/SwarmInfo.h"
#include "std_msgs/Float32.h"
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <glog/logging.h>

#include "LowPassFilter.hpp"

using namespace std;
using namespace Eigen;

// Measurements
struct Jupiter
{
    int id;
    Vector3d position;
    Quaterniond orientation;
};
struct Ganymede
{
    int id;
    Vector3d position;
    Quaterniond orientation;
};
struct DistMeas
{
    ros::Time timestamp;
    Jupiter jupiter;
    Ganymede ganymede;
    double distance;
};
struct AnonymousBearingMeas
{
    ros::Time timestamp;
    Jupiter jupiter;
    Vector3d bearing;
};
struct BearingMeas
{
    ros::Time timestamp;
    Jupiter jupiter;
    Ganymede ganymede;
    Vector3d bearing;
};

// Decision variable
struct Drift
{
    Vector3d translation = Vector3d::Zero();
    double yaw = 0.0;

    Drift() {}
    Drift(const Vector3d &t, double y) : translation(t), yaw(y) {}
};

// Cost functions
struct RegularizationCostFunctor
{
    RegularizationCostFunctor(const Vector3d &initial_translation, double initial_yaw, double prior_weight, double regularization_weight, bool optimize_z, bool optimize_yaw)
        : initial_translation_(initial_translation), initial_yaw_(initial_yaw), prior_weight_(prior_weight), regularization_weight_(regularization_weight), optimize_z_(optimize_z), optimize_yaw_(optimize_yaw) {}

    template <typename T>
    bool operator()(const T *const translation, const T *const yaw, T *residuals) const
    {
        residuals[0] = T(regularization_weight_) * (translation[0] - T(initial_translation_(0))) + T(prior_weight_) * (translation[0] - T(0.0));
        residuals[1] = T(regularization_weight_) * (translation[1] - T(initial_translation_(1))) + T(prior_weight_) * (translation[1] - T(0.0));
        if (optimize_z_)
            residuals[2] = T(regularization_weight_) * (translation[2] - T(initial_translation_(2))) + T(prior_weight_) * (translation[2] - T(0.0));
        else
            residuals[2] = T(0.0);
        if (optimize_yaw_)
            residuals[3] = T(regularization_weight_) * (*yaw - T(initial_yaw_)) + T(prior_weight_) * (*yaw - T(0.0));
        else
            residuals[3] = T(0.0);
        return true;
    }

    static ceres::CostFunction *Create(const Vector3d &initial_translation, double initial_yaw, double prior_weight, double regularization_weight, bool optimize_z, bool optimize_yaw)
    {
        return (new ceres::AutoDiffCostFunction<RegularizationCostFunctor, 4, 3, 1>(
            new RegularizationCostFunctor(initial_translation, initial_yaw, prior_weight, regularization_weight, optimize_z, optimize_yaw)));
    }

private:
    const Vector3d initial_translation_;
    const double initial_yaw_;
    const double prior_weight_;
    const double regularization_weight_;
    const bool optimize_z_;
    const bool optimize_yaw_;
};
struct DistanceCostFunctor
{
    DistanceCostFunctor(const Vector3d &jupiter_odom_p, const Vector3d &ganymede_odom_p,
                        double distance, double weight, bool optimize_z, bool optimize_yaw)
        : jupiter_odom_p_(jupiter_odom_p), ganymede_odom_p_(ganymede_odom_p), distance_(distance), weight_(weight), optimize_z_(optimize_z), optimize_yaw_(optimize_yaw) {}

    template <typename T>
    bool operator()(const T *const jupiter_drift_p, const T *const jupiter_drift_yaw,
                    const T *const ganymede_drift_p, const T *const ganymede_drift_yaw, T *residuals) const
    {
        // Compute drift quaternion and translation for Jupiter
        Map<const Matrix<T, 3, 1>> jupiter_drift_p_(jupiter_drift_p);
        Quaternion<T> jupiter_drift_q = Quaternion<T>(AngleAxis<T>(*jupiter_drift_yaw, Matrix<T, 3, 1>::UnitZ()));

        // Apply drift to Jupiter's odometry position
        Matrix<T, 3, 1> jupiter_p;
        if (optimize_yaw_)
            jupiter_p = jupiter_drift_q * jupiter_odom_p_.cast<T>() + jupiter_drift_p_;
        else
            jupiter_p = jupiter_odom_p_.cast<T>() + jupiter_drift_p_;

        // Compute drift quaternion and translation for Ganymede
        Map<const Matrix<T, 3, 1>> ganymede_drift_p_(ganymede_drift_p);
        Quaternion<T> ganymede_drift_q = Quaternion<T>(AngleAxis<T>(*ganymede_drift_yaw, Matrix<T, 3, 1>::UnitZ()));

        // Apply drift to Ganymede's odometry position
        Matrix<T, 3, 1> ganymede_p;
        if (optimize_yaw_)
            ganymede_p = ganymede_drift_q * ganymede_odom_p_.cast<T>() + ganymede_drift_p_;
        else
            ganymede_p = ganymede_odom_p_.cast<T>() + ganymede_drift_p_;

        // FIXME: the correct premise here is that drift_q is only rotating in the yaw direction
        if (!optimize_z_)
        {
            jupiter_p(2) = T(jupiter_odom_p_(2));
            ganymede_p(2) = T(ganymede_odom_p_(2));
        }

        Matrix<T, 3, 1> relative_p = ganymede_p - jupiter_p;

        residuals[0] = T(weight_) * (T(distance_) - relative_p.norm());

        return true;
    }

    template <typename T>
    bool operator()(const T *const jupiter_drift_p, const T *const jupiter_drift_yaw, T *residuals) const
    {
        // Compute drift quaternion and translation for Jupiter
        Map<const Matrix<T, 3, 1>> jupiter_drift_p_(jupiter_drift_p);
        Quaternion<T> jupiter_drift_q = Quaternion<T>(AngleAxis<T>(*jupiter_drift_yaw, Matrix<T, 3, 1>::UnitZ()));

        // Apply drift to Jupiter's odometry position
        Matrix<T, 3, 1> jupiter_p;
        if (optimize_yaw_)
            jupiter_p = jupiter_drift_q * jupiter_odom_p_.cast<T>() + jupiter_drift_p_;
        else
            jupiter_p = jupiter_odom_p_.cast<T>() + jupiter_drift_p_;

        // FIXME: the correct premise here is that drift_q is only rotating in the yaw direction
        if (!optimize_z_)
        {
            jupiter_p(2) = T(jupiter_odom_p_(2));
        }

        Matrix<T, 3, 1> relative_p = ganymede_odom_p_.cast<T>() - jupiter_p;

        residuals[0] = T(weight_) * (T(distance_) - relative_p.norm());

        return true;
    }

    static ceres::CostFunction *Create(const Vector3d &jupiter_odom_p, const Vector3d &ganymede_odom_p,
                                       double distance, double weight, bool optimize_z, bool optimize_yaw)
    {
        return (new ceres::AutoDiffCostFunction<DistanceCostFunctor, 1, 3, 1, 3, 1>(
            new DistanceCostFunctor(jupiter_odom_p, ganymede_odom_p, distance, weight, optimize_z, optimize_yaw)));
    }

    static ceres::CostFunction *CreateAnchorCost(const Vector3d &jupiter_odom_p, const Vector3d &ganymede_odom_p,
                                                 double distance, double weight, bool optimize_z, bool optimize_yaw)
    {
        return (new ceres::AutoDiffCostFunction<DistanceCostFunctor, 1, 3, 1>(
            new DistanceCostFunctor(jupiter_odom_p, ganymede_odom_p, distance, weight, optimize_z, optimize_yaw)));
    }

private:
    const Vector3d jupiter_odom_p_;
    const Vector3d ganymede_odom_p_;
    const double distance_;
    const double weight_;
    const bool optimize_z_;
    const bool optimize_yaw_;
};
struct BearingCostFunctor
{
    BearingCostFunctor(const Vector3d &jupiter_odom_p, const Quaterniond &jupiter_odom_q,
                       const Vector3d &ganymede_odom_p, const Quaterniond &ganymede_odom_q,
                       const Vector3d &bearing, double weight, bool optimize_z, bool optimize_yaw)
        : jupiter_odom_p_(jupiter_odom_p), jupiter_odom_q_(jupiter_odom_q), ganymede_odom_p_(ganymede_odom_p), ganymede_odom_q_(ganymede_odom_q), bearing_(bearing), weight_(weight), optimize_z_(optimize_z), optimize_yaw_(optimize_yaw) {}

    template <typename T>
    bool operator()(const T *const jupiter_drift_p, const T *const jupiter_drift_yaw,
                    const T *const ganymede_drift_p, const T *const ganymede_drift_yaw, T *residuals) const
    {
        // Compute drift quaternion and translation for Jupiter
        Map<const Matrix<T, 3, 1>> jupiter_drift_p_(jupiter_drift_p);
        Quaternion<T> jupiter_drift_q = Quaternion<T>(AngleAxis<T>(*jupiter_drift_yaw, Matrix<T, 3, 1>::UnitZ()));

        // Apply drift to Jupiter's odometry
        Matrix<T, 3, 1> jupiter_p;
        Quaternion<T> jupiter_q;
        if (optimize_yaw_)
        {
            jupiter_p = jupiter_drift_q * jupiter_odom_p_.cast<T>() + jupiter_drift_p_;
            jupiter_q = jupiter_drift_q * jupiter_odom_q_.cast<T>();
        }
        else
        {
            jupiter_p = jupiter_odom_p_.cast<T>() + jupiter_drift_p_;
            jupiter_q = jupiter_odom_q_.cast<T>();
        }

        // Compute drift quaternion and translation for Ganymede
        Map<const Matrix<T, 3, 1>> ganymede_drift_p_(ganymede_drift_p);
        Quaternion<T> ganymede_drift_q = Quaternion<T>(AngleAxis<T>(*ganymede_drift_yaw, Matrix<T, 3, 1>::UnitZ()));

        // Apply drift to Ganymede's odometry position
        Matrix<T, 3, 1> ganymede_p;
        if (optimize_yaw_)
            ganymede_p = ganymede_drift_q * ganymede_odom_p_.cast<T>() + ganymede_drift_p_;
        else
            ganymede_p = ganymede_odom_p_.cast<T>() + ganymede_drift_p_;

        // FIXME: the correct premise here is that drift_q is only rotating in the yaw direction
        if (!optimize_z_)
        {
            jupiter_p(2) = T(jupiter_odom_p_(2));
            ganymede_p(2) = T(ganymede_odom_p_(2));
        }

        Matrix<T, 3, 1> bearing_from_odom = (jupiter_q.inverse() * (ganymede_p - jupiter_p)).normalized();

        residuals[0] = T(weight_) * (T(1.0) - bearing_from_odom.dot(bearing_.cast<T>()));

        return true;
    }

    static ceres::CostFunction *Create(const Vector3d &jupiter_odom_p, const Quaterniond &jupiter_odom_q,
                                       const Vector3d &ganymede_odom_p, const Quaterniond &ganymede_odom_q,
                                       const Vector3d &bearing, double weight, bool optimize_z, bool optimize_yaw)
    {
        return (new ceres::AutoDiffCostFunction<BearingCostFunctor, 1, 3, 1, 3, 1>(
            new BearingCostFunctor(jupiter_odom_p, jupiter_odom_q, ganymede_odom_p, ganymede_odom_q, bearing, weight, optimize_z, optimize_yaw)));
    }

private:
    const Vector3d jupiter_odom_p_;
    const Quaterniond jupiter_odom_q_;
    const Vector3d ganymede_odom_p_;
    const Quaterniond ganymede_odom_q_; // FIXME: unused actually
    const Vector3d bearing_;
    const double weight_;
    const bool optimize_z_;
    const bool optimize_yaw_;
};

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;

    ros::init(argc, argv, "relative_loc_node");
    ros::NodeHandle nh("~");

    string hyperparam_path = argv[1];
    cv::FileStorage param(hyperparam_path, cv::FileStorage::READ);

    int glog_severity_level = param["glog_severity_level"];
    switch (glog_severity_level)
    {
    case 0:
        FLAGS_stderrthreshold = google::INFO;
        break;
    case 1:
        FLAGS_stderrthreshold = google::WARNING;
        break;
    case 2:
        FLAGS_stderrthreshold = google::ERROR;
        break;
    case 3:
        FLAGS_stderrthreshold = google::FATAL;
        break;
    default:
        break;
    }

    CHECK(getenv("DRONE_ID") != nullptr);
    int self_id = atoi(getenv("DRONE_ID"));

    // Output thread
    Vector3d drift_p_(0.0, 0.0, 0.0);
    Quaterniond drift_q_(1.0, 0.0, 0.0, 0.0);
    double drift_yaw_ = 0.0;
    bool drift_initialized = false;
    // Enable self revised odom output of lidar drones before initialization
    cv::FileNode id_node = param["drone_id_and_uwb_ext"];
    for (auto it = id_node.begin(); it != id_node.end(); ++it)
    {
        int id = (*it)["id"];
        if (self_id == id)
        {
            bool is_lidar = (int)(*it)["is_lidar"];
            if (is_lidar)
                drift_initialized = true;
            break;
        }
    }
    auto applyDrift = [](const Vector3d &drift_p, const Quaterniond &drift_q,
                         Vector3d &odom_p, Quaterniond &odom_q)
    {
        odom_p = drift_q * odom_p + drift_p;
        odom_q = drift_q * odom_q;
    };

    LowPassFilter<double> output_filter_x(param["cutoff_frequency_revised_p"], param["max_delta_revised_p"]);
    LowPassFilter<double> output_filter_y(param["cutoff_frequency_revised_p"], param["max_delta_revised_p"]);
    ros::Publisher revised_odom_pub = nh.advertise<nav_msgs::Odometry>(param["revised_odom_topic"], 1000);
    auto odomCallback = [&](const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (drift_initialized)
        {
            Vector3d revised_odom_p(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z);
            Quaterniond revised_odom_q(msg->pose.pose.orientation.w,
                                       msg->pose.pose.orientation.x,
                                       msg->pose.pose.orientation.y,
                                       msg->pose.pose.orientation.z);

            applyDrift(drift_p_, drift_q_, revised_odom_p, revised_odom_q);

            double now = chrono::duration_cast<chrono::duration<double>>(chrono::high_resolution_clock::now().time_since_epoch()).count();
            output_filter_x.input(revised_odom_p.x(), now);
            output_filter_y.input(revised_odom_p.y(), now);

            nav_msgs::Odometry revised_odom_msg(*msg);
            revised_odom_msg.pose.pose.position.x = output_filter_x.output();
            revised_odom_msg.pose.pose.position.y = output_filter_y.output();
            revised_odom_msg.pose.pose.position.z = revised_odom_p.z();
            revised_odom_msg.pose.pose.orientation.w = revised_odom_q.w();
            revised_odom_msg.pose.pose.orientation.x = revised_odom_q.x();
            revised_odom_msg.pose.pose.orientation.y = revised_odom_q.y();
            revised_odom_msg.pose.pose.orientation.z = revised_odom_q.z();

            revised_odom_pub.publish(revised_odom_msg);
        }
    };
    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("self_odom", 1000,
                                         odomCallback,
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    LowPassFilter<Vector3d> drift_filter_p(param["cutoff_frequency_drift_p"], param["max_delta_drift_p"]);
    LowPassFilter<double> drift_filter_yaw(param["cutoff_frequency_drift_yaw"], (double)param["max_delta_drift_yaw"] * M_PI / 180.0);
    double max_abs_yaw = (double)param["max_abs_yaw"] * M_PI / 180.0;
    bool publish_debug_topics = (int)param["publish_debug_topics"];
    ros::Publisher drift_after_lpf_pub = nh.advertise<relative_loc::Drift>(param["drift_after_lpf_topic"], 1000);
    auto filterDrift = [&]()
    {
        drift_yaw_ = clamp(drift_yaw_, -max_abs_yaw, max_abs_yaw);

        double now = chrono::duration_cast<chrono::duration<double>>(chrono::high_resolution_clock::now().time_since_epoch()).count();
        drift_filter_p.input(drift_p_, now);
        drift_p_ = drift_filter_p.output();

        drift_filter_yaw.input(drift_yaw_, now);
        drift_q_ = Quaterniond(AngleAxisd(drift_filter_yaw.output(), Vector3d::UnitZ()));

        if (publish_debug_topics)
        {
            relative_loc::Drift drift_filtered_with_msg;
            drift_filtered_with_msg.id = self_id;
            drift_filtered_with_msg.drift.header.stamp = ros::Time::now();

            drift_filtered_with_msg.drift.pose.position.x = drift_p_.x();
            drift_filtered_with_msg.drift.pose.position.y = drift_p_.y();
            drift_filtered_with_msg.drift.pose.position.z = drift_p_.z();

            drift_filtered_with_msg.drift.pose.orientation.w = drift_q_.w();
            drift_filtered_with_msg.drift.pose.orientation.x = drift_q_.x();
            drift_filtered_with_msg.drift.pose.orientation.y = drift_q_.y();
            drift_filtered_with_msg.drift.pose.orientation.z = drift_q_.z();

            drift_after_lpf_pub.publish(drift_filtered_with_msg);
        }
    };

    // Input buffers
    map<int, deque<nav_msgs::Odometry>> swarm_odom_raw;
    map<int, deque<relative_loc::DistanceMeas>> swarm_dist_meas_raw;
    map<int, deque<relative_loc::AnonymousBearingMeas>> swarm_anonymous_bearing_meas_raw;
    deque<DistMeas> swarm_dist_meas;
    deque<AnonymousBearingMeas> swarm_anonymous_bearing_meas;
    deque<BearingMeas> swarm_bearing_meas;

    // Decision variable
    map<int, Drift> swarm_drift;
    swarm_drift.clear();
    mutex swarm_drift_mtx;

    // Configs and utils
    int center_id;
    set<int> drone_id_all; // Contains the ids of drones in all the swarms
    set<int> drone_id;     // Contains only the ids of drones in the belonging swarm
    map<int, bool> is_lidar_all;
    map<int, bool> is_lidar;
    map<int, Vector3d> uwb_extrinsic_all;
    map<int, Vector3d> uwb_extrinsic;
    map<int, bool> drift_initialized_;
    map<int, shared_ptr<LowPassFilter<double>>> swarm_drift_opt_yaw_filter;
    map<int, Vector3d> swarm_drift_prev_p;
    map<int, double> swarm_drift_prev_yaw;
    swarm_drift_opt_yaw_filter.clear();
    swarm_drift_prev_p.clear();
    swarm_drift_prev_yaw.clear();
    map<int, bool> inform_no_odom;
    map<int, bool> inform_insufficient_dist_meas;
    bool recv_swarm_info = false;
    auto reset = [&]()
    {
        // Reset everything
        swarm_odom_raw.clear();
        swarm_dist_meas_raw.clear();
        swarm_anonymous_bearing_meas_raw.clear();
        swarm_dist_meas.clear();
        swarm_anonymous_bearing_meas.clear();
        swarm_bearing_meas.clear();

        is_lidar.clear();
        uwb_extrinsic.clear();
        drift_initialized_.clear();
        inform_no_odom.clear();
        inform_insufficient_dist_meas.clear();

        if (!recv_swarm_info)
            for (int id : drone_id)
                swarm_drift[id] = Drift(); // Initialize to zero
        else
            for (int id : drone_id)
                if (swarm_drift.find(id) == swarm_drift.end())
                    swarm_drift[id] = Drift(); // Initialize to zero

        for (auto id : drone_id)
        {
            is_lidar[id] = is_lidar_all[id];
            uwb_extrinsic[id] = uwb_extrinsic_all[id];
        }

        for (auto id : drone_id)
        {
            if (is_lidar[id])
                drift_initialized_[id] = true;
            else
                drift_initialized_[id] = false;
        }

        for (int id : drone_id)
        {
            if (!is_lidar[id] && swarm_drift_opt_yaw_filter.find(id) == swarm_drift_opt_yaw_filter.end())
            {
                swarm_drift_opt_yaw_filter.emplace(id, make_shared<LowPassFilter<double>>(0.1, 23333));
                swarm_drift_opt_yaw_filter[id]->input(0.0, chrono::duration_cast<chrono::duration<double>>(chrono::high_resolution_clock::now().time_since_epoch()).count());

                swarm_drift_prev_p.emplace(id, Vector3d::Zero());
                swarm_drift_prev_yaw.emplace(id, 0.0);
            }
        }

        for (auto id : drone_id)
            inform_no_odom[id] = true;

        for (auto id : drone_id)
            inform_insufficient_dist_meas[id] = true;
    };

    // Configs for all the swarms
    for (auto it = id_node.begin(); it != id_node.end(); ++it)
    {
        int id = (*it)["id"];
        CHECK((drone_id_all.insert(id)).second);

        bool is_lidar_ = (int)(*it)["is_lidar"];
        is_lidar_all[id] = is_lidar_;

        vector<double> ext;
        (*it)["uwb_ext"] >> ext;
        CHECK(ext.size() == 3) << "We live in a 3 dimensional world, homie ^^";
        uwb_extrinsic_all[id] = Vector3d(ext[0], ext[1], ext[2]);
    }
    CHECK(drone_id_all.find(self_id) != drone_id_all.end());

    // Anchor configs
    set<int> uwb_anchor_id;
    map<int, Vector3d> uwb_anchor_position;
    cv::FileNode uwb_node = param["uwb_anchor_id_and_position"];
    for (auto it = uwb_node.begin(); it != uwb_node.end(); ++it)
    {
        int id = (*it)["id"];
        CHECK((uwb_anchor_id.insert(id)).second);

        vector<double> pos;
        (*it)["position"] >> pos;
        CHECK(pos.size() == 3) << "We live in a 3 dimensional world, homie ^^";
        uwb_anchor_position[id] = Vector3d(pos[0], pos[1], pos[2]);
    }
    auto checkNoDuplicateID = [](const set<int> &drone_id, const set<int> &uwb_anchor_id)
    {
        set<int> union_;
        set_union(drone_id.begin(), drone_id.end(),
                  uwb_anchor_id.begin(), uwb_anchor_id.end(),
                  inserter(union_, union_.begin()));
        return union_.size() == drone_id.size() + uwb_anchor_id.size();
    };

    mutex swarm_odom_reset_mtx,
        dist_reset_mtx,
        bearing_reset_mtx,
        align_reset_mtx,
        opt_reset_mtx;
    auto swarmInfoCallback = [&](const quadrotor_msgs::SwarmInfo::ConstPtr &msg)
    {
        // Waiting for all threads to finish executing
        lock_guard<mutex> lock0(swarm_odom_reset_mtx);
        lock_guard<mutex> lock1(dist_reset_mtx);
        lock_guard<mutex> lock2(bearing_reset_mtx);
        lock_guard<mutex> lock3(align_reset_mtx);
        lock_guard<mutex> lock4(opt_reset_mtx);

        set<int> fake_drone_id;
        for (auto id : msg->fake_drones_ids)
            fake_drone_id.insert(id);

        set<int> drone_id_;
        for (auto id : msg->robot_ids)
        {
            if (fake_drone_id.find(id) != fake_drone_id.end())
                continue;
            drone_id_.insert(id);
        }

        for (auto id : drone_id_)
        {
            if (drone_id_all.find(id) == drone_id_all.end())
            {
                LOG(ERROR) << "!(drone_id_all.find(id) != drone_id_all.end()) #^#";
                return;
            }
        }

        if (drone_id_.find(msg->leader_id) == drone_id_.end())
        {
            LOG(ERROR) << "leader_id is not in the robot_ids #^#";
            return;
        }
        if (drone_id_.find(self_id) == drone_id_.end())
        {
            LOG(ERROR) << "Self id is not in the swarm info received #^#";
            return;
        }

        if (recv_swarm_info)
        {
            set<int> prev_vision;
            set<int> curr_vision;

            for (int id : drone_id)
                if (!is_lidar_all[id])
                    prev_vision.insert(id);
            for (int id : drone_id_)
                if (!is_lidar_all[id])
                    curr_vision.insert(id);

            if (prev_vision != curr_vision)
                LOG(WARNING) << "Group changes occur within visual drones #^#";
        }

        if (!checkNoDuplicateID(drone_id_, uwb_anchor_id))
        {
            LOG(ERROR) << "!checkNoDuplicateID(drone_id_, uwb_anchor_id) #^#";
            return;
        }

        center_id = msg->leader_id; // Assume the swarm leader is the relative localization center
        drone_id = drone_id_;

        reset();

        recv_swarm_info = true;
        LOG(INFO) << "Swarm info received 😆😆😆";
    };
    ros::Subscriber swarm_info_sub =
        nh.subscribe<quadrotor_msgs::SwarmInfo>(param["swarm_info_topic"], 1000,
                                                swarmInfoCallback,
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());

    bool init_from_swarm_info = (int)param["init_from_swarm_info"];
    if (init_from_swarm_info)
    {
        LOG(INFO) << "Waiting for swarm info ...";
        while (!recv_swarm_info && ros::ok())
        {
            ros::spinOnce();
            this_thread::sleep_for(chrono::milliseconds(5));
        }
    }
    else
    {
        LOG(INFO) << "Loading relative localization center id from environment variable ...";
        CHECK(getenv("REL_LOC_CENTER_ID") != nullptr);
        center_id = atoi(getenv("REL_LOC_CENTER_ID"));

        drone_id = drone_id_all;
        CHECK(checkNoDuplicateID(drone_id, uwb_anchor_id));
        is_lidar = is_lidar_all;
        uwb_extrinsic = uwb_extrinsic_all;

        reset();
    }

    if (center_id == self_id)
    {
        cout << "\033[32m============= This is the central node of the relative localization system *~* =============\033[0m" << endl;

        double sliding_window_length = param["sliding_window_length"];
        mutex swarm_odom_mtx,
            dist_meas_raw_mtx,
            dist_meas_mtx,
            anonymous_bearing_meas_raw_mtx,
            anonymous_bearing_meas_mtx,
            bearing_meas_mtx;

        double swarm_odom_freq = param["swarm_odom_freq"];
        auto swarmOdomCallback = [&](const nav_msgs::Odometry::ConstPtr &msg)
        {
            lock_guard<mutex> lock0(swarm_odom_reset_mtx);

            CHECK_EQ(msg->child_frame_id.substr(0, 6), "drone_");

            int id = atoi(msg->child_frame_id.substr(6, 10).c_str());
            if (drone_id_all.find(id) == drone_id_all.end())
            {
                LOG(ERROR) << "Drone " << id << " is not in any swarm #^#";
                return;
            }

            if (drone_id.find(id) == drone_id.end())
                return;

            if (abs((msg->header.stamp - ros::Time::now()).toSec()) > 1.0)
                LOG(WARNING) << "Timestamp of a odom from drone " << id << " is more than 1.0s later (or earlier) than current time @_@ T_odom - T_now = " << (msg->header.stamp - ros::Time::now()).toSec() << "s";

            lock_guard<mutex> lock1(swarm_odom_mtx);

            swarm_odom_raw[id].push_back(*msg);

            if (swarm_odom_raw[id].size() > 5.20 * sliding_window_length * swarm_odom_freq)
                swarm_odom_raw[id].pop_front();
        };
        ros::Subscriber swarm_odom_sub =
            nh.subscribe<nav_msgs::Odometry>(param["swarm_odom_topic"], 1000,
                                             swarmOdomCallback,
                                             ros::VoidConstPtr(),
                                             ros::TransportHints().tcpNoDelay());

        double dist_meas_freq = param["distance_measurement_freq"];
        auto distanceCallback = [&](const relative_loc::DistanceMeas::ConstPtr &msg)
        {
            lock_guard<mutex> lock0(dist_reset_mtx);

            int id = msg->distance_meas.id;
            if (drone_id.find(id) == drone_id.end())
            {
                LOG(ERROR) << "UWB id (= " << id << ") of an receiving distance measurement is not in the belonging swarm #^#";
                return;
            }

            if (uwb_anchor_id.find(id) != uwb_anchor_id.end())
            {
                LOG(ERROR) << "Drone " << id << " is a fxxking uwb anchor #^#";
                return;
            }

            if (abs((msg->header.stamp - ros::Time::now()).toSec()) > 1.0)
                LOG(WARNING) << "Timestamp of a distance measurement from drone " << id << " is more than 1.0s later (or earlier) than current time @_@ T_meas - T_now = " << (msg->header.stamp - ros::Time::now()).toSec() << "s";

            set<int> swarm_uwb_id;
            swarm_uwb_id.insert(id);
            for (const nlink_parser::LinktrackNode2 &ganymede : msg->distance_meas.nodes)
                swarm_uwb_id.insert(ganymede.id);
            if (!includes(swarm_uwb_id.begin(), swarm_uwb_id.end(), drone_id.begin(), drone_id.end()))
                LOG(ERROR) << "Not all drones in the swarm can conduct distance measurements with each other #^#";

            lock_guard<mutex> lock1(dist_meas_raw_mtx);

            swarm_dist_meas_raw[id].push_back(*msg);

            if (swarm_dist_meas_raw[id].size() > 5.20 * sliding_window_length * dist_meas_freq)
                swarm_dist_meas_raw[id].pop_front();
        };
        ros::Subscriber distance_sub =
            nh.subscribe<relative_loc::DistanceMeas>(param["distance_measurement_topic"], 1000,
                                                     distanceCallback,
                                                     ros::VoidConstPtr(),
                                                     ros::TransportHints().tcpNoDelay());

        atomic<bool> enable_bearing((int)param["enable_bearing"]); // FIXME: a lot of implicit conversions in this code #^#
        double bearing_meas_freq = param["bearing_measurement_freq"];
        auto bearingCallback = [&](const relative_loc::AnonymousBearingMeas::ConstPtr &msg)
        {
            lock_guard<mutex> lock0(bearing_reset_mtx);

            if (drone_id.find(msg->id) == drone_id.end())
            {
                LOG(ERROR) << "Drone id (= " << msg->id << ") of an receiving bearing measurement is not in the belonging swarm #^#";
                return;
            }
            if (abs((msg->anonymous_bearing.header.stamp - ros::Time::now()).toSec()) > 1.0)
                LOG(WARNING) << "Timestamp of a bearing measurement from drone " << msg->id << " is more than 1.0s later (or earlier) than current time @_@ T_meas - T_now = " << (msg->anonymous_bearing.header.stamp - ros::Time::now()).toSec() << "s";

            lock_guard<mutex> lock1(anonymous_bearing_meas_raw_mtx);

            swarm_anonymous_bearing_meas_raw[msg->id].push_back(*msg);

            if (swarm_anonymous_bearing_meas_raw[msg->id].size() > 5.20 * sliding_window_length * bearing_meas_freq)
                swarm_anonymous_bearing_meas_raw[msg->id].pop_front();
        };
        ros::Subscriber bearing_sub;
        if (enable_bearing)
            bearing_sub =
                nh.subscribe<relative_loc::AnonymousBearingMeas>(param["bearing_measurement_topic"], 1000,
                                                                 bearingCallback,
                                                                 ros::VoidConstPtr(),
                                                                 ros::TransportHints().tcpNoDelay());

        ros::AsyncSpinner spinner(6);
        spinner.start();

        auto alignMeasAndOdomByTimestamp = [&](const int id, const ros::Time &meas_timestamp, geometry_msgs::Pose &pose_from_odom)
        {
            lock_guard<mutex> lock(swarm_odom_mtx);

            const deque<nav_msgs::Odometry> &odoms = swarm_odom_raw[id];
            if (odoms.empty())
            {
                if (inform_no_odom[id])
                {
                    LOG(WARNING) << "No odom received from drone " << id << " #^#";
                    inform_no_odom[id] = false;
                }
                return false;
            }
            else
                inform_no_odom[id] = true;

            const double tolerance = 3.0 / swarm_odom_freq; // 3 odom periods
            if (meas_timestamp < odoms.front().header.stamp - ros::Duration(tolerance) ||
                meas_timestamp > odoms.back().header.stamp + ros::Duration(tolerance))
            {
                LOG(ERROR) << ((meas_timestamp > odoms.back().header.stamp + ros::Duration(tolerance)) ? "Drone " + to_string(id) + "'s distance or bearing measurement time >> lastest odom time (" + to_string((meas_timestamp - odoms.back().header.stamp).toSec()) + "s) #^#" : "Drone " + to_string(id) + "'s distance or bearing measurement time << earliest odom time (" + to_string((odoms.front().header.stamp - meas_timestamp).toSec()) + "s) #^#");
                return false;
            }

            // Find the closest odom by iterating from the end
            auto closest_it = odoms.rbegin();
            double closest_diff = numeric_limits<double>::infinity();

            for (auto it = odoms.rbegin(); it != odoms.rend(); ++it)
            {
                double diff = abs((meas_timestamp - it->header.stamp).toSec());
                if (diff < closest_diff)
                {
                    closest_diff = diff;
                    closest_it = it;
                }
                else
                {
                    // Since the deque is ordered, we can break early if the difference starts increasing
                    break;
                }
            }
            if (closest_diff > 2.0 / swarm_odom_freq) // 2 odom periods
                LOG(WARNING) << "Distance from meas to drone " << id << "'s nearest odom is " << closest_diff << "s";

            pose_from_odom = (*closest_it).pose.pose;
            return true;
        };
        bool consider_uwb_ext = (int)param["consider_uwb_extrinsic"];
        double uwb_bias = param["uwb_bias"];
        auto applyDrift = [](const Drift &drift, Vector3d &odom_p, Quaterniond &odom_q)
        {
            Quaterniond drift_q(AngleAxisd(drift.yaw, Vector3d::UnitZ()));
            odom_p = drift_q * odom_p + drift.translation;
            odom_q = drift_q * odom_q;
        };
        bool custom_debug_output = (int)param["custom_debug_output"];
        ros::Publisher swarm_dist_est_pub = nh.advertise<std_msgs::Float32>(param["swarm_dist_est_topic"], 1000);
        ros::Publisher swarm_dist_meas_pub = nh.advertise<std_msgs::Float32>(param["swarm_dist_meas_topic"], 1000);
        thread align_meas_and_odom_by_timestamp_thread(
            [&]()
            {
                ros::Rate r(max({swarm_odom_freq, dist_meas_freq, bearing_meas_freq}));
                while (ros::ok())
                {
                    {
                        lock_guard<mutex> lock(align_reset_mtx);
                        { // Align distance measurements with odom
                            lock_guard<mutex> lock(dist_meas_raw_mtx);
                            for (auto &[jupiter_id, jupiters] : swarm_dist_meas_raw)
                            {
                                while (!jupiters.empty())
                                {
                                    const nlink_parser::LinktrackNodeframe3 &jupiter_raw = jupiters.front().distance_meas;

                                    ros::Time jupiter_time = jupiters.front().header.stamp;
                                    geometry_msgs::Pose pose_from_odom;
                                    bool jupiter_odom_found = alignMeasAndOdomByTimestamp(jupiter_id, jupiter_time, pose_from_odom);
                                    if (!jupiter_odom_found)
                                    {
                                        jupiters.pop_front();
                                        continue;
                                    }

                                    Jupiter jupiter;
                                    jupiter.id = jupiter_id;
                                    jupiter.position = Vector3d(pose_from_odom.position.x,
                                                                pose_from_odom.position.y,
                                                                pose_from_odom.position.z);
                                    jupiter.orientation = Quaterniond(pose_from_odom.orientation.w,
                                                                      pose_from_odom.orientation.x,
                                                                      pose_from_odom.orientation.y,
                                                                      pose_from_odom.orientation.z);
                                    if (consider_uwb_ext)
                                        jupiter.position += jupiter.orientation * uwb_extrinsic[jupiter_id];

                                    for (const nlink_parser::LinktrackNode2 &ganymede_raw : jupiter_raw.nodes)
                                    {
                                        int ganymede_id = ganymede_raw.id;
                                        bool ganymede_is_uwb_anchor = uwb_anchor_id.find(ganymede_id) != uwb_anchor_id.end();

                                        if (!(drone_id_all.find(ganymede_id) != drone_id_all.end() || ganymede_is_uwb_anchor))
                                        {
                                            LOG(ERROR) << "There's a ganymede uwb id that doesn't belong to any of the swarms and is not an anchor id #^#";
                                            continue;
                                        }

                                        if (drone_id.find(ganymede_id) == drone_id.end())
                                            continue;

                                        Ganymede ganymede;
                                        ganymede.id = ganymede_id;

                                        if (ganymede_is_uwb_anchor)
                                        {
                                            ganymede.position = uwb_anchor_position[ganymede_id];
                                            ganymede.orientation = Quaterniond(1.0, 0.0, 0.0, 0.0);
                                        }
                                        else
                                        {
                                            ros::Time ganymede_time = jupiter_time;

                                            geometry_msgs::Pose pose_from_odom;
                                            bool ganymede_odom_found = alignMeasAndOdomByTimestamp(ganymede_id, ganymede_time, pose_from_odom);
                                            if (!ganymede_odom_found)
                                                continue;

                                            ganymede.position = Vector3d(pose_from_odom.position.x,
                                                                         pose_from_odom.position.y,
                                                                         pose_from_odom.position.z);
                                            ganymede.orientation = Quaterniond(pose_from_odom.orientation.w,
                                                                               pose_from_odom.orientation.x,
                                                                               pose_from_odom.orientation.y,
                                                                               pose_from_odom.orientation.z);
                                            if (consider_uwb_ext)
                                                ganymede.position += ganymede.orientation * uwb_extrinsic[ganymede_id];
                                        }

                                        double distance = ganymede_raw.dis + uwb_bias;
                                        // FIXME: a dirty trick
                                        if (distance < 1.3)
                                            distance *= 0.8;

                                        DistMeas dist_meas;
                                        dist_meas.timestamp = jupiter_time;
                                        dist_meas.jupiter = jupiter;
                                        dist_meas.ganymede = ganymede;
                                        dist_meas.distance = distance;

                                        {
                                            lock_guard<mutex> lock(dist_meas_mtx);
                                            swarm_dist_meas.push_back(dist_meas);
                                            // TODO: a lot of copy so far #^# Try to use move instead
                                        }

                                        if (publish_debug_topics)
                                        {
                                            Vector3d j_p(jupiter.position);
                                            Quaterniond j_q(jupiter.orientation);
                                            Vector3d g_p(ganymede.position);
                                            Quaterniond g_q(ganymede.orientation);

                                            {
                                                lock_guard<mutex> lock(swarm_drift_mtx);
                                                applyDrift(swarm_drift[jupiter_id], j_p, j_q);
                                                applyDrift(swarm_drift[ganymede_id], g_p, g_q);
                                            }

                                            std_msgs::Float32 dist_est_msg;
                                            dist_est_msg.data = (j_p - g_p).norm();
                                            swarm_dist_est_pub.publish(dist_est_msg);

                                            if (abs(dist_est_msg.data - distance) < 25.0)
                                            {
                                                std_msgs::Float32 dist_meas_msg;
                                                dist_meas_msg.data = distance;
                                                swarm_dist_meas_pub.publish(dist_meas_msg);
                                            }

                                            this_thread::sleep_for(chrono::milliseconds(1));
                                        }
                                    }
                                    jupiters.pop_front();
                                }
                            }

                            // FIXME: swarm_dist_meas is not ordered in time #^#
                            {
                                lock_guard<mutex> lock(dist_meas_mtx);
                                while (!swarm_dist_meas.empty() && swarm_dist_meas.front().timestamp < ros::Time::now() - ros::Duration(sliding_window_length))
                                    swarm_dist_meas.pop_front();
                            }
                        }

                        if (enable_bearing)
                        { // Align anonymous bearing measurements with odom
                            lock_guard<mutex> lock(anonymous_bearing_meas_raw_mtx);
                            for (auto &[jupiter_id, jupiters] : swarm_anonymous_bearing_meas_raw)
                            {
                                while (!jupiters.empty())
                                {
                                    const geometry_msgs::PointStamped &jupiter_raw = jupiters.front().anonymous_bearing;
                                    if (jupiters.front().id != jupiter_id)
                                    {
                                        LOG(ERROR) << "jupiters.front().id != jupiter_id #^#";
                                        continue;
                                    }

                                    ros::Time jupiter_time = jupiter_raw.header.stamp;
                                    geometry_msgs::Pose pose_from_odom;
                                    bool jupiter_odom_found = alignMeasAndOdomByTimestamp(jupiter_id, jupiter_time, pose_from_odom);
                                    if (!jupiter_odom_found)
                                    {
                                        jupiters.pop_front();
                                        continue;
                                    }

                                    Jupiter jupiter;
                                    jupiter.id = jupiter_id;
                                    jupiter.position = Vector3d(pose_from_odom.position.x,
                                                                pose_from_odom.position.y,
                                                                pose_from_odom.position.z);
                                    jupiter.orientation = Quaterniond(pose_from_odom.orientation.w,
                                                                      pose_from_odom.orientation.x,
                                                                      pose_from_odom.orientation.y,
                                                                      pose_from_odom.orientation.z);

                                    Vector3d bearing(jupiter_raw.point.x,
                                                     jupiter_raw.point.y,
                                                     jupiter_raw.point.z);
                                    CHECK_DOUBLE_EQ(bearing.norm(), 1.0);

                                    AnonymousBearingMeas bearing_meas;
                                    bearing_meas.timestamp = jupiter_time;
                                    bearing_meas.jupiter = jupiter;
                                    bearing_meas.bearing = bearing;

                                    {
                                        lock_guard<mutex> lock(anonymous_bearing_meas_mtx);
                                        swarm_anonymous_bearing_meas.push_back(bearing_meas);
                                        // TODO: a lot of copy so far #^# Use move instead
                                    }

                                    jupiters.pop_front();
                                }
                            }
                        }
                    }
                    r.sleep();
                }
            });

        int ceres_verbosity_level = param["ceres_verbosity_level"];
        CHECK(ceres_verbosity_level == 0 || ceres_verbosity_level == 1 || ceres_verbosity_level == 2) << "Invalid Ceres verbosity level #^#";
        double prior_cost_weight = param["prior_cost_weight"];
        double regularization_cost_weight = param["regularization_cost_weight"];
        double distance_cost_weight = param["distance_cost_weight"];
        double bearing_cost_weight = param["bearing_cost_weight"];
        double distance_outlier_thr = param["distance_outlier_threshold"];
        bool optimize_z = (int)param["optimize_z"];
        bool optimize_yaw = (int)param["optimize_yaw"];
        double max_deanonymization_bearing_angle = (double)param["max_deanonymization_bearing_angle"] * M_PI / 180.0;
        double huber_threshold = param["huber_threshold"];
        bool drift_updated = false; // Indicates that the drift of at least one drone has been updated by optimization

        auto setupDistAndRegCost = [&](ceres::Problem &problem)
        {
            lock_guard<mutex> lock(swarm_drift_mtx);

            int N_dist_meas_;
            map<int, int> N_dist_meas;
            map<int, bool> sufficient_dist_meas;
            for (int id : drone_id)
            {
                N_dist_meas[id] = 0;
                sufficient_dist_meas[id] = false;
            }

            { // Distance measurement cost
                lock_guard<mutex> lock(dist_meas_mtx);
                N_dist_meas_ = swarm_dist_meas.size();

                for (const DistMeas &dist_meas : swarm_dist_meas)
                {
                    // Keep sliding_window_length seconds measurements in the sliding window
                    if (dist_meas.timestamp < ros::Time::now() - ros::Duration(sliding_window_length))
                        continue;

                    // FIXME: enhance outlier detection
                    Vector3d jupiter_p(dist_meas.jupiter.position);
                    Quaterniond jupiter_q(dist_meas.jupiter.orientation);
                    Vector3d ganymede_p(dist_meas.ganymede.position);
                    Quaterniond ganymede_q(dist_meas.ganymede.orientation);
                    applyDrift(swarm_drift[dist_meas.jupiter.id], jupiter_p, jupiter_q);
                    applyDrift(swarm_drift[dist_meas.ganymede.id], ganymede_p, ganymede_q);
                    if (abs(dist_meas.distance - (jupiter_p - ganymede_p).norm()) > distance_outlier_thr)
                        continue;

                    bool ganymede_is_uwb_anchor = uwb_anchor_id.find(dist_meas.ganymede.id) != uwb_anchor_id.end();
                    if (ganymede_is_uwb_anchor)
                    {
                        ceres::CostFunction *distance_cost =
                            DistanceCostFunctor::CreateAnchorCost(dist_meas.jupiter.position,
                                                                  dist_meas.ganymede.position,
                                                                  dist_meas.distance,
                                                                  distance_cost_weight,
                                                                  optimize_z,
                                                                  optimize_yaw);

                        ceres::LossFunction *huber_kernel = new ceres::HuberLoss(huber_threshold);

                        Drift &jupiter_drift = swarm_drift[dist_meas.jupiter.id];

                        problem.AddResidualBlock(distance_cost, huber_kernel,
                                                 jupiter_drift.translation.data(), &jupiter_drift.yaw);

                        N_dist_meas[dist_meas.jupiter.id]++;
                    }
                    else
                    {
                        ceres::CostFunction *distance_cost = DistanceCostFunctor::Create(dist_meas.jupiter.position,
                                                                                         dist_meas.ganymede.position,
                                                                                         dist_meas.distance,
                                                                                         distance_cost_weight,
                                                                                         optimize_z,
                                                                                         optimize_yaw);

                        ceres::LossFunction *huber_kernel = new ceres::HuberLoss(huber_threshold);

                        Drift &jupiter_drift = swarm_drift[dist_meas.jupiter.id];
                        Drift &ganymede_drift = swarm_drift[dist_meas.ganymede.id];

                        problem.AddResidualBlock(distance_cost, huber_kernel,
                                                 jupiter_drift.translation.data(), &jupiter_drift.yaw,
                                                 ganymede_drift.translation.data(), &ganymede_drift.yaw);

                        N_dist_meas[dist_meas.jupiter.id]++;
                        N_dist_meas[dist_meas.ganymede.id]++;
                    }
                }
            }

            // Regularization cost: keep the drift close to initial value
            for (auto &[id, drift] : swarm_drift)
            {
                if (drone_id.find(id) != drone_id.end())
                {
                    ceres::CostFunction *regularization_cost = RegularizationCostFunctor::Create(drift.translation,
                                                                                                 drift.yaw,
                                                                                                 N_dist_meas[id] * prior_cost_weight,
                                                                                                 N_dist_meas[id] * regularization_cost_weight,
                                                                                                 optimize_z,
                                                                                                 optimize_yaw);

                    problem.AddResidualBlock(regularization_cost, nullptr,
                                             drift.translation.data(), &drift.yaw);
                }
            }

            // FIXME: improve the condition for ensuring sufficient measurements in the sliding window
            for (int id : drone_id)
            {
                if (N_dist_meas[id] > 0.5 * sliding_window_length * dist_meas_freq)
                {
                    if (!is_lidar[id])
                        drift_updated = true;
                    drift_initialized_[id] = true;
                    sufficient_dist_meas[id] = true;
                    inform_insufficient_dist_meas[id] = true;
                }
            }

            for (auto &[id, ready_for_opt] : sufficient_dist_meas)
            {
                if (!ready_for_opt)
                {
                    Drift &drift = swarm_drift[id];
                    problem.SetParameterBlockConstant(drift.translation.data());
                    problem.SetParameterBlockConstant(&drift.yaw);
                    if (inform_insufficient_dist_meas[id])
                    {
                        LOG(WARNING) << "Insufficient distance measurements associated with drone " << id << ", fixing its drift @_@";
                        inform_insufficient_dist_meas[id] = false;
                    }
                }
            }

            return N_dist_meas_;
        };

        const double max_abs_p_diff = 0.13;
        const double max_abs_yaw_diff = 0.13 * M_PI / 180.0;
        auto smoothDrift = [&]()
        {
            double now = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
            for (auto &[id, drift] : swarm_drift)
            {
                if (!is_lidar[id] && (drone_id.find(id) != drone_id.end()))
                {
                    Vector3d delta_drift_p = drift.translation - swarm_drift_prev_p[id];
                    double delta_morm = delta_drift_p.norm();
                    if (delta_morm > max_abs_p_diff)
                    {
                        delta_drift_p = delta_drift_p * (max_abs_p_diff / delta_morm);
                        drift.translation = swarm_drift_prev_p[id] + delta_drift_p;
                    }
                    swarm_drift_prev_p[id] = drift.translation;

                    if (optimize_yaw)
                    {
                        if (drift.yaw > swarm_drift_prev_yaw[id] + max_abs_yaw_diff)
                            drift.yaw = swarm_drift_prev_yaw[id] + max_abs_yaw_diff;
                        else if (drift.yaw < swarm_drift_prev_yaw[id] - max_abs_yaw_diff)
                            drift.yaw = swarm_drift_prev_yaw[id] - max_abs_yaw_diff;

                        swarm_drift_opt_yaw_filter[id]->input(drift.yaw, now);
                        drift.yaw = swarm_drift_opt_yaw_filter[id]->output();

                        swarm_drift_prev_yaw[id] = drift.yaw;
                    }
                }
            }
        };

        auto optimizeOverDistanceMeas = [&]()
        {
            ceres::Problem problem;

            setupDistAndRegCost(problem);

            lock_guard<mutex> lock(swarm_drift_mtx);

            for (auto &[id, is_lidar_] : is_lidar)
            {
                if (is_lidar_)
                {
                    Drift &lidar_drift = swarm_drift[id];
                    problem.SetParameterBlockConstant(lidar_drift.translation.data());
                    problem.SetParameterBlockConstant(&lidar_drift.yaw);
                }
            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            if (ceres_verbosity_level == 1)
                options.minimizer_progress_to_stdout = true;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            if (ceres_verbosity_level == 2)
                cout << summary.FullReport() << endl;

            smoothDrift();
        };

        ros::Publisher swarm_bearing_diff_pub = nh.advertise<std_msgs::Float32>(param["swarm_bearing_diff_topic"], 1000);
        ros::Publisher swarm_bearing_diff_before_opt_pub = nh.advertise<std_msgs::Float32>(param["swarm_bearing_diff_before_opt_topic"], 1000);
        auto deanonymizeBearingMeas = [&]()
        {
            lock_guard<mutex> lock0(anonymous_bearing_meas_mtx);
            lock_guard<mutex> lock1(bearing_meas_mtx);
            lock_guard<mutex> lock2(swarm_drift_mtx);
            while (!swarm_anonymous_bearing_meas.empty())
            {
                const AnonymousBearingMeas &anonymous_bearing_meas = swarm_anonymous_bearing_meas.front();

                int jupiter_id = anonymous_bearing_meas.jupiter.id;
                if (drone_id.find(jupiter_id) == drone_id.end())
                {
                    LOG(ERROR) << "drone_id.find(jupiter_id) == drone_id.end() #^#";
                    continue;
                }

                ros::Time jupiter_time = anonymous_bearing_meas.timestamp;

                Vector3d jupiter_p = anonymous_bearing_meas.jupiter.position;
                Quaterniond jupiter_q = anonymous_bearing_meas.jupiter.orientation;
                Vector3d jupiter_p_raw(jupiter_p);
                Quaterniond jupiter_q_raw(jupiter_q);
                applyDrift(swarm_drift[jupiter_id], jupiter_p, jupiter_q);

                Vector3d bearing = anonymous_bearing_meas.bearing;
                CHECK_DOUBLE_EQ(bearing.norm(), 1.0);

                Ganymede ganymede;
                double closest_diff = numeric_limits<double>::infinity();
                double closest_diff_raw = numeric_limits<double>::infinity();

                for (int ganymede_id : drone_id)
                {
                    if (ganymede_id == jupiter_id)
                        continue;

                    ros::Time ganymede_time = jupiter_time;
                    geometry_msgs::Pose pose_from_odom;
                    bool ganymede_odom_found = alignMeasAndOdomByTimestamp(ganymede_id, ganymede_time, pose_from_odom);
                    if (!ganymede_odom_found)
                        continue;

                    Vector3d ganymede_p(pose_from_odom.position.x,
                                        pose_from_odom.position.y,
                                        pose_from_odom.position.z);
                    Quaterniond ganymede_q(pose_from_odom.orientation.w,
                                           pose_from_odom.orientation.x,
                                           pose_from_odom.orientation.y,
                                           pose_from_odom.orientation.z);
                    Vector3d ganymede_p_raw(ganymede_p);
                    Quaterniond ganymede_q_raw(ganymede_q);
                    applyDrift(swarm_drift[ganymede_id], ganymede_p, ganymede_q);

                    Vector3d bearing_from_odom = (jupiter_q.inverse() * (ganymede_p - jupiter_p)).normalized();
                    Vector3d bearing_from_odom_raw = (jupiter_q_raw.inverse() * (ganymede_p_raw - jupiter_p_raw)).normalized();

                    double diff = acos(clamp(bearing.dot(bearing_from_odom), -1.0, 1.0));
                    double diff_raw = acos(clamp(bearing.dot(bearing_from_odom_raw), -1.0, 1.0));

                    if (diff < closest_diff)
                    {
                        closest_diff = diff;
                        closest_diff_raw = diff_raw;
                        ganymede.id = ganymede_id;
                        ganymede.position = ganymede_p;
                        ganymede.orientation = ganymede_q;
                    }
                }

                if (closest_diff < max_deanonymization_bearing_angle)
                {
                    BearingMeas bearing_meas;
                    bearing_meas.timestamp = jupiter_time;
                    bearing_meas.jupiter = anonymous_bearing_meas.jupiter;
                    bearing_meas.ganymede = ganymede;
                    bearing_meas.bearing = bearing;

                    if (custom_debug_output)
                        cout << "\033[32mAccept bearing: " << bearing.x() << " " << bearing.y() << " " << bearing.z() << "\033[0m" << endl;

                    swarm_bearing_meas.push_back(bearing_meas);

                    if (publish_debug_topics)
                    {
                        std_msgs::Float32 bearing_diff_msg, bearing_diff_before_opt_msg;
                        bearing_diff_msg.data = closest_diff * 180.0 / M_PI;
                        bearing_diff_before_opt_msg.data = closest_diff_raw * 180.0 / M_PI;
                        swarm_bearing_diff_pub.publish(bearing_diff_msg);
                        swarm_bearing_diff_before_opt_pub.publish(bearing_diff_before_opt_msg);
                        this_thread::sleep_for(chrono::milliseconds(1));
                    }
                }
                else
                {
                    LOG(WARNING) << "Deanonymization failed (closest_diff = " << closest_diff * 180.0 / M_PI << " deg) for an anonymous bearing measurement from drone " << jupiter_id << " #^#";
                    if (custom_debug_output)
                        cout << "\033[31mReject bearing: " << bearing.x() << " " << bearing.y() << " " << bearing.z() << "\033[0m" << endl;
                }
                swarm_anonymous_bearing_meas.pop_front();
            }
            // FIXME: swarm_bearing_meas is not ordered in time #^#
            while (!swarm_bearing_meas.empty() && swarm_bearing_meas.front().timestamp < ros::Time::now() - ros::Duration(sliding_window_length))
                swarm_bearing_meas.pop_front();
        };

        bool inform_insufficient_bearing_meas = true;
        auto optimizeOverDistanceAndBearingMeas = [&]()
        {
            ceres::Problem problem;

            int N_dist_meas_ = setupDistAndRegCost(problem);

            lock_guard<mutex> lock(swarm_drift_mtx);

            if (drift_updated)
            { // Bearing measurement cost
                lock_guard<mutex> lock(bearing_meas_mtx);
                int N_bearing_meas = swarm_bearing_meas.size();

                if (N_bearing_meas <= 3)
                {
                    if (inform_insufficient_bearing_meas)
                    {
                        LOG(WARNING) << "Number of bearing measurements is limited, may lead to degradation of the optimization #^#";
                        inform_insufficient_bearing_meas = false;
                    }
                }
                else
                    inform_insufficient_bearing_meas = true;

                for (const BearingMeas &bearing_meas : swarm_bearing_meas)
                {
                    // Keep sliding_window_length seconds measurements in the sliding window
                    if (bearing_meas.timestamp < ros::Time::now() - ros::Duration(sliding_window_length))
                        continue;

                    ceres::CostFunction *bearing_cost = BearingCostFunctor::Create(bearing_meas.jupiter.position,
                                                                                   bearing_meas.jupiter.orientation,
                                                                                   bearing_meas.ganymede.position,
                                                                                   bearing_meas.ganymede.orientation,
                                                                                   bearing_meas.bearing,
                                                                                   ((double)N_dist_meas_ / N_bearing_meas) * bearing_cost_weight,
                                                                                   optimize_z,
                                                                                   optimize_yaw);

                    ceres::LossFunction *huber_kernel = new ceres::HuberLoss(huber_threshold);

                    Drift &jupiter_drift = swarm_drift[bearing_meas.jupiter.id];
                    Drift &ganymede_drift = swarm_drift[bearing_meas.ganymede.id];

                    problem.AddResidualBlock(bearing_cost, huber_kernel,
                                             jupiter_drift.translation.data(), &jupiter_drift.yaw,
                                             ganymede_drift.translation.data(), &ganymede_drift.yaw);
                }
            }

            for (auto &[id, is_lidar_] : is_lidar)
            {
                if (is_lidar_)
                {
                    Drift &lidar_drift = swarm_drift[id];
                    problem.SetParameterBlockConstant(lidar_drift.translation.data());
                    problem.SetParameterBlockConstant(&lidar_drift.yaw);
                }
            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            if (ceres_verbosity_level == 1)
                options.minimizer_progress_to_stdout = true;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            if (ceres_verbosity_level == 2)
                cout << summary.FullReport() << endl;

            smoothDrift();
        };

        ros::Publisher swarm_drift_pub = nh.advertise<relative_loc::Drift>(param["drift_to_edges_topic"], 1000);
        auto publishDrift = [&]()
        {
            lock_guard<mutex> lock(swarm_drift_mtx);

            for (int id : drone_id)
            {
                if (id == self_id || !drift_initialized_[id] || is_lidar[id])
                    continue;

                relative_loc::Drift drift_with_id;
                drift_with_id.to_drone_ids.push_back(id);
                drift_with_id.id = id;
                drift_with_id.drift.header.stamp = ros::Time::now();

                Vector3d drift_p = swarm_drift[id].translation;
                drift_with_id.drift.pose.position.x = drift_p.x();
                drift_with_id.drift.pose.position.y = drift_p.y();
                drift_with_id.drift.pose.position.z = drift_p.z();

                drift_with_id.drift.pose.orientation.w = swarm_drift[id].yaw;
                drift_with_id.drift.pose.orientation.x = 0.0;
                drift_with_id.drift.pose.orientation.y = 0.0;
                drift_with_id.drift.pose.orientation.z = 0.0;

                swarm_drift_pub.publish(drift_with_id);
                this_thread::sleep_for(chrono::milliseconds(1));
            }
        };
        ros::Publisher swarm_revised_odom_pub = nh.advertise<nav_msgs::Odometry>(param["swarm_revised_odom_topic"], 1000);
        auto publishRevisedOdom = [&]()
        {
            lock_guard<mutex> lock(swarm_odom_mtx);
            for (int id : drone_id)
            {
                if (!drift_initialized_[id] ||
                    swarm_odom_raw.find(id) == swarm_odom_raw.end() ||
                    swarm_odom_raw[id].empty())
                    continue;

                nav_msgs::Odometry revised_odom(swarm_odom_raw[id].back());
                Vector3d odom_p(revised_odom.pose.pose.position.x,
                                revised_odom.pose.pose.position.y,
                                revised_odom.pose.pose.position.z);
                Quaterniond odom_q(revised_odom.pose.pose.orientation.w,
                                   revised_odom.pose.pose.orientation.x,
                                   revised_odom.pose.pose.orientation.y,
                                   revised_odom.pose.pose.orientation.z);
                Drift drift(swarm_drift[id].translation, swarm_drift[id].yaw);
                applyDrift(drift, odom_p, odom_q);

                revised_odom.header.stamp = ros::Time::now();
                revised_odom.header.frame_id = "world";
                revised_odom.child_frame_id = "drone_" + to_string(id);
                revised_odom.pose.pose.position.x = odom_p.x();
                revised_odom.pose.pose.position.y = odom_p.y();
                revised_odom.pose.pose.position.z = odom_p.z();
                revised_odom.pose.pose.orientation.w = odom_q.w();
                revised_odom.pose.pose.orientation.x = odom_q.x();
                revised_odom.pose.pose.orientation.y = odom_q.y();
                revised_odom.pose.pose.orientation.z = odom_q.z();

                swarm_revised_odom_pub.publish(revised_odom);
                this_thread::sleep_for(chrono::milliseconds(1));
            }
        };

        int optimize_per_N_meas = param["optimize_per_N_meas"];
        ros::Rate optimizaion_rate(max({swarm_odom_freq, dist_meas_freq, bearing_meas_freq}) / optimize_per_N_meas);
        while (ros::ok())
        {
            {
                lock_guard<mutex> lock(opt_reset_mtx);
                if (!enable_bearing)
                    optimizeOverDistanceMeas();
                else
                {
                    deanonymizeBearingMeas();
                    optimizeOverDistanceAndBearingMeas();
                }

                if (drift_updated)
                { // Send latest estimated drift to all drones
                    if (drift_initialized_[self_id])
                    {
                        lock_guard<mutex> lock(swarm_drift_mtx);
                        drift_p_ = swarm_drift[self_id].translation;
                        drift_yaw_ = swarm_drift[self_id].yaw;
                        filterDrift();
                        drift_initialized = true;
                    }
                    publishDrift();
                    drift_updated = false;
                }
                publishRevisedOdom();
            }

            optimizaion_rate.sleep();
        }
    }
    else
    {
        cout << "\033[32m============= This is an edge node of the relative localization system ^^ =============\033[0m" << endl;

        auto recvDriftCallback = [&](const relative_loc::Drift::ConstPtr &msg)
        {
            if (msg->id != self_id)
                return;

            if (abs((msg->drift.header.stamp - ros::Time::now()).toSec()) > 1.0)
                LOG(WARNING) << "Timestamp of drift from center is more than 1.0s later (or earlier) than current time @_@ T_drift - T_now = " << (msg->drift.header.stamp - ros::Time::now()).toSec() << "s";

            drift_p_ = Vector3d(msg->drift.pose.position.x,
                                msg->drift.pose.position.y,
                                msg->drift.pose.position.z);
            drift_yaw_ = msg->drift.pose.orientation.w;
            filterDrift();
            drift_initialized = true;
        };
        ros::Subscriber drift_sub =
            nh.subscribe<relative_loc::Drift>(param["drift_from_center_topic"], 10,
                                              recvDriftCallback,
                                              ros::VoidConstPtr(),
                                              ros::TransportHints().tcpNoDelay());

        ros::spin();
    }

    return 0;
}
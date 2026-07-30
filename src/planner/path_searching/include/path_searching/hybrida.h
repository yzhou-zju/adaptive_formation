#ifndef _HYBRID_ASTAR_H
#define _HYBRID_ASTAR_H

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
// #include "global_map.hpp"
#include <plan_env/grid_map.h>

#define CLOSE 'a'
#define OPEN 'b'
#define NOEXPAND 'c'
using namespace std;
// namespace hybrid{
class PathNode
{
public:
  /* -------------------- */
  Eigen::Vector3i posindex;
  int radius_idx;
  Eigen::Matrix<double, 8, 1> state;
  double g_score, f_score;
  Eigen::Vector4d input;
  double duration;
  double time;
  PathNode *parent;
  char node_state;

  /* -------------------- */
  PathNode()
  {
    parent = NULL;
    node_state = NOEXPAND;
  }
  ~PathNode() {};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode *PathNodePtr;

class NodeComparator
{
public:
  bool operator()(PathNodePtr node1, PathNodePtr node2)
  {
    return node1->f_score > node2->f_score;
  }
};

// template <typename T>
// struct matrix_hash : std::unary_function<T, size_t>
// {
//   std::size_t operator()(T const &matrix) const
//   {
//     size_t seed = 0;
//     for (size_t i = 0; i < matrix.size(); ++i)
//     {
//       auto elem = *(matrix.data() + i);
//       seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
//               (seed >> 2);
//     }
//     return seed;
//   }
// };

class NodeHashTable
{
private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>>
      data_4d_;

public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3i idx, PathNodePtr node)
  {
    data_3d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int radius_idx, PathNodePtr node)
  {
    data_4d_.insert(std::make_pair(
        Eigen::Vector4i(idx(0), idx(1), idx(2), radius_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx)
  {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int radius_idx)
  {
    auto iter =
        data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), radius_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear()
  {
    data_3d_.clear();
    data_4d_.clear();
  }
};

class HybridAstar
{
private:
  std::vector<Eigen::Vector4i> debug_visited_nodes_;

  /* ---------- main data structure ---------- */
  vector<PathNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable expanded_nodes_;
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      open_set_;
  std::vector<PathNodePtr> path_nodes_;

  /* ---------- record data ---------- */
  Eigen::Vector4d start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 8, 8> phi_; // state transit matrix
  // map_util::Ptr edt_environment_;
  GridMap::Ptr edt_environment_;
  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_, init_max_tau_;
  double max_vel_, max_acc_, max_radius_, min_radius_, max_radius_vel_, max_radius_acc_, max_lll, min_lll, max_lll_vel, max_lll_acc;
  double w_time_, horizon_, lambda_heu_;
  int allocate_num_, check_num_;
  int front_is_deform_, back_is_deform_;
  double tie_breaker_;
  double robot_z;
  double fake_z;
  std::vector<Eigen::Vector3d> object;
  std::vector<Eigen::Vector3d> shape_to_graph;

  /* map */
  double resolution_, inv_resolution_, radius_resolution_, lll_resolution, inv_radius_resolution_, inv_lll_resolution;
  Eigen::Vector3d origin_, map_size_3d_;

  /* helper */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  int radiusToIndex(double radius);
  int lToIndex(double lll);
  void retrievePath(PathNodePtr end_node);

  /* shot trajectory */
  vector<double> cubic(double a, double b, double c, double d);
  vector<double> quartic(double a, double b, double c, double d, double e);
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                       double time_to_goal);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                           double &optimal_time);

  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 8, 1> &state0,
                    Eigen::Matrix<double, 8, 1> &state1, Eigen::Vector4d um,
                    double tau);

public:

  HybridAstar() {};
  ~HybridAstar();

  enum
  {
    REACH_HORIZON = 1,
    REACH_END = 2,
    NO_PATH = 3,
    NEAR_END = 4
  };

  /* main API */
  void setParam(ros::NodeHandle &nh);
  void init();
  void reset();
  int search(Eigen::Vector4d start_pt, Eigen::Vector4d start_vel,
             Eigen::Vector4d start_acc, Eigen::Vector4d end_pt,
             Eigen::Vector4d end_vel, bool use_shaping = true);

  void setEnvironment(const GridMap::Ptr &env);
  bool isCollision(Eigen::Vector3d c, double r);
  bool isCollisionwithObject(Eigen::Vector3d c, double r);

  std::vector<Eigen::Vector4d> getKinoTraj(double delta_t);

  // void getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
  //                 vector<Eigen::Vector3d>& start_end_derivatives);

  std::vector<PathNodePtr> getVisitedNodes();

  typedef shared_ptr<HybridAstar> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
// }

#endif

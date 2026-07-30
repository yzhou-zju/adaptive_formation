/***
 * @Author: ztr
 * @Date: 2022-05-12 00:00:09
 * @LastEditTime: 2022-05-12 00:00:10
 * @LastEditors: ztr
 * @Description:
 * @FilePath:
 * /Pipline-Swarm-Formation/src/planner/traj_opt/include/optimizer/poly_traj_optimizer.h
 */
#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
// #include <path_searching/kinodynamic_astar.h>
// #include <path_searching/hybrida.h>
#include "optimizer/global_traj_opt.hpp"
#include "optimizer/lbfgs.hpp"
#include "poly_traj_utils.hpp"
#include <frob_test/swarm_graph.hpp>
#include <fstream>
#include <path_searching/graph_class.hpp>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include <traj_utils/FormationType.h>
#include <traj_utils/plan_container.hpp>
namespace ego_planner {

class ConstrainPoints {
public:
  int cp_size; // deformation points
  Eigen::MatrixXd points;

  void resize_cp(const int size_set) {
    cp_size = size_set;
    points.resize(3, size_set);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class FixedTimePoints {
public:
  double start_time;
  double sampling_time_step;
  Eigen::MatrixXd points;           // visualization
  bool enable_decouple_swarm_graph; // if calculate local min of swarm graph in
                                    // advance

  double duration;
  int sampling_num;
  int idx_of_sets;

  Eigen::VectorXd relative_time_ahead;
  std::vector<SwarmGraph> swarm_graph_advance_sets;
  std::vector<std::vector<Eigen::Vector3d>> swarm_pos_advance_sets;
  std::vector<std::vector<Eigen::Vector3d>> swarm_vel_advance_sets;
  std::vector<Eigen::Vector3d> local_min_advance_sets;

  void resetBuffer() {
    start_time = 0.0;
    duration = 0.0;
    sampling_num = 0;
    idx_of_sets = 0;

    relative_time_ahead.resize(0);
    swarm_graph_advance_sets.clear();
    swarm_pos_advance_sets.clear();
    swarm_vel_advance_sets.clear();
    local_min_advance_sets.clear();
    return;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class PolyTrajOptimizer {

private:
  GridMap::Ptr grid_map_;
  bool use_kino_;
  AStar::Ptr a_star_;
  // KinodynamicAstar::Ptr kino_a_star_;
  // HybridAstar kino_search_;
  ros::Subscriber formationchange;
  traj_utils::FormationType F_type;

  poly_traj::MinJerkOpt jerkOpt_;
  SwarmTrajData *swarm_trajs_{
      NULL};                 // Can not use shared_ptr and no need to free
  ConstrainPoints cps_;      // uniform sampling需要修改这部分?????
  FixedTimePoints time_cps_; // fixed time step sampling
  SwarmGraph::Ptr swarm_graph_;
  int drone_id_;
  int cps_num_prePiece_; // number of distinctive constrain points each piece
  int variable_num_;     // optimization variables
  int piece_num_;        // poly traj piece numbers
  int iter_num_;         // iteration of the solver
  bool enable_fix_step_ = false;

  string result_fn_;
  fstream result_file_;

  bool record_once_ =
      true; // debug for calculating the local min of swarm graph
  std::ofstream log_;
  int opt_local_min_loop_num_;
  int opt_local_min_loop_sum_num_ = 0;
  double collision_check_time_end_ = 0.0;

  enum FORCE_STOP_OPTIMIZE_TYPE {
    DONT_STOP,
    STOP_FOR_REBOUND,
    STOP_FOR_ERROR
  } force_stop_type_;

  enum FORMATION_TYPE {
    NONE_FORMATION = 0,
    RECTANGLE_WITH_CENTER = 1,
    REGULAR_TETRAHEDRON = 2,
    HEXAGON = 3,
    Z_LETTER = 4,
    J_LETTER = 5,
    U_LETTER = 6,
    REGULAR_HEXAGON = 7,
    ARROW = 8,
    HEART = 9,
    RECTANGLE = 10,
    TWENTY_FIVE = 25,
    OCTAHEDRON = 13,
    ztr_1 = 11,
    ztr_2 = 12,
    equ_triangle = 14
  };

  enum FORMATION_METHOD_TYPE {
    SWARM_GRAPH = 0, // (default)
    LEADER_POSITION = 1,
    RELATIVE_POSITION = 2,
    VRB_METHOD = 3
  };

  /* optimization parameters */
  double wei_smooth_; // smooth weight
  double wei_obs_;    // obstacle weight
  double wei_swarm_;  // swarm weight
  double wei_feas_;   // feasibility weight
  double wei_sqrvar_; // squared variance weight
  double wei_time_;   // time weight
  double wei_gather_; // swarm gathering penalty

  double obs_clearance_;          // safe distance between uav and obstacles
  double swarm_clearance_;        // safe distance between uav and uav
  double swarm_gather_threshold_; // threshold distance between uav and swarm
                                  // center
  double max_vel_, max_vel2_, max_acc_; // dynamic limits
  double max_vel_real;

  bool formation_change_get;

  bool is_set_swarm_advanced_;
  int size_swarm;
  double max_graph_aware;
  int vis_step;
  double vis_hor;
  int formation_type_;
  int formation_method_type_;
  int formation_size_;
  int id_in_group;
  int formation_size_all;
  bool use_formation_ = true;
  bool is_other_assigning_ = false;

  bool formation_change_flag = false;
  std::vector<Eigen::Vector3d> swarm_des_;
  std::vector<Eigen::Vector3d> swarm_des_opt;
  std::vector<Eigen::Vector3d> swarm_des_all;
  std::vector<Eigen::Vector3d> swarm_des_1;
  std::vector<std::vector<int>> group_id;
  int id_group_own = 0;
  std::vector<Eigen::Vector3d> swarm_des_old;
  Eigen::Vector3d init_start_zy;
  Eigen::Vector3d init_end_zy;
  std::vector<int> adj_in_, adj_out_;
  Eigen::VectorXi assignment_;
  std::vector<Eigen::VectorXi> max_cline_id_;
  Eigen::VectorXi max_cline_id_m_z;
  bool cline_begin = false;

  double t_now_;
  // benchmark
  vector<Eigen::Vector3d> formation_relative_dist_;

  double fixed_formation_z_;
  bool is_opt_formation_z_ = true;

public:
  PolyTrajOptimizer() {}
  ~PolyTrajOptimizer() { ztr_log3.close(); }

  /////////////for kino dynamic
  /// planner////////////////-----------------------------------------
  bool hybridastarWithMinTraj(const Eigen::MatrixXd &iniState,
                              const Eigen::MatrixXd &finState,
                              std::vector<Eigen::Vector3d> &simple_path,
                              Eigen::MatrixXd &ctl_points,
                              poly_traj::MinJerkOpt &frontendMJ);
  bool hybridastarWithMinTraj_QL(const Eigen::MatrixXd &iniState,
                                 const Eigen::MatrixXd &finState,
                                 std::vector<Eigen::Vector3d> &simple_path,
                                 Eigen::MatrixXd &ctl_points,
                                 poly_traj::MinJerkOpt &frontendMJ);
  void set_leader_path(const Eigen::Matrix3d &start_leader,
                       const Eigen::Matrix3d &end_leader,
                       const vector<double> &rrr_start,
                       const vector<double> &rrr_end,
                       const vector<double> &lll_start,
                       const vector<double> &lll_end,
                       std::vector<Eigen::Vector4d> &path_R_,
                       const double &opt_time_now, double &opt_score);
  void leader_opt_ok();
  bool clc_form_error(const Eigen::Vector3d &pos_mine);
  void change_id(int id_drone, int type) {
    if (type == 0) {
      sparse_id = {0, 1, 2, 3, 4, 11,12, 13,14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
    }
    if (type == 1) {
      sparse_id = {0, 1, 2, 3, 4, 5,9,10,11,12, 13,14, 15, 16, 17, 18, 19, 20,21, 22, 23, 24,25, 26,27,28,29};
    }
    if (type == 2) {
      sparse_id = {0, 1, 2, 3, 4, 8,9,10, 12,13, 15, 16, 17, 18, 19, 20,21, 23, 24,25,27,28,29};
    }
    if (type == 3) {
      sparse_id = {0, 1, 2, 3, 4,12,13, 15, 16, 17, 18, 19, 20,21, 23, 24,25,27,28,29};
    }
    // 查找插入位置
    auto insert_pos =
        std::lower_bound(sparse_id.begin(), sparse_id.end(), drone_id_);

    // 判重并插入
    if (insert_pos == sparse_id.end() || *insert_pos != drone_id_) {
      sparse_id.insert(insert_pos, drone_id_);
    }
    clc_error_id = sparse_id;
    id_in_cline = id_drone;
  }
  double clc_des_swarm(std::vector<Eigen::Vector3d> swarm_cur_set,
                       std::vector<int> des_id_swarm, double futur_time) {
    double clc_swarm = 0.0;
    for (int i = 0; i < swarm_des_opt.size(); i++) {
      double t_iii =
          std::min(std::max(futur_time, 0.0), posTraj_sub.getTotalDuration());
      Eigen::Vector3d pos_traj_ = posTraj_sub.getPos(t_iii);
      Eigen::Matrix<double, 1, 1> r_traj_ = radiusTraj_sub.getPos(t_iii);
      Eigen::Matrix<double, 1, 1> l_traj_ = lllTraj_sub.getPos(t_iii);
      Eigen::Vector3d pos_bias =
          r_traj_(0, 0) *
          Eigen::Vector3d(l_traj_(0, 0) * swarm_des_opt[i][0],
                          (1.0 / l_traj_(0, 0)) * swarm_des_opt[i][1], 0.0);
      Eigen::Vector3d pos_formation =
          Eigen::Vector3d(pos_traj_[0], pos_traj_[1], 1.0) + pos_bias;
      if (des_id_swarm[i] >= 0) {
        clc_swarm += (pos_formation - swarm_cur_set[i]).norm();
      }
    }
    return clc_swarm;
  }
  void set_des_swarm(std::vector<Eigen::Vector3d> swarm_des_set) {
    /*******************************************TODO**********graph_size***************************************
     */
    graph_size = 0;
    number_change = true;
    swarm_des_opt.clear();
    clc_error_id.clear();
    swarm_des_opt = swarm_des_set;
    for (int id_real = 0; id_real < swarm_des_opt.size(); id_real++) {
      if (swarm_des_opt[id_real].z() != -99.0) {
        graph_size = graph_size + 1;
        clc_error_id.push_back(id_real);
      }
    }
  }
  void set_max_vel(bool fast_or_slow) {
    if (fast_or_slow) {
      max_vel_real = max_vel_;
    } else {
      max_vel_real = 0.1;
    }
  }
  /////////////for kino dynamic planner///////////////auto cMat =
  /// data_pos.getPiece(i).getCoeffMat();/-------------------------------------------
  Eigen::MatrixXd
  compute_normalized_laplacian(const std::vector<Eigen::Vector3d> &points);
  Eigen::MatrixXd
  compute_laplacian_difference(const std::vector<Eigen::Vector3d> &des_pos,
                               const std::vector<Eigen::Vector3d> &cur_pos);
  Eigen::MatrixXd process_difference_matrix(const Eigen::MatrixXd &diff_matrix);
  /* set variables */
  void formation_change(bool get_change);
  void setParam(ros::NodeHandle &nh, double id);
  void setEnvironment(ros::NodeHandle &nh, const GridMap::Ptr &map, int id);
  void setControlPoints(const Eigen::MatrixXd &points);
  void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
  void setDroneId(const int drone_id);
  void FormationChange(const traj_utils::FormationType::ConstPtr &msg);
  /* helper functions */
  inline ConstrainPoints getControlPoints() { return cps_; }
  inline const ConstrainPoints *getControlPointsPtr(void) { return &cps_; }
  inline const poly_traj::MinJerkOpt *getMinJerkOptPtr(void) {
    return &jerkOpt_;
  }
  inline int get_cps_num_prePiece_() { return cps_num_prePiece_; }
  inline double getSwarmClearance(void) { return swarm_clearance_; }
  inline int getFormationSize() { return graph_size; }
  // std::vector<Eigen::Vector3d> getSwarmGraphInit() { return
  // swarm_graph_->getDesNodesInit(); }
  std::vector<Eigen::Vector3d> getSwarmGraphInit() { return swarm_des_opt; }
  double getFormationError(std::vector<Eigen::Vector3d> swarm_graph_pos);
  double gettopo_formation_err(const poly_traj::Trajectory &traj_topo);
  void updateSwarmGraph(Eigen::VectorXi assignment);
  double getCollisionCheckTimeEnd() { return collision_check_time_end_; }

  /* main planning API */
  bool optimizeTrajectory_lbfgs(const Eigen::MatrixXd &iniState,
                                const Eigen::MatrixXd &finState,
                                const Eigen::MatrixXd &initInnerPts,
                                const Eigen::VectorXd &initT,
                                Eigen::MatrixXd &optimal_points,
                                const bool use_formation);

  bool sumConstrainAware(Eigen::Vector3d &form_grad_sum,
                         Eigen::Vector3d &obs_grad_sum, double &obs_dist_sum,
                         double &aware_sum, double &t_duration);
  double sumvisAware();
  double gettopo_vismy_err(const poly_traj::Trajectory &traj_topo_my);
  double gettopo_visother_err(const poly_traj::Trajectory &traj_topo_my);

  bool topoWithMinTraj(const Eigen::MatrixXd &iniState,
                       const Eigen::MatrixXd &finState,
                       vector<Eigen::Vector3d> &topo_path,
                       Eigen::MatrixXd &ctl_points,
                       poly_traj::MinJerkOpt &frontendMJ);
  bool astarWithMinTraj(const Eigen::MatrixXd &iniState,
                        const Eigen::MatrixXd &finState,
                        std::vector<Eigen::Vector3d> &simple_path,
                        Eigen::MatrixXd &ctl_points,
                        poly_traj::MinJerkOpt &frontendMJ);
  bool pathR_MinTraj(const Eigen::MatrixXd &iniState,
                     const Eigen::MatrixXd &finState,
                     vector<Eigen::Vector3d> &simple_path,
                     Eigen::MatrixXd &ctl_points,
                     poly_traj::MinJerkOpt &frontendMJ);

private:
  /* callbacks by the L-BFGS optimizer */
  static double costFunctionCallback(void *func_data, const double *x,
                                     double *grad, const int n);

  static int earlyExitCallback(void *func_data, const double *x,
                               const double *g, const double fx,
                               const double xnorm, const double gnorm,
                               const double step, int n, int k, int ls);

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

  template <typename EIGENVEC>
  void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

  template <typename EIGENVEC, typename EIGENVECGD>
  void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                        const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                        double &costT);

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

  /*
    calculate cost of trajectory with fixed control points sampling
  */
  double calculateMean(const std::vector<double> &data);
  double calculateVariance(const std::vector<double> &data);
  template <typename EIGENVEC>
  void addPVAGradCost2CTwithFixedCtrlPoints(EIGENVEC &gdT,
                                            Eigen::VectorXd &costs,
                                            const int &K);
  void getFixedTimePos();
  bool obstacleGradCostP(const int i_dp, const Eigen::Vector3d &p,
                         Eigen::Vector3d &gradp, double &costp);

  bool swarmGradCostP(const int i_dp, const double t, const Eigen::Vector3d &p,
                      const Eigen::Vector3d &v, Eigen::Vector3d &gradp,
                      double &gradt, double &grad_prev_t, double &costp);

  bool swarmGraphGradCostP(const int i_dp, const double t,
                           const Eigen::Vector3d &p, const Eigen::Vector3d &v,
                           Eigen::Vector3d &gradp, double &gradt,
                           double &grad_prev_t, double &costp);

  bool swarmGatherCostGradP(const int i_dp, const double t,
                            const Eigen::Vector3d &p, const Eigen::Vector3d &v,
                            Eigen::Vector3d &gradp, double &gradt,
                            double &grad_prev_t, double &costp);

  // benchmark
  bool leaderPosFormationCostGradP(const int i_dp, const double t,
                                   const Eigen::Vector3d &p,
                                   const Eigen::Vector3d &v,
                                   Eigen::Vector3d &gradp, double &gradt,
                                   double &grad_prev_t, double &costp);

  bool relativePosFormationCostGradP(const int i_dp, const double t,
                                     const Eigen::Vector3d &p,
                                     const Eigen::Vector3d &v,
                                     Eigen::Vector3d &gradp, double &gradt,
                                     double &grad_prev_t, double &costp);

  bool feasibilityGradCostV(const Eigen::Vector3d &v, Eigen::Vector3d &gradv,
                            double &costv);

  bool feasibilityGradCostA(const Eigen::Vector3d &a, Eigen::Vector3d &grada,
                            double &costa);

  void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                         Eigen::MatrixXd &gdp, double &var);

  /*
    calculate cost of trajectory with fixed time step sampling
  */
  template <typename EIGENVEC>
  void addPVAGradCost2CTwithFixedTimeSteps(EIGENVEC &gdT,
                                           Eigen::VectorXd &costs);

  /* use L-NFGS optimizer to calculate the local minimum of swarm graph */
  void setSwarmGraphInAdavanced(const Eigen::VectorXd initT);
  static double swarmGraphCostCallback(void *func_data, const double *x,
                                       double *grad, const int n);

  double obstacleGradCostP(Eigen::VectorXd &gdT);

  double swarmGradCostP(Eigen::VectorXd &gdT);

  double swarmGraphGradCostP(Eigen::VectorXd &gdT);

  double leaderGradCostP(Eigen::VectorXd &gdT);

  void feasibilityGradCostVandA(Eigen::VectorXd &gdT, double &vel_cost,
                                double acc_cost);

  /* useful function */

  bool checkCollision(double &collision_relative_time);

  bool getFormationPos(std::vector<Eigen::Vector3d> &swarm_graph_pos,
                       Eigen::Vector3d pos);
  double swarm_pts_[200][3]; // formation change

  void setDesiredFormation2(std::vector<Eigen::Vector3d> swarm_des_zy) {
    // std::vector<Eigen::Vector3d> swarm_des_zy;
    formation_size_ = swarm_des_zy.size();
    for (int i = 0; i < formation_size_; i++) {
      for (int j = 0; j < i; j++) {
        adj_in_.push_back(i);
        adj_out_.push_back(j);
      }
    }
    assignment_ =
        Eigen::VectorXi::LinSpaced(formation_size_, 0, formation_size_ - 1);
    swarm_graph_->setDesiredForm(swarm_des_zy, adj_in_, adj_out_);
  }

  //  void setDesiredFormation(int type, int id_drone)//必须知晓队形中size个数
  // {
  //   /*************************************************************************************************/
  //   drone_id_ = id_drone;
  //   id_group_own = id_drone;
  //   group_id.resize(3);
  //   group_id[0] = {0,1,2};
  //   group_id[1] = {0,1,2};
  //   group_id[2] = {0,1,2};
  //   for(int kkm=0;kkm<group_id[id_drone].size();kkm++)
  //   {
  //     if(id_drone==group_id[id_drone][kkm])
  //     {
  //       id_in_group = kkm;
  //     }
  //   }
  //   formation_size_all = 3;
  //   /*************************************************************************************************/
  //   swarm_des_.clear();//之后要重新赋予数值，填充新的formation
  //   swarm_des_old.clear();
  // for (int i = 0; i < size_swarm; i++)
  // {
  //   nh_.param("t"+to_string((int)type)+"_relative_pos_" + to_string(i) +
  //   "/x", swarm_pts_[i][0], -1.0);
  //   nh_.param("t"+to_string((int)type)+"_relative_pos_" + to_string(i) +
  //   "/y", swarm_pts_[i][1], -1.0);
  //   nh_.param("t"+to_string((int)type)+"_relative_pos_" + to_string(i) +
  //   "/z", swarm_pts_[i][2], -1.0);
  // }

  // for (int j = 0; j < group_id[id_group_own].size(); j++)
  // {
  //  swarm_des_.push_back(Eigen::Vector3d(swarm_pts_[group_id[id_group_own][j]][0],swarm_pts_[group_id[id_group_own][j]][1],swarm_pts_[group_id[id_group_own][j]][2]));
  // }

  // formation_size_ = swarm_des_.size();
  // // construct the desired swarm graph
  // // adj_in_ =  {0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6};
  // // adj_out_ = {1, 2, 3, 4, 5, 6, 2, 3, 4, 5, 6, 1};
  // for (int i=0; i<formation_size_; i++){
  //   for ( int j=0; j<i; j++){
  //     adj_in_.push_back(i);
  //     adj_out_.push_back(j);
  //   }
  // }
  // assignment_ = Eigen::VectorXi::LinSpaced(formation_size_, 0,
  // formation_size_ - 1 ); swarm_graph_->setDesiredForm(swarm_des_, adj_in_,
  // adj_out_);

  // }

  void group_in_advance(int type, int num_form, int id_drone,
                        std::vector<std::vector<int>> group_id_in) {
    /*************************************************************************************************/
    drone_id_ = id_drone;
    id_group_own = id_drone;
    group_id = group_id_in;

    for (int kkm = 0; kkm < group_id[id_drone].size(); kkm++) {
      if (id_drone == group_id[id_drone][kkm]) {
        id_in_group = kkm;
      }
    }
    formation_size_all = num_form;
    // formation_size_all = 100;
    // id_in_group = id_drone;

    /*************************************************************************************************/
    ztr_log3 << "in setDesiredFormation" << endl;
    swarm_des_.clear(); // 之后要重新赋予数值，填充新的formation
    swarm_des_old.clear();
    for (int i = 0; i < size_swarm; i++) {
      nh_.param("t" + to_string((int)type) + "_relative_pos_" + to_string(i) +
                    "/x",
                swarm_pts_[i][0], -1.0);
      nh_.param("t" + to_string((int)type) + "_relative_pos_" + to_string(i) +
                    "/y",
                swarm_pts_[i][1], -1.0);
      nh_.param("t" + to_string((int)type) + "_relative_pos_" + to_string(i) +
                    "/z",
                swarm_pts_[i][2], -1.0);
    }
    for (int i = 0; i < sparse_id.size(); i++) {
      if (sparse_id[i] != drone_id_) {
        swarm_des_all.push_back(Eigen::Vector3d(swarm_pts_[sparse_id[i]][0],
                                                swarm_pts_[sparse_id[i]][1],
                                                swarm_pts_[sparse_id[i]][2]));
      }
    }
    for (int j = 0; j < group_id[id_group_own].size(); j++) {
      swarm_des_.push_back(
          Eigen::Vector3d(swarm_pts_[group_id[id_group_own][j]][0],
                          swarm_pts_[group_id[id_group_own][j]][1],
                          swarm_pts_[group_id[id_group_own][j]][2]));
    }

    formation_size_ = swarm_des_.size();
    // construct the desired swarm graph
    // adj_in_ =  {0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6};
    // adj_out_ = {1, 2, 3, 4, 5, 6, 2, 3, 4, 5, 6, 1};
    for (int i = 0; i < formation_size_; i++) {
      for (int j = 0; j < i; j++) {
        adj_in_.push_back(i);
        adj_out_.push_back(j);
      }
    }
    assignment_ =
        Eigen::VectorXi::LinSpaced(formation_size_, 0, formation_size_ - 1);
    swarm_graph_->setDesiredForm(swarm_des_, adj_in_, adj_out_);
  }
  void setDesiredFormation(int type, int id_drone) // 必须知晓队形中size个数
  {
    bool is_all = false;
    int num_form = size_swarm;
    if (is_all) {
      /*************************************************************************************************/
      drone_id_ = id_drone;
      id_group_own = id_drone;
      group_id.resize(num_form);
      // group_id[0] =
      // {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59};
      formation_size_all = num_form;
      id_in_group = id_drone;
      for (int kkm = 0; kkm < group_id[id_drone].size(); kkm++) {
        if (id_drone == group_id[id_drone][kkm]) {
          id_in_group = kkm;
        }
      }
      /*************************************************************************************************/
      ztr_log3 << "in setDesiredFormation" << endl;
      swarm_des_.clear(); // 之后要重新赋予数值，填充新的formation
      swarm_des_old.clear();
      for (int i = 0; i < size_swarm; i++) {
        nh_.param("t" + to_string((int)type) + "_relative_pos_" + to_string(i) +
                      "/x",
                  swarm_pts_[i][0], -1.0);
        nh_.param("t" + to_string((int)type) + "_relative_pos_" + to_string(i) +
                      "/y",
                  swarm_pts_[i][1], -1.0);
        nh_.param("t" + to_string((int)type) + "_relative_pos_" + to_string(i) +
                      "/z",
                  swarm_pts_[i][2], -1.0);
      }

      for (int j = 0; j < group_id[id_group_own].size(); j++) {
        swarm_des_.push_back(
            Eigen::Vector3d(swarm_pts_[group_id[id_group_own][j]][0],
                            swarm_pts_[group_id[id_group_own][j]][1],
                            swarm_pts_[group_id[id_group_own][j]][2]));
      }

      formation_size_ = swarm_des_.size();
      // construct the desired swarm graph
      // adj_in_ =  {0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6};
      // adj_out_ = {1, 2, 3, 4, 5, 6, 2, 3, 4, 5, 6, 1};
      for (int i = 0; i < formation_size_; i++) {
        for (int j = 0; j < i; j++) {
          adj_in_.push_back(i);
          adj_out_.push_back(j);
        }
      }
      assignment_ =
          Eigen::VectorXi::LinSpaced(formation_size_, 0, formation_size_ - 1);
      swarm_graph_->setDesiredForm(swarm_des_, adj_in_, adj_out_);
      std::cout << "formation_size_:" << formation_size_ << std::endl;
      std::cout << "id_group_own:" << id_group_own << std::endl;
      std::cout << "id_in_group:" << id_in_group << std::endl;
      std::cout << "drone_id_:" << drone_id_ << std::endl;
      std::cout << std::endl;
    } else {
      /*************************************************************************************************/
      group_id.resize(num_form);
      //////////////////////////////cube/////////////////////
      for (int ii_group = 0; ii_group < num_form; ii_group++) {
        for (int ii_ = 0; ii_ < num_form; ii_++) {
          group_id[ii_group].push_back(ii_);
        }
      }
      group_in_advance(type, num_form, id_drone, group_id);

      swarm_des_opt = {
          {0.0, 0.0, 0.0},     {0.4, 1.34, 0.0},    {0.31, 2.63, 0.0},
          {-1.09, 1.83, 0.0},  {-2.60, 2.83, 0.0},  {-3.68, 2.17, 0.0},
          {-3.60, 1.01, 0.0},  {-3.13, 0.01, 0.0},  {-2.51, -0.92, 0.0},
          {-1.88, -1.63, 0.0}, {-0.84, -2.01, 0.0}, {0.09, -1.43, 0.0},
          {0.91, -0.64, 0.0},  {1.41, 0.39, 0.0},   {1.84, 1.57, 0.0},
          {1.41, 2.69, 0.0}, // 15
          {-2.31, 1.49, 0.0},  {-1.94, 0.25, 0.0},  {-1.05, -0.68, 0.0},
          {-0.9, 0.74, 0.0},

          {0.0, 0.0, -99.0},   {0.0, 0.0, -99.0},   {0.0, 0.0, -99.0},
          {0.0, 0.0, -99.0},   {0.0, 0.0, -99.0},   {0.0, 0.0, -99.0},
          {0.0, 0.0, -99.0},   {0.0, 0.0, -99.0},   {0.0, 0.0, -99.0},
          {0.0, 0.0, -99.0}};
      graph_size = 20;
      sparse_id = {0, 1, 2, 3, 4, 12, 13,14, 15, 16, 17, 18, 19};
      // 查找插入位置
      auto insert_pos =
          std::lower_bound(sparse_id.begin(), sparse_id.end(), drone_id_);

      // 判重并插入
      if (insert_pos == sparse_id.end() || *insert_pos != drone_id_) {
        sparse_id.insert(insert_pos, drone_id_);
      }
      clc_error_id = sparse_id;
      id_in_cline = id_drone;
    }
  }

  ros::NodeHandle nh_;

public:
  std::ofstream ztr_log3;
  typedef unique_ptr<PolyTrajOptimizer> Ptr;
  global_traj::GlobalTrajOptimizer<GridMap::Ptr> trajROpt;
  bool opt_ok = false;
  bool number_change = false;
  bool use_leader_ = false;
  double leader_opt_time = 0.0;
  int front_traj_piecenum = 0;
  int rank_hhh_old = 0;
  int rank_lll_old = 0;
  double wei_formation_prl;
  double wei_formation_;
  double wei_formation2_;
  double wei_graph_;
  bool use_graph_;
  double error_to_leader_;
  int graph_size;
  int id_in_cline;
  std::vector<int> sparse_id;
  std::vector<int> clc_error_id;
  ros::Time number_change_time = ros::Time::now();
  poly_traj_leader::Trajectory<3> posTraj_sub;
  poly_traj_leader::Trajectory<1> radiusTraj_sub;
  poly_traj_leader::Trajectory<1> lllTraj_sub;
};

} // namespace ego_planner
#endif
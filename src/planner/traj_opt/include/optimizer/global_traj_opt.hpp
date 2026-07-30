#ifndef _GLOBAL_TRAJ_OPT_H_
#define _GLOBAL_TRAJ_OPT_H_

#include <Eigen/Eigen>
#include <ros/ros.h>

#include "poly_traj_utils_leader.hpp"
#include "lbfgs_hzc.hpp"
// #include "global_map.hpp"
#include <plan_env/grid_map.h>
namespace global_traj
{

  using namespace std;
  template <typename gridmap>

  class GlobalTrajOptimizer
  {

  private:
    int piecenum;
    double gslar_ = 0.0;
    double esdfWei = 1000.0;
    double esdfWei_local = 500.0;
    double penaWei = 1000.0;
    double penaform = 1000.0;
    double energyWei = 1.0; // 1
    int traj_res = 16;

    double safeMargin = 0.1;

    GridMap::Ptr map_itf_;
    // gridmap *map_itf_;
    Eigen::MatrixXd headState_local, tailState_local;
    Eigen::MatrixXd headState_, tailState_;
    Eigen::MatrixXd headRadius_, tailRadius_;
    Eigen::MatrixXd headlll_, taillll_;

    double wei_time_ = 1.0;
    double min_radius = 0.2;
    double max_radius = 30.0;
    double min_lll = 0.2;
    double max_lll = 4.0;
    double vmax = 3.0;
    double vmax_local = 3.0;
    double amax = 3.0;
    double amax_local = 3.0;
    double omegamax = 5.0;
    double omegadotmax = 10.0;
    double lllmax = 5.0;
    double llldotmax = 10.0;
    double length_per_piece = 1.5;
    double min_dis = 100;
    double robot_z = 0.2;
    int back_is_deform_;
    std::vector<Eigen::Vector3d> object;

    Eigen::Vector3d swarm_des_one;
    double opt_fin_time;

  public:
    poly_traj_leader::MinJerkOpt<3> jerkOpt_local;
    poly_traj_leader::Trajectory<3> posTraj_local;
    poly_traj_leader::MinJerkOpt<3> jerkOpt_;
    poly_traj_leader::MinJerkOpt<1> radiusOpt_;
    poly_traj_leader::MinJerkOpt<1> lllOpt_;
    poly_traj_leader::Trajectory<3> posTraj_;
    poly_traj_leader::Trajectory<1> radiusTraj_;
    poly_traj_leader::Trajectory<1> lllTraj_;
    double jerkposCost_, jerkradiusCost_, jerklllCost_, timeCost_, energyCost_;
    double jerkposCost_local, timeCost_local, energyCost_local;

    GlobalTrajOptimizer() {}
    ~GlobalTrajOptimizer() {}
    void init(ros::NodeHandle &nh, GridMap::Ptr env)
    {
      nh.param("max_vel", vmax, 3.0);
      nh.param("max_vel_local", vmax_local, 3.0);
      nh.param("max_acc", amax, 3.0);
      nh.param("max_acc_local", amax_local, 3.0);
      nh.param("min_radius", min_radius, 0.1);
      nh.param("max_radius", max_radius, 0.5);
      nh.param("weight_time", wei_time_, 500.0);
      nh.param("polyTraj_piece_length", length_per_piece, 1.0);
      nh.param("omegamax", omegamax, 1.0);
      nh.param("omegadotmax", omegadotmax, 1.0);
      nh.param("robot_z", robot_z, 0.2);
      nh.param("back_is_deform_", back_is_deform_, 3);

      map_itf_ = env;
    }
    inline int OptimizeGlobalTrajectory(
        const Eigen::MatrixXd &headState,
        const Eigen::MatrixXd &tailState,
        const Eigen::MatrixXd &headRadius,
        const Eigen::MatrixXd &tailRadius,
        std::vector<Eigen::Vector3d> initial_path)
    {
      headState_ = headState;
      tailState_ = tailState;
      headRadius_ = headRadius;
      tailRadius_ = tailRadius;

      double totalLength = 0.0;
      for (int i = 0; i < initial_path.size() - 1; i++)
      {
        totalLength += (initial_path[i + 1] - initial_path[i]).norm();
      }
      piecenum = std::max(int(totalLength / length_per_piece), 2);
      double tmpl = totalLength / piecenum;
      Eigen::MatrixXd wps;
      wps.resize(3, piecenum - 1);
      double curArc = 0.0;
      int index = 0;
      for (int i = 0; i < initial_path.size() - 1; i++)
      {
        curArc += (initial_path[i + 1] - initial_path[i]).norm();
        if (curArc > (index + 1) * tmpl)
        {
          wps.col(index) = initial_path[i];
          index++;
          if (index >= piecenum - 1)
          {
            break;
          }
        }
      }
      Eigen::VectorXd initialT;
      initialT.resize(piecenum);
      initialT.setConstant(tmpl / (vmax / 1.2));

      Eigen::MatrixXd wps_radius;
      wps_radius.resize(1, piecenum - 1);
      wps_radius.setConstant((min_radius + max_radius) / 2.0);
      std::cout << "piecenum: " << piecenum << std::endl;
      jerkOpt_.reset(headState_, tailState_, piecenum);
      radiusOpt_.reset(headRadius, tailRadius, piecenum);

      int variable_num_ = 3 * (piecenum - 1) + piecenum + 1 * (piecenum - 1);

      Eigen::VectorXd x;
      x.resize(variable_num_);
      int offset = 0;
      memcpy(x.data() + offset, wps.data(), wps.size() * sizeof(x[0]));
      offset += wps.size();

      Eigen::Map<Eigen::VectorXd> Vt(x.data() + offset, initialT.size());
      RealT2VirtualT(initialT, Vt);
      offset += initialT.size();

      memcpy(x.data() + offset, wps_radius.data(), wps_radius.size() * sizeof(x[0]));
      offset += wps_radius.size();

      lbfgs_hzc::lbfgs_parameter_t lbfgs_params;
      lbfgs_params.mem_size = 256; // 128
      lbfgs_params.past = 8;       // 3
      lbfgs_params.g_epsilon = 1.0e-4;
      lbfgs_params.min_step = 1.0e-32;
      lbfgs_params.delta = 1.0e-6;
      lbfgs_params.max_iterations = 10000;
      int result;
      double final_cost;
      double t1 = ros::Time::now().toSec();
      result = lbfgs_hzc::lbfgs_optimize(
          x,
          final_cost,
          GlobalTrajOptimizer::costFunctionCallback,
          NULL,
          NULL,
          this,
          lbfgs_params);
      double t2 = ros::Time::now().toSec();
      ROS_WARN_STREAM("ring planning time: " << 1000.0 * (t2 - t1) << " ms");

      /* ---------- get result and check collision ---------- */
      if (result == lbfgs_hzc::LBFGS_CONVERGENCE ||
          result == lbfgs_hzc::LBFGS_CANCELED ||
          result == lbfgs_hzc::LBFGS_STOP || result == lbfgs_hzc::LBFGSERR_MAXIMUMITERATION)
      {
        ROS_WARN_STREAM("ring planner worked cost:" << final_cost);
      }
      else if (result == lbfgs_hzc::LBFGSERR_MAXIMUMLINESEARCH)
      {
        ROS_WARN_STREAM("ring planner worked cost:" << final_cost);
        ROS_WARN("Lbfgs: The line-search routine reaches the maximum number of evaluations.");
      }
      else
      {
        ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs_hzc::lbfgs_strerror(result));
      }

      std::cout << "after optimized: " << jerkOpt_.get_T1().transpose() << std::endl;

      posTraj_ = jerkOpt_.getTraj();
      radiusTraj_ = radiusOpt_.getTraj();

      double r = radiusTraj_.getPos(0.0)(0, 0);
      return true;
    }

    inline int OptimizeGlobalTrajectory(
        const Eigen::MatrixXd &headState,
        const Eigen::MatrixXd &tailState,
        const Eigen::MatrixXd &headRadius,
        const Eigen::MatrixXd &tailRadius,
        const Eigen::MatrixXd &headlll,
        const Eigen::MatrixXd &taillll,
        std::vector<Eigen::Vector4d> initial_path,
        std::vector<double> lll_path)
    {
      headState_ = headState;
      tailState_ = tailState;
      headRadius_ = headRadius;
      tailRadius_ = tailRadius;
      headlll_ = headlll;
      taillll_ = taillll;
      double totalLength = 0.0;
      for (int i = 0; i < initial_path.size() - 1; i++)
      {
        totalLength += (initial_path[i + 1].head(3) - initial_path[i].head(3)).norm();
      }
      piecenum = std::max(int(totalLength / length_per_piece), 2);
      double tmpl = totalLength / piecenum;
      Eigen::MatrixXd wps;
      wps.resize(3, piecenum - 1);
      Eigen::MatrixXd wps_radius;
      wps_radius.resize(1, piecenum - 1);
      Eigen::MatrixXd wps_lll;
      wps_lll.resize(1, piecenum - 1);
      double curArc = 0.0;
      int index = 0;
      for (int i = 0; i < initial_path.size() - 1; i++)
      {
        curArc += (initial_path[i + 1].head(3) - initial_path[i].head(3)).norm();
        if (curArc > (index + 1) * tmpl)
        {
          wps.col(index) = initial_path[i].head(3);
          wps_radius(0, index) = initial_path[i][3];
          wps_lll(0, index) = lll_path[i];
          index++;
          if (index >= piecenum - 1)
          {
            break;
          }
        }
      }
      Eigen::VectorXd initialT;
      initialT.resize(piecenum);
      initialT.setConstant(tmpl / (vmax / 1.2));

      // (min_radius + max_radius) / 2.0);

      std::cout << "piecenum: " << piecenum << std::endl;
      std::cout << "initial_path: " << initial_path.size() << std::endl;
      std::cout << "lll_path: " << lll_path.size() << std::endl;
      jerkOpt_.reset(headState_, tailState_, piecenum);
      radiusOpt_.reset(headRadius, tailRadius, piecenum);
      lllOpt_.reset(headlll, taillll, piecenum);

      int variable_num_ = 3 * (piecenum - 1) + piecenum + 1 * (piecenum - 1) + 1 * (piecenum - 1);

      Eigen::VectorXd x;
      x.resize(variable_num_);
      int offset = 0;
      memcpy(x.data() + offset, wps.data(), wps.size() * sizeof(x[0]));
      offset += wps.size();

      Eigen::Map<Eigen::VectorXd> Vt(x.data() + offset, initialT.size());
      RealT2VirtualT(initialT, Vt);
      offset += initialT.size();

      memcpy(x.data() + offset, wps_radius.data(), wps_radius.size() * sizeof(x[0]));
      offset += wps_radius.size();

      memcpy(x.data() + offset, wps_lll.data(), wps_lll.size() * sizeof(x[0]));
      offset += wps_lll.size();
      lbfgs_hzc::lbfgs_parameter_t lbfgs_params;
      lbfgs_params.mem_size = 256; // 128
      lbfgs_params.past = 8;       // 3
      lbfgs_params.g_epsilon = 1.0e-4;
      lbfgs_params.min_step = 1.0e-32;
      lbfgs_params.delta = 1.0e-6;
      lbfgs_params.max_iterations = 10000;
      int result;
      double final_cost;
      double t1 = ros::Time::now().toSec();
      result = lbfgs_hzc::lbfgs_optimize(
          x,
          final_cost,
          GlobalTrajOptimizer::costFunctionCallback,
          NULL,
          NULL,
          this,
          lbfgs_params);
      double t2 = ros::Time::now().toSec();
      ROS_WARN_STREAM("ring planning time: " << 1000.0 * (t2 - t1) << " ms");
      ROS_WARN_STREAM("############################################################");

      ROS_WARN_STREAM("opt->jerkposCost_: " << jerkOpt_.getTrajJerkCost());
      ROS_WARN_STREAM("opt->jerkradiusCost_: " << radiusOpt_.getTrajJerkCost());
      ROS_WARN_STREAM("traj totaltime: " << jerkOpt_.get_T1().sum());

      ROS_WARN_STREAM("############################################################");

      /* ---------- get result and check collision ---------- */
      if (result == lbfgs_hzc::LBFGS_CONVERGENCE ||
          result == lbfgs_hzc::LBFGS_CANCELED ||
          result == lbfgs_hzc::LBFGS_STOP || result == lbfgs_hzc::LBFGSERR_MAXIMUMITERATION)
      {
        ROS_WARN_STREAM("ring planner worked cost:" << final_cost);
      }
      else if (result == lbfgs_hzc::LBFGSERR_MAXIMUMLINESEARCH)
      {
        ROS_WARN_STREAM("ring planner worked cost:" << final_cost);
        // ROS_WARN("Lbfgs: The line-search routine reaches the maximum number of evaluations.");
      }
      else
      {
        ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs_hzc::lbfgs_strerror(result));
      }

      std::cout << "after optimized: " << jerkOpt_.get_T1().transpose() << std::endl;

      posTraj_ = jerkOpt_.getTraj();
      radiusTraj_ = radiusOpt_.getTraj();
      lllTraj_ = lllOpt_.getTraj();

      double r = radiusTraj_.getPos(0.0)(0, 0);
      return true;
    }

    inline int Optimize_localTrajectory(
        const Eigen::MatrixXd &headState,
        const Eigen::MatrixXd &tailState,
        std::vector<Eigen::Vector3d> initial_path)
    {
      headState_local = headState;
      tailState_local = tailState;
      double totalLength = 0.0;
      for (int i = 0; i < initial_path.size() - 1; i++)
      {
        totalLength += (initial_path[i + 1] - initial_path[i]).norm();
      }
      piecenum = std::max(int(totalLength / length_per_piece), 2);
      double tmpl = totalLength / piecenum;
      Eigen::MatrixXd wps;
      wps.resize(3, piecenum - 1);
      double curArc = 0.0;
      int index = 0;
      std::cout << "visualization1111111111111155555555555\n";

      for (int i = 0; i < initial_path.size() - 1; i++)
      {
        curArc += (initial_path[i + 1] - initial_path[i]).norm();
        if (curArc > (index + 1) * tmpl)
        {
          wps.col(index) = initial_path[i];
          index++;
          if (index >= piecenum - 1)
          {
            break;
          }
        }
      }
      Eigen::VectorXd initialT;
      initialT.resize(piecenum);
      initialT.setConstant(tmpl / (vmax / 1.2));
      std::cout << "visualization1111111111111166666666666\n";

      // (min_radius + max_radius) / 2.0);

      std::cout << "piecenum: " << piecenum << std::endl;
      jerkOpt_local.reset(headState_local, tailState_local, piecenum);

      int variable_num_ = 3 * (piecenum - 1) + piecenum;

      Eigen::VectorXd x;
      x.resize(variable_num_);
      int offset = 0;
      memcpy(x.data() + offset, wps.data(), wps.size() * sizeof(x[0]));
      offset += wps.size();

      Eigen::Map<Eigen::VectorXd> Vt(x.data() + offset, initialT.size());
      RealT2VirtualT(initialT, Vt);
      std::cout << "visualization11111111111111777777777777777\n";

      lbfgs_hzc::lbfgs_parameter_t lbfgs_params;
      lbfgs_params.mem_size = 256; // 128
      lbfgs_params.past = 5;       // 3
      lbfgs_params.g_epsilon = 1.0e-4;
      lbfgs_params.min_step = 1.0e-32;
      lbfgs_params.delta = 1.0e-5;
      lbfgs_params.max_iterations = 10000;
      int result;
      double final_cost;
      double t1 = ros::Time::now().toSec();
      result = lbfgs_hzc::lbfgs_optimize(
          x,
          final_cost,
          GlobalTrajOptimizer::costFunctionCallback_local,
          NULL,
          NULL,
          this,
          lbfgs_params);
      double t2 = ros::Time::now().toSec();
      ROS_WARN_STREAM("ring planning time: " << 1000.0 * (t2 - t1) << " ms");
      ROS_WARN_STREAM("############################################################");

      ROS_WARN_STREAM("opt->jerkposCost_: " << jerkOpt_local.getTrajJerkCost());

      ROS_WARN_STREAM("############################################################");

      /* ---------- get result and check collision ---------- */
      if (result == lbfgs_hzc::LBFGS_CONVERGENCE ||
          result == lbfgs_hzc::LBFGS_CANCELED ||
          result == lbfgs_hzc::LBFGS_STOP || result == lbfgs_hzc::LBFGSERR_MAXIMUMITERATION)
      {
        ROS_WARN_STREAM("ring planner worked cost:" << final_cost);
      }
      else if (result == lbfgs_hzc::LBFGSERR_MAXIMUMLINESEARCH)
      {
        ROS_WARN_STREAM("ring planner worked cost:" << final_cost);
        ROS_WARN("Lbfgs: The line-search routine reaches the maximum number of evaluations.");
      }
      else
      {
        ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs_hzc::lbfgs_strerror(result));
      }

      posTraj_local = jerkOpt_local.getTraj();

      return true;
    }
    inline Eigen::Vector3d get_local(double t)
    {
      t = std::min(std::max(t, 0.0), jerkOpt_local.get_T1().sum());
      Eigen::Vector3d pos = posTraj_local.getPos(t);
      return pos;
    }
    inline Eigen::Vector4d getPwithR(double t)
    {
      t = std::min(std::max(t, 0.0), getTotalDuration());
      Eigen::Vector3d pos = posTraj_.getPos(t);
      Eigen::Matrix<double, 1, 1> r = radiusTraj_.getPos(t);
      return Eigen::Vector4d(pos[0], pos[1], pos[2], r(0, 0));
    }
    inline double get_lll(double t)
    {
      t = std::min(std::max(t, 0.0), getTotalDuration());
      Eigen::Matrix<double, 1, 1> lll_fin = lllTraj_.getPos(t);
      return lll_fin(0, 0);
    }
    inline Eigen::Vector3d getAcc_(double t)
    {
      Eigen::Vector3d acc_ = posTraj_.getAcc(t);
      return acc_;
    }
    inline Eigen::Vector3d getVel_(double t)
    {
      Eigen::Vector3d vel_ = posTraj_.getVel(t);
      return vel_;
    }
    inline Eigen::Vector3d getPos_(double t)
    {
      Eigen::Vector3d pos_ = posTraj_.getPos(t);
      return pos_;
    }
    inline double get_max_vel()
    {
      double acc_max = posTraj_.getMaxVelRate();
      return acc_max;
    }
    inline double getTotalDuration()
    {
      return jerkOpt_.get_T1().sum();
    }
    inline double getMaxVelRadius()
    {
      double max_vel_radius = 0.0;
      for (double t = 0.0; t < getTotalDuration(); t += 0.01)
      {
        Eigen::Matrix<double, 1, 1> dr = radiusTraj_.getVel(t);
        max_vel_radius = std::max(max_vel_radius, dr(0, 0));
      }
      return max_vel_radius;
    }
    inline double getMaxAccRadius()
    {
      double max_acc_radius = 0.0;
      for (double t = 0.0; t < getTotalDuration(); t += 0.01)
      {
        Eigen::Matrix<double, 1, 1> ddr = radiusTraj_.getAcc(t);
        max_acc_radius = std::max(max_acc_radius, ddr(0, 0));
      }
      return max_acc_radius;
    }
    inline void get_swarm_des_one(const Eigen::Vector3d &swarm_des_one_)
    {
      swarm_des_one = swarm_des_one_;
    }
    inline void get_opt_fin_time(const double &opt_fin_time_)
    {
      opt_fin_time = opt_fin_time_;
    }

  private:
    void positiveSmoothedL1(const double &x, double &f, double &df)
    {
      const double pe = 1.0e-4;
      const double half = 0.5 * pe;
      const double f3c = 1.0 / (pe * pe);
      const double f4c = -0.5 * f3c / pe;
      const double d2c = 3.0 * f3c;
      const double d3c = 4.0 * f4c;

      if (x < pe)
      {
        f = (f4c * x + f3c) * x * x * x;
        df = (d3c * x + d2c) * x * x;
      }
      else
      {
        f = x - half;
        df = 1.0;
      }
      return;
    }
    static double costFunctionCallback_local(void *func_data, const Eigen::VectorXd &x, Eigen::VectorXd &grad)
    {
      double smcost = 0.0, timecost = 0.0, qvarcost = 0.0;
      GlobalTrajOptimizer *opt = reinterpret_cast<GlobalTrajOptimizer *>(func_data);
      int offset = 0;
      Eigen::Map<const Eigen::MatrixXd> P(x.data() + offset, 3, opt->piecenum - 1);
      Eigen::Map<Eigen::MatrixXd> gradP(grad.data() + offset, 3, opt->piecenum - 1);
      offset += 3 * (opt->piecenum - 1);
      gradP.setZero();

      Eigen::Map<const Eigen::VectorXd> Vdts(x.data() + offset, opt->piecenum);
      Eigen::Map<Eigen::VectorXd> gradVdts(grad.data() + offset, opt->piecenum);
      Eigen::VectorXd dts(opt->piecenum);
      Eigen::VectorXd gradDts(opt->piecenum);
      gradDts.setZero();
      opt->VirtualT2RealT(Vdts, dts);
      offset += opt->piecenum;

      opt->jerkposCost_local = 0.0;

      opt->timeCost_local = 0.0;

      opt->jerkOpt_local.generate(P, dts);              // Generate trajectory from {P,T}
      opt->jerkOpt_local.initGradCost(gradDts, smcost); // Smoothness cost

      opt->jerkposCost_local = smcost;

      opt->addPVAJGradCost2CT_local(gradDts, qvarcost); // Time int cost
      opt->jerkOpt_local.getGrad2TP(gradDts, gradP);    // Gradient prepagation

      // gradDts.setConstant(gradDts.sum() / gradDts.size());
      opt->VirtualTGradCost(dts, Vdts, gradDts, gradVdts, timecost); // Real time back to virtual time
      opt->timeCost_ = timecost;
      return smcost + qvarcost + timecost;
    }
    static double costFunctionCallback(void *func_data, const Eigen::VectorXd &x, Eigen::VectorXd &grad)
    {
      double smcost = 0.0, smcost_radius = 0.0, smcost_lll = 0.0, timecost = 0.0, qvarcost = 0.0;
      GlobalTrajOptimizer *opt = reinterpret_cast<GlobalTrajOptimizer *>(func_data);
      int offset = 0;
      Eigen::Map<const Eigen::MatrixXd> P(x.data() + offset, 3, opt->piecenum - 1);
      Eigen::Map<Eigen::MatrixXd> gradP(grad.data() + offset, 3, opt->piecenum - 1);
      offset += 3 * (opt->piecenum - 1);
      gradP.setZero();

      Eigen::Map<const Eigen::VectorXd> Vdts(x.data() + offset, opt->piecenum);
      Eigen::Map<Eigen::VectorXd> gradVdts(grad.data() + offset, opt->piecenum);
      Eigen::VectorXd dts(opt->piecenum);
      Eigen::VectorXd gradDts(opt->piecenum);
      gradDts.setZero();
      opt->VirtualT2RealT(Vdts, dts);
      offset += opt->piecenum;

      Eigen::Map<const Eigen::MatrixXd> P_radius(x.data() + offset, 1, opt->piecenum - 1);
      Eigen::Map<Eigen::MatrixXd> gradP_radius(grad.data() + offset, 1, opt->piecenum - 1);
      gradP_radius.setZero();
      offset += opt->piecenum - 1;

      Eigen::Map<const Eigen::MatrixXd> P_lll(x.data() + offset, 1, opt->piecenum - 1);
      Eigen::Map<Eigen::MatrixXd> gradP_lll(grad.data() + offset, 1, opt->piecenum - 1);
      gradP_lll.setZero();
      opt->jerkposCost_ = 0.0;
      opt->jerkradiusCost_ = 0.0;
      opt->jerklllCost_ = 0.0;

      opt->timeCost_ = 0.0;
      opt->energyCost_ = 0.0;
      opt->min_dis = 100.0;

      opt->jerkOpt_.generate(P, dts);                       // Generate trajectory from {P,T}
      opt->radiusOpt_.generate(P_radius, dts);              // Generate trajectory from {P,T}
      opt->lllOpt_.generate(P_lll, dts);                    // Generate trajectory from {P,T}
      opt->jerkOpt_.initGradCost(gradDts, smcost);          // Smoothness cost
      opt->radiusOpt_.initGradCost(gradDts, smcost_radius); // Smoothness cost
      opt->lllOpt_.initGradCost(gradDts, smcost_lll);       // Smoothness cost

      opt->jerkposCost_ = smcost;
      opt->jerkradiusCost_ = smcost_radius;
      opt->jerklllCost_ = smcost_lll;

      opt->addPVAJGradCost2CT(gradDts, qvarcost); // Time int cost
      opt->jerkOpt_.getGrad2TP(gradDts, gradP);   // Gradient prepagation

      opt->radiusOpt_.getGrad2TP(gradDts, gradP_radius); // Gradient prepagation

      opt->lllOpt_.getGrad2TP(gradDts, gradP_lll); // Gradient prepagation
      // gradDts.setConstant(gradDts.sum() / gradDts.size());
      opt->VirtualTGradCost(dts, Vdts, gradDts, gradVdts, timecost); // Real time back to virtual time
      opt->timeCost_ = timecost;
      return smcost + smcost_radius + smcost_lll + qvarcost + timecost;
    }
    void addPVAJGradCost2CT(Eigen::VectorXd &gdT, double &cost)
    {
      Eigen::Vector3d sigma, dsigma, ddsigma, dddsigma, ddddsigma, s_sigma;
      Eigen::Vector3d gradPos, gradVel, gradAcc, gradJerk;

      Eigen::Matrix<double, 1, 1> r, dr, ddr, dddr, ddddr;
      Eigen::Matrix<double, 1, 1> gradR, gradDr, gradDdr, gradDddr, gradDdddr;
      Eigen::Matrix<double, 1, 1> l, dl, ddl, dddl, ddddl;
      Eigen::Matrix<double, 1, 1> gradL, gradDl, gradDdl, gradDddl, gradDdddl;
      Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4, beta5;

      double s1, s2, s3, s4, s5;
      double step, alpha;

      for (int i = 0; i < piecenum; ++i)
      {
        const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
        const Eigen::Matrix<double, 6, 1> &c_R = radiusOpt_.get_b().block<6, 1>(i * 6, 0);
        const Eigen::Matrix<double, 6, 1> &c_L = lllOpt_.get_b().block<6, 1>(i * 6, 0);
        step = jerkOpt_.get_T1()(i) / traj_res; // T_i /k
        s1 = 0.0;
        for (int j = 1; j <= traj_res; ++j)
        {
          s2 = s1 * s1;
          s3 = s2 * s1;
          s4 = s2 * s2;
          s5 = s4 * s1;
          beta0 << 1.0, s1, s2, s3, s4, s5;
          beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
          beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
          beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
          beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * s1;
          beta5 << 0.0, 0.0, 0.0, 0.0, 24.0, 120;
          alpha = 1.0 / traj_res * j;

          // update s1 for the next iteration
          s1 += step;

          sigma = c.transpose() * beta0;
          dsigma = c.transpose() * beta1;
          ddsigma = c.transpose() * beta2;
          dddsigma = c.transpose() * beta3;
          ddddsigma = c.transpose() * beta4;
          s_sigma = c.transpose() * beta5;
          gradPos.setZero();
          gradVel.setZero();
          gradAcc.setZero();
          gradJerk.setZero();
          Eigen::Vector3d pos, vel, acc, jerk;
          pos = sigma;
          vel = dsigma;
          acc = ddsigma;
          jerk = dddsigma;

          r = c_R.transpose() * beta0;
          dr = c_R.transpose() * beta1;
          ddr = c_R.transpose() * beta2;
          dddr = c_R.transpose() * beta3;
          ddddr = c_R.transpose() * beta4;
          gradR.setZero();
          gradDr.setZero();
          gradDdr.setZero();
          gradDddr.setZero();
          gradDdddr.setZero();

          l = c_L.transpose() * beta0;
          dl = c_L.transpose() * beta1;
          ddl = c_L.transpose() * beta2;
          dddl = c_L.transpose() * beta3;
          ddddl = c_L.transpose() * beta4;
          gradL.setZero();
          gradDl.setZero();
          gradDdl.setZero();
          gradDddl.setZero();
          gradDdddl.setZero();
          /////LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
          {
            // L-Lmax<0
            double vioLmax = l.col(0)[0] - max_lll;
            if (vioLmax > 0)
            {
              double pena, penaD;
              positiveSmoothedL1(vioLmax, pena, penaD);
              cost += penaWei * pena;
              gradL.col(0)[0] += penaWei * penaD * 1.0;
            }
          }
          {
            // Lmin - L < 0
            double vioLmin = min_lll - l.col(0)[0];
            if (vioLmin > 0)
            {
              double pena, penaD;
              positiveSmoothedL1(vioLmin, pena, penaD);
              cost += penaWei * pena;
              gradL.col(0)[0] += penaWei * penaD * -1.0;
            }
          }
          {
            double vioLvel = dl(0, 0) * dl(0, 0) - lllmax * lllmax;
            if (vioLvel > 0)
            {
              double pena, penaD;
              positiveSmoothedL1(vioLvel, pena, penaD);
              cost += penaWei * pena;
              gradDl.col(0)[0] += penaWei * penaD * 2.0 * dl(0, 0);
            }
          }
          {
            double vioLacc = ddl(0, 0) * ddl(0, 0) - llldotmax * llldotmax;
            if (vioLacc > 0)
            {
              double pena, penaD;
              positiveSmoothedL1(vioLacc, pena, penaD);
              cost += penaWei * pena;
              gradDdl.col(0)[0] += penaWei * penaD * 2.0 * ddl(0, 0);
            }
          }
          /////RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR
          {
            // r-rmax<0
            double vioRmax = r.col(0)[0] - max_radius;
            if (vioRmax > 0)
            {
              double pena, penaD;
              positiveSmoothedL1(vioRmax, pena, penaD);
              cost += penaWei * pena;
              gradR.col(0)[0] += penaWei * penaD * 1.0;
            }
          }
          {
            // rmin - r < 0
            double vioRmin = min_radius - r.col(0)[0];
            if (vioRmin > 0)
            {
              double pena, penaD;
              positiveSmoothedL1(vioRmin, pena, penaD);
              cost += penaWei * pena;
              gradR.col(0)[0] += penaWei * penaD * -1.0;
            }
          }
          {
            double vioRvel = dr(0, 0) * dr(0, 0) - omegamax * omegamax;
            if (vioRvel > 0)
            {
              double pena, penaD;
              positiveSmoothedL1(vioRvel, pena, penaD);
              cost += penaWei * pena;
              gradDr.col(0)[0] += penaWei * penaD * 2.0 * dr(0, 0);
            }
          }
          {
            double vioRacc = ddr(0, 0) * ddr(0, 0) - omegadotmax * omegadotmax;
            if (vioRacc > 0)
            {
              double pena, penaD;
              positiveSmoothedL1(vioRacc, pena, penaD);
              cost += penaWei * pena;
              gradDdr.col(0)[0] += penaWei * penaD * 2.0 * ddr(0, 0);
            }
          }
          /////PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
          {
            double vioVel = vel.squaredNorm() - vmax * vmax;
            if (vioVel > 0)
            {
              double pena, penaD;
              positiveSmoothedL1(vioVel, pena, penaD);
              cost += penaWei * pena;
              gradVel += penaWei * penaD * 2.0 * dsigma;
            }
          }

          {
            double vioAcc = acc.squaredNorm() - amax * amax;
            if (vioAcc > 0)
            {
              double pena, penaD;
              positiveSmoothedL1(vioAcc, pena, penaD);
              cost += penaWei * pena;
              gradAcc += penaWei * penaD * 2.0 * ddsigma;
            }
          }

          ///////////obssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss
          // {
          //   for (double theta = 0.0; theta < 2 * M_PI; theta += M_PI / 12.0)
          //   {
          //     Eigen::Vector3d coord = pos + r(0, 0) * Eigen::Vector3d(l(0, 0) * cos(theta), (1.0 / l(0, 0)) * sin(theta), 0.0);
          //     Eigen::Vector3d gradViolaSdpos;
          //     double dist_map;
          //     map_itf_->evaluateEDTWithGrad(coord, dist_map, gradViolaSdpos);
          //     double dis = dist_map;
          //     double vioSdist = (-dis + safeMargin);
          //     min_dis = std::min(min_dis, dis);
          //     // std::cout << "min_dis: " << min_dis << std::endl;
          //     if (vioSdist > 0)
          //     {
          //       double pena, penaD;
          //       positiveSmoothedL1(vioSdist, pena, penaD);
          //       cost += esdfWei * pena;
          //       gradPos += esdfWei * penaD * (-1.0) * gradViolaSdpos;

          //       gradR.col(0)[0] += esdfWei * penaD * (-1.0) * gradViolaSdpos.dot(Eigen::Vector3d(l(0, 0) * cos(theta), (1.0 / l(0, 0)) * sin(theta), 0.0));

          //       gradL.col(0)[0] += esdfWei * penaD * (-1.0) * gradViolaSdpos.dot(r(0, 0) * Eigen::Vector3d(cos(theta), -(1.0 / (l(0, 0) * l(0, 0))) * sin(theta), 0.0));
          //     }
          //   }
          // }
          ///////////every fessssssssssssssssssssssssssssssssssssssssssssssssssssssssss
          {
            for (double theta = 0.0; theta < 2 * M_PI; theta += M_PI / 12.0)
            {
              Eigen::Vector3d coord_vel = vel + dr(0, 0) * Eigen::Vector3d(l(0, 0) * cos(theta), (1.0 / l(0, 0)) * sin(theta), 0.0) +
                                          r(0, 0) * Eigen::Vector3d(dl(0, 0) * cos(theta), -(dl(0, 0) / (l(0, 0) * l(0, 0))) * sin(theta), 0.0);
              double vioVel = coord_vel.squaredNorm() - vmax * vmax;
              if (vioVel > 0)
              {
                double pena, penaD;
                positiveSmoothedL1(vioVel, pena, penaD);
                cost += penaWei * pena;
                gradVel += penaWei * penaD * 2.0 * coord_vel;
                gradDr.col(0)[0] += penaWei * penaD * 2.0 * coord_vel.dot(Eigen::Vector3d(l(0, 0) * cos(theta), (1.0 / l(0, 0)) * sin(theta), 0.0));
                gradDl.col(0)[0] += penaWei * penaD * 2.0 * coord_vel.dot(r(0, 0) * Eigen::Vector3d(cos(theta), -(1.0 / (l(0, 0) * l(0, 0))) * sin(theta), 0.0));
              }
            }
          }

          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += beta0 * gradPos.transpose() +
                                                      beta1 * gradVel.transpose() +
                                                      beta2 * gradAcc.transpose() +
                                                      beta3 * gradJerk.transpose();

          radiusOpt_.get_gdC().block<6, 1>(i * 6, 0) += beta0 * gradR.transpose() +
                                                        beta1 * gradDr.transpose() +
                                                        beta2 * gradDdr.transpose() +
                                                        beta3 * gradDddr.transpose();

          lllOpt_.get_gdC().block<6, 1>(i * 6, 0) += beta0 * gradL.transpose() +
                                                     beta1 * gradDl.transpose() +
                                                     beta2 * gradDdl.transpose() +
                                                     beta3 * gradDddl.transpose();

          gdT(i) += (gradPos.dot(vel) +
                     gradVel.dot(acc) +
                     gradAcc.dot(jerk) +
                     gradJerk.dot(ddddsigma)) *
                        alpha +
                    (gradR.dot(dr) +
                     gradDr.dot(ddr) +
                     gradDdr.dot(dddr) +
                     gradDddr.dot(ddddr)) *
                        alpha +
                    (gradL.dot(dl) +
                     gradDl.dot(ddl) +
                     gradDdl.dot(dddl) +
                     gradDddl.dot(ddddl)) *
                        alpha;
        }
      }
    }
    void addPVAJGradCost2CT_local(Eigen::VectorXd &gdT, double &cost)
    {
      Eigen::Vector3d sigma, dsigma, ddsigma, dddsigma, ddddsigma, s_sigma;
      Eigen::Vector3d gradPos, gradVel, gradAcc, gradJerk;
      double t_cur = 0.0;
      double t_sum = 0.0;

      Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4, beta5;

      double s1, s2, s3, s4, s5;
      double step, alpha;

      for (int i = 0; i < piecenum; ++i)
      {
        const Eigen::Matrix<double, 6, 3> &c = jerkOpt_local.get_b().block<6, 3>(i * 6, 0);
        step = jerkOpt_local.get_T1()(i) / traj_res; // T_i /k
        s1 = 0.0;
        for (int j = 1; j <= traj_res; ++j)
        {
          t_cur = t_sum + step * j;
          s2 = s1 * s1;
          s3 = s2 * s1;
          s4 = s2 * s2;
          s5 = s4 * s1;
          beta0 << 1.0, s1, s2, s3, s4, s5;
          beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
          beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
          beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
          beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * s1;
          beta5 << 0.0, 0.0, 0.0, 0.0, 24.0, 120;
          alpha = 1.0 / traj_res * j;

          // update s1 for the next iteration
          s1 += step;

          sigma = c.transpose() * beta0;
          dsigma = c.transpose() * beta1;
          ddsigma = c.transpose() * beta2;
          dddsigma = c.transpose() * beta3;
          ddddsigma = c.transpose() * beta4;
          s_sigma = c.transpose() * beta5;
          gradPos.setZero();
          gradVel.setZero();
          gradAcc.setZero();
          gradJerk.setZero();
          Eigen::Vector3d pos, vel, acc, jerk;
          pos = sigma;
          vel = dsigma;
          acc = ddsigma;
          jerk = dddsigma;

          /////PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
          {
            double vioVel = vel.squaredNorm() - vmax_local * vmax_local;
            if (vioVel > 0)
            {
              double pena, penaD;
              positiveSmoothedL1(vioVel, pena, penaD);
              cost += penaWei * pena;
              gradVel += penaWei * penaD * 2.0 * dsigma;
            }
          }

          {
            double vioAcc = acc.squaredNorm() - amax_local * amax_local;
            if (vioAcc > 0)
            {
              double pena, penaD;
              positiveSmoothedL1(vioAcc, pena, penaD);
              cost += penaWei * pena;
              gradAcc += penaWei * penaD * 2.0 * ddsigma;
            }
          }

          ///////////obssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss
          // {
          //   Eigen::Vector3d coord = pos;
          //   Eigen::Vector3d gradViolaSdpos;
          //   double dist_map;
          //   map_itf_->evaluateEDTWithGrad(coord, dist_map, gradViolaSdpos);
          //   double dis = dist_map;
          //   double vioSdist = (-dis + safeMargin / 10);
          //   // std::cout << "min_dis: " << min_dis << std::endl;
          //   if (vioSdist > 0)
          //   {
          //     double pena, penaD;
          //     positiveSmoothedL1(vioSdist, pena, penaD);
          //     cost += esdfWei_local * pena;
          //     gradPos += esdfWei_local * penaD * (-1.0) * gradViolaSdpos;
          //   }
          // }
          ///////////formation_timeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
          Eigen::Vector4d trajP_local;
          double traj_lll_local;
          if (t_cur <= opt_fin_time)
          {
            trajP_local = getPwithR(t_cur);
            traj_lll_local = get_lll(t_cur);
          }
          else
          {
            trajP_local = getPwithR(opt_fin_time);
            traj_lll_local = get_lll(opt_fin_time);
          }
          Eigen::Vector3d pos_formation_des = trajP_local.head(3) + trajP_local[3] * Eigen::Vector3d(traj_lll_local * swarm_des_one[0], (1.0 / traj_lll_local) * swarm_des_one[1], swarm_des_one[2]); // swarm_des_one
          Eigen::Vector3d formation_Viola = pos - pos_formation_des;
          double dist_real = formation_Viola.norm();
          double dist2_real = formation_Viola.squaredNorm();
          if (dist2_real > 0)
          {
            cost += penaform * dist2_real;
            gradPos += penaform * formation_Viola;
          }

          jerkOpt_local.get_gdC()
              .block<6, 3>(i * 6, 0) += beta0 * gradPos.transpose() +
                                        beta1 * gradVel.transpose() +
                                        beta2 * gradAcc.transpose() +
                                        beta3 * gradJerk.transpose();

          gdT(i) += (gradPos.dot(vel) +
                     gradVel.dot(acc) +
                     gradAcc.dot(jerk) +
                     gradJerk.dot(ddddsigma)) *
                    alpha;
        }
        t_sum += jerkOpt_local.get_T1()(i);
      }
    }
    void positiveSmoothedL3(const double &x, double &f, double &df)
    {
      df = x * x;
      f = df * x;
      df *= 3.0;

      return;
    }

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
    {
      for (int i = 0; i < VT.size(); ++i)
      {
        RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0) + gslar_
                            : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0) + gslar_;
      }
    }
    void VirtualT2RealT(const double &VT, double &RT)
    {

      RT = VT > 0.0 ? ((0.5 * VT + 1.0) * VT + 1.0) + gslar_
                    : 1.0 / ((0.5 * VT - 1.0) * VT + 1.0) + gslar_;
    }
    template <typename EIGENVEC>
    inline void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
    {
      for (int i = 0; i < RT.size(); ++i)
      {
        VT(i) = RT(i) > 1.0 + gslar_
                    ? (sqrt(2.0 * RT(i) - 1.0 - 2 * gslar_) - 1.0)
                    : (1.0 - sqrt(2.0 / (RT(i) - gslar_) - 1.0));
      }
    }
    inline void RealT2VirtualT(const double &RT, double &VT)
    {
      VT = RT > 1.0 + gslar_
               ? (sqrt(2.0 * RT - 1.0 - 2 * gslar_) - 1.0)
               : (1.0 - sqrt(2.0 / (RT - gslar_) - 1.0));
    }
    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT, const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT, double &costT)
    {
      for (int i = 0; i < VT.size(); ++i)
      {
        double gdVT2Rt;
        if (VT(i) > 0)
        {
          gdVT2Rt = VT(i) + 1.0;
        }
        else
        {
          double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
          gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
        }

        gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
      }

      costT = RT.sum() * wei_time_;
    }
    void VirtualTGradCost(const double &RT, const double &VT, const double &gdRT, double &gdVT, double &costT)
    {
      double gdVT2Rt;
      if (VT > 0)
      {
        gdVT2Rt = VT + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT - 1.0) * VT + 1.0;
        gdVT2Rt = (1.0 - VT) / (denSqrt * denSqrt);
      }

      gdVT = (gdRT + wei_time_) * gdVT2Rt;
      costT = RT * wei_time_;
    }
    void VirtualTGrad2t(const double &VT, const double &gdRT, double &gdVT)
    {
      double gdVT2Rt;
      if (VT > 0)
      {
        gdVT2Rt = VT + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT - 1.0) * VT + 1.0;
        gdVT2Rt = (1.0 - VT) / (denSqrt * denSqrt);
      }
      gdVT = gdRT * gdVT2Rt;
    }
    template <typename EIGENVEC, typename EIGENVECGD>
    void Virtual2Grad(const EIGENVEC &VT, const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT)
    {
      for (int i = 0; i < VT.size(); ++i)
      {
        double gdVT2Rt;
        if (VT(i) > 0)
        {
          gdVT2Rt = VT(i) + 1.0;
        }
        else
        {
          double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
          gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
        }

        gdVT(i) = (gdRT(i))*gdVT2Rt;
      }
    }
    double expC2(double t)
    {
      return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                     : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
    }
    double logC2(double T)
    {
      return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
    }
    void forwardT(const Eigen::Ref<const Eigen::VectorXd> &t, const double &sT, Eigen::Ref<Eigen::VectorXd> vecT)
    {
      int M = t.size();
      for (int i = 0; i < M; ++i)
      {
        vecT(i) = expC2(t(i));
      }
      vecT(M) = 0.0;
      vecT /= 1.0 + vecT.sum();
      vecT(M) = 1.0 - vecT.sum();
      vecT *= sT;
      return;
    }
    void backwardT(const Eigen::Ref<const Eigen::VectorXd> &vecT, Eigen::Ref<Eigen::VectorXd> t)
    {
      int M = t.size();
      t = vecT.head(M) / vecT(M);
      for (int i = 0; i < M; ++i)
      {
        t(i) = logC2(vecT(i));
      }
      return;
    }

  public:
    typedef unique_ptr<GlobalTrajOptimizer> Ptr;
  };

} // namespace plan_manage
#endif
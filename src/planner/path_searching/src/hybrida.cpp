#include "path_searching/hybrida.h"
#include <sstream>
using namespace std;
using namespace Eigen;

// namespace hybrid
// {
HybridAstar::~HybridAstar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

int HybridAstar::search(Eigen::Vector4d start_pt, Eigen::Vector4d start_vel,
                             Eigen::Vector4d start_acc, Eigen::Vector4d end_pt,
                             Eigen::Vector4d end_vel, bool use_shaping)
{
  use_shaping = true;
  start_vel_ = start_vel;
  start_acc_ = start_acc;
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(4) = start_pt;
  cur_node->state.tail(4) = start_vel;
  Eigen::Vector3i pos_index_begin;
  edt_environment_->posToIndex(start_pt.head(3), pos_index_begin);
  cur_node->posindex = pos_index_begin;
  cur_node->radius_idx = radiusToIndex(start_pt[3]);
  cur_node->g_score = 0.0;

  Eigen::VectorXd end_state(8);
  Eigen::Vector3i end_index;
  int end_radius_idx;
  end_state.head(4) = end_pt;
  end_state.tail(4) = end_vel;
  Eigen::Vector3i pos_index_end;
  edt_environment_->posToIndex(end_pt.head(3), pos_index_end);
  end_index = pos_index_end;
  end_radius_idx = radiusToIndex(end_pt[3]);
  double time_to_goal;
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = OPEN;
  open_set_.push(cur_node);
  use_node_num_ += 1;

  // if(use_shaping){
  expanded_nodes_.insert(cur_node->posindex, cur_node->radius_idx, cur_node);
  // }
  // else{
  //   expanded_nodes_.insert(cur_node->posindex, cur_node);
  // }
  //

  PathNodePtr neighbor = NULL;
  PathNodePtr terminate_node = NULL;
  bool init_search = false;
  const int tolerance = ceil(1 / resolution_);

  // if(map_util_->getDistance(end_pt)<0.2){
  //   ROS_ERROR("hybridastart end is not free");
  // }
  if (isCollision(start_pt.head(3), start_pt[3]) || isCollision(end_pt.head(3), end_pt[3]))
  {
    ROS_ERROR("hybridastart start end is not free");
  }

  while (!open_set_.empty())
  {
    cur_node = open_set_.top();

    // Terminate?
    bool reach_horizon = (cur_node->state.head(2) - start_pt.head(2)).norm() >= horizon_;
    bool near_end = abs(cur_node->posindex(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->posindex(1) - end_index(1)) <= tolerance;

    if (reach_horizon || near_end)
    {
      terminate_node = cur_node;
      retrievePath(terminate_node);
      if (near_end)
      {
        // Check whether shot traj exist
        estimateHeuristic(cur_node->state, end_state, time_to_goal);
        computeShotTraj(cur_node->state, end_state, time_to_goal);
      }
    }
    if (reach_horizon)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      else
      {
        std::cout << "reach horizon" << std::endl;
        return REACH_HORIZON;
      }
    }

    if (near_end)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        std::cout << "use_node_num: " << use_node_num_ << std::endl;
        return REACH_END;
      }
      else if (cur_node->parent != NULL)
      {
        std::cout << "near end" << std::endl;
        return NEAR_END;
      }
      else
      {
        std::cout << "no path" << std::endl;
        return NO_PATH;
      }
    }
    open_set_.pop();
    cur_node->node_state = CLOSE;

    auto it = std::find(debug_visited_nodes_.begin(), debug_visited_nodes_.end(),
                        Eigen::Vector4i(cur_node->posindex[0], cur_node->posindex[1], cur_node->posindex[2], cur_node->radius_idx));
    // 判断是否找到
    if (it != debug_visited_nodes_.end())
    {
      ROS_ERROR("11111111111111111111111111111111");
    }

    debug_visited_nodes_.emplace_back(cur_node->posindex[0], cur_node->posindex[1], cur_node->posindex[2], cur_node->radius_idx);

    iter_num_ += 1;

    double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 2.0;
    Eigen::Matrix<double, 8, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 8, 1> pro_state;
    vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector4d um;
    double pro_t;
    vector<Eigen::Vector4d> inputs;
    vector<double> durations;

    if (init_search)
    {
      inputs.push_back(start_acc_);
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
           tau += time_res_init * init_max_tau_)
        durations.push_back(tau);
      init_search = false;
    }
    else
    {
      if (use_shaping)
      {
        for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
          for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
            for (double al = -max_lll_acc; al <= max_lll_acc + 1e-3; al += 0.5)
              for (double ar = -max_radius_acc_; ar <= max_radius_acc_ + 1e-3; ar += 0.5)
              {
                um << ax, ay, al, ar;
                inputs.push_back(um);
              }
      }
      else
      {
        for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
          for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
            for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
            {
              um << ax, ay, az, 0.0;
              inputs.push_back(um);
            }
      }

      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        durations.push_back(tau);
    }
    // cout << "cur state:" << cur_state.head(3).transpose() << endl;
    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j)
      {
        um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);
        Eigen::Vector3d pro_pos = Eigen::Vector3d(pro_state.x(), pro_state.y(), fake_z);
        Eigen::Vector3i pro_id_index;
        edt_environment_->posToIndex(pro_pos, pro_id_index);
        Eigen::Vector3i pro_id = pro_id_index;
        int radius_idx = radiusToIndex(pro_state[3]);
        int l_idx = lToIndex(pro_state[2]);
        // pro_id[2] = l_idx;
        Eigen::Vector3i pro_id_fake = Eigen::Vector3i(pro_id.x(), pro_id.y(), l_idx);
        PathNodePtr pro_node = expanded_nodes_.find(pro_id_fake, radius_idx);
        // PathNodePtr pro_node =expanded_nodes_.find(pro_id);

        if (pro_node != NULL && pro_node->node_state == CLOSE)
        {
          continue;
        }
        // Check maximal velocity
        Eigen::Vector3d pro_v = pro_state.segment(4, 3);
        if (pro_v.head(2).norm() > max_vel_ || pro_state[3] < min_radius_ || pro_state[3] > max_radius_ || pro_state[2] < min_lll || pro_state[2] > max_lll || fabs(pro_state[6]) > max_lll_vel ||
            fabs(pro_state[7]) > max_radius_vel_)
        {
          continue;
        }

        // Check not in the same voxel
        Eigen::Vector3i diff = pro_id_fake - Eigen::Vector3i(cur_node->posindex.x(), cur_node->posindex.y(), pro_id_fake[2]);
        int diff_radius = radius_idx - cur_node->radius_idx;
        int diff_lll = l_idx - cur_node->posindex.z();
        if (diff.norm() == 0 && diff_radius == 0 && diff_lll == 0)
        {
          continue;
        }

        // Check safety
        Eigen::Vector3d pos = pro_state.head(3);
        double radius = pro_state[3];
        Eigen::Matrix<double, 8, 1> xt;
        bool is_occ = false;
        for (int k = 1; k <= check_num_; ++k)
        {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);
          pos = xt.head(3);
          // if (edt_environment_->getDistance(pos)<xt[3])
          // {
          //   is_occ = true;
          //   break;
          // }
          if (use_shaping)
          {
            if (isCollision(pos, xt[3]))
            {
              is_occ = true;
              break;
            }
          }
          else
          {
            double rrr = max_radius_;
            if (front_is_deform_ == 1)
            {
              rrr = max_radius_;
            }
            if (front_is_deform_ == 2 || back_is_deform_ == 4)
            {
              rrr = min_radius_;
            }
            pro_state[3] = rrr;
            if (isCollision(pos, rrr))
            {
              is_occ = true;
              break;
            }
          }
        }
        if (is_occ)
        {
          continue;
        }

        double time_to_goal, tmp_g_score, tmp_f_score;
        double middle_r = (pro_state[3] + cur_state[3]) / 2.0;
        double ew = 0.1 / (middle_r * middle_r + 0.00000001);
        Eigen::Vector3d pos_um = Eigen::Vector3d(um[0], um[1], um[2]);
        double aaa = 1.0;
        tmp_g_score = (um.squaredNorm() + um[2] * um[2] + um[3] * um[3]) * tau + w_time_ * tau + cur_node->g_score;
        // if(front_is_deform_==3&&back_is_deform_!=4)
        // {
        //   tmp_g_score = (um.squaredNorm()) * tau  + w_time_ * tau + aaa * ((middle_r-max_radius_)*(middle_r-max_radius_)/(max_radius_*max_radius_)) * tau + cur_node->g_score;
        // }else{
        //   tmp_g_score = (pos_um.squaredNorm()) * tau  + w_time_ * tau + cur_node->g_score;
        // }

        // tmp_g_score = (aaa * pow((pro_state[3] - max_radius_)/(max_radius_ * max_radius_) , 2.0) + 1.0) * pos_um.squaredNorm() * tau + um[3] * um[3] * tau + w_time_ * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);
        // std::cout << "pro_state: "<<pro_state.transpose() << std::endl;
        // std::cout << "heuristic: "<< estimateHeuristic(pro_state, end_state, time_to_goal) << std::endl;
        // ros::Duration(0.1).sleep();

        // Compare nodes expanded from the same parent
        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id_fake - Eigen::Vector3i(expand_node->posindex.x(), expand_node->posindex.y(), fake_z)).norm() == 0 && (radius_idx == expand_node->radius_idx) && (l_idx == expand_node->posindex.z()))
          {
            prune = true;
            if (tmp_f_score < expand_node->f_score)
            {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
            }
            break;
          }
        }

        // This node end up in a voxel different from others
        if (!prune)
        {
          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->posindex = pro_id_fake;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = OPEN;
            pro_node->radius_idx = radiusToIndex(radius);
            open_set_.push(pro_node);
            expanded_nodes_.insert(pro_id, pro_node->radius_idx, pro_node);
            // expanded_nodes_.insert(pro_id, pro_node);
            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;
            // std::cout << "use_node_num: " << use_node_num_ << std::endl;
            if (use_node_num_ == allocate_num_)
            {
              cout << "run out of memory." << endl;
              return NO_PATH;
            }
          }
          else if (pro_node->node_state == OPEN)
          {
            if (tmp_g_score < pro_node->g_score)
            {
              pro_node->posindex = pro_id_fake;
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              pro_node->radius_idx = radiusToIndex(radius);
            }
          }
          else
          {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }
        }
        // init_search = false;
      }
  }

  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

void HybridAstar::setParam(ros::NodeHandle &nh)
{
  nh.param("kinosearch/max_tau", max_tau_, 0.4);
  nh.param("kinosearch/init_max_tau", init_max_tau_, 0.4);
  nh.param("max_vel", max_vel_, 5.0);
  nh.param("max_acc", max_acc_, 6.0);
  nh.param("min_radius", min_radius_, 0.8);
  nh.param("max_radius", max_radius_, 1.0);
  nh.param("min_lll", min_lll, 0.25);
  nh.param("max_lll", max_lll, 4.0);
  nh.param("grid_map/resolution", resolution_, 0.1);
  nh.param("omegamax", max_radius_vel_, 1.0);
  nh.param("omegadotmax", max_radius_acc_, 0.25);
  nh.param("lllvel_max", max_lll_vel, 1.0);
  nh.param("llldot_max", max_lll_acc, 0.25);
  nh.param("weight_time", w_time_, 1000.0);
  nh.param("kinosearch/horizon", horizon_, 1000.0);
  nh.param("kinosearch/radius_resolution", radius_resolution_, 0.02);
  nh.param("kinosearch/lll_resolution", lll_resolution, 0.04);
  nh.param("kinosearch/lambda_heu", lambda_heu_, 2.0);
  nh.param("kinosearch/allocate_num", allocate_num_, 1000000);
  nh.param("kinosearch/check_num", check_num_, 15);
  nh.param("robot_z", robot_z, 0.2);
  nh.param("front_is_deform_", front_is_deform_, 3);
  nh.param("back_is_deform_", back_is_deform_, 4);
  nh.param("fack_z_", fake_z, 1.0);
  std::cout << "resolution: " << resolution_ << std::endl;
  std::cout << "hybridastar: max_vel" << max_vel_ << std::endl;
  std::cout << "hybridastar: max_acc" << max_acc_ << std::endl;
  std::cout << "front_is_deform_" << front_is_deform_ << std::endl;
  tie_breaker_ = 1.0 + 1.0 / 10000;
  w_time_ = 1.0;

}

void HybridAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}
double HybridAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time)
{
  const Vector4d dp = x2.head(4) - x1.head(4);
  const Vector4d v0 = x1.segment(4, 4);
  const Vector4d v1 = x2.segment(4, 4);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_ * 0.5;
  double t_bar = (x1.head(2) - x2.head(2)).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d = t_bar;

  for (auto t : ts)
  {
    if (t < t_bar)
      continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
}

bool HybridAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  /* ---------- get coefficient ---------- */
  const Vector4d p0 = state1.head(4);
  const Vector4d dp = state2.head(4) - p0;
  const Vector4d v0 = state1.segment(4, 4);
  const Vector4d v1 = state2.segment(4, 4);
  const Vector4d dv = v1 - v0;
  double t_d = time_to_goal;
  MatrixXd coef(4, 4);
  end_vel_ = v1;

  Vector4d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector4d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector4d c = v0;
  Vector4d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Vector4d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector4i index;

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta)
  {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t(j) = pow(time, j);

    for (int dim = 0; dim < 4; dim++)
    {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      // vel(dim) = (Tm * poly1d).dot(t);
      // acc(dim) = (Tm * Tm * poly1d).dot(t);

      // if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
      // {
      // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
      // return false;
      // }
    }

    if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
        coord(2) < origin_(2) || coord(2) >= map_size_3d_(2))
    {
      ROS_ERROR("shot out of map");
      return false;
    }
    Eigen::Vector3d pos = coord.head(3);
    if (edt_environment_->getDistance(pos) < coord[3])
    {
      ROS_ERROR("shot occupied");
      return false;
    }
    // if(isCollision(pos, coord(3))){
    //   ROS_ERROR("shot occupied");
    //   return false;
    // }
  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  ROS_WARN_STREAM("shot traj computed");
  return true;
}
bool HybridAstar::isCollision(Eigen::Vector3d c, double r)
{
  //  for(double theta = 0.0; theta < 2*M_PI; theta += M_PI/6.0){
  //   for(double z = -robot_z/2.0; z <= robot_z/2.0; z += 0.01){
  //     Eigen::Vector3d pos = c + r*Eigen::Vector3d(cos(theta), sin(theta), 0.0) + Eigen::Vector3d(0.0, 0.0, z);
  //     // Eigen::Vector3d pos = c;
  //     if (edt_environment_->getDistance(pos)<0.01)
  //     {
  //       return true;
  //     }
  //  }
  // }

  for (double theta = 0.0; theta < 2 * M_PI; theta += M_PI / 8.0)
  {
    for (double step_obs = 0.0; step_obs < 1.0; step_obs = step_obs + 0.05)
    {
      // Eigen::Vector2d pos = Eigen::Vector2d(c.x(), c.y()) + 0.0 * Eigen::Vector2d(1.0*(shape_to_graph[point_i].x() - c.x()), (1.0/1.0)*(shape_to_graph[point_i].y()-c.y()));
      Eigen::Vector3d pos_real = Eigen::Vector3d(c.x(), c.y(), fake_z) + step_obs * r * Eigen::Vector3d(c.z() * cos(theta), (1 / c.z()) * sin(theta), 0.0);

      if (edt_environment_->getDistance(pos_real) < 0.01)
      {
        return true;
      }
    }
  }
  return false;
}

vector<double> HybridAstar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

vector<double> HybridAstar::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

void HybridAstar::init()
{
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  inv_radius_resolution_ = 1.0 / radius_resolution_;
  inv_lll_resolution = 1.0 / lll_resolution;
  origin_ = edt_environment_->getOrigin();
  map_size_3d_ = edt_environment_->getMapSize();
  ROS_INFO_STREAM("origin_: " << origin_.transpose());
  ROS_INFO_STREAM("map_size_3d_: " << map_size_3d_.transpose());

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;
  }

  phi_ = Eigen::MatrixXd::Identity(8, 8);
  use_node_num_ = 0;
  iter_num_ = 0;
}

void HybridAstar::setEnvironment(const GridMap::Ptr &env)
{
  edt_environment_ = env;
}

void HybridAstar::reset()
{
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOEXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_ = false;
}

std::vector<Eigen::Vector4d> HybridAstar::getKinoTraj(double delta_t)
{
  vector<Vector4d> state_list;

  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();
  Matrix<double, 8, 1> x0, xt;

  while (node->parent != NULL)
  {
    Vector4d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(4));
    }
    node = node->parent;
  }
  reverse(state_list.begin(), state_list.end());
  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    Vector4d coord;
    VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 4; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}

std::vector<PathNodePtr> HybridAstar::getVisitedNodes()
{
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i HybridAstar::posToIndex(Eigen::Vector3d pt)
{
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

  return idx;
}

int HybridAstar::radiusToIndex(double radius)
{
  int idx = floor((radius - min_radius_) * inv_radius_resolution_);
  return idx;
}
int HybridAstar::lToIndex(double lll)
{
  int idx = floor((lll - min_lll) * inv_lll_resolution);
  return idx;
}
void HybridAstar::stateTransit(Eigen::Matrix<double, 8, 1> &state0, Eigen::Matrix<double, 8, 1> &state1,
                                    Eigen::Vector4d um, double tau)
{
  for (int i = 0; i < 4; ++i)
    phi_(i, i + 4) = tau;

  Eigen::Matrix<double, 8, 1> integral;
  integral.head(4) = 0.5 * pow(tau, 2) * um;
  integral.tail(4) = tau * um;

  state1 = phi_ * state0 + integral;
}
// }
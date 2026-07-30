#include <path_searching/kinodynamic_astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

  KinodynamicAstar::~KinodynamicAstar()
  {
    for (int i = 0; i < allocate_num_; i++)
    {
      delete path_node_pool_[i];
    }
  }
  //以及两个标志位init 和dynamic，还有一个搜索起始时间。将起始点及目标点的三维位置转化至栅格地图的index. 并计算第一个扩展点的Heuristic cost.
  int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                               Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
  {
    ztr_log4<<"--------------in search kino A star---------------------"<<endl;
    start_vel_ = start_v;
    start_acc_ = start_a;

    PathNodePtr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->state.head(3) = start_pt; //成员是Eigen::Matrix<double, 6, 1> state;
    cur_node->state.tail(3) = start_v;
    cur_node->index = posToIndex(start_pt);
    cur_node->g_score = 0.0;

    Eigen::VectorXd end_state(6);
    Eigen::Vector3i end_index;
    double time_to_goal;

    end_state.head(3) = end_pt;
    end_state.tail(3) = end_v;      //对应文章中的III.B小节，主要原理是利用庞特里亚金原理解决两点边值问题
    end_index = posToIndex(end_pt); //，得到最优解后用最优解的控制代价作为启发函数。
    cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
    cur_node->node_state = IN_OPEN_SET;
    open_set_.push(cur_node);
    use_node_num_ += 1;

    //bug出现在dynamic为false,导致time_origin_没有赋予数值
    //https://github.com/HKUST-Aerial-Robotics/Fast-Planner/issues/117
    //bug出现在这里!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if (dynamic)
    {
      time_origin_ = time_start;
      cur_node->time = time_start;
      cur_node->time_idx = timeToIndex(time_start);
      expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
      // cout << "time start: " << time_start << endl;
    }    // expanded_nodes_ 内含有一个unordered_map 再调用insert方法
    else // std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      expanded_nodes_.insert(cur_node->index, cur_node);

    PathNodePtr neighbor = NULL;
    PathNodePtr terminate_node = NULL;
    bool init_search = init;
    const int tolerance = ceil(1 / resolution_);

    while (!open_set_.empty())
    {if(iter_num_>10) break;//避免超时
      ztr_log4<<"iter_num_:"<<iter_num_<<endl;
      cur_node = open_set_.top();
      // reach_horizon 与出发点距离超过horizon_
      //  Terminate?
      bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;
      bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                      abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                      abs(cur_node->index(2) - end_index(2)) <= tolerance;

      if (reach_horizon || near_end)
      {
        terminate_node = cur_node;
        retrievePath(terminate_node); //回溯路径
        if (near_end)
        {
          // Check whether shot traj exist
          estimateHeuristic(cur_node->state, end_state, time_to_goal);
          // time_to_goal最优控制时间已经在estimateHeuristic中计算过了，这里只用引入
          //该时间进行多项式计算即可，验证轨迹安全无碰撞，速度加速度都不超限
          computeShotTraj(cur_node->state, end_state, time_to_goal);
          if (init_search)
            ROS_ERROR("Shot in first search loop!");
        }
      }
      if (reach_horizon)
      {
        if (is_shot_succ_) // is_shot_succ_在computeShotTraj中进行了设置
        {
          std::cout << "[kino]: reach end" << std::endl;
          return REACH_END;
        }
        else
        {
          std::cout << "[kino]: reach horizon" << std::endl;
          return REACH_HORIZON;
        }
      }

      if (near_end)
      {
        if (is_shot_succ_)
        {
          std::cout << "[kino]: reach end" << std::endl;
          return REACH_END;
        }
        else if (cur_node->parent != NULL)
        {
          std::cout << "[kino]: near end" << std::endl;
          return NEAR_END;
        }
        else
        {
          std::cout << "[kino]: no path" << std::endl;
          return NO_PATH;
        }
      }
      open_set_.pop();
      cur_node->node_state = IN_CLOSE_SET;
      iter_num_ += 1;

      double res = 1 / reso_acc, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
      Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
      Eigen::Matrix<double, 6, 1> pro_state;
      vector<PathNodePtr> tmp_expand_nodes;
      Eigen::Vector3d um;             //控制空间加速度的数值
      double pro_t;                   //记录离散控制空间，时间作为输入时此时的时间
      vector<Eigen::Vector3d> inputs; //输入量即加速度3d向量 的离散量的集合
      vector<double> durations;       //时刻的容器，装入时间tau 时间段离散量的集合
      // init标志位控制是否是最开始查找，是由search函数里面的init_search接受，其中判断是否是第一次搜索
      //最开始就start_acc作为起始点的加速度输入进行查找，如果找不到，则再进行一轮离散化输入加速度查找
      //若都找不到，则路径寻找失败
      if (init_search) // tau是时间离散量
      {                //如果是初始第一次search
        inputs.push_back(start_acc_);
        for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
             tau += time_res_init * init_max_tau_)
          durations.push_back(tau);
        init_search = false;
      }
      else //控制空间离散，加速度
      {
        for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
          for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
            for (double az = -max_acc_z; az <= max_acc_z + 1e-3; az += max_acc_z * res)
            {
              um << ax, ay, az;
              inputs.push_back(um);
            }
        for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
          durations.push_back(tau); //时间空间离散
      }

      // cout << "cur state:" << cur_state.head(3).transpose() << endl;
      for (int i = 0; i < inputs.size(); ++i)
        for (int j = 0; j < durations.size(); ++j)
        {
          um = inputs[i];
          double tau = durations[j];
          stateTransit(cur_state, pro_state, um, tau); //根据输入加速度um,持续时间tau，得到pro_state
          pro_t = cur_node->time + tau;                //更新此时的时间

          Eigen::Vector3d pro_pos = pro_state.head(3); //新的点的位置xyz
          // ztr_log4<<"--------------in search kino A star---------999-----------"<<endl;
          // Check if in close set
          Eigen::Vector3i pro_id = posToIndex(pro_pos);
          int pro_t_id = timeToIndex(pro_t);

          PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);

          if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
          {
            if (init_search)
              std::cout << "close" << std::endl;
            continue;
          }

          // Check maximal velocity 否则continue进行下一个点检验
          Eigen::Vector3d pro_v = pro_state.tail(3);
          if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_)
          {
            if (init_search)
              std::cout << "vel" << std::endl;
            continue;
          }

          // Check not in the same voxel 否则continue进行下一个点检验
          Eigen::Vector3i diff = pro_id - cur_node->index;
          int diff_time = pro_t_id - cur_node->time_idx;
          if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
          {
            if (init_search)
              std::cout << "same" << std::endl;
            continue;
          }
//////////////这部分好像有问题????检查collision????????????????????????????


          // Check safety
          Eigen::Vector3d pos;
          Eigen::Matrix<double, 6, 1> xt;
          bool is_occ = false;
          for (int k = 1; k <= check_num_; ++k)
          { //检测这个过程当中0-tao细分为check_num_段，检查每段是否有碰撞，有的话设置为is_occ，break停止检测
            double dt = tau * double(k) / double(check_num_);
            stateTransit(cur_state, xt, um, dt);
            pos = xt.head(3);
            
            // ztr_log4<<"第"<<k<<"次"<<"---------------------collision check-------------"<<endl;
            // ztr_log4<<"coord"<<pos.transpose()<<endl<<"coll? "<<grid_map_->getInflateOccupancy(pos)<<endl;
            
            if (grid_map_->getInflateOccupancy(pos) == 1)
            {
              is_occ = true;
              break;
            }
          }
          if (is_occ) //检测碰撞了
          {
            ztr_log4<<"--------------is_occ-----------"<<endl;
            if (init_search)
              std::cout << "safe" << std::endl;
            continue;
          }
          double time_to_goal, tmp_g_score, tmp_f_score; // g考虑了加速度大小um的长度
          tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
          tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);
          //以上代码在当前节点的基础上，根据对输入时间的离散进行拓展得到tmp临时节点，首先判断节点是否已经被拓展过，是否与当前节点在同一节点
          //检查速度约束，检查碰撞，若都没有问题就计算当前节点的g,f。state transit函数即通过向前积分得到拓展节点的速度位置
          //接下来进行节点剪枝
          // Compare nodes expanded from the same parent
          bool prune = false;
          /*         首先判断当前临时扩展节点与current node的其他临时扩展节点是否在同一个voxel中，如果是的话，就要进行剪枝。
                  要判断当前临时扩展节点的fscore是否比同一个voxel的对比fscore小，
                  如果是的话，则更新这一Voxel节点为当前临时扩展节点。 */
          for (int j = 0; j < tmp_expand_nodes.size(); ++j)
          {
            PathNodePtr expand_node = tmp_expand_nodes[j];
            if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
            {
              prune = true;
              if (tmp_f_score < expand_node->f_score)
              {
                expand_node->f_score = tmp_f_score;
                expand_node->g_score = tmp_g_score;
                expand_node->state = pro_state;
                expand_node->input = um;
                expand_node->duration = tau;
                if (dynamic)
                  expand_node->time = cur_node->time + tau;
              }
              break;
            }
          }
          // ztr_log4<<"--------------in search kino A star----------------"<<endl;
          // This node end up in a voxel different from others
          /*         如果不剪枝的话，则首先判断当前临时扩展节点pro_node是否出现在open集中，
                  如果不是的话，则可以直接将pro_node加入open集中。如果存在于open集但还未扩展的话，
                  则比较当前临时扩展节点与对应VOXEL节点的fscore,若更小，则更新voxel中的节点。 */
          if (!prune)
          {
            if (pro_node == NULL)
            {
              pro_node = path_node_pool_[use_node_num_];
              pro_node->index = pro_id;
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              pro_node->node_state = IN_OPEN_SET;
              if (dynamic)
              {
                pro_node->time = cur_node->time + tau;
                pro_node->time_idx = timeToIndex(pro_node->time);
              }
              open_set_.push(pro_node);
              if (dynamic)
                expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
              else
                expanded_nodes_.insert(pro_id, pro_node);

              tmp_expand_nodes.push_back(pro_node);

              use_node_num_ += 1;
              // ztr_log4<<"use_node_num_"<<use_node_num_<<endl;
              if (use_node_num_ == allocate_num_)
              {
                ztr_log4<<"--------------run out of memory. -----NO_PATH--------------"<<endl;
                cout << "run out of memory." << endl;
                return NO_PATH;
              }
            }
            else if (pro_node->node_state == IN_OPEN_SET)
            {
              if (tmp_g_score < pro_node->g_score)
              {
                // pro_node->index = pro_id;
                pro_node->state = pro_state;
                pro_node->f_score = tmp_f_score;
                pro_node->g_score = tmp_g_score;
                pro_node->input = um;
                pro_node->duration = tau;
                pro_node->parent = cur_node;
                if (dynamic)
                  pro_node->time = cur_node->time + tau;
              }
            }
            else
            {
  ztr_log4<<"-------------error type in searching:-----------"<<endl;
              cout << "error type in searching: " << pro_node->node_state << endl;
            }
          }
        }
      // init_search = false;
    }
    /* open集是通过两个数据结构实现的，一个队列用来存储，弹出open集中的节点。
    另一个哈希表NodeHashtable 用来查询节点是否已经存在于open集中。
    而判断一个节点是否存在于close set中，则是通过Nodehashtable 与nodestate来决定的，如果nodeState
    是 InCloseSet, 且存在于NodeHashtable, 则说明该节点已经被扩展过了，存在于close set中。 */
//  ztr_log4<<"--------------in search kino A star--------ii-----------"<<endl;
    // cout << "open set empty, no path!" << endl;
    // cout << "use node num: " << use_node_num_ << endl;
    // cout << "iter num: " << iter_num_ << endl;
    return NO_PATH;
  }

  void KinodynamicAstar::setParam(ros::NodeHandle &nh)
  {
    nh.param("search/max_tau", max_tau_, -1.0);
    nh.param("search/init_max_tau", init_max_tau_, -1.0);
    nh.param("search/max_vel", max_vel_, -1.0);
    nh.param("search/max_acc", max_acc_, -1.0);
    nh.param("search/max_acc_z", max_acc_z, -1.0);
    nh.param("search/w_time", w_time_, -1.0);
    nh.param("search/horizon", horizon_, -1.0);
    nh.param("search/resolution_astar", resolution_, -1.0);
    nh.param("search/time_resolution", time_resolution_, -1.0);
    nh.param("search/lambda_heu", lambda_heu_, -1.0);
    nh.param("search/allocate_num", allocate_num_, -1);
    nh.param("search/check_num", check_num_, -1);
    nh.param("search/optimistic", optimistic_, true);
    nh.param("search/reso_acc", reso_acc, 3.0);
    tie_breaker_ = 1.0 + 1.0 / 10000;

    double vel_margin;
    nh.param("search/vel_margin", vel_margin, 0.0);
    max_vel_ += vel_margin;
  }

  void KinodynamicAstar::retrievePath(PathNodePtr end_node)
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

  /* 首先通过设置启发函数对时间求导等于0，得到启发函数关于时间T的四次方程，再通过求解该四次方程，
  得到一系列实根，通过比较这些实根所对应的cost大小，得到最优时间。这里需要注意，
  关于时间的一元四次方程是通过费拉里方法求解的，需要嵌套一个元三次方程进行求解，
  也就是代码中应的cubic（）函数。

  一元四次方程的求解过程参见wikipedia中的费拉里方法：
  https://zh.wikipedia.org/wiki/%E5%9B%9B%E6%AC%A1%E6%96%B9%E7%A8%8B

  一元三次方程的求见过程参见wikipedia中的求根系数法。需要加以说明的是，代码中的判别式大于0及等于0的情况利用了求根公式，判别式小于0的情况则是使用了三角函数解法：
  https://zh.wikipedia.org/wiki/%E4%B8%89%E6%AC%A1%E6%96%B9%E7%A8%8B
   */
  // estimateHeuristic（）对应文章中的III.B小节，主要原理是利用庞特里亚金原理解决两点边值问题，得到最优解后用最优解的控制代价作为启发函数
  // estimateHeuristic计算返回最优时间optimal_time
  double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time)
  {
    const Vector3d dp = x2.head(3) - x1.head(3);
    const Vector3d v0 = x1.segment(3, 3);
    const Vector3d v1 = x2.segment(3, 3);

    double c1 = -36 * dp.dot(dp);
    double c2 = 24 * (v0 + v1).dot(dp);
    double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
    double c4 = 0;
    double c5 = w_time_;

    std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

    double v_max = max_vel_ * 0.5;
    double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
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

  /* 计算出起始节点的启发项后，就基本进入搜索循环，第一步是判断当前节点是否超出horizon或是离终点较近了，
  并计算一条直达曲线，检查这条曲线上是否存在。若存在，则搜索完成，返回路径点即可。 */
  /* 利用庞特里亚金原理解一个两点边值问题。因为最优控制时间已经在estimateHeuristic中计算过了，
  所以这里只要引入该时间进行多项式计算即可。
  这部分的目的是为了验证该轨迹是安全的，即不发生碰撞，速度、加速度不超限。 */
  bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
  {
    /* ---------- get coefficient ---------- */
    const Vector3d p0 = state1.head(3);
    const Vector3d dp = state2.head(3) - p0;
    const Vector3d v0 = state1.segment(3, 3);
    const Vector3d v1 = state2.segment(3, 3);
    const Vector3d dv = v1 - v0;
    double t_d = time_to_goal;
    MatrixXd coef(3, 4);
    end_vel_ = v1;

    Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
    Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
    Vector3d c = v0;
    Vector3d d = p0;

    // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
    // a*t^3 + b*t^2 + v0*t + p0
    coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

    Vector3d coord, vel, acc;
    VectorXd poly1d, t, polyv, polya;
    Vector3i index;

    Eigen::MatrixXd Tm(4, 4);
    Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

    /* ---------- forward checking of trajectory ---------- */
    double t_delta = t_d / 10;
    for (double time = t_delta; time <= t_d; time += t_delta)
    {
      t = VectorXd::Zero(4);
      for (int j = 0; j < 4; j++)
        t(j) = pow(time, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef.row(dim);
        coord(dim) = poly1d.dot(t);
        vel(dim) = (Tm * poly1d).dot(t);
        acc(dim) = (Tm * Tm * poly1d).dot(t);

        if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
        {
          // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
          // return false;
        }
      }

      if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
          coord(2) < origin_(2) || coord(2) >= map_size_3d_(2))
      {
        return false;
      }

      if (grid_map_->getInflateOccupancy(coord) == 1 ) {
        return false;
      }

    }
    coef_shot_ = coef;
    t_shot_ = t_d;
    is_shot_succ_ = true;
    return true;
  }

  vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
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

  vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e)
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

  void KinodynamicAstar::init(int id)
  {

//否则有bug??????????????????????????????????????
time_origin_=0;
//否则有bug??????????????????????????????????????

    drone_id = id;
    ztr_log4.open("/home/ztr/workspace/swarm/Pipline-Swarm-Formation/log/drone_kino_" + to_string(drone_id) + ".txt");
    /* ---------- map params ---------- */

    this->inv_resolution_ = 1.0 / resolution_;

    inv_time_resolution_ = 1.0 / time_resolution_;

    ztr_log4<<"inv_time_resolution_"<<inv_time_resolution_<<endl;
 
    grid_map_->getRegion(origin_, map_size_3d_);

    // cout << "origin_: " << origin_.transpose() << endl;
    // cout << "map size: " << map_size_3d_.transpose() << endl;
    ztr_log4 << "before pre-allocated node" << endl;
    ztr_log4<<"allocate_num_"<<allocate_num_<<endl;
    /* ---------- pre-allocated node ---------- */
    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++)
    {
      path_node_pool_[i] = new PathNode;
    }
    ztr_log4 << "after pre-allocated node" << endl;
    phi_ = Eigen::MatrixXd::Identity(6, 6); //状态转移矩阵单位阵
    use_node_num_ = 0;
    iter_num_ = 0;
  }

  void KinodynamicAstar::setEnvironment(const GridMap::Ptr &env)
  {
    this->grid_map_ = env;
  }

  void KinodynamicAstar::reset()
  {
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++)
    {
      PathNodePtr node = path_node_pool_[i];
      node->parent = NULL;
      node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
    is_shot_succ_ = false;
    has_path_ = false;
  }
  
  /* getKinoTraj这一函数多作用是在完成路径搜索后按照预设的时间分辨率delta_t通过节点回溯和状态前向积分
  得到分辨率更高的路径点。如果最后的shot trajectory存在的话，则还要加上最后一段shot trajectory
  (即通过computeshottraj)算出来得到的。 */
  std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t)
  {
    vector<Vector3d> state_list;

    /* ---------- get traj of searching ---------- */
    PathNodePtr node = path_nodes_.back();
    Matrix<double, 6, 1> x0, xt;

    while (node->parent != NULL)
    {
      Vector3d ut = node->input; //加速度
      double duration = node->duration;
      x0 = node->parent->state;
      //按照预设的时间分辨率delta_t通过节点回溯和状态前向积分得到分辨率更高的路径点。
      for (double t = duration; t >= -1e-5; t -= delta_t)
      {
        stateTransit(x0, xt, ut, t);
        state_list.push_back(xt.head(3));
      }
      node = node->parent;
    }
    reverse(state_list.begin(), state_list.end());
    /* ---------- get traj of one shot ---------- */
    if (is_shot_succ_)
    {
      Vector3d coord;
      VectorXd poly1d, time(4);

      for (double t = delta_t; t <= t_shot_; t += delta_t)
      {
        for (int j = 0; j < 4; j++)
          time(j) = pow(t, j);

        for (int dim = 0; dim < 3; dim++)
        { // computeShotTraj中计算了one shot的多项式系数
          poly1d = coef_shot_.row(dim);
          coord(dim) = poly1d.dot(time);
        }
        state_list.push_back(coord); //如果最后的shot trajectory存在的话，则还要加上最后一段shot trajectory
                                     //                                (即通过computeshottraj)算出来得到的
      }
    }

    return state_list;
  }

  // getSamples这一函数则是离散获得一些轨迹点以及起始点的速度与加速度。
  void KinodynamicAstar::getSamples(double &ts, vector<Eigen::Vector3d> &point_set,
                                    vector<Eigen::Vector3d> &start_end_derivatives)
  {
    /* ---------- path duration ---------- */
    double T_sum = 0.0;
    if (is_shot_succ_)
      T_sum += t_shot_;
    PathNodePtr node = path_nodes_.back();
    while (node->parent != NULL)
    {
      T_sum += node->duration;
      node = node->parent;
    }
    // cout << "duration:" << T_sum << endl;

    // Calculate boundary vel and acc
    Eigen::Vector3d end_vel, end_acc;
    double t;
    if (is_shot_succ_)
    {
      t = t_shot_;
      end_vel = end_vel_;
      for (int dim = 0; dim < 3; ++dim)
      {
        Vector4d coe = coef_shot_.row(dim);
        end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;
      }
    }
    else
    {
      t = path_nodes_.back()->duration;
      end_vel = node->state.tail(3);
      end_acc = path_nodes_.back()->input;
    }

    // Get point samples
    int seg_num = floor(T_sum / ts);
    seg_num = max(8, seg_num);
    ts = T_sum / double(seg_num);
    bool sample_shot_traj = is_shot_succ_;
    node = path_nodes_.back();

    for (double ti = T_sum; ti > -1e-5; ti -= ts)
    {
      if (sample_shot_traj)
      {
        // samples on shot traj
        Vector3d coord;
        Vector4d poly1d, time;

        for (int j = 0; j < 4; j++)
          time(j) = pow(t, j);

        for (int dim = 0; dim < 3; dim++)
        {
          poly1d = coef_shot_.row(dim);
          coord(dim) = poly1d.dot(time);
        }

        point_set.push_back(coord);
        t -= ts;

        /* end of segment */
        if (t < -1e-5)
        {
          sample_shot_traj = false;
          if (node->parent != NULL)
            t += node->duration;
        }
      }
      else
      {
        // samples on searched traj
        Eigen::Matrix<double, 6, 1> x0 = node->parent->state;
        Eigen::Matrix<double, 6, 1> xt;
        Vector3d ut = node->input;

        stateTransit(x0, xt, ut, t);

        point_set.push_back(xt.head(3));
        t -= ts;

        // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
        if (t < -1e-5 && node->parent->parent != NULL)
        {
          node = node->parent;
          t += node->duration;
        }
      }
    }
    reverse(point_set.begin(), point_set.end());

    // calculate start acc
    Eigen::Vector3d start_acc;
    if (path_nodes_.back()->parent == NULL)
    {
      // no searched traj, calculate by shot traj
      start_acc = 2 * coef_shot_.col(2);
    }
    else
    {
      // input of searched traj
      start_acc = node->input;
    }

    start_end_derivatives.push_back(start_vel_);
    start_end_derivatives.push_back(end_vel);
    start_end_derivatives.push_back(start_acc);
    start_end_derivatives.push_back(end_acc);
  }

  std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
  {
    vector<PathNodePtr> visited;
    visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
    return visited;
  }

  Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
  {
    Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

    // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
    // origin_(1)) * inv_resolution_),
    //     floor((pt(2) - origin_(2)) * inv_resolution_);

    return idx;
  }

  int KinodynamicAstar::timeToIndex(double time)
  {
    int idx = floor((time - time_origin_) * inv_time_resolution_);
    return idx;
  }
  //前向积分得到的拓展节点的位置与速度  um是加速度输入，tau是离散的时间段（持续时间）
  void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1> &state0, Eigen::Matrix<double, 6, 1> &state1,
                                      Eigen::Vector3d um, double tau)
  {
    for (int i = 0; i < 3; ++i)
      phi_(i, i + 3) = tau;
    // tau在初始化时为  init中为phi_ = Eigen::MatrixXd::Identity(6, 6);
    // Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
    Eigen::Matrix<double, 6, 1> integral;
    integral.head(3) = 0.5 * pow(tau, 2) * um; // 1/2*a*t^2，位置增加二分之一a t方
    integral.tail(3) = tau * um;               //速度增加 at

    state1 = phi_ * state0 + integral;
  }

  //矩阵phi_ 实现x=x+vx*tau y=y+vy*tau z=z+vz*tau 然后在控制Input加速度上还有
  // x,y,z位置增加二分之一a t方，a为新的输入    速度增加 at
//  1  0  0  tau  0   0
//  0  1  0   0  tau  0
//  0  0  1   0   0  tau
//  0  0  0   1   0   0
//  0  0  0   0   1   0
//  0  0  0   0   0   1
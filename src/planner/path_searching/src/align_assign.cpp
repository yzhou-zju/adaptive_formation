#include "path_searching/align_assign.hpp"

// Update the pseudo cost mat for assignment
void AlignAssign::updateALASCostMat(const std::vector<Eigen::Vector3d> &nodes_horizon,
                                    const std::vector<Eigen::Vector3d> &nodes_des)
{
    cost_ALAS_mat_ = Eigen::MatrixXd::Zero(swarm_size_, swarm_size_);
    for (int i = 0; i < swarm_size_; i++)
    {
        for (int j = 0; j < swarm_size_; j++)
        {
            cost_ALAS_mat_(i, j) = -nodes_horizon[i].dot(nodes_des[j]);
        }
    }
    // std::cout << "cost_mat: " << std::endl;
    // std::cout << cost_ALAS_mat_ << std::endl;
}

void AlignAssign::updateRACostMat(const std::vector<Eigen::Vector3d> &nodes_cur,
                                  const std::vector<Eigen::Vector3d> &ALAS_goals)
{
    cost_RA_mat_ = Eigen::MatrixXd::Zero(swarm_size_, swarm_size_);
    for (int i = 0; i < swarm_size_; i++)
    {
        for (int j = 0; j < swarm_size_; j++)
        {
            cost_RA_mat_(i, j) = calcDist2(nodes_cur[i], ALAS_goals[j]);
        }
    }
}

void AlignAssign::getOptGoals(const std::vector<Eigen::Vector3d> &nodes_horizon,
                              const std::vector<Eigen::Vector3d> &nodes_cur,
                              const std::vector<Eigen::Vector3d> &nodes_des,
                              const Eigen::VectorXd &aware_weight,
                              std::vector<Eigen::Vector3d> &opt_goals,
                              Eigen::VectorXi &opt_assignment,
                              bool with_ALAS_assign, bool with_RA_assign)
{
    swarm_size_ = nodes_des.size();
    std::vector<Eigen::Vector3d> ALAS_goals(swarm_size_);
    Eigen::VectorXi ALAS_assignment(swarm_size_);

    // Conduct the ALAS phase
    getALASGoals(nodes_horizon, nodes_des, aware_weight, ALAS_goals, ALAS_assignment, with_ALAS_assign);

    if (with_RA_assign)
    {
        updateRACostMat(nodes_cur, ALAS_goals);
        // Get the optimal assignment for RA phase
        assigner_.run_munkres_algorithm(cost_RA_mat_);
        assign_RA_munkres_ = assigner_.output();
    }
    else
    {
        assign_RA_munkres_ = Eigen::VectorXi::LinSpaced(swarm_size_, 0, swarm_size_ - 1);
    }
    opt_assignment = assign_RA_munkres_;
    reArrangeGoals(ALAS_goals, opt_goals);
}
void AlignAssign::getOptassig(const std::vector<Eigen::Vector3d> &nodes_cur,
                              const std::vector<Eigen::Vector3d> &nodes_des,
                              Eigen::VectorXi &opt_assignment)
{
    swarm_size_ = nodes_des.size();

    updateRACostMat(nodes_cur, nodes_des);
    // Get the optimal assignment for RA phase
    assigner_.run_munkres_algorithm(cost_RA_mat_);
    assign_RA_munkres_ = assigner_.output();

    opt_assignment = assign_RA_munkres_;
    // reArrangeGoals(ALAS_goals, opt_goals);
}
void AlignAssign::getALASGoals(const std::vector<Eigen::Vector3d> &nodes_horizon,
                               const std::vector<Eigen::Vector3d> &nodes_des,
                               const Eigen::VectorXd &aware_weight,
                               std::vector<Eigen::Vector3d> &ALAS_goals,
                               Eigen::VectorXi &ALAS_assignment, bool with_assign)
{
    if (with_assign)
    {
        updateALASCostMat(nodes_horizon, nodes_des);
        // Generate best assignment for ALAS phase
        assigner_.run_munkres_algorithm(cost_ALAS_mat_);
        assign_ALAS_munkres_ = assigner_.output();
    }
    else
    {
        assign_ALAS_munkres_ = Eigen::VectorXi::LinSpaced(swarm_size_, 0, swarm_size_ - 1);
    }
    // Restore the optimal assignment
    ALAS_assignment = assign_ALAS_munkres_;

    // Rearrange nodes_des into nodes_assign according to assign_opt
    std::vector<Eigen::Vector3d> nodes_assign(swarm_size_);
    reArrangeDes(nodes_des, nodes_assign);

    // Softmax the aware weights
    softmaxLayer(aware_weight, soft_weight_);

    // Calculate the optimal weighted alignment
    double scale_opt;
    Eigen::Vector3d translation_opt;
    calcOptAlign(nodes_horizon, nodes_assign, scale_opt, translation_opt);

    // Get transformed goals with optimal scale and translation
    // Note this step we transform the initial nodes_des for RA phase
    transformGoals(ALAS_goals, nodes_des, scale_opt, translation_opt);
}

void AlignAssign::reArrangeDes(const std::vector<Eigen::Vector3d> &nodes_des,
                               std::vector<Eigen::Vector3d> &nodes_assign)
{
    for (int i = 0; i < swarm_size_; i++)
        nodes_assign[i] = nodes_des[assign_ALAS_munkres_(i)];
}

void AlignAssign::reArrangeGoals(const std::vector<Eigen::Vector3d> &ALAS_goals,
                                 std::vector<Eigen::Vector3d> &opt_goals)
{
    opt_goals.clear();
    for (int i = 0; i < swarm_size_; i++)
        opt_goals.push_back(ALAS_goals[assign_RA_munkres_(i)]);
}

void AlignAssign::calcOptAlign(const std::vector<Eigen::Vector3d> &nodes_horizon,
                               const std::vector<Eigen::Vector3d> &nodes_assign,
                               double &scale, Eigen::Vector3d &translation)
{
    // init params: weighted sum of p, q, pTq, qTq.
    double b_q = 0, b_pq = 0;
    Eigen::Vector3d q_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_sum = Eigen::Vector3d::Zero();

    // Update params
    double w = 0;
    for (int i = 0; i < swarm_size_; i++)
    {
        w = soft_weight_(i);
        q_sum += w * nodes_assign[i];
        p_sum += w * nodes_horizon[i];
        b_q += w * nodes_assign[i].dot(nodes_assign[i]);
        b_pq += w * nodes_horizon[i].dot(nodes_assign[i]);
    }

    scale = (b_pq - p_sum.dot(q_sum)) / (b_q - q_sum.dot(q_sum));
    translation = (p_sum - scale * q_sum);

    // std::cout << "Optimal scale: " << scale << std::endl;
    // std::cout << "Optimal translation: " << std::endl;
    // std::cout << translation << std::endl;
}

void AlignAssign::transformGoals(std::vector<Eigen::Vector3d> &nodes_goal, const std::vector<Eigen::Vector3d> &nodes_assign,
                                 double &scale, Eigen::Vector3d &translation)
{
    nodes_goal.clear();
    for (auto node : nodes_assign)
    {
        nodes_goal.push_back(scale * node + translation);
    }
}
// 尚未仔细查看
void AlignAssign::softmaxLayer(const Eigen::VectorXd &aware_weight, Eigen::VectorXd &soft_weight)
{
    soft_weight = aware_weight.array().exp();
    double exp_aware_sum = soft_weight.sum();
    soft_weight /= exp_aware_sum;
}

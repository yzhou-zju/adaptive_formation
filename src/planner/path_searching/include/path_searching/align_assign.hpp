#ifndef ALIGN_ASSIGN_HPP
#define ALIGN_ASSIGN_HPP

#include <algorithm>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <cfloat>
#include "path_searching/munkres_algorithm.hpp"
// #include <frob_test/swarm_graph.hpp>

class AlignAssign
{

private:
    Munkres assigner_; // Hungarian

    int swarm_size_;                      // Also the dim of c_mat
    Eigen::MatrixXd cost_ALAS_mat_;       // ALAS Cost matrix for Hungarian
    Eigen::MatrixXd cost_RA_mat_;         // ReAssignment Cost matrix
    Eigen::VectorXd soft_weight_;         // Weights after softmax
    Eigen::VectorXi assign_ALAS_munkres_; // Optimal ALAS assignment from Munkres
    Eigen::VectorXi assign_RA_munkres_;   // Optimal ReAssignment from Munkres
    // assignment[ drone_id ] indicates the optimal position index
    // in desired formation for the drone_id one.

    // Renew the cost matrix for ALAS phase
    void updateALASCostMat(const std::vector<Eigen::Vector3d> &nodes_horizon,
                           const std::vector<Eigen::Vector3d> &nodes_des);

    // Renew the cost matrix for the ReAssignment phase
    void updateRACostMat(const std::vector<Eigen::Vector3d> &nodes_cur,
                         const std::vector<Eigen::Vector3d> &ALAS_goals);

    // Rearrange the order of the nodes according to optimal ALAS assignment
    void reArrangeDes(const std::vector<Eigen::Vector3d> &nodes_des,
                      std::vector<Eigen::Vector3d> &nodes_assign);

    // Rearrange the order of the nodes according to the optimal ReAssignment
    void reArrangeGoals(const std::vector<Eigen::Vector3d> &ALAS_goals,
                        std::vector<Eigen::Vector3d> &opt_goals);

    // Calculate the close-form alignment solution
    void calcOptAlign(const std::vector<Eigen::Vector3d> &nodes_cur,
                      const std::vector<Eigen::Vector3d> &nodes_assign,
                      double &scale, Eigen::Vector3d &translation);

    // Transform the nodes with given scale and translation
    void transformGoals(std::vector<Eigen::Vector3d> &nodes_goal,
                        const std::vector<Eigen::Vector3d> &nodes_assign,
                        double &scale, Eigen::Vector3d &translation);

    inline double calcDist2(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
    {
        return (v1 - v2).cwiseAbs2().sum();
    }

public:
    AlignAssign() {}
    ~AlignAssign() {}

    // Main API
    void getALASGoals(const std::vector<Eigen::Vector3d> &nodes_horizon,
                      const std::vector<Eigen::Vector3d> &nodes_des,
                      const Eigen::VectorXd &aware_weight,
                      std::vector<Eigen::Vector3d> &ALAS_goals,
                      Eigen::VectorXi &ALAS_assignment, bool with_assign);

    void getOptGoals(const std::vector<Eigen::Vector3d> &nodes_horizon,
                     const std::vector<Eigen::Vector3d> &nodes_cur,
                     const std::vector<Eigen::Vector3d> &nodes_des,
                     const Eigen::VectorXd &aware_weight,
                     std::vector<Eigen::Vector3d> &opt_goals,
                     Eigen::VectorXi &opt_assignment,
                     bool with_ALAS_assign, bool with_RA_assign);
    void getOptassig(const std::vector<Eigen::Vector3d> &nodes_cur,
                     const std::vector<Eigen::Vector3d> &nodes_des,
                     Eigen::VectorXi &opt_assignment);
        // Softmax
        void softmaxLayer(const Eigen::VectorXd &aware_weight, Eigen::VectorXd &soft_weight);
};

#endif
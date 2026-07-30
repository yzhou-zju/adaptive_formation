#ifndef _SWARM_GRAPH_H_
#define _SWARM_GRAPH_H_

#include <memory>
#include <Eigen/Eigen>
#include <ros/ros.h>


class SwarmGraph{
    
private:
    std::vector<Eigen::Vector3d> nodes_;  //Contains the pos of the vertices in swarm_graph
    std::vector<Eigen::Vector3d> nodes_des_;  //Desired graph (permutation could change after assignment)
    std::vector<Eigen::Vector3d> nodes_des_init_; //Initial desired graph (never change once set)

    std::vector<Eigen::Vector3d> agent_grad_; //Contains the gradp of the swarm_graph
    std::vector<Eigen::Vector3d> agent_grad_org_; //gradp before normalization

    std::vector<int> adj_in_;  // The head nodes of the edges in graph
    std::vector<int> adj_out_; // The tail nodes of the edges in graph
    std::vector< std::vector<int> > adj_arr_; // Arrays indicating the adjacency relations

    bool have_desired_;

    Eigen::MatrixXd A_;   //Adjacency matrix
    Eigen::VectorXd D_;   //Degree matrix
    Eigen::MatrixXd Lhat_; //Symmetric Normed Laplacian
    Eigen::MatrixXd A_des_;   //Desired adjacency matrix 
    Eigen::VectorXd D_des_;   //Desired degree matrix
    Eigen::MatrixXd Lhat_des_;  //Desired SNL 
    Eigen::MatrixXd DLhat_;     //Difference of SNL---->DLhat = Lhat - Lhat_des;
    Eigen::VectorXi last_assignment_; //Last optimal assignment

    //Calculate the graph feature matrices
    bool calcMatrices( const std::vector<Eigen::Vector3d> &swarm,
                        const std::vector<int> &adj_in,
                        const std::vector<int> &adj_out,
                        Eigen::MatrixXd &Adj, Eigen::VectorXd &Deg,
                        Eigen::MatrixXd &SNL );

    //E-norm distance
    double calcDist2( const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);                   

    //Calculate the gradients over positions
    bool calcFGrad( Eigen::Vector3d &gradp, Eigen::Vector3d &gradp_org, int query_idx );

public:

    SwarmGraph(); 
    ~SwarmGraph(){}

    //Update the nodes, feature matrices & desired matrices
    inline void setnewform(){have_desired_ = false; adj_arr_.clear();};
    bool updateGraph( const std::vector<Eigen::Vector3d> &swarm );
    bool updatePartGraphAndGetGrad( const int idx, const std::vector<Eigen::Vector3d> &swarm, 
                                    Eigen::Vector3d &gradp);

    //Set desired swarm nodes
    bool setDesiredForm( const std::vector<Eigen::Vector3d> &swarm_des,
                          const std::vector<int> &adj_in,
                          const std::vector<int> &adj_out );

    //Calculate the squared F-Normed difference of SNL matrix
    bool calcFNorm2( double &cost );

    //Get the id_th gradient over position
    Eigen::Vector3d getGrad(int id);
    Eigen::Vector3d getGradOrg(int id);
    bool getGrad(std::vector<Eigen::Vector3d> &swarm_grad);

    //Helper functions for assignment tasks
    std::vector<Eigen::Vector3d> getDesNodesInit() { return nodes_des_init_; }
    std::vector<Eigen::Vector3d> getDesNodesCur() { return nodes_des_; }

    //Update the swarm graph with new assignment
    void setAssignment( const Eigen::VectorXi &assignment );

    /* --- When use aware-matrix for adaptive metric --- */
    // double calcAware( const Eigen::Vector3d &obs_grad, const Eigen::Vector3d &form_grad, const double obs_dist);
    // void softmaxLayer( const Eigen::VectorXd &adapt_aware, Eigen::VectorXd &soft_aware );
    // void updateWeightMat( const Eigen::VectorXd &adapt_aware );

    typedef std::unique_ptr<SwarmGraph> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

};

#endif
#include <frob_test/swarm_graph.hpp>
   
SwarmGraph::SwarmGraph(){   
    have_desired_ = false;
}

bool SwarmGraph::updateGraph( const std::vector<Eigen::Vector3d> &swarm )
{
    nodes_ = swarm;
    if(nodes_.size() != nodes_des_.size()){
        std::cout <<"swarm size : " << nodes_.size() << std::endl;
        ROS_WARN("Size of swarm formation vector is incorrect. ");
        return false;
    }

    if( have_desired_ ){
        calcMatrices(nodes_, adj_in_, adj_out_, A_, D_, Lhat_);
        DLhat_ = Lhat_ - Lhat_des_;

        //Update the gradient vector
        Eigen::Vector3d gradp, gradp_org;
        agent_grad_.clear();
        agent_grad_org_.clear();
        for( int idx = 0; idx < nodes_.size(); idx++ ){
            calcFGrad(gradp, gradp_org, idx);
            agent_grad_.push_back( gradp );
            agent_grad_org_.push_back( gradp_org );
        }
    }else{
        ROS_WARN( "Please enter the desired formation!!" );
    }
}

bool SwarmGraph::updatePartGraphAndGetGrad( const int idx, const std::vector<Eigen::Vector3d> &swarm, 
                                            Eigen::Vector3d &gradp)
{
    // need to change: only update information of local uav
    nodes_ = swarm;
    if(nodes_.size() != nodes_des_.size()){
        std::cout <<"swarm size2 : " << nodes_.size() << std::endl;
        ROS_WARN("Size of swarm formation vector is incorrect. ");
        return false;
    }
    
    if( have_desired_ ){
        calcMatrices(nodes_, adj_in_, adj_out_, A_, D_, Lhat_);
        DLhat_ = Lhat_ - Lhat_des_;
        
        //Only update the gradient vector of local uav 
        Eigen::Vector3d gradp_org;
        calcFGrad(gradp, gradp_org, idx);
    }else{
        ROS_WARN( "Please enter the desired formation!!" );
    }
}

bool SwarmGraph::setDesiredForm( const std::vector<Eigen::Vector3d> &swarm_des,
                                  const std::vector<int> &adj_in,
                                  const std::vector<int> &adj_out )
{
    //Save the initial desired nodes
    if( !have_desired_ ){
        nodes_des_init_ = swarm_des;
        last_assignment_ = Eigen::VectorXi::LinSpaced( swarm_des.size(), 0, swarm_des.size() - 1 );
    }

    nodes_des_ = swarm_des;
    have_desired_ = true;

    //Update the adjacency relations
    adj_in_ = adj_in;
    adj_out_ = adj_out;
    adj_arr_.resize( swarm_des.size() );
    for( int i = 0; i < adj_in_.size(); i++ )
    {   
        adj_arr_[adj_in_[i]].push_back( adj_out_[i] );
        adj_arr_[adj_out_[i]].push_back( adj_in_[i] ); }
    
    //Update the desired matrix
    calcMatrices(nodes_des_, adj_in_, adj_out_, A_des_, D_des_, Lhat_des_);
    return have_desired_;
}

//根据3*n位置向量计算Adj,Deg，SNL分别存入
bool SwarmGraph::calcMatrices( const std::vector<Eigen::Vector3d> &swarm,
                                const std::vector<int> &adj_in,
                                const std::vector<int> &adj_out,
                                Eigen::MatrixXd &Adj, Eigen::VectorXd &Deg,
                                Eigen::MatrixXd &SNL )
{
    //Init the matrices
    Adj = Eigen::MatrixXd::Zero( swarm.size(), swarm.size() );//n*n
    Deg = Eigen::VectorXd::Zero( swarm.size() );
    SNL = Eigen::MatrixXd::Identity( swarm.size(), swarm.size() );

    int id_1, id_2;
    double dist;
    // Update the Adj and Deg 
    for( int i = 0; i < adj_in.size(); i++ ){
        id_1 = adj_in[i];
        id_2 = adj_out[i];

        dist = calcDist2( swarm[id_1], swarm[id_2] );
        Adj(id_1,id_2) = Adj(id_2,id_1) = dist;
        Deg(id_1) += dist;
        Deg(id_2) += dist;
    }

    // Update the SNL matrix
    for( int i = 0; i < adj_in.size(); i++ ){
        id_1 = adj_in[i];
        id_2 = adj_out[i];
        SNL(id_1,id_2) = SNL(id_2,id_1) = - Adj(id_1,id_2) / (sqrt(Deg(id_1)) * sqrt(Deg(id_2)));
    }
    return true;
}

double SwarmGraph::calcDist2( const Eigen::Vector3d &v1, const Eigen::Vector3d &v2){
    return (v1-v2).cwiseAbs2().sum(); 
}


bool SwarmGraph::calcFNorm2( double &cost ){
    //Check if have the desired formation
    if( have_desired_ ){
        cost = DLhat_.cwiseAbs2().sum();
        return true;
    }else{
        ROS_WARN( "Invalid desired formation." );
        return false;
    }
}

bool SwarmGraph::calcFGrad( Eigen::Vector3d &gradp, Eigen::Vector3d &gradp_org, int query_idx )
{
    if( have_desired_ ){

        int N = adj_arr_[query_idx].size();
        Eigen::VectorXd dfde = Eigen::VectorXd::Zero(N);
        Eigen::MatrixXd dedp(N, 3);

        int iter = 0;
        double b0 = 0;

        for(int idx_i: adj_arr_[query_idx])
            b0 += A_(query_idx,idx_i) * DLhat_(query_idx, idx_i) / sqrt(D_(idx_i));
        b0 = 2 * pow(D_(query_idx), -1.5) * b0;

        for(int idx_j: adj_arr_[query_idx]){
            for(int idx_k: adj_arr_[idx_j])
                dfde(iter) += A_(idx_j,idx_k) * DLhat_(idx_j,idx_k) / sqrt(D_(idx_k));
            dfde(iter) = 2 * pow(D_(idx_j), -1.5) * dfde(iter) + b0 + 4 * ( -1/(sqrt(D_(idx_j))*sqrt(D_(query_idx)))) * DLhat_(idx_j, query_idx);
            iter++;
        }
        
        for(int i = 0; i < N; i++)
            for( int j = 0; j < 3; j++ )
                dedp(i, j) = 2 * (nodes_[query_idx](j) - nodes_[adj_arr_[query_idx][i]](j));
        
        //Ignore the machine epsilon
        gradp_org = dfde.transpose() * dedp;
        if(gradp_org.norm() > 1e-7){
            // gradp = gradp_org.normalized();
            gradp = gradp_org;
        }else{
            gradp_org = Eigen::VectorXd::Zero(3);
            gradp = gradp_org;
        }
    }else{
        ROS_WARN( "Invalid desired formation." );
        return false;
    }
}


Eigen::Vector3d SwarmGraph::getGrad(int id){
    if (have_desired_ && id<nodes_des_.size()){
        return agent_grad_[id];
    } else {
        Eigen::Vector3d grad = Eigen::Vector3d::Zero();
        ROS_WARN("id is error !!! or desired graph has not been setup !!!");
        return grad;
    }
}

bool SwarmGraph::getGrad(std::vector<Eigen::Vector3d> &swarm_grad){
    if (have_desired_){
        swarm_grad = agent_grad_;
        return true;
    }else{
        ROS_WARN("desired graph has not been setup !!!");
        return false;
    }
} 

Eigen::Vector3d SwarmGraph::getGradOrg(int id){
    if (have_desired_ && id<nodes_des_.size()){
        return agent_grad_org_[id];
    } else {
        Eigen::Vector3d grad = Eigen::Vector3d::Zero();
        ROS_WARN("id is error !!! or desired graph has not been setup !!!");
        return grad;
    }
}

void SwarmGraph::setAssignment( const Eigen::VectorXi &assignment )
{
    if( assignment.isApprox(last_assignment_) ){
        return;
    }else{
        nodes_des_.clear();
        Eigen::VectorXi assignment_inv(assignment.size());
        
        for( int i = 0; i < assignment.size(); i++ ){
            nodes_des_.push_back( nodes_des_init_[ assignment(i) ] );
            adj_arr_[i].clear();
            assignment_inv(assignment(i)) = i;
        }
        
        // Update adjacency relations in Graph
        for( int i = 0; i < adj_in_.size(); i++ ){
            adj_in_[i] = assignment_inv(adj_in_[i]);
            adj_out_[i] = assignment_inv(adj_out_[i]);
        }
        for( int i = 0; i < adj_in_.size(); i++ )
        {   adj_arr_[adj_in_[i]].push_back( adj_out_[i] );
            adj_arr_[adj_out_[i]].push_back( adj_in_[i] ); }

        calcMatrices( nodes_des_, adj_in_, adj_out_, A_des_, D_des_, Lhat_des_ );
        last_assignment_ = assignment;
        return;
    }
}

/* --- When use aware-matrix for adaptive metric --- */

// void SwarmGraph::softmaxLayer( const Eigen::VectorXd &adapt_aware, Eigen::VectorXd &soft_aware ){
//     soft_aware = adapt_aware.array().exp();
//     double exp_aware_sum = soft_aware.sum();
//     soft_aware /= exp_aware_sum;
// }

// void SwarmGraph::updateWeightMat( const Eigen::VectorXd &adapt_aware ){
    
//     Eigen::VectorXd soft_aware( adapt_aware.size() );
//     softmaxLayer( adapt_aware, soft_aware );
    
//     weight_mat_ = Eigen::MatrixXd::Zero( soft_aware.size(), soft_aware.size() );
//     for (int i = 0; i < soft_aware.size(); i++){
//         for(int j = 0; j < i; j++){
//             weight_mat_(i,j) = weight_mat_(j,i) = soft_aware(i) * soft_aware(j);
//         }
//     } 
// }

// double SwarmGraph::calcAware(const Eigen::Vector3d &obs_grad, const Eigen::Vector3d &form_grad, const double obs_dist)
// {
//     double cosbeta = 0, alpha = 5.0, gamma = -1.0, lambda = 10.0;

//     cosbeta = (form_grad.dot(obs_grad)) / (form_grad.norm() * obs_grad.norm());
//     double aware = lambda * form_grad.norm() / ( 1 + exp( alpha * cosbeta + gamma ) ) / obs_dist;
//     return aware;
// }

#include "brnelib/brne.hpp"

namespace brne
{
  BRNE::BRNE(double kernel_a1, double kernel_a2,
             double cost_a1, double cost_a2, double cost_a3,
             double dt, int n_steps, int n_samples,
             double max_ang_vel, double max_lin_vel,
             double y_min, double y_max):
    kernel_a1{kernel_a1},
    kernel_a2{kernel_a2},
    cost_a1{cost_a1},
    cost_a2{cost_a2},
    cost_a3{cost_a3},
    dt{dt},
    n_steps{n_steps},
    n_samples{n_samples},
    max_ang_vel{max_ang_vel},
    max_lin_vel{max_lin_vel},
    y_min{y_min},
    y_max{y_max}
    {}
  void BRNE::print_params(){
    std::cout << "kernel a1:\t" << kernel_a1 << std::endl;
    std::cout << "kernel a2:\t" << kernel_a2 << std::endl;
    std::cout << "cost a1:\t" << cost_a1 << std::endl;
    std::cout << "cost a2:\t" << cost_a2 << std::endl;
    std::cout << "cost a3:\t" << cost_a3 << std::endl;
    std::cout << "dt:\t\t" << dt << std::endl;
    std::cout << "n steps:\t" << n_steps << std::endl;
    std::cout << "n samples:\t" << n_samples << std::endl;
    std::cout << "max ang vel:\t" << max_ang_vel << std::endl;
    std::cout << "max lin vel:\t" << dt << std::endl;
    std::cout << "y min:\t\t" << dt << std::endl;
    std::cout << "y max:\t\t" << dt << std::endl;
  }

  arma::mat BRNE::compute_kernel_mat(arma::vec t1, arma::vec t2){
    arma::mat res(t1.size(), t2.size(), arma::fill::zeros);
    for (auto i=0; i<t1.size(); i++){
      for (auto j=0; j<t2.size(); j++){
        res.at(i,j) = kernel_a2 * exp(-kernel_a1 * pow((t1.at(i) - t2.at(j)),2));
      }
    }
    return res;
  }

  void BRNE::compute_Lmat(){
    cov_mat.reset();
    cov_Lmat.reset();
    arma::vec tlist = arma::linspace<arma::vec>(0, (n_steps-1)*dt, n_steps);
    std::cout << "Tlist\n" << tlist << std::endl;
    arma::vec train_ts(1, arma::fill::value(tlist.at(0)));
    std::cout << "train ts\n" << train_ts << std::endl;
    arma::vec train_noise(1, arma::fill::value(1e-3));
    std::cout << "Train noise\n" << train_noise << std::endl;

    auto cm_11 = compute_kernel_mat(train_ts, train_ts);
    cm_11 += train_noise.diag();
    auto cm_12 = compute_kernel_mat(tlist, train_ts);
    auto cm_22 = compute_kernel_mat(tlist, tlist);
    cov_mat = cm_22 - cm_12 * cm_11.i() * cm_12.t();
    // std::cout << "Cov mat\n" << cov_mat << std::endl;
    auto success = arma::chol(cov_Lmat, cov_mat, "lower");
    std::cout << "Cholesky success? " << success << std::endl;
    while (!success){
      cov_mat += arma::eye(cov_mat.n_rows,cov_mat.n_rows) * 1e-6;
      success = arma::chol(cov_Lmat, cov_mat, "lower");
      std::cout << "Cholesky success? " << success << std::endl;
    }
    // std::cout << "Cov Lmat\n" << cov_Lmat << std::endl;
  }

  arma::mat BRNE::mvn_sample_normal(){
    // TODO should do error checking to make sure Lmat is set
    arma::mat res(n_steps, n_samples, arma::fill::randn);
    // std::cout << "Random sample\n" << res << std::endl;
    return (cov_Lmat * res).t();
  }
  
  void BRNE::compute_index_table(){
    index_table.reset();
    arma::mat table(n_agents, n_agents, arma::fill::zeros);
    for (int i=0; i<n_agents; i++){
      table.at(i,0) = i;
      auto idx = 1;
      for (int j=0; j<n_agents; j++){
        if (i != j){
          table.at(i,idx) = j;
          idx++;
        }
      }
    }
    index_table = table;
  }

  void BRNE::compute_costs(arma::mat xtraj, arma::mat ytraj){
    costs.reset();
    auto size = n_agents*n_samples;
    arma::mat new_costs(size, size, arma::fill::zeros);
    for (auto i=0; i<size; i++){
      for (auto j=0; j<size; j++){
        arma::vec traj_costs(n_steps, arma::fill::zeros);
        for (auto t=0; t<n_steps; t++){
          auto dst = pow(xtraj.at(i,t) - xtraj.at(j,t), 2) + pow(ytraj.at(i,t) - ytraj.at(j,t), 2);
          // std::cout << "dist: "<< dst << std::endl;
          traj_costs.at(t) = 2.0 - 2.0/(1.0 + exp(-cost_a1 *pow(dst, cost_a2)));
        }
        new_costs.at(i,j) = traj_costs.max() * cost_a3;
      }
    }
    costs = new_costs;
  }

  void BRNE::collision_check(arma::mat ytraj){
    // only keep where y_min < y_traj < y_max
    coll_mask.reset();
    arma::vec valid(ytraj.n_rows);
    for (int r=0; r<ytraj.n_rows; r++){
      int is_valid = 1;
      for (int c=0; c<ytraj.n_cols; c++){
        if ((ytraj.at(r,c) < y_min) || (ytraj.at(r,c) > y_max)){
          is_valid = 0;
          break;
        }
      }
      valid.at(r) = is_valid;
    }
    coll_mask = arma::conv_to<arma::mat>::from(valid);
    coll_mask.reshape(n_samples, n_agents);
    coll_mask = coll_mask.t();
  }

  void BRNE::update_weights(){
    for (int i=0; i<n_agents; i++){
      auto row = arma::conv_to<arma::rowvec>::from(index_table.row(i));
      for (int j=0; j<n_samples; j++){
        auto c = 0.0;
        auto idx1 = all_pts.at(row.at(0), j);
        for (int k=0; k<n_agents-1; k++){
          for (int l=0; l<n_samples; l++){
            auto idx2 = all_pts.at(row.at(k+1), l);
            c += costs.at(idx1, idx2) * weights.at(row.at(k+1), l);
          }
        }
        c /= ((n_agents - 1) * n_samples);
        weights.at(i,j) = exp(-1.0 * c);
      }
      weights.row(i) /= arma::mean(weights.row(i));
    }
  }

  arma::mat BRNE::brne_nav(arma::mat xtraj_samples, arma::mat ytraj_samples){
    weights.reset();
    all_pts.reset();
    // compute number of agents
    n_agents = xtraj_samples.n_rows / n_samples;
    std::cout << "N agents: " << n_agents << std::endl;
    // compute all points index
    all_pts = arma::conv_to<arma::mat>::from(arma::linspace<arma::rowvec>(0, n_agents*n_samples-1, n_agents*n_samples));
    all_pts.reshape(n_samples, n_agents);
    all_pts = all_pts.t();
    // TODO I could cache this for different numbers of agents
    compute_index_table();
    // get the costs
    compute_costs(xtraj_samples, ytraj_samples);
    weights = arma::mat(n_agents, n_samples, arma::fill::ones);
    for (int i=0; i<10; i++){
      std::cout << "weights update #" << i << std::endl;
      std::cout << "weights\n" << weights << std::endl;
      update_weights();
    }
    std::cout << "Final weights\n" << weights << std::endl;

    collision_check(ytraj_samples);
    // TODO: add a check to make sure that at least some of these are true for agent.

    for (int a=0; a<n_agents; a++){
      // element wise multiplication
      auto agent_weights = arma::conv_to<arma::rowvec>::from(weights.row(a));
      auto agent_mask = arma::conv_to<arma::rowvec>::from(coll_mask.row(a));
      auto masked_weights = agent_weights % agent_mask;
      weights.row(a) = masked_weights / arma::mean(masked_weights);
    }

    std::cout << "Weights after masking\n" << weights << std::endl;
    return weights;
  }
}
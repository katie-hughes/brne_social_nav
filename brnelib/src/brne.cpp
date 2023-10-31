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
  
  arma::mat BRNE::compute_index_table(){
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
    return table;
  }

  arma::mat BRNE::compute_costs(arma::mat xtraj, arma::mat ytraj){
    auto size = n_agents*n_samples;
    arma::mat costs(size, size, arma::fill::zeros);
    for (auto i=0; i<size; i++){
      for (auto j=0; j<size; j++){
        arma::vec traj_costs(n_steps, arma::fill::zeros);
        for (auto t=0; t<n_steps; t++){
          auto dst = pow(xtraj.at(i,t) - xtraj.at(j,t), 2) + pow(ytraj.at(i,t) - ytraj.at(j,t), 2);
          // std::cout << "dist: "<< dst << std::endl;
          traj_costs.at(t) = 2.0 - 2.0/(1.0 + exp(-cost_a1 *pow(dst, cost_a2)));
        }
        costs.at(i,j) = traj_costs.max() * cost_a3;
      }
    }
    return costs;
  }

  // arma::mat BRNE::collision_check(arma::mat ytraj){
    
  // }

  arma::mat BRNE::brne_nav(arma::mat xtraj_samples, arma::mat ytraj_samples){
    n_agents = xtraj_samples.n_rows / n_samples;
    std::cout << "N agents: " << n_agents << std::endl;
    // TODO I could cache this for different numbers of agents
    auto index_table = compute_index_table();
    std::cout << "Index table\n" << index_table << std::endl;
    auto costs = compute_costs(xtraj_samples, ytraj_samples);
    std::cout << "Costs\n" << costs << std::endl;
    arma::mat weights(n_agents, n_samples, arma::fill::zeros);
    for (int i=0; i<10; i++){
      std::cout << "weights update #" << i << std::endl;
      // weights = update_weights(costs, weights, index_table);
    }
    return weights;
  }
}
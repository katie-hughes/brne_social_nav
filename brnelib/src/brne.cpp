#include "brnelib/brne.hpp"

namespace brnelib
{
  arma::mat get_kernel_mat(arma::vec t1, arma::vec t2, double a1, double a2){
    arma::mat res;
    return res;
  }

  arma::vec mvn_sample_normal(int nsamples, int tsteps, arma::mat Lmat){
    arma::vec res;
    return res;
  }

  double get_min_dist(arma::vec xtraj, arma::vec ytraj){
    double res = 0;
    return res;
  }

  arma::mat get_Lmat(arma::vec train_ts, arma::vec test_ts, 
                     arma::vec train_noise, double a1, double a2){
    arma::mat res;
    return res;
  }

  arma::mat costs_nb(arma::vec trajx, arma::vec trajy, int n_agents, int n_points, int tsteps, 
                     double a1, double a2, double a3){
    arma::mat res;
    return res;
  }

  // 
  arma::vec weights_update(arma::mat all_costs, arma::vec old_weights, arma::vec index_table, 
                           arma::mat all_pt_index, int num_agents, int num_points){
    arma::vec res;
    return res;
  }

  arma::mat get_index_table(int num_agents){
    arma::mat res;
    return res;
  }

  arma::vec coll_beck(arma::vec trajy, double ymin, double ymax){
    arma::vec res;
    return res;
  }

  // very unsure of types
  std::vector<arma::vec> brne_nav(std::vector<arma::vec> xtraj_samples, std::vector<arma::vec> ytraj_samples, 
                     int n_agents, int tsteps, int n_points, double a1, double a2, double a3, 
                     double ymin, double ymax){
    std::vector<arma::vec> res;
    return res;
  }
}
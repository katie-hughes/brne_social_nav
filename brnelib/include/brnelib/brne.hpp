#include <iostream>
#include <cmath> 
#include <string>
#include <stdexcept>
#include <iosfwd>
#include <vector>
#include <armadillo>

namespace brnelib
{
  // Return the kernel matrix for parameters a1 and a2
  // feel confident about these types
  arma::mat get_kernel_mat(arma::vec t1, arma::vec t2, double a1, double a2);

  // multivariate normal distribution sampling
  // do not feel confident for return type
  arma::vec mvn_sample_normal(int nsamples, int tsteps, arma::mat Lmat);

  // no idea of types
  double get_min_dist(arma::vec xtraj, arma::vec ytraj);

  // cholesky: already exists

  // feel ok about types
  arma::mat get_Lmat(arma::vec train_ts, arma::vec test_ts, 
                     double train_noise, double a1, double a2);

  // feel ok about types
  arma::mat costs_nb(arma::vec trajx, arma::vec trajy, int n_agents, int n_points, int tsteps, 
                     double a1, double a2, double a3);

  // 
  arma::vec weights_update(arma::mat all_costs, arma::vec old_weights, arma::vec index_table, 
                           arma::mat all_pt_index, int num_agents, int num_points);

  // types good
  arma::mat get_index_table(int num_agents);

  // no idea of types
  arma::vec coll_beck(arma::vec trajy, double ymin, double ymax);

  // very unsure of types
  arma::vec brne_nav(arma::vec xtraj_samples, arma::vec ytraj_samples, 
                     int n_agents, int tsteps, int n_points, double a1, double a2, double a3, 
                     double ymin, double ymax);
}
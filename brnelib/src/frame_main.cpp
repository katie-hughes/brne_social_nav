#include "brnelib/brne.hpp"

int main(){
  double kernel_a1 =    0.5;
  double kernel_a2 =    0.2;
  double cost_a1 =      8.0;
  double cost_a2 =      5.0;
  double cost_a3 =      40.0;
  double dt =           0.1;
  int n_steps =         6;
  int n_samples =       5;
  double max_ang_vel =  1;
  double max_lin_vel =  1;
  double y_min =        -0.5;
  double y_max =        0.5;

  brne::BRNE brne_test{kernel_a1, kernel_a2,
                       cost_a1, cost_a2, cost_a3,
                       dt, n_steps, n_samples,
                       max_ang_vel, max_lin_vel,
                       y_min, y_max};
  brne_test.print_params();
  brne_test.compute_Lmat();

  auto sample = brne_test.mvn_sample_normal();
  std::cout << "sample \n" << sample << std::endl;

  // agent 1: x from -2 to 0, y at -0.1
  const auto x1_start = -2.0;
  const auto x1_end = 0;
  arma::rowvec xlist_1 = arma::linspace<arma::rowvec>(-2, 0, n_steps);
  arma::rowvec ylist_1(n_steps, arma::fill::value(-0.1));

  // agent 2: from 0 to -2, y = -0.3
  const auto x2_start = 0;
  const auto x2_end = -2.0;
  arma::rowvec xlist_2 = arma::linspace<arma::rowvec>(0, -2, n_steps);
  arma::rowvec ylist_2(n_steps, arma::fill::value(-0.3));

  std::cout << "X1\n" << xlist_1 << std::endl;
  std::cout << "Y1\n" << ylist_1 << std::endl;
  std::cout << "X2\n" << xlist_2 << std::endl;
  std::cout << "Y2\n" << ylist_2 << std::endl;

  return 0;
}
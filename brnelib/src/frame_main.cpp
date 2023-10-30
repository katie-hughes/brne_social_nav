#include "brnelib/brne.hpp"

int main(){
  double kernel_a1 =    0.5;
  double kernel_a2 =    0.2;
  double cost_a1 =      8.0;
  double cost_a2 =      5.0;
  double cost_a3 =      40.0;
  double dt =           0.1;
  int n_steps =         20;
  int n_samples =       5;
  double max_ang_vel =  1;
  double max_lin_vel =  1;
  double y_min =        -0.5;
  double y_max =        0.5;

  brne::BRNE test{kernel_a1, kernel_a2,
                  cost_a1, cost_a2, cost_a3,
                  dt, n_steps, n_samples,
                  max_ang_vel, max_lin_vel,
                  y_min, y_max};
  test.print_params();
  return 0;
}
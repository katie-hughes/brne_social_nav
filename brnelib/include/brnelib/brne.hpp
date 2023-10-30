#include <iostream>
#include <cmath> 
#include <string>
#include <stdexcept>
#include <iosfwd>
#include <vector>
#include <armadillo>

namespace brnelib
{

  class BRNE{
    private:
      double kernel_a1;
      double kernel_a2;
      double cost_a1;
      double cost_a2;
      double cost_a3;
      double dt;
      int n_steps;
      int n_samples;
      double max_ang_vel;
      double max_lin_vel;
      double y_min;
      double y_max;
    public:
      explicit BRNE(double kernel_a1, double kernel_a2,
                    double cost_a1, double cost_a2, double cost_a3);
      void print_params();
  };
}
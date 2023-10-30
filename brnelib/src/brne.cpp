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
}
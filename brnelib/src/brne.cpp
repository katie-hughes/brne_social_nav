#include "brnelib/brne.hpp"

namespace brnelib
{
  BRNE::BRNE(double kernel_a1, double kernel_a2,
             double cost_a1, double cost_a2, double cost_a3):
    kernel_a1{kernel_a1},
    kernel_a2{kernel_a2},
    cost_a1{cost_a1},
    cost_a2{cost_a2},
    cost_a3{cost_a3}
    {}
  void BRNE::print_params(){
    std::cout << "Kernel a1:\t" << kernel_a1 << std::endl;
    std::cout << "Kernel a2:\t" << kernel_a2 << std::endl;
    std::cout << "Cost a1:\t" << cost_a1 << std::endl;
    std::cout << "Cost a2:\t" << cost_a2 << std::endl;
    std::cout << "Cost a3:\t" << cost_a3 << std::endl;
  }
}
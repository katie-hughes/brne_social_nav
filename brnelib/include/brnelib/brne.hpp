#include <iostream>
#include <cmath> 
#include <string>
#include <stdexcept>
#include <iosfwd>
#include <vector>
#include <armadillo>

namespace brne
{
  /// @brief Class for Bayes Rule Nash Equilibrium calculations
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
      /// @brief Construct an instance of the BRNE class
      /// @param kernel_a1 control the "straightness" of trajectory samples. 
      /// The larger the value is, the less straight the trajectory sample will be.
      /// @param kernel_a2 control the "width/spreadness" of trajectory samples. 
      /// The larger the value is, more spread the trajectory samples are.
      /// @param cost_a1 control the safety zone
      /// The smaller the value is, the more conversative the robot will be.
      /// @param cost_a2 control the safety zone
      /// The larger the value is, the more conservative the robot will be.
      /// @param cost_a3  control the safety penalty weight
      /// The larger the value is, more conservative the robot will be.
      /// @param dt time between ticks (seconds)
      /// @param n_steps number of timesteps in the planning horizon
      /// @param n_samples number of samples each agent generates
      /// @param max_ang_vel maximum angular velocity of the robot
      /// @param max_lin_vel maximum linear velocity of the robot
      /// @param y_min minimum coordinate of the corridor
      /// @param y_max maximum coordinate of the corridor
      explicit BRNE(double kernel_a1, double kernel_a2,
                    double cost_a1, double cost_a2, double cost_a3,
                    double dt, int n_steps, int n_samples,
                    double max_ang_vel, double max_lin_vel,
                    double y_min, double y_max);
      /// @brief Print out the constants of the BRNE class.
      void print_params();
      arma::mat compute_kernel_mat(arma::vec t1, arma::vec t2);
      void compute_Lmat();
  };
}
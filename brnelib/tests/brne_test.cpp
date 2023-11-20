#include "brnelib/brne.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <string>
#include <sstream>


namespace brne {

    TEST_CASE("one_static_ped", "brne"){
        int n_steps =         5;
        int n_samples =       16;

        double kernel_a1 =    0.5;
        double kernel_a2 =    0.2;
        double cost_a1 =      8.0;
        double cost_a2 =      5.0;
        double cost_a3 =      20.0;
        double dt =           0.1;
        double y_min =        -0.5;
        double y_max =        0.5;

        int n_agents = 2;

        brne::BRNE brne_test{kernel_a1, kernel_a2,
                            cost_a1, cost_a2, cost_a3,
                            dt, n_steps, n_samples,
                            y_min, y_max};
        brne_test.print_params();

        double max_lin_vel = 0.6;
        double max_ang_vel = 1.0;

        double nominal_lin_vel = 0.4;

        brne::TrajGen trajgen{max_lin_vel, max_ang_vel, n_samples, n_steps, dt};

        // nominal trajectories
        arma::mat x_nominal(n_agents, n_steps, arma::fill::zeros);
        arma::mat y_nominal(n_agents, n_steps, arma::fill::zeros);

        // Robot goes forward. starts at (0,0,0)
        arma::rowvec robot_pose(3, arma::fill::zeros);
        auto traj_samples = trajgen.traj_sample(nominal_lin_vel, 0.0, robot_pose);

        arma::mat xtraj_samples(n_agents*n_samples, n_steps, arma::fill::zeros);
        arma::mat ytraj_samples(n_agents*n_samples, n_steps, arma::fill::zeros);

        xtraj_samples.submat(0, 0, n_samples-1, n_steps-1) = trajgen.get_xtraj_samples();
        ytraj_samples.submat(0, 0, n_samples-1, n_steps-1) = trajgen.get_ytraj_samples();

        // pedestrian static at (1.0, 0.01)
        double ped_x = 1.0;
        double ped_y = 0.01;
        xtraj_samples.submat((1)*n_samples, 0, (2)*n_samples-1, n_steps-1) = arma::mat(n_samples, n_steps, arma::fill::value(ped_x));
        ytraj_samples.submat((1)*n_samples, 0, (2)*n_samples-1, n_steps-1) = arma::mat(n_samples, n_steps, arma::fill::value(ped_y));

        // std::cout << "Xtraj samples \n" << xtraj_samples << std::endl;
        // std::cout << "Ytraj samples \n" << ytraj_samples << std::endl;

        auto weights = brne_test.brne_nav(xtraj_samples, ytraj_samples);

        std::cout << "Weights\n" << weights << std::endl;

        REQUIRE(static_cast<int>(weights.n_rows) == n_agents);
        REQUIRE(static_cast<int>(weights.n_cols) == n_samples);

        arma::mat desired_weights(n_agents, n_samples, arma::fill::zeros);
        // hardcoded from a version of the algorithm that I know works
        desired_weights.row(0) = arma::rowvec(std::vector<double>{7.1966e+00,8.8948e-01,6.0468e-02,3.5967e-03,2.7752e-04,3.3587e-05,6.5357e-06,1.9297e-06,6.9580e+00,8.3311e-01,5.4973e-02,3.2140e-03,2.4731e-04,3.0161e-05,5.9462e-06,1.7818e-06});
        desired_weights.row(1) = arma::rowvec(n_samples, arma::fill::ones);

        std::cout << "Desired Weights\n" << desired_weights << std::endl;

        REQUIRE(arma::approx_equal(weights, desired_weights, "absdiff", 0.0001));
        // REQUIRE(0);
    }

}
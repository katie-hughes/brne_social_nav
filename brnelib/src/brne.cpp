#include "brnelib/brne.hpp"

namespace brne
{
  BRNE::BRNE(){};
  BRNE::BRNE(double kernel_a1, double kernel_a2,
             double cost_a1, double cost_a2, double cost_a3,
             double dt, int n_steps, int n_samples,
             double y_min, double y_max):
    kernel_a1{kernel_a1},
    kernel_a2{kernel_a2},
    cost_a1{cost_a1},
    cost_a2{cost_a2},
    cost_a3{cost_a3},
    dt{dt},
    n_steps{n_steps},
    n_samples{n_samples},
    y_min{y_min},
    y_max{y_max}
    {
      compute_Lmat();
    }
  void BRNE::print_params(){
    auto line = "-----------------------------------------------";
    std::cout << "\n\t\tBRNE PARAMETERS" << std::endl;
    std::cout << line << std::endl;
    std::cout << "kernel a1:\t" << kernel_a1 << std::endl;
    std::cout << "kernel a2:\t" << kernel_a2 << std::endl;
    std::cout << "cost a1:\t" << cost_a1 << std::endl;
    std::cout << "cost a2:\t" << cost_a2 << std::endl;
    std::cout << "cost a3:\t" << cost_a3 << std::endl;
    std::cout << "dt:\t\t" << dt << std::endl;
    std::cout << "n steps:\t" << n_steps << std::endl;
    std::cout << "n samples:\t" << n_samples << std::endl;
    std::cout << "y min:\t\t" << y_min << std::endl;
    std::cout << "y max:\t\t" << y_max << std::endl;
    std::cout << "covariance matrix\n" << cov_mat << std::endl;
    std::cout << "covariance Lmat:\n" << cov_Lmat << std::endl;
    std::cout << line << std::endl;
  }

  std::string BRNE::param_string(){
    std::string res;
    std::string line = "-----------------------------------------------\n";
    res = "\n\t\tBRNE PARAMETERS\n" + 
          line + 
          "kernel_a1:\t" + std::to_string(kernel_a1) + "\n" +
          "kernel_a2:\t" + std::to_string(kernel_a2) + "\n" +
          "cost_a1:\t" + std::to_string(cost_a1) + "\n" +
          "cost_a2:\t" + std::to_string(cost_a2) + "\n" +
          "cost_a3:\t" + std::to_string(cost_a3) + "\n" +
          "dt:\t\t" + std::to_string(dt) + "\n" +
          "n_steps:\t" + std::to_string(n_steps) + "\n" +
          "n_samples:\t" + std::to_string(n_samples) + "\n" +
          "y_min:\t\t" + std::to_string(y_min) + "\n" +
          "y_max:\t\t" + std::to_string(y_max) + "\n" +
          line; 
    return res;
  }

  arma::mat BRNE::compute_kernel_mat(arma::vec t1, arma::vec t2){
    // here I should only be doing the diagonal or something like this
    // to enforce symmetry
    // t1.at(i) and t2.at(i) are the same
    // smallest eigtenvalue of the matrix is??
    // largest / smallest
    arma::mat res(t1.size(), t2.size(), arma::fill::zeros);
    for (auto i=0; i<static_cast<int>(t1.size()); i++){
      for (auto j=0; j<static_cast<int>(t2.size()); j++){
        res.at(i,j) = kernel_a2 * exp(-kernel_a1 * pow((t1.at(i) - t2.at(j)),2));
      }
    }
    return res;
  }

  void BRNE::compute_Lmat(){
    cov_mat.reset();
    cov_Lmat.reset();
    arma::vec tlist = arma::linspace<arma::vec>(0, (n_steps-1)*dt, n_steps);
    arma::vec train_ts(1, arma::fill::value(tlist.at(0)));
    arma::vec train_noise(1, arma::fill::value(1e-3));

    auto cm_11 = compute_kernel_mat(train_ts, train_ts);
    cm_11 += train_noise.diag();
    auto cm_12 = compute_kernel_mat(tlist, train_ts);
    auto cm_22 = compute_kernel_mat(tlist, tlist);
    // use left division to do the inverse
    // do this with solve instead. arma::solve
    cov_mat = cm_22 - cm_12 * cm_11.i() * cm_12.t();
    // smallest eigenvalue
    const auto eigenvalue = arma::eig_sym(cov_mat);
    cov_mat += arma::eye(cov_mat.n_rows,cov_mat.n_rows) * 1e-6;
    auto success = arma::chol(cov_Lmat, cov_mat, "lower");
    // std::cout << "Cholesky success? " << success << std::endl;
    while (!success){
      cov_mat += arma::eye(cov_mat.n_rows,cov_mat.n_rows) * 1e-6;
      success = arma::chol(cov_Lmat, cov_mat, "lower");
      // std::cout << "Cholesky retry success? " << success << std::endl;
    }
  }

  arma::mat BRNE::mvn_sample_normal(){
    // TODO should do error checking to make sure Lmat is set
    arma::mat res(n_steps, n_samples, arma::fill::randn);
    return (cov_Lmat * res).t();
  }
  
  void BRNE::compute_index_table(){
    index_table.reset();
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
    index_table = table;
  }

  void BRNE::compute_costs(arma::mat xtraj, arma::mat ytraj){
    costs.reset();
    auto size = n_agents*n_samples;
    arma::mat new_costs(size, size, arma::fill::zeros);
    for (auto i=0; i<size; i++){
      for (auto j=0; j<size; j++){
        arma::vec traj_costs(n_steps, arma::fill::zeros);
        for (auto t=0; t<n_steps; t++){
          auto dst = pow(xtraj.at(i,t) - xtraj.at(j,t), 2) + pow(ytraj.at(i,t) - ytraj.at(j,t), 2);
          traj_costs.at(t) = 2.0 - 2.0/(1.0 + exp(-cost_a1 *pow(dst, cost_a2)));
        }
        new_costs.at(i,j) = traj_costs.max() * cost_a3;
      }
    }
    costs = new_costs;
  }

  void BRNE::collision_check(arma::mat ytraj){
    // only keep where y_min < y_traj < y_max
    coll_mask.reset();
    arma::vec valid(ytraj.n_rows);
    for (int r=0; r<static_cast<int>(ytraj.n_rows); r++){
      int is_valid = 1;
      for (int c=0; c<static_cast<int>(ytraj.n_cols); c++){
        if ((ytraj.at(r,c) < y_min) || (ytraj.at(r,c) > y_max)){
          is_valid = 0;
          break;
        }
      }
      valid.at(r) = is_valid;
    }
    coll_mask = arma::conv_to<arma::mat>::from(valid);
    coll_mask.reshape(n_samples, n_agents);
    coll_mask = coll_mask.t();
  }

  void BRNE::update_weights(){
    for (int i=0; i<n_agents; i++){
      auto row = arma::conv_to<arma::rowvec>::from(index_table.row(i));
      for (int j=0; j<n_samples; j++){
        auto c = 0.0;
        auto idx1 = all_pts.at(row.at(0), j);
        for (int k=0; k<n_agents-1; k++){
          for (int l=0; l<n_samples; l++){
            auto idx2 = all_pts.at(row.at(k+1), l);
            c += costs.at(idx1, idx2) * weights.at(row.at(k+1), l);
          }
        }
        c /= ((n_agents - 1) * n_samples);
        weights.at(i,j) = exp(-1.0 * c);
      }
      weights.row(i) /= arma::mean(weights.row(i));
    }
  }

  arma::mat BRNE::brne_nav(arma::mat xtraj_samples, arma::mat ytraj_samples){
    weights.reset();
    all_pts.reset();
    // compute number of agents
    n_agents = xtraj_samples.n_rows / n_samples;
    std::cout << "N agents: " << n_agents << std::endl;
    // compute all points index
    all_pts = arma::conv_to<arma::mat>::from(arma::linspace<arma::rowvec>(0, n_agents*n_samples-1, n_agents*n_samples));
    all_pts.reshape(n_samples, n_agents);
    all_pts = all_pts.t();
    // TODO I could cache this for different numbers of agents
    compute_index_table();
    // get the costs
    compute_costs(xtraj_samples, ytraj_samples);
    weights = arma::mat(n_agents, n_samples, arma::fill::ones);
    for (int i=0; i<10; i++){
      std::cout << "weights update #" << i << std::endl;
      std::cout << "weights\n" << weights << std::endl;
      update_weights();
    }
    std::cout << "Final weights\n" << weights << std::endl;

    collision_check(ytraj_samples);
    // TODO: add a check to make sure that at least some of these are true for agent.

    for (int a=0; a<n_agents; a++){
      // element wise multiplication
      auto agent_weights = arma::conv_to<arma::rowvec>::from(weights.row(a));
      auto agent_mask = arma::conv_to<arma::rowvec>::from(coll_mask.row(a));
      auto masked_weights = agent_weights % agent_mask;
      weights.row(a) = masked_weights / arma::mean(masked_weights);
    }

    std::cout << "Weights after masking\n" << weights << std::endl;
    return weights;
  }

  std::vector<traj> BRNE::compute_optimal_trajectory(arma::mat x_nominal, arma::mat y_nominal,
                                                     arma::mat x_samples, arma::mat y_samples){
    std::vector<traj> res;
    for (int a=0; a<n_agents; a++){
      auto agent_weights = arma::conv_to<arma::vec>::from(weights.row(a));
      auto agent_x_samples = arma::conv_to<arma::mat>::from(x_samples.submat(a*n_samples, 0, (a+1)*n_samples-1, n_steps-1));
      auto agent_y_samples = arma::conv_to<arma::mat>::from(y_samples.submat(a*n_samples, 0, (a+1)*n_samples-1, n_steps-1));
      for (int i=0; i<n_samples; i++){
        agent_x_samples.row(i) *= agent_weights.at(i);
        agent_y_samples.row(i) *= agent_weights.at(i);
      }
      traj agent_traj;
      // std::cout << "new agent samples\n" << agent_x_samples << std::endl;
      // auto test = x_nominal.row(a) + arma::mean(agent_x_samples,0);
      agent_traj.x = arma::conv_to<std::vector<double>>::from(x_nominal.row(a) + arma::mean(agent_x_samples,0));
      agent_traj.y = arma::conv_to<std::vector<double>>::from(y_nominal.row(a) + arma::mean(agent_y_samples,0));
      res.push_back(agent_traj);
    }
    return res;
  }


  TrajGen::TrajGen(){}

  TrajGen::TrajGen(double max_lin_vel, double max_ang_vel, int n_samples, int n_steps, double dt):
    max_lin_vel{max_lin_vel},
    max_ang_vel{max_ang_vel},
    n_samples{n_samples},
    n_steps{n_steps},
    dt{dt}
  {}

  std::vector<arma::mat> TrajGen::traj_sample(double lin_vel, double ang_vel, arma::rowvec state){
    arma::vec lin_vel_vec(n_steps, arma::fill::value(lin_vel));
    arma::vec ang_vel_vec(n_steps, arma::fill::value(ang_vel));
    arma::mat nominal_commands(n_steps, 2, arma::fill::zeros);
    nominal_commands.col(0) = lin_vel_vec;
    nominal_commands.col(1) = ang_vel_vec;

    std::cout << "Commands\n" << nominal_commands << std::endl;
    auto n_per_dim = static_cast<int>(sqrt(n_samples));
    auto n_per_lin = static_cast<int>(n_per_dim*2);
    auto n_per_ang = static_cast<int>(n_per_dim/2);
    std::cout << "N per dim: " << n_per_dim << " per lin " << n_per_lin << " per ang " << n_per_ang << std::endl;


    auto lin_offset = std::min(lin_vel_vec.min(), max_lin_vel-lin_vel_vec.max());
    auto ang_offset = std::min(max_ang_vel + ang_vel_vec.min(), max_ang_vel-ang_vel_vec.max());

    std::cout << "Lin offset " << lin_offset << " ang offset " << ang_offset << std::endl;


    auto lin_ls = arma::linspace<arma::vec>(-lin_offset, lin_offset, n_per_lin);
    auto ang_ls = arma::linspace<arma::vec>(-ang_offset, ang_offset, n_per_ang);

    std::cout << "Lin linspace\n" << lin_ls << std::endl;
    std::cout << "Ang linspace\n" << ang_ls << std::endl;

    // want to get the combination of every single pair of lin/ang in these vectors

    arma::mat u_perturbs(n_samples, 2, arma::fill::zeros);
    arma::vec lin_col(n_samples, arma::fill::zeros);
    arma::vec ang_col(n_samples, arma::fill::zeros);
    for (int i=0; i<n_per_ang; i++){
      lin_col.subvec(i*n_per_lin, (i+1)*n_per_lin-1) = lin_ls;
      ang_col.subvec(i*n_per_lin, (i+1)*n_per_lin-1) = arma::vec(n_per_lin, arma::fill::value(ang_ls.at(i)));
    }
    u_perturbs.col(0) = lin_col;
    u_perturbs.col(1) = ang_col;
    std::cout << "u perturbs\n" << u_perturbs << std::endl;

    ulist = arma::mat(u_perturbs);
    ulist.col(0) += lin_vel;
    ulist.col(1) += ang_vel;
    std::cout << "ulist\n" << ulist << std::endl;

    // next I need to make arrays of states

    arma::mat states(n_samples, 3, arma::fill::zeros);
    states.each_row() = state;

    // arma::mat sdot = dyn(states, ulist);

    std::vector<arma::mat> traj;

    xtraj_samples = arma::mat(n_samples, n_steps, arma::fill::zeros);
    ytraj_samples = arma::mat(n_samples, n_steps, arma::fill::zeros);
    end_pose = arma::mat(n_samples, 2, arma::fill::zeros);

    for (int t=0; t<n_steps; t++){
      states = dyn_step(states, ulist);
      traj.push_back(arma::mat(states));
      std::cout << "State\n" << states << std::endl;
      xtraj_samples.col(t) = states.col(0);
      ytraj_samples.col(t) = states.col(1);
      if (t == n_steps - 1){
        end_pose.col(0) = states.col(0);
        end_pose.col(1) = states.col(1);
      }

    }

    std::cout << "X traj samples\n" << xtraj_samples << std::endl;
    std::cout << "Y traj samples\n" << ytraj_samples << std::endl;
    

    return traj;
  }

  arma::mat TrajGen::get_xtraj_samples(){
    return xtraj_samples;
  }

  arma::mat TrajGen::get_ytraj_samples(){
    return ytraj_samples;
  }

  arma::mat TrajGen::opt_controls(arma::rowvec goal){
    std::cout << "End pose\n" << end_pose << std::endl;
    arma::mat goal_mat(n_samples, 2, arma::fill::zeros);
    goal_mat.each_row() = goal;
    std::cout << "Goal mat\n" << goal_mat << std::endl;
    auto opt_idx = (arma::vecnorm(end_pose - goal_mat, 2, 1)).index_min();
    std::cout << "optimal_index\n" << opt_idx << std::endl;
    arma::mat opt_cmds(n_steps, 2, arma::fill::zeros);
    opt_cmds.each_row() = ulist.row(opt_idx);
    return opt_cmds;
  }


  arma::mat TrajGen::dyn(arma::mat state, arma::mat controls){
    // lin vel is controls.col(0)
    // ang vel is controls.col(1)
    // angles is state.col(2), x is col(0), y is col(1)
    arma::mat sdot(n_samples, 3, arma::fill::zeros);
    // % is element wise multiplication
    sdot.col(0) = controls.col(0) % arma::cos(state.col(2));
    sdot.col(1) = controls.col(0) % arma::sin(state.col(2));
    sdot.col(2) = controls.col(1);
    // std::cout << "Sdot\n" << sdot << std::endl;
    return sdot;
  }

  arma::mat TrajGen::dyn_step(arma::mat state, arma::mat controls){
    arma::mat k1 = dt * dyn(state, controls);
    arma::mat k2 = dt * dyn(state + 0.5*k1, controls);
    arma::mat k3 = dt * dyn(state + 0.5*k2, controls);
    arma::mat k4 = dt * dyn(state + k3, controls);

    return state + (k1 + 2.0*k2 + 2.0*k3 + k4)/6.0;
  }

  arma::rowvec TrajGen::dyn(arma::rowvec state, arma::rowvec controls){
    arma::rowvec sdot(3, arma::fill::zeros);
    sdot.at(0) = controls.at(0) * cos(state.at(2));
    sdot.at(1) = controls.at(0) * sin(state.at(2));
    sdot.at(2) = controls.at(1);
    // std::cout << "Sdot\n" << sdot << std::endl;
    return sdot;
  }

  arma::rowvec TrajGen::dyn_step(arma::rowvec state, arma::rowvec controls){
    arma::rowvec k1 = dt * dyn(state, controls);
    arma::rowvec k2 = dt * dyn(state + 0.5*k1, controls);
    arma::rowvec k3 = dt * dyn(state + 0.5*k2, controls);
    arma::rowvec k4 = dt * dyn(state + k3, controls);
    return state + (k1 + 2.0*k2 + 2.0*k3 + k4)/6.0;
  }

  arma::mat TrajGen::sim_traj(arma::rowvec state, arma::mat controls){
    std::cout << "State\n" << state << std::endl;
    std::cout << "Controls\n" << controls << std::endl;
    arma::mat opt_traj(n_steps, 3, arma::fill::zeros);
    arma::rowvec current_state(state);
    std::cout << "Current State" << current_state << std::endl;
    opt_traj.row(0) = arma::rowvec(current_state);
    std::cout << "Traj\n" << opt_traj << std::endl;
    for (int i=1; i<n_steps; i++){
      current_state = dyn_step(current_state, controls.row(i-1));
      std::cout << "Current State" << current_state << std::endl;
      opt_traj.row(i) = arma::rowvec(current_state);
    }
    std::cout << "Traj\n" << opt_traj << std::endl;
    return opt_traj;
  }

}
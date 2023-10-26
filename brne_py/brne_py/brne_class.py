########################################################## FUNCTIONS
import numpy as np
import numba as nb

# import os
# os.environ['NUMBA_NUM_THREADS'] = '12'
# nb.config.NUMBA_NUM_THREADS = 4

rng = np.random.default_rng(1)


"""
BRNE parameters:
kernel_a1: Control the "straightness" of trajectory samples. Larger the value is, less straight the trajectory sample will be.
kernel_a2: Control the "width/spreadness" of trajectory samples. Larger the value is, more spread the trajectory samples are.
cost_a1: Control the safety zone, smaller the value is, more conversative the robot will be.
cost_a2: Control the safety zone, larger the value is, more conservative the robot will be.
cost_a3: Control the safety penalty weight, larger the value is, more conservative the robot will be.
"""
# kernel_a1 = 0.5
# kernel_a2 = 0.2
# cost_a1 = 8.0
# cost_a2 = 1.0
# cost_a3 = 20.0


@nb.njit  # ('float64[:, :](float64[:, :])')
def cholesky_numba(A):
    n = A.shape[0]
    L = np.zeros_like(A)
    for i in range(n):
        for j in range(i + 1):
            s = 0
            for k in range(j):
                s += L[i][k] * L[j][k]

            if (i == j):
                L[i][j] = (A[i][i] - s) ** 0.5
            else:
                L[i][j] = (1.0 / L[j][j] * (A[i][j] - s))
    return L


class BRNE:
    def __init__(self, 
                 kernel_a1, kernel_a2, 
                 cost_a1, cost_a2, cost_a3, 
                 dt, plan_steps, n_samples,
                 max_ang_vel, max_lin_vel, nominal_vel,
                 ped_sample_scale, ymin, ymax):
        self.kernel_a1 = kernel_a1
        self.kernel_a2 = kernel_a2
        self.cost_a1 = cost_a1
        self.cost_a2 = cost_a2
        self.cost_a3 = cost_a3
        self.dt = dt
        self.plan_steps = plan_steps
        self.n_samples = n_samples
        self.max_ang_vel = max_ang_vel
        self.max_lin_vel = max_lin_vel
        self.nominal_vel = nominal_vel
        self.ped_sample_scale = ped_sample_scale
        self.ymin = ymin
        self.ymax = ymax

        self.tlist = np.arange(self.plan_steps) * 0.1
        self.train_ts = np.array([self.tlist[0]])
        self.train_noise = np.array([1e-04])
        self.test_ts = self.tlist

        self.cov_Lmat = None
        self.cov_mat = None
        # fills these in
        self.get_Lmat()

        # x,y,yaw
        self.state = np.array([0,0,0],dtype=float)
        self.peds = []
        self.n_agents = 1
        self.goal = None

        self.index_table = None
        self.get_index_table()

    @nb.jit(nopython=True, parallel=True)
    def get_kernel_mat(self, t1list, t2list):
        mat = np.zeros((t1list.shape[0], t2list.shape[0]))
        for i in nb.prange(t1list.shape[0]):
            for j in nb.prange(t2list.shape[0]):
                mat[i][j] = np.exp(-self.kernel_a1 * (t1list[i] - t2list[j]) ** 2) * self.kernel_a2
        return mat.T
    

    def get_Lmat(self):
        covmat_11 = self.get_kernel_mat(self.train_ts, self.train_ts)
        covmat_11 += np.diag(self.train_noise)
        covmat_12 = self.get_kernel_mat(self.test_ts, self.train_ts).T
        covmat_22 = self.get_kernel_mat(self.test_ts, self.test_ts)
        cov_mat = covmat_22 - covmat_12 @ np.linalg.inv(covmat_11) @ covmat_12.T
        cov_mat += np.eye(self.test_ts.shape[0]) * 1e-06
        self.cov_Lmat = cholesky_numba(cov_mat)
        self.cov_mat = cov_mat

    def mvn_sample_normal(self):
        init_samples = rng.standard_normal(size=(self.plan_steps, (self.n_agents - 1)*self.n_samples))
        new_samples = self.cov_Lmat @ init_samples
        return new_samples.T
    
    @nb.jit(nopython=True, parallel=True)
    def costs_nb(self, trajs_x, trajs_y):
        vals = np.zeros((self.n_agents * self.n_samples, self.n_agents * self.n_samples))
        for i in nb.prange(self.n_samples * self.n_agents):
            traj_xi = trajs_x[i]
            traj_yi = trajs_y[i]
            for j in nb.prange(self.n_samples * self.n_agents):
                traj_xj = trajs_x[j]
                traj_yj = trajs_y[j]
                traj_costs = np.zeros(self.plan_steps)
                for t in nb.prange(self.plan_steps):
                    dist = (traj_xi[t] - traj_xj[t]) ** 2 + (traj_yi[t] - traj_yj[t]) ** 2
                    traj_costs[t] = 2 - 2 / (1.0 + np.exp(-self.cost_a1 * np.power(dist, self.cost_a2)))
                vals[i, j] = np.max(traj_costs) * self.cost_a3
        return vals
    
    @nb.jit(nopython=True, parallel=True)
    def get_index_table(self):
        index_table = np.zeros((self.n_agents, self.n_agents))
        for i in nb.prange(self.n_agents):
            index_table[i][0] = i
            idx = 1
            for j in range(self.n_agents):
                if i == j:
                    continue
                index_table[i][idx] = j
                idx += 1
        self.index_table = index_table.astype(int)

    
    @nb.jit(nopython=True, parallel=True)
    def weights_update_nb(self, all_costs, old_weights):
        all_pt_index = np.arange(self.n_agents * self.n_samples).reshape(self.n_agents, self.n_samples)
        weights = old_weights.copy()
        for i in range(self.n_agents):
            row = self.index_table[i]
            for j1 in nb.prange(self.n_samples):
                cost1 = 0.0
                idx1 = all_pt_index[row[0], j1]
                for i2 in nb.prange(self.n_agents - 1):
                    for j2 in range(self.n_samples):
                        idx2 = all_pt_index[row[i2 + 1], j2]
                        cost1 += all_costs[idx1, idx2] * weights[row[i2 + 1], j2]
                cost1 /= (self.n_agents - 1) * self.n_samples
                weights[i, j1] = np.exp(-1.0 * cost1)
            weights[i] /= np.mean(weights[i])
        return weights
    
    @nb.jit(nopython=True, parallel=True)
    def coll_beck(self, trajs_y):
        lower_mask = trajs_y > self.ymin
        upper_mask = trajs_y < self.ymax
        return lower_mask * upper_mask

    def brne_nav(self, xtraj_samples, ytraj_samples, iterations=10):
        self.get_index_table()
        weights = np.ones((self.n_agents, self.n_samples))
        all_costs = self.costs_nb(xtraj_samples, ytraj_samples)
        coll_mask = self.coll_beck(ytraj_samples[0:self.n_samples]).all(axis=1).astype(float)
        # if we are going out of bounds, coll_mask is all 0s and we will encounter divide by 0 error.
        if not np.any(coll_mask):
            return None
        for i in range(iterations+1):
            weights = self.weights_update_nb(all_costs, weights)

        agent_weights = weights[0] * coll_mask
        agent_weights /= np.mean(agent_weights)
        weights[0] = agent_weights.copy()
        return weights
    

    def update(self):
        self.n_agents = len(self.peds) + 1
        x_pts = self.mvn_sample_normal()
        y_pts = self.mvn_sample_normal()
        print(f"X pts {x_pts.shape}")
        print(f"Y pts {y_pts.shape}")



class Dyn:
    def __init__(self, dt, u1_max, u2_max, n_samples):
        self.dt = dt
        self.u1_max = u1_max
        self.u2_max = u2_max
        self.n_samples = n_samples
    def dyn(self, st, ut):
        sdot = np.array([
            ut[0] * np.cos(st[2]),
            ut[0] * np.sin(st[2]),
            ut[1]
        ])
        return sdot
    def dyn_step(self, st, ut):
        k1 = self.dt * self.dyn(st, ut)
        k2 = self.dt * self.dyn(st + k1/2.0, ut)
        k3 = self.dt * self.dyn(st + k2/2.0, ut)
        k4 = self.dt * self.dyn(st + k3, ut)
        return st + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    def traj_sim(self, st, ulist):
        tsteps = len(ulist)
        traj = np.zeros((tsteps, 3))
        for t in range(tsteps):
            st = self.dyn_step(st, ulist[t])
            traj[t] = st.copy()
        return traj
    def traj_sim_essemble(self, st, ulist,):
        """
        st.shape = (3, num_samples)
        ulist = (tsteps, num_samples, 2)
        """
        tsteps = ulist.shape[0]
        num_samples = ulist.shape[1]
        assert st.shape[1] == ulist.shape[1]
        traj = np.zeros((tsteps, 3, num_samples))
        for t in range(tsteps):
            st = self.dyn_step(st, ulist[t].T)
            traj[t] = st.copy()
        return traj
    
    def get_ulist_essemble(self, ulist):
        num_essembles_per_dim = int(np.sqrt(self.n_samples))
        num_essembles_per_dim_u1 = int(num_essembles_per_dim * 2)
        num_essembles_per_dim_u2 = int(num_essembles_per_dim / 2)

        u1_offset = np.minimum(
            ulist[:,0].min(),
            self.u1_max-ulist[:,0].max()
        )
        u2_offset = np.minimum(
            self.u2_max+ulist[:,1].min(),
            self.u2_max-ulist[:,1].max()
        )
        u1_perturbs, u2_perturbs = np.meshgrid(
        np.linspace(-u1_offset, u1_offset, num_essembles_per_dim_u1),
        np.linspace(-u2_offset, u2_offset, num_essembles_per_dim_u2)
        )
        u_perturbs = np.array([
            u1_perturbs.ravel(), u2_perturbs.ravel()
        ]).T

        ulist_essemeble = ulist[:,np.newaxis] + u_perturbs
        assert ulist_essemeble.shape[0] == ulist.shape[0]
        assert ulist_essemeble.shape[1] == u_perturbs.shape[0]
        assert ulist_essemeble.shape[2] == ulist.shape[1]

        return ulist_essemeble
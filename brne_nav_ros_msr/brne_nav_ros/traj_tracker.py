import numpy as np


class TrajTracker():
    def __init__(self, dt, max_lin_vel=0.6, max_ang_vel=1.2):
        self.dt = dt
        self.ad = -5  # range between -2 to -10, prfer to be smaller than -5
        self.R = np.diag(np.array([1.0, 2.0]))
        self.umax = np.array([max_lin_vel, max_ang_vel])
        # self.umin = -1.0 * self.umax
        self.umin = np.array([0.0, -max_ang_vel])

    def wrap(self, a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    def dyn(self, st, ut):
        xdot = ut[0] * np.cos(st[2])
        ydot = ut[0] * np.sin(st[2])
        thdot = ut[1]
        sdot = np.array([xdot, ydot, thdot])
        return sdot

    def dyn_step(self, st, ut):
        k1 = self.dyn(st, ut) * self.dt
        k2 = self.dyn(st+k1/2, ut) * self.dt
        k3 = self.dyn(st+k2/2, ut) * self.dt
        k4 = self.dyn(st+k3, ut) * self.dt
        st_new = st + (k1 + 2*k2 + 2*k3 + k4) / 6
        st_new[2] = self.wrap(st_new[2])
        return st_new

    def sim_traj(self, s0, ulist):
        tsteps = len(ulist) + 1
        ndim = len(s0)
        traj = np.zeros((tsteps, ndim))
        traj[0] = s0.copy()
        st = s0.copy()
        for t in range(1, tsteps):
            st = self.dyn_step(st, ulist[t-1])
            traj[t] = st.copy()
        return traj.copy()

    def l1(ref_st, st):
        diff_s = st - ref_st
        val = np.sum(np.square(diff_s))
        return val

    def dl1ds(self, ref_st, st):
        diff_s = st - ref_st
        diff_s[2] = self.wrap(diff_s[2])
        return 2 * diff_s

    def rho_dyn(self, rhot, st, ut, ref_st):
        A = np.array([
            [0, 0, ut[0] * -np.sin(st[2])],
            [0, 0, ut[0] * np.cos(st[2])],
            [0, 0, 0]
        ])
        return A.T @ rhot + self.dl1ds(ref_st, st)

    def rho_step(self, rhot, st, ut, ref_st):
        k1 = self.rho_dyn(rhot, st, ut, ref_st) * self.dt
        k2 = self.rho_dyn(rhot+k1/2, st, ut, ref_st) * self.dt
        k3 = self.rho_dyn(rhot+k2/2, st, ut, ref_st) * self.dt
        k4 = self.rho_dyn(rhot+k3, st, ut, ref_st) * self.dt
        rhot_new = rhot + (k1 + 2*k2 + 2*k3 + k4) / 6
        return rhot_new

    def rho_sim(self, rhoT, slist, ulist, ref_slist):
        tsteps = len(slist)
        ndim = len(rhoT)
        rho_traj = np.zeros((tsteps, ndim))
        rho_traj[0] = rhoT.copy()
        rhot = rhoT.copy()
        for t in range(1, tsteps):
            rhot = self.rho_step(rhot, slist[t-1], ulist[t-1], ref_slist[t-1])
            rho_traj[t] = rhot.copy()
        return rho_traj.copy()

    def compute_u2(self, st, ut, rhot, R, ad, umin, umax):
        hx = np.array([
            [np.cos(st[2]), 0],
            [np.sin(st[2]), 0],
            [0, 1]
        ])
        temp_vec = hx.T @ rhot
        sig = np.outer(temp_vec, temp_vec)
        u2 = np.linalg.solve(sig + R.T, sig@ut + hx.T@rhot * ad)
        return np.clip(u2, umin, umax)

    def compute_u2list(self, slist, ulist, rho_list, R, ad, umin, umax):
        u2list = np.zeros_like(ulist)
        for i, (st, ut, rhot) in enumerate(zip(slist, ulist, rho_list)):
            u2list[i] = self.compute_u2(st, ut, rhot, R, ad, umin, umax)
        return u2list.copy()

    def get_controls(self, s0, ref_traj, udef):
        isac_horizon = 5

        sim_tsteps = ref_traj.shape[0]
        recon_ref_traj = np.concatenate([ref_traj, np.tile(ref_traj[-1], (isac_horizon, 1))])

        ref_slist = recon_ref_traj.copy()
        ref_thlist = np.arctan2(ref_slist[1:, 1] - ref_slist[:-1, 1], ref_slist[1:, 0] - ref_slist[:-1, 0])
        ref_thlist = self.wrap(ref_thlist)
        ref_thlist = np.array([*ref_thlist, ref_thlist[-1]])
        ref_slist = np.concatenate([ref_slist, ref_thlist[:, np.newaxis]], axis=1)

        ulist0 = np.zeros((isac_horizon, 2)) + udef  # np.array([0.0, 0.0])
        s_curr = s0.copy()
        opt_ulist = np.zeros((sim_tsteps-1, 2))
        opt_traj = np.zeros((sim_tsteps, 3))
        opt_traj[0] = s0.copy()

        for sim_t in range(sim_tsteps-1):
            ulist = ulist0.copy()

            curr_ref_slist = ref_slist[sim_t: sim_t + isac_horizon]
            slist = self.sim_traj(s_curr, ulist)
            # tlist = np.arange(isac_horizon) * self.dt + sim_t * self.dt
            rhoT = np.zeros_like(slist[-1])
            rho_list_back = self.rho_sim(rhoT, slist[::-1], ulist[::-1], curr_ref_slist[::-1])
            rho_list = rho_list_back[::-1]
            u2list = ulist.copy()
            u2list[0] = self.compute_u2(slist[0], ulist[0], rho_list[0], self.R, self.ad, self.umin, self.umax)

            opt_ulist[sim_t] = u2list[0].copy()
            s_curr = self.sim_traj(s_curr, u2list)[1]
            opt_traj[sim_t+1] = s_curr.copy()

        return opt_ulist, opt_traj


# # This is a hand-designed non-linear reference trajectory to track
# def ref_traj(t):
#     ref_loc_x = 0.5 * t * (1 - 0.1 * np.sin(1.6*t))
#     ref_loc_y = np.sin(0.2 * t) * 0.2 + 0.3 * t
#     ref_loc = np.array([ref_loc_x, ref_loc_y]).T
#     return ref_loc
# ref_slist = np.array([ref_traj(_t) for _t in np.arange(50) * 0.1])

# #### Trajectory tracking
# tracker = TrajTracker()

# tic = time.time()
# sac_ulist, sac_traj = tracker.get_controls(np.zeros(3), ref_slist)
# recon_traj = tracker.sim_traj(np.zeros(3), sac_ulist)
# toc = time.time()
# print('elapsed time: ', toc-tic)
# print(sac_traj)

# #### VISUALIZATION
# fig, ax = plt.subplots(1, 1, tight_layout=True, figsize=(8, 6))
# ax.plot(sac_traj[:,0], sac_traj[:,1], linestyle='-', marker='o', color='C0')
# ax.plot(ref_slist[:,0], ref_slist[:,1], linestyle='-', marker='o', color='C1')
# # ax.plot(recon_traj[:,0], recon_traj[:,1], linestyle='-', marker='o', color='C2')
# ax.set_aspect('equal')
# plt.show()
# plt.close()

# print()
# print(sac_ulist)

########################################################################################


# ## SHOOTING METHOD

# import numpy as np


# class TrajTracker:
#     def __init__(self):
#         self.umin = np.array([-0.5, -0.5])
#         self.umax = np.array([0.5, 0.5])
#         self.dt = 0.1

#     def wrap(self, a):
#         return (a + np.pi) % (2 * np.pi) - np.pi

#     def dyn(self, st, ut):
#         xdot = ut[0] * np.cos(st[2])
#         ydot = ut[0] * np.sin(st[2])
#         thdot = ut[1]
#         sdot = np.array([xdot, ydot, thdot])
#         return sdot

#     def dyn_step(self, st, ut):
#         k1 = self.dyn(st, ut) * self.dt
#         k2 = self.dyn(st+k1/2, ut) * self.dt
#         k3 = self.dyn(st+k2/2, ut) * self.dt
#         k4 = self.dyn(st+k3, ut) * self.dt
#         st_new = st + (k1 + 2*k2 + 2*k3 + k4) / 6
#         st_new[2] = self.wrap(st_new[2])
#         return st_new

#     def sim_traj(self, s0, ulist):
#         tsteps = len(ulist) + 1
#         ndim = len(s0)
#         traj = np.zeros((tsteps, ndim))
#         traj[0] = s0.copy()
#         st = s0.copy()
#         for t in range(1, tsteps):
#             st = self.dyn_step(st, ulist[t-1])
#             traj[t] = st.copy()
#         return traj.copy()

#     def get_controls(self, s0, ref_traj, udef):
#         s_curr = s0.copy()
#         plan_steps = len(ref_traj)
#         ulist = np.zeros((plan_steps-1, 2))

#         ref_slist = ref_traj.copy()
#         # ref_thlist = np.arctan2(ref_slist[1:,1]-ref_slist[:-1,1], ref_slist[1:,0]-ref_slist[:-1,0])
#         # ref_thlist = self.wrap(ref_thlist)
#         # ref_thlist = np.array([*ref_thlist, ref_thlist[-1]])
#         # ref_slist = np.concatenate([ref_slist, ref_thlist[:,np.newaxis]], axis=1)

#         for t in range(1, plan_steps):
#             s_next = ref_slist[t]
#             diff_s = s_next - s_curr[:2]

#             curr_vline = np.array([np.cos(s_curr[2]), np.sin(s_curr[2])])
#             projection = curr_vline * (curr_vline@diff_s)# / (curr_vline@curr_vline)

#             lin_vel = np.linalg.norm(projection) / self.dt
#             ang_vel = (np.arctan2(
#                 ref_traj[t+0,1] - ref_slist[t-1,1],
#                 ref_slist[t+0,0] - ref_slist[t-1,0])
#             - s_curr[2]) / self.dt

#             u_curr = np.array([lin_vel, ang_vel])
#             u_curr = np.clip(u_curr, self.umin, self.umax)
#             s_curr = self.dyn_step(s_curr, u_curr)
#             ulist[t-1] = u_curr.copy()

#         ulist_traj = self.sim_traj(s0, ulist)
#         return ulist, ulist_traj

from brne import *
np.set_printoptions(suppress=True)


plan_steps = 25
dt = 0.1
kernel_a1 = 0.2
kernel_a2 = 0.2
cost_a1 = 4.0
cost_a2 = 1.0
cost_a3 = 80.0
n_samples = 196
n_agents = 1
max_ang_vel = 1.0
nominal_vel = 0.4
ped_sample_scale = 0.1
ymin = -0.75
ymax = 0.75

# x, y, yaw.
state = np.array([0,0,0],dtype=float)
# X, Y
goal = np.array([5,0],dtype=float)
# ped
peds = []

# this is only done once during the initialization
tlist = np.arange(plan_steps) * 0.1
train_ts = np.array([tlist[0]])
train_noise = np.array([1e-04])
test_ts = tlist

print(f"Tlist {tlist.shape}")
print(f"train ts {train_ts.shape}")
print(f"test ts {test_ts.shape}")
print(f"Train noise {train_noise.shape}")

cov_Lmat, cov_mat = get_Lmat_nb(train_ts, test_ts, train_noise, kernel_a1, kernel_a2)
print(f"Cov lmat {cov_Lmat.shape}")
print(f"Cov mat {cov_mat.shape}")

# Test one iteration
x_pts = mvn_sample_normal((n_agents-1)*n_samples, plan_steps, cov_Lmat)
y_pts = mvn_sample_normal((n_agents-1)*n_samples, plan_steps, cov_Lmat)

print(f"X pts {x_pts.shape}")
print(f"Y pts {y_pts.shape}")

theta_a = state[2]
if state[2] > 0.0:
    theta_a -= np.pi/2
else:
    theta_a += np.pi/2
axis_vec = np.array([np.cos(theta_a), np.sin(theta_a)])
vec_to_goal = goal - state[:2]
dist_to_goal = np.linalg.norm(vec_to_goal)
proj_len = (axis_vec @ vec_to_goal) / (vec_to_goal @ vec_to_goal) * dist_to_goal
radius = 0.5 * dist_to_goal / proj_len

ut = np.array([nominal_vel, 0])
if state[2] > 0.0:
    ut[1] -= nominal_vel/radius
else:
    ut[1] += nominal_vel/radius

nominal_cmds = np.tile(ut, reps=(plan_steps,1))

print(f"Nominal cmds {nominal_cmds.shape}")

ulists = get_ulist_essemble(
                nominal_cmds, nominal_vel+0.05, max_ang_vel, n_samples)

print(f"Ulists {ulists.shape}")

trajs = traj_sim_essemble(
                np.tile(state, reps=(n_samples,1)).T,
                ulists,
                dt)

print(f"Trajs {trajs.shape}")

xtraj_samples = np.zeros((n_agents * n_samples, plan_steps))
ytraj_samples = np.zeros((n_agents * n_samples, plan_steps))

# update samples with pedestrian info. for now assume none

xtraj_samples[0:n_samples] = trajs[:,0,:].T
ytraj_samples[0:n_samples] = trajs[:,1,:].T

weights = brne_nav(
            xtraj_samples, ytraj_samples,
            n_agents, plan_steps, n_samples,
            cost_a1, cost_a2, cost_a3, ped_sample_scale,
            ymin, ymax
            )
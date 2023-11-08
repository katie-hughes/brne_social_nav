import numpy as np 
import matplotlib.pyplot as plt 
import brne as brne
import time 
np.set_printoptions(suppress=True, linewidth=10000, precision=4)

dt = 0.1
num_samples = 16
plan_steps = 5
num_agents = 2
ped_sample_scale = 1.0

# agent 1 nominal trajectory
xlist_1 = np.linspace(-2, 0, plan_steps)
ylist_1 = np.ones_like(xlist_1) * -0.1

# agent 2 nominal trajectory 
xlist_2 = np.linspace(0, -2, plan_steps)
ylist_2 = np.ones_like(xlist_2) * -0.3

xmean_list = np.array([
    xlist_1, xlist_2
])
ymean_list = np.array([
    ylist_1, ylist_2
])
print(f"Xmean list\n{xmean_list}")

# we visual nominal trajectories and the environment (corridor)
fig, (ax1, ax2) = plt.subplots(1,2, sharey=True, figsize=(8,4), dpi=150)
fig.suptitle('Pure python test')
ax1.set_xlim(-3, 3)
ax1.set_ylim(-1, 1)

ax1.plot(xlist_1, ylist_1, linestyle='-', color='C0')
ax1.plot(xlist_1[0], ylist_1[0], marker='o', markersize=15, color='C0')
ax1.plot(xlist_2, ylist_2, linestyle='-', color='C1')
ax1.plot(xlist_2[0], ylist_2[0], marker='o', markersize=15, color='C1')
ax1.axhline(0.5, 0.0, 1.0, linestyle='--', color='k')
ax1.axhline(-0.5, 0.0, 1.0, linestyle='--', color='k')

# here we define kernel parameter
tlist = np.arange(plan_steps) * dt 
print(f"Tlist {tlist}")
# train_ts = np.array([tlist[0], tlist[-1]])
# train_noise = np.array([1e-03, 1e-03])

train_ts = np.array([tlist[0]])
train_noise = np.array([1e-03])

test_ts = tlist
cov_Lmat, cov_mat = brne.get_Lmat_nb(train_ts, test_ts, train_noise, 0.5, 0.2)
# this is where the verification begins
# print('verify cov_Lmat:')
# print(cov_Lmat)

# samples using the BRNE function
x_pts_1 = brne.mvn_sample_normal(num_samples, plan_steps, cov_Lmat)
y_pts_1 = brne.mvn_sample_normal(num_samples, plan_steps, cov_Lmat)
x_pts_2 = brne.mvn_sample_normal(num_samples, plan_steps, cov_Lmat)
y_pts_2 = brne.mvn_sample_normal(num_samples, plan_steps, cov_Lmat)
# two-step verification for sampling:
# (1) write the sampling code in C++, make sure
# (2) make the C++ code read in the python-generated samples
# print(f"X pts 1{x_pts_1}")
# print(f"X pts 2{x_pts_2}")
x_pts = np.vstack([x_pts_1, x_pts_2])
# print(f"X_pts {x_pts}")
y_pts = np.vstack([y_pts_1, y_pts_2])
width_scale = (0.5 + 0.5) / (y_pts.max() - y_pts.min()) 
# print('width_scale: ', width_scale)
x_pts *= width_scale
y_pts *= width_scale

# visualize samples here
for i in range(num_samples):
    ax1.plot(xlist_1 + x_pts_1[i] * width_scale, ylist_1 + y_pts_1[i] * width_scale,
            linestyle='--', color='C0')
    ax1.plot(xlist_2 + x_pts_2[i] * width_scale, ylist_2 + y_pts_2[i] * width_scale,
            linestyle='--', color='C1')

ax1.set_title("Python sampling")

ax2.set_xlim(-3, 3)
ax2.set_ylim(-1, 1)
ax2.plot(xlist_1, ylist_1, linestyle='--', color='C0')
ax2.plot(xlist_1[0], ylist_1[0], marker='o', markersize=15, color='C0')
ax2.plot(xlist_2, ylist_2, linestyle='--', color='C1')
ax2.plot(xlist_2[0], ylist_2[0], marker='o', markersize=15, color='C1')
ax2.axhline(0.5, 0.0, 1.0, linestyle='--', color='k')
ax2.axhline(-0.5, 0.0, 1.0, linestyle='--', color='k')

xtraj_samples = np.zeros((num_agents * num_samples, plan_steps))
ytraj_samples = np.zeros((num_agents * num_samples, plan_steps))

print(f"Xtraj samples\n{xtraj_samples}")

i = 0
xtraj_samples[i * num_samples:i * num_samples + num_samples] = xmean_list[i] + x_pts[i * num_samples:i * num_samples + num_samples]
ytraj_samples[i * num_samples:i * num_samples + num_samples] = ymean_list[i] + y_pts[i * num_samples:i * num_samples + num_samples]

print(f"Xmeanlisti\n{xmean_list[i]}")
print(f"Xpts\n{x_pts[i * num_samples:i * num_samples + num_samples]}")

print(f"Xtraj samples\n{xtraj_samples}")
for i in range(1, num_agents):
    xtraj_samples[i * num_samples:i * num_samples + num_samples] = xmean_list[i] + x_pts[i * num_samples:i * num_samples + num_samples] * ped_sample_scale
    ytraj_samples[i * num_samples:i * num_samples + num_samples] = ymean_list[i] + y_pts[i * num_samples:i * num_samples + num_samples] * ped_sample_scale

print(f"Test index table\n{brne.get_index_table(5)}")
# exit()
weights = brne.brne_nav(xtraj_samples=xtraj_samples,
                        ytraj_samples=ytraj_samples,
                        num_agents=num_agents,
                        tsteps=plan_steps,
                        num_pts=num_samples,
                        cost_a1=8.0,
                        cost_a2=5.0,
                        cost_a3=20.0,
                        ped_sample_scale=1.0,
                        y_min=-0.5,
                        y_max=0.5)
print(f"Weights FINAL\n{weights}")

opt_trajs_x = np.zeros((num_agents, plan_steps))
opt_trajs_y = np.zeros((num_agents, plan_steps))
for i in range(num_agents):
    agent_weights = weights[i]
    opt_trajs_x[i] = xmean_list[i] + \
        np.mean(x_pts[(i)*num_samples : (i+1)*num_samples] * agent_weights[:,np.newaxis], axis=0)
    opt_trajs_y[i] = ymean_list[i]  + \
        np.mean(y_pts[(i)*num_samples : (i+1)*num_samples] * agent_weights[:,np.newaxis], axis=0)

# visualize the final optimal trajectories
ax2.plot(opt_trajs_x[0], opt_trajs_y[0], linestyle='-', color='C0')
ax2.plot(opt_trajs_x[1], opt_trajs_y[1], linestyle='-', color='C1')
ax2.set_title('Optimal trajectories with Python')


# plt.savefig('after_corridor_avoidance.png')
# plt.show()
plt.close()

print('\n\n\n')


test_nsamples = num_samples
test_nsteps = 5

ut = np.array([0.4, 0.1])
nominal_cmds = np.tile(ut, reps=(test_nsteps,1))
ulist = brne.get_ulist_essemble(nominal_cmds, 0.6, 1.0, test_nsamples)
print(f"Ulist {ulist.shape}")
iteration = 0
for i in ulist:
    print(f'iteration {iteration}')
    print(i.shape)
    print(i)
    iteration += 1

print('\n\n')

# exit()

robot_state = np.array([20, 100, 0])

tiles = np.tile(robot_state, reps=(test_nsamples,1)).T
print(f"tiles {tiles.shape}\n{tiles}")
traj = brne.traj_sim_essemble(tiles, ulist,dt)
print(f"traj {traj.shape}")
iteration = 0
for i in traj:
    print(f'\n\n{iteration}')
    print(i.shape)
    print(i.T)
    iteration += 1

import numpy as np 
import matplotlib.pyplot as plt 
import brne as brne
import time 
np.set_printoptions(suppress=True, linewidth=10000, precision=4)

plan_steps = 5
num_samples = 3

dt = 0.1
num_agents = 2
ped_sample_scale = 1.0

# bad practice but ok
xmean_list = np.genfromtxt("../../brnelib/build/x_nominal.csv", delimiter=",")
ymean_list = np.genfromtxt("../../brnelib/build/y_nominal.csv", delimiter=",")
print(f"Xmean list\n{xmean_list}")
# we visual nominal trajectories and the environment (corridor)
fig, (ax1, ax2) = plt.subplots(1,2, sharey=True, figsize=(8,4), dpi=150)
fig.suptitle('C++ sampling test')

ax1.set_xlim(-3, 3)
ax1.set_ylim(-1, 1)

ax1.plot(xmean_list[0], ymean_list[0], linestyle='-', color='C0')
ax1.plot(xmean_list[0][0], ymean_list[0][0], marker='o', markersize=15, color='C0')
ax1.plot(xmean_list[1], ymean_list[1], linestyle='-', color='C1')
ax1.plot(xmean_list[1][0], ymean_list[1][0], marker='o', markersize=15, color='C1')
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
# two-step verification for sampling:
# (1) write the sampling code in C++, make sure
# (2) make the C++ code read in the python-generated samples
x_pts = np.genfromtxt("../../brnelib/build/x_samples.csv", delimiter=",")
y_pts = np.genfromtxt("../../brnelib/build/y_samples.csv", delimiter=",")
width_scale = (0.5 + 0.5) / (y_pts.max() - y_pts.min()) 
print('width_scale: ', width_scale)
x_pts *= width_scale
y_pts *= width_scale

# visualize samples here
for i in range(num_samples):
    ax1.plot(xmean_list[0] + x_pts[i] * width_scale, ymean_list[0] + y_pts[i] * width_scale,
            linestyle='--', color='C0')
    ax1.plot(xmean_list[1] + x_pts[num_samples + i] * width_scale, ymean_list[1] + y_pts[num_samples + i] * width_scale,
            linestyle='--', color='C1')

ax1.set_title('C++ sampling')

# exit()

ax2.set_xlim(-3, 3)
ax2.set_ylim(-1, 1)

ax2.plot(xmean_list[0], ymean_list[0], linestyle='--', color='C0')
ax2.plot(xmean_list[0][0], ymean_list[0][0], marker='o', markersize=15, color='C0')

ax2.plot(xmean_list[1], ymean_list[1], linestyle='--', color='C1')
ax2.plot(xmean_list[1][0], ymean_list[1][0], marker='o', markersize=15, color='C1')

ax2.axhline(0.5, 0.0, 1.0, linestyle='--', color='k')
ax2.axhline(-0.5, 0.0, 1.0, linestyle='--', color='k')

xtraj_samples = np.zeros((num_agents * num_samples, plan_steps))
ytraj_samples = np.zeros((num_agents * num_samples, plan_steps))

i = 0
xtraj_samples[i * num_samples:i * num_samples + num_samples] = xmean_list[i] + x_pts[i * num_samples:i * num_samples + num_samples]
ytraj_samples[i * num_samples:i * num_samples + num_samples] = ymean_list[i] + y_pts[i * num_samples:i * num_samples + num_samples]

for i in range(1, num_agents):
    xtraj_samples[i * num_samples:i * num_samples + num_samples] = xmean_list[i] + x_pts[i * num_samples:i * num_samples + num_samples] * ped_sample_scale
    ytraj_samples[i * num_samples:i * num_samples + num_samples] = ymean_list[i] + y_pts[i * num_samples:i * num_samples + num_samples] * ped_sample_scale

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
# print(f"Weights FINAL\n{weights}")

opt_trajs_x = np.zeros((num_agents, plan_steps))
opt_trajs_y = np.zeros((num_agents, plan_steps))
for i in range(num_agents):
    agent_weights = weights[i]
    opt_trajs_x[i] = xmean_list[i] + \
        np.mean(x_pts[(i)*num_samples : (i+1)*num_samples] * agent_weights[:,np.newaxis], axis=0)
    print(f"\nFirst\n{x_pts[(i)*num_samples : (i+1)*num_samples]}")
    print(f"Second\n{agent_weights[:,np.newaxis]}")
    print(f"Product\n{x_pts[(i)*num_samples : (i+1)*num_samples] * agent_weights[:,np.newaxis]}")
    print(f"Mean\n{np.mean(x_pts[(i)*num_samples : (i+1)*num_samples] * agent_weights[:,np.newaxis], axis=0)}")
    opt_trajs_y[i] = ymean_list[i]  + \
        np.mean(y_pts[(i)*num_samples : (i+1)*num_samples] * agent_weights[:,np.newaxis], axis=0)

print(f"Opt traj x {opt_trajs_x}")
print(f"Opt traj y {opt_trajs_y}")

# visualize the final optimal trajectories
ax2.plot(opt_trajs_x[0], opt_trajs_y[0], linestyle='-', color='C0')
ax2.plot(opt_trajs_x[1], opt_trajs_y[1], linestyle='-', color='C1')
ax2.set_title('Optimal trajectories with Python')

# plt.savefig('after_corridor_avoidance.png')
plt.show()
plt.close()
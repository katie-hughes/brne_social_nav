import numpy as np 
import matplotlib.pyplot as plt 
import brne as brne
import time 
np.set_printoptions(suppress=True)

dt = 0.1
num_samples = 5
plan_steps = 10
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

# we visual nominal trajectories and the environment (corridor)
fig, ax = plt.subplots(1, 1, dpi=150)
ax.set_xlim(-3, 3)
ax.set_ylim(-1, 1)

ax.plot(xlist_1, ylist_1, linestyle='-', color='C0')
ax.plot(xlist_1[0], ylist_1[0], marker='o', markersize=15, color='C0')

ax.plot(xlist_2, ylist_2, linestyle='-', color='C1')
ax.plot(xlist_2[0], ylist_2[0], marker='o', markersize=15, color='C1')

ax.axhline(0.5, 0.0, 1.0, linestyle='--', color='k')
ax.axhline(-0.5, 0.0, 1.0, linestyle='--', color='k')

# here we define kernel parameter
tlist = np.arange(plan_steps) * dt 
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

x_pts = np.vstack([x_pts_1, x_pts_2])
y_pts = np.vstack([y_pts_1, y_pts_2])
width_scale = (0.5 + 0.5) / (y_pts.max() - y_pts.min()) 
print('width_scale: ', width_scale)
x_pts *= width_scale
y_pts *= width_scale

# visualize samples here
for i in range(num_samples):
    ax.plot(xlist_1 + x_pts_1[i] * width_scale, ylist_1 + y_pts_1[i] * width_scale,
            linestyle='--', color='C0')
    ax.plot(xlist_2 + x_pts_2[i] * width_scale, ylist_2 + y_pts_2[i] * width_scale,
            linestyle='--', color='C1')

plt.show()
plt.close()


# exit()

fig, ax = plt.subplots(1, 1, dpi=150)
ax.set_xlim(-3, 3)
ax.set_ylim(-1, 1)

ax.plot(xlist_1, ylist_1, linestyle='--', color='C0')
ax.plot(xlist_1[0], ylist_1[0], marker='o', markersize=15, color='C0')

ax.plot(xlist_2, ylist_2, linestyle='--', color='C1')
ax.plot(xlist_2[0], ylist_2[0], marker='o', markersize=15, color='C1')

ax.axhline(0.5, 0.0, 1.0, linestyle='--', color='k')
ax.axhline(-0.5, 0.0, 1.0, linestyle='--', color='k')

all_pt_index = np.arange(num_agents * num_samples).reshape(num_agents, num_samples)

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
print(f"Weights FINAL\n{weights}")

coll_mask = brne.coll_beck(ytraj_samples, -0.5, 0.5).all(axis=1).astype(float).reshape(num_agents, num_samples)
opt_trajs_x = np.zeros((num_agents, plan_steps))
opt_trajs_y = np.zeros((num_agents, plan_steps))
for i in range(num_agents):
    opt_trajs_x[i] = \
        np.mean(xtraj_samples[(i)*num_samples : (i+1)*num_samples] * weights[i][:,np.newaxis], axis=0)
    opt_trajs_y[i] = \
        np.mean(ytraj_samples[(i)*num_samples : (i+1)*num_samples] * weights[i][:,np.newaxis], axis=0)


# visualize the final optimal trajectories
ax.plot(opt_trajs_x[0], opt_trajs_y[0], linestyle='-', color='C0')
ax.plot(opt_trajs_x[1], opt_trajs_x[1], linestyle='-', color='C1')
ax.set_title('New Test')

# plt.savefig('after_corridor_avoidance.png')
plt.show()
plt.close()
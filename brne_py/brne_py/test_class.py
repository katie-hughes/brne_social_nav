from brne_class import BRNE, Dyn
import numpy as np
np.set_printoptions(suppress=True)


plan_steps = 25
dt = 0.1
kernel_a1 = 0.2
kernel_a2 = 0.2
cost_a1 = 4.0
cost_a2 = 1.0
cost_a3 = 80.0
n_samples = 196

max_ang_vel = 1.0
max_lin_vel = 0.6
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

n_agents = len(peds) + 1


brne = BRNE(kernel_a1=kernel_a1,
            kernel_a2=kernel_a2,
            cost_a1=cost_a1,
            cost_a2=cost_a2,
            cost_a3=cost_a3,
            dt=dt,
            plan_steps=plan_steps,
            n_samples=n_samples,
            max_ang_vel=max_ang_vel,
            max_lin_vel=max_lin_vel,
            nominal_vel=nominal_vel,
            ymin=ymin,
            ymax=ymax
            )
        
print(f"Tlist {brne.tlist.shape}")
print(f"train ts {brne.train_ts.shape}")
print(f"test ts {brne.test_ts.shape}")
print(f"Train noise {brne.train_noise.shape}")
print(f"Cov lmat {brne.cov_Lmat.shape}")
print(f"Cov mat {brne.cov_mat.shape}")


brne.update(state=np.array([0,0,0]))

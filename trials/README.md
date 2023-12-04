# Trial Results

## Small Hallway 

Goal pose set to (6,0) from starting square:

```
ros2 service call /set_goal_pose crowd_nav_interfaces/srv/GoalReq "x: 6.0        
y: 0.0" 
```

BRNE Params:

```
replan_freq: 10.0 
dt: 0.1
maximum_agents: 5 
n_samples: 196 
n_steps: 25 
max_lin_vel: 0.6 
max_ang_vel: 1.0 
nominal_lin_vel: 0.4 
open_space_velocity: 0.6 
kernel_a1: 0.2 
kernel_a2: 0.2 
cost_a1: 15.0 
cost_a2: 3.0 
cost_a3: 20.0 
ped_sample_scale: 0.1 
people_timeout: 0.5  
y_min: -0.75 
y_max:  0.75 
people_timeout_off: true
close_stop_threshold: 0.5 
brne_activate_threshold: 6.0 
goal_threshold: 0.5 
```

### Test 1: One pedestrian standing on blue X

All goals were reached

| Trial | Time        | Straight Line Path Distance | Path Distance | Path Ratio | Closest dist to ped | # of E-STOPs|
| ----- | ----------- | --------------------------- | ------------- | ---------- | ------------------- | ----------- |
|  1    |  11.076 s   |    5.55326 m                |   5.8623 m    |  1.05565   |  1.33483 m          |  0          |
|  2    |  11.0081 s  |    5.55987 m                |   5.91017 m   |  1.063     |  1.36396 m          |  0          |
|  3    |  11.0858 s  |    5.55203 m                |   5.83696 m   |  1.05132   |  1.35577 m          |  0          |
| 4 |  10.9634 |  5.55233 |  5.83745 |  1.05135 |  1.32836 |  0 | 
| 5 |  10.914 |  5.55035 |  5.74271 |  1.03466 |  1.54368 |  0 | 



### Test 2: One Ped on blue x, one ped on Pink X

| Trial | Time | Straight Line Path Distance | Path Distance | Path Ratio | Closest Dist To Pedestrian | Number Of E-STOPs | 
| ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| 1 |  14.9403 |  5.84514 |  6.83149 |  1.16875 |  1.33692 |  0 | 
| 2 |  14.9403 |  5.84514 |  6.83149 |  1.16875 |  1.33692 |  0 | 
| 3 |  13.5654 |  5.7647 |  6.25842 |  1.08565 |  1.41831 |  0 | 
| 4 |  13.6856 |  5.76291 |  6.32004 |  1.09667 |  1.39749 |  0 | 
| 5 |  14.2977 |  5.82363 |  6.81682 |  1.17055 |  1.28643 |  0 |

Something got messed up here as trial 1 and 2 have the same data but as my screen recordings got corrupted I can't say exactly what. 


### Test 3: One Pedestrian walking straight down the right hand side of squares

| Trial | Time | Straight Line Path Distance | Path Distance | Path Ratio | Closest Dist To Pedestrian | Number Of E-STOPs | 
| ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| 1 |  11.9856 |  5.62163 |  5.83216 |  1.03745 |  3.07084 |  0 | 
| 2 |  11.6663 |  5.56225 |  5.77776 |  1.03875 |  2.40846 |  0 | 
| 3 |  11.7836 |  5.5596 |  5.70086 |  1.02541 |  2.31984 |  6 | 
| 4 |  12.5897 |  5.53588 |  5.89053 |  1.06406 |  2.45876 |  0 | 
| 5 |  11.9793 |  5.54135 |  5.89904 |  1.06455 |  2.27335 |  0 |


### Test 4: Two walking down hallway, slightly spaced out

| Trial | Time | Straight Line Path Distance | Path Distance | Path Ratio | Closest Dist To Pedestrian | Number Of E-STOPs | 
| ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| 1 |  23.6915 |  5.5456 |  5.92541 |  1.06849 |  1.84429 |  110 | 
| 2 |  12.7445 |  5.73958 |  6.1234 |  1.06687 |  2.79309 |  0 | 
| 3 |  12.1764 |  5.72099 |  5.99224 |  1.04741 |  2.46016 |  0 | 

Didn't have time to do the last two tests as the wifi hotspot died and the dog was overheating

### Cool Test with 4 people

```

[brne_nav-1] [INFO] [1701472153.590476969] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701472153.590557836] [brne]: =========================================================
[brne_nav-1] [INFO] [1701472153.590577987] [brne]: Time: 17.7492 s
[brne_nav-1] [INFO] [1701472153.590592592] [brne]: Straight Line Path: 5.52631 m
[brne_nav-1] [INFO] [1701472153.591006103] [brne]: Trial Path: 6.81133 m
[brne_nav-1] [INFO] [1701472153.591021876] [brne]: Path Ratio: 1.23253
[brne_nav-1] [INFO] [1701472153.591034552] [brne]: Closest Dist to Ped: 1.66046 m
[brne_nav-1] [INFO] [1701472153.591127408] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701472153.591139160] [brne]: =========================================================

```
# Small Hallway 

set goal pose to (6,0) from starting square

```
ros2 service call /set_goal_pose crowd_nav_interfaces/srv/GoalReq "x: 6.0        
y: 0.0" 
```

BRNE Params

```
    replan_freq: 10.0  # unit: Hz
    dt: 0.1
    maximum_agents: 5  # maximum number of agents BRNE will consider (including the robot)
    n_samples: 196  # number of samples assigned to each agent. needs to be the square of an even number
    n_steps: 25  # time steps of the planning horizon
    max_lin_vel: 0.6  # 0.6,0.8 maximum linear velocity allowed on the robot
    max_ang_vel: 1.0  # 1.0,1.2 maximum angular velocity allowed on the robot
    nominal_lin_vel: 0.4  #0.4,0.5 nomimal (linear) velocity when plannig the initial trajectory
    open_space_velocity: 0.6  # nominal velocity when the robot is in open space
    kernel_a1: 0.2  # control the "straightness" of trajectory samples. Larger the value is, less straight the trajectory sample will be.
    kernel_a2: 0.2  # control the "width/spreadness" of trajectory samples. Larger the value is, more spread the trajectory samples are.
    # a1 probably makes safety zone narrower. try tuning a1 and a3
    cost_a1: 15.0  # control the safety zone, smaller the value is, more conversative the robot will be.
    cost_a2: 3.0  # control the safety zone, larger the value is, more conservative the robot will be.
    cost_a3: 20.0  #  control the safety penalty weight, larger the value is, more conservative the robot will be.
    ped_sample_scale: 0.1  # pedestrian's willingness for cooperation, default value is 1.0, the smaller it is, the less the robot would expect the pedestrians to make space for it
    people_timeout: 0.5  # unit: seconds
    y_min: -0.75  # lower bound of y coordinate (one side of corridor)
    y_max:  0.75  # upper bound of y coordinate (the other side of corridor)
    people_timeout_off: true
    close_stop_threshold: 0.5  # threshold for safety mask, leading to estop
    brne_activate_threshold: 6.0  # distance threshold from a pedestrian to enable BRNE
    goal_threshold: 0.5 # threshold for reaching the goal position (m)
```

## Test 1: One pedestrian standing on blue X

Trial 1
```
[brne]: Goal Reached!
[brne_nav-1] [INFO] [1701466941.283024159] [brne]: =========================================================
[brne_nav-1] [INFO] [1701466941.283040446] [brne]: Time: 11.0796 s
[brne_nav-1] [INFO] [1701466941.283051634] [brne]: Straight Line Path: 5.55326 m
[brne_nav-1] [INFO] [1701466941.283062317] [brne]: Trial Path: 5.8623 m
[brne_nav-1] [INFO] [1701466941.283072206] [brne]: Path Ratio: 1.05565
[brne_nav-1] [INFO] [1701466941.283082085] [brne]: Closest Dist to Ped: 1.33483 m
[brne_nav-1] [INFO] [1701466941.283091948] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701466941.283100874] [brne]: =========================================================

```

Trial 2
```
[brne_nav-1] [INFO] [1701467246.918303819] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701467246.918358684] [brne]: =========================================================
[brne_nav-1] [INFO] [1701467246.918375466] [brne]: Time: 11.0081 s
[brne_nav-1] [INFO] [1701467246.918384609] [brne]: Straight Line Path: 5.55987 m
[brne_nav-1] [INFO] [1701467246.918393082] [brne]: Trial Path: 5.91017 m
[brne_nav-1] [INFO] [1701467246.918400584] [brne]: Path Ratio: 1.063
[brne_nav-1] [INFO] [1701467246.918408139] [brne]: Closest Dist to Ped: 1.36396 m
[brne_nav-1] [INFO] [1701467246.918427583] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701467246.918437088] [brne]: =========================================================
```

Trial 3

```
[brne_nav-1] [INFO] [1701467435.239112756] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701467435.239173575] [brne]: =========================================================
[brne_nav-1] [INFO] [1701467435.239190358] [brne]: Time: 11.0858 s
[brne_nav-1] [INFO] [1701467435.239201960] [brne]: Straight Line Path: 5.55203 m
[brne_nav-1] [INFO] [1701467435.239223015] [brne]: Trial Path: 5.83696 m
[brne_nav-1] [INFO] [1701467435.239237267] [brne]: Path Ratio: 1.05132
[brne_nav-1] [INFO] [1701467435.239250771] [brne]: Closest Dist to Ped: 1.35577 m
[brne_nav-1] [INFO] [1701467435.239264140] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701467435.239276711] [brne]: =========================================================

```

Trial 4
```
[brne_nav-1] [INFO] [1701467610.542835254] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701467610.542890849] [brne]: =========================================================
[brne_nav-1] [INFO] [1701467610.542906862] [brne]: Time: 10.9634 s
[brne_nav-1] [INFO] [1701467610.542918568] [brne]: Straight Line Path: 5.55233 m
[brne_nav-1] [INFO] [1701467610.542930172] [brne]: Trial Path: 5.83745 m
[brne_nav-1] [INFO] [1701467610.542941089] [brne]: Path Ratio: 1.05135
[brne_nav-1] [INFO] [1701467610.542952180] [brne]: Closest Dist to Ped: 1.32836 m
[brne_nav-1] [INFO] [1701467610.542962913] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701467610.542972559] [brne]: =========================================================

```

Trial 5

```
[brne_nav-1] [INFO] [1701467804.876945081] [brne]: Goal Received: 6, 0
[brne_nav-1] [INFO] [1701467815.791014079] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701467815.791109651] [brne]: =========================================================
[brne_nav-1] [INFO] [1701467815.791142304] [brne]: Time: 10.914 s
[brne_nav-1] [INFO] [1701467815.791162685] [brne]: Straight Line Path: 5.55035 m
[brne_nav-1] [INFO] [1701467815.791181493] [brne]: Trial Path: 5.74271 m
[brne_nav-1] [INFO] [1701467815.791197285] [brne]: Path Ratio: 1.03466
[brne_nav-1] [INFO] [1701467815.791213148] [brne]: Closest Dist to Ped: 1.54368 m
[brne_nav-1] [INFO] [1701467815.791228929] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701467815.791243729] [brne]: =========================================================

```


## One Ped on blue x, one ped on Pink X


Trial 1: failure??? 
```
[brne_nav-1] [WARN] [1701471809.521703797] [brne]: No path found -- stopping navigation to this goal.

```


Trial 2

```
[brne_nav-1] [INFO] [1701472015.680164417] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701472015.680226032] [brne]: =========================================================
[brne_nav-1] [INFO] [1701472015.680246318] [brne]: Time: 14.496 s
[brne_nav-1] [INFO] [1701472015.680257968] [brne]: Straight Line Path: 5.78871 m
[brne_nav-1] [INFO] [1701472015.680270498] [brne]: Trial Path: 6.63759 m
[brne_nav-1] [INFO] [1701472015.680281123] [brne]: Path Ratio: 1.14664
[brne_nav-1] [INFO] [1701472015.680291390] [brne]: Closest Dist to Ped: 1.13486 m
[brne_nav-1] [INFO] [1701472015.680302275] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701472015.680311107] [brne]: =========================================================

```

   
## Test 3: One Pedestrian walking straight down the right hand side of squares

Trial 1

```
[brne_nav-1] [INFO] [1701475671.189178135] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701475671.189262739] [brne]: =========================================================
[brne_nav-1] [INFO] [1701475671.189291113] [brne]: Time: 11.9856 s
[brne_nav-1] [INFO] [1701475671.189308074] [brne]: Straight Line Path: 5.62163 m
[brne_nav-1] [INFO] [1701475671.189330668] [brne]: Trial Path: 5.83216 m
[brne_nav-1] [INFO] [1701475671.189346398] [brne]: Path Ratio: 1.03745
[brne_nav-1] [INFO] [1701475671.189364668] [brne]: Closest Dist to Ped: 3.07084 m
[brne_nav-1] [INFO] [1701475671.189380597] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701475671.189394784] [brne]: =========================================================

```


Trial 2

```
[brne_nav-1] [INFO] [1701475829.506781352] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701475829.506835991] [brne]: =========================================================
[brne_nav-1] [INFO] [1701475829.506854312] [brne]: Time: 11.6663 s
[brne_nav-1] [INFO] [1701475829.506866582] [brne]: Straight Line Path: 5.56225 m
[brne_nav-1] [INFO] [1701475829.506878949] [brne]: Trial Path: 5.77776 m
[brne_nav-1] [INFO] [1701475829.506890474] [brne]: Path Ratio: 1.03875
[brne_nav-1] [INFO] [1701475829.506903593] [brne]: Closest Dist to Ped: 2.40846 m
[brne_nav-1] [INFO] [1701475829.506914930] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701475829.506925461] [brne]: =========================================================

```


Trial 3

```
[brne_nav-1] [INFO] [1701475948.716996593] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701475948.717085043] [brne]: =========================================================
[brne_nav-1] [INFO] [1701475948.717113321] [brne]: Time: 11.7836 s
[brne_nav-1] [INFO] [1701475948.717125477] [brne]: Straight Line Path: 5.5596 m
[brne_nav-1] [INFO] [1701475948.717138159] [brne]: Trial Path: 5.70086 m
[brne_nav-1] [INFO] [1701475948.717148925] [brne]: Path Ratio: 1.02541
[brne_nav-1] [INFO] [1701475948.717159604] [brne]: Closest Dist to Ped: 2.31984 m
[brne_nav-1] [INFO] [1701475948.717171104] [brne]: Number of E-STOPs: 6
[brne_nav-1] [INFO] [1701475948.717180555] [brne]: =========================================================

```

Trial 4

```
[brne_nav-1] [INFO] [1701476150.660465504] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701476150.660561078] [brne]: =========================================================
[brne_nav-1] [INFO] [1701476150.660591221] [brne]: Time: 12.5897 s
[brne_nav-1] [INFO] [1701476150.660610214] [brne]: Straight Line Path: 5.53588 m
[brne_nav-1] [INFO] [1701476150.660627359] [brne]: Trial Path: 5.89053 m
[brne_nav-1] [INFO] [1701476150.660642393] [brne]: Path Ratio: 1.06406
[brne_nav-1] [INFO] [1701476150.660655616] [brne]: Closest Dist to Ped: 2.45876 m
[brne_nav-1] [INFO] [1701476150.660669208] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701476150.660680877] [brne]: =========================================================

```


Trial 5

```
[brne_nav-1] [INFO] [1701476321.073141273] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701476321.073230592] [brne]: =========================================================
[brne_nav-1] [INFO] [1701476321.073262127] [brne]: Time: 11.9793 s
[brne_nav-1] [INFO] [1701476321.073280870] [brne]: Straight Line Path: 5.54135 m
[brne_nav-1] [INFO] [1701476321.073298889] [brne]: Trial Path: 5.89904 m
[brne_nav-1] [INFO] [1701476321.073315894] [brne]: Path Ratio: 1.06455
[brne_nav-1] [INFO] [1701476321.073332686] [brne]: Closest Dist to Ped: 2.27335 m
[brne_nav-1] [INFO] [1701476321.073349584] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701476321.073364468] [brne]: =========================================================

```


## Two walking down hallway, slightly spaced out





Test with 4 people


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
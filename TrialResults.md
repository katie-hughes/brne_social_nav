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
   
## one walking on the hallway on the right hand side
Trial 1

```
[brne_nav-1] [INFO] [1701469799.846495893] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701469799.846563898] [brne]: =========================================================
[brne_nav-1] [INFO] [1701469799.846588373] [brne]: Time: 12.5086 s
[brne_nav-1] [INFO] [1701469799.846603763] [brne]: Straight Line Path: 5.53289 m
[brne_nav-1] [INFO] [1701469799.846619096] [brne]: Trial Path: 5.94115 m
[brne_nav-1] [INFO] [1701469799.846632886] [brne]: Path Ratio: 1.07379
[brne_nav-1] [INFO] [1701469799.846646889] [brne]: Closest Dist to Ped: 2.38545 m
[brne_nav-1] [INFO] [1701469799.846661130] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701469799.846673474] [brne]: =========================================================

```

Trial 2

```
[brne_nav-1] [INFO] [1701469972.284623273] [brne]: =========================================================
[brne_nav-1] [INFO] [1701469972.284901325] [brne]: Time: 11.6943 s
[brne_nav-1] [INFO] [1701469972.285298306] [brne]: Straight Line Path: 5.5139 m
[brne_nav-1] [INFO] [1701469972.296938733] [brne]: Trial Path: 5.75984 m
[brne_nav-1] [INFO] [1701469972.298332272] [brne]: Path Ratio: 1.0446
[brne_nav-1] [INFO] [1701469972.298867302] [brne]: Closest Dist to Ped: 2.46476 m
[brne_nav-1] [INFO] [1701469972.299232188] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701469972.299609150] [brne]: =========================================================
```

Trial 3

```
[brne_nav-1] [INFO] [1701470124.868412220] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701470124.869031973] [brne]: =========================================================
[brne_nav-1] [INFO] [1701470124.869365211] [brne]: Time: 11.7817 s
[brne_nav-1] [INFO] [1701470124.869620649] [brne]: Straight Line Path: 5.57591 m
[brne_nav-1] [INFO] [1701470124.869860713] [brne]: Trial Path: 5.74593 m
[brne_nav-1] [INFO] [1701470124.870098802] [brne]: Path Ratio: 1.03049
[brne_nav-1] [INFO] [1701470124.870576522] [brne]: Closest Dist to Ped: 2.15796 m
[brne_nav-1] [INFO] [1701470124.871010863] [brne]: Number of E-STOPs: 6
[brne_nav-1] [INFO] [1701470124.871421535] [brne]: =========================================================

```


Trial 4
```
[brne_nav-1] [INFO] [1701470366.805354369] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701470366.805448598] [brne]: =========================================================
[brne_nav-1] [INFO] [1701470366.805470263] [brne]: Time: 11.6882 s
[brne_nav-1] [INFO] [1701470366.805482622] [brne]: Straight Line Path: 5.54166 m
[brne_nav-1] [INFO] [1701470366.805494770] [brne]: Trial Path: 5.75213 m
[brne_nav-1] [INFO] [1701470366.805505549] [brne]: Path Ratio: 1.03798
[brne_nav-1] [INFO] [1701470366.805516263] [brne]: Closest Dist to Ped: 2.69147 m
[brne_nav-1] [INFO] [1701470366.805530059] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701470366.805539889] [brne]: =========================================================

```




4. two walking down the hallway down the dogs
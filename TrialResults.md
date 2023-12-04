# Small Hallway 

set goal pose to (6,0) from starting square

```
ros2 service call /set_goal_pose crowd_nav_interfaces/srv/GoalReq "x: 6.0        
y: 0.0" 
```

BRNE Params

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


## Test 2: One Ped on blue x, one ped on Pink X


Trial 1
```
[brne_nav-1] [INFO] [1701478481.914657806] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701478481.914715373] [brne]: =========================================================
[brne_nav-1] [INFO] [1701478481.914733129] [brne]: Time: 14.9403 s
[brne_nav-1] [INFO] [1701478481.914742260] [brne]: Straight Line Path: 5.84514 m
[brne_nav-1] [INFO] [1701478481.914751080] [brne]: Trial Path: 6.83149 m
[brne_nav-1] [INFO] [1701478481.914758817] [brne]: Path Ratio: 1.16875
[brne_nav-1] [INFO] [1701478481.914766673] [brne]: Closest Dist to Ped: 1.33692 m
[brne_nav-1] [INFO] [1701478481.914775766] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701478481.914782927] [brne]: =========================================================
```


Trial 2
```
[brne_nav-1] [INFO] [1701478481.914657806] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701478481.914715373] [brne]: =========================================================
[brne_nav-1] [INFO] [1701478481.914733129] [brne]: Time: 14.9403 s
[brne_nav-1] [INFO] [1701478481.914742260] [brne]: Straight Line Path: 5.84514 m
[brne_nav-1] [INFO] [1701478481.914751080] [brne]: Trial Path: 6.83149 m
[brne_nav-1] [INFO] [1701478481.914758817] [brne]: Path Ratio: 1.16875
[brne_nav-1] [INFO] [1701478481.914766673] [brne]: Closest Dist to Ped: 1.33692 m
[brne_nav-1] [INFO] [1701478481.914775766] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701478481.914782927] [brne]: =========================================================

```

Trial 3

```
[brne_nav-1] [INFO] [1701478843.442503701] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701478843.442555453] [brne]: =========================================================
[brne_nav-1] [INFO] [1701478843.442572091] [brne]: Time: 13.5654 s
[brne_nav-1] [INFO] [1701478843.442579957] [brne]: Straight Line Path: 5.7647 m
[brne_nav-1] [INFO] [1701478843.442587506] [brne]: Trial Path: 6.25842 m
[brne_nav-1] [INFO] [1701478843.442594288] [brne]: Path Ratio: 1.08565
[brne_nav-1] [INFO] [1701478843.442601300] [brne]: Closest Dist to Ped: 1.41831 m
[brne_nav-1] [INFO] [1701478843.442608916] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701478843.442615236] [brne]: =========================================================

```

Trial 4

```
[brne_nav-1] [INFO] [1701478987.655391105] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701478987.655461356] [brne]: =========================================================
[brne_nav-1] [INFO] [1701478987.655482459] [brne]: Time: 13.6856 s
[brne_nav-1] [INFO] [1701478987.655494113] [brne]: Straight Line Path: 5.76291 m
[brne_nav-1] [INFO] [1701478987.655504788] [brne]: Trial Path: 6.32004 m
[brne_nav-1] [INFO] [1701478987.655513735] [brne]: Path Ratio: 1.09667
[brne_nav-1] [INFO] [1701478987.655522631] [brne]: Closest Dist to Ped: 1.39749 m
[brne_nav-1] [INFO] [1701478987.655532322] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701478987.655540630] [brne]: =========================================================

```

Trial 5

```
[brne_nav-1] [INFO] [1701479188.079573299] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701479188.079640171] [brne]: =========================================================
[brne_nav-1] [INFO] [1701479188.079662028] [brne]: Time: 14.2977 s
[brne_nav-1] [INFO] [1701479188.079675869] [brne]: Straight Line Path: 5.82363 m
[brne_nav-1] [INFO] [1701479188.079688885] [brne]: Trial Path: 6.81682 m
[brne_nav-1] [INFO] [1701479188.079700886] [brne]: Path Ratio: 1.17055
[brne_nav-1] [INFO] [1701479188.079716968] [brne]: Closest Dist to Ped: 1.28643 m
[brne_nav-1] [INFO] [1701479188.079728891] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701479188.079739491] [brne]: =========================================================

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


## Test 4: Two walking down hallway, slightly spaced out


Trial 1

```
[brne_nav-1] [INFO] [1701479529.443674546] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701479529.444585525] [brne]: =========================================================
[brne_nav-1] [INFO] [1701479529.445035967] [brne]: Time: 23.6915 s
[brne_nav-1] [INFO] [1701479529.445644304] [brne]: Straight Line Path: 5.5456 m
[brne_nav-1] [INFO] [1701479529.446024679] [brne]: Trial Path: 5.92541 m
[brne_nav-1] [INFO] [1701479529.446350927] [brne]: Path Ratio: 1.06849
[brne_nav-1] [INFO] [1701479529.446668147] [brne]: Closest Dist to Ped: 1.84429 m
[brne_nav-1] [INFO] [1701479529.446943551] [brne]: Number of E-STOPs: 110
[brne_nav-1] [INFO] [1701479529.447221924] [brne]: =========================================================

```

Trial 2

```
[brne_nav-1] [INFO] [1701479770.769069578] [brne]: Goal Reached!
[brne_nav-1] [INFO] [1701479770.769176954] [brne]: =========================================================
[brne_nav-1] [INFO] [1701479770.769213902] [brne]: Time: 12.7445 s
[brne_nav-1] [INFO] [1701479770.769236950] [brne]: Straight Line Path: 5.73958 m
[brne_nav-1] [INFO] [1701479770.769256612] [brne]: Trial Path: 6.1234 m
[brne_nav-1] [INFO] [1701479770.769275728] [brne]: Path Ratio: 1.06687
[brne_nav-1] [INFO] [1701479770.769295328] [brne]: Closest Dist to Ped: 2.79309 m
[brne_nav-1] [INFO] [1701479770.769317203] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701479770.769334096] [brne]: =========================================================
```

Trial 3

```
[brne_nav-1] [INFO] [1701479952.082634830] [brne]: =========================================================
[brne_nav-1] [INFO] [1701479952.082657109] [brne]: Time: 12.1764 s
[brne_nav-1] [INFO] [1701479952.082671588] [brne]: Straight Line Path: 5.72099 m
[brne_nav-1] [INFO] [1701479952.082685326] [brne]: Trial Path: 5.99224 m
[brne_nav-1] [INFO] [1701479952.082698387] [brne]: Path Ratio: 1.04741
[brne_nav-1] [INFO] [1701479952.082714608] [brne]: Closest Dist to Ped: 2.46016 m
[brne_nav-1] [INFO] [1701479952.082742280] [brne]: Number of E-STOPs: 0
[brne_nav-1] [INFO] [1701479952.082752995] [brne]: =========================================================

```

## Cool Test with 4 people


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
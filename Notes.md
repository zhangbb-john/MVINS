- [1. Debug](#1-debug)
  - [1.1. AUV mode](#11-auv-mode)
  - [1.2. TANK1](#12-tank1)
    - [1.2.1. vins\_estimator/launch/auv\_right.launch](#121-vins_estimatorlaunchauv_rightlaunch)
    - [1.2.2. vins\_estimator/src/estimator.h](#122-vins_estimatorsrcestimatorh)
    - [1.2.3. vins\_estimator/src/estimator.cpp](#123-vins_estimatorsrcestimatorcpp)
    - [1.2.4. feature\_tracker/src/feature\_tracker.cpp](#124-feature_trackersrcfeature_trackercpp)
    - [1.2.5. pose\_graph/src/pose\_graph.h](#125-pose_graphsrcpose_graphh)
    - [1.2.6. pose\_graph/src/pose\_graph.c](#126-pose_graphsrcpose_graphc)
    - [1.2.7. run\_auv\_loop.sh](#127-run_auv_loopsh)
  - [1.3. TANK2](#13-tank2)
  - [1.4. FIELD](#14-field)
    - [1.4.1. vins\_estimator/src/estimator.h](#141-vins_estimatorsrcestimatorh)
    - [1.4.2. pose\_graph/src/pose\_graph.h](#142-pose_graphsrcpose_graphh)
    - [1.4.3. feature\_tracker/src/feature\_tracker.cpp](#143-feature_trackersrcfeature_trackercpp)
- [2. Steps conversion from mvi-slam benchmark](#2-steps-conversion-from-mvi-slam-benchmark)
  - [2.1. estimator.c](#21-estimatorc)

# 1. Debug
## 1.1. AUV mode 
```
        para_Feature[feature_index][0] = 1.0 / 2.1;
        problem.AddParameterBlock(para_Feature[feature_index], 1);

        problem.SetParameterBlockConstant(para_Feature[feature_index]);

```


## 1.2. TANK1
### 1.2.1. vins_estimator/launch/auv_right.launch
auv_enhance1030

 /media/ubuntu/Bingbing-Data2/MVINS-Data/20231020-20231105-idsse-auv-test/scene_2023-10-30-12-05-20-time-16000-17650-modified.bag -r 1"

 recordd tank1
### 1.2.2. vins_estimator/src/estimator.h
#define AUV
### 1.2.3. vins_estimator/src/estimator.cpp
        para_Feature[feature_index][0] = 1.0 / 2.15;//2.25
### 1.2.4. feature_tracker/src/feature_tracker.cpp
            double threshold = 50;//AUV mode 50
### 1.2.5. pose_graph/src/pose_graph.h
#define FRAME_NUM 8
#define THRE_IMG 0.016//field0.032//tank2 0.011

1e6 - 45//mag threshold
### 1.2.6. pose_graph/src/pose_graph.c
    if (flag_detect_loop && (global_index % 3) == 0)
    if (flag_detect_loop && (global_index % 1) == 0)

### 1.2.7. run_auv_loop.sh
    sleep 1800;

## 1.3. TANK2


## 1.4. FIELD
### 1.4.1. vins_estimator/src/estimator.h
//#define AUV
### 1.4.2. pose_graph/src/pose_graph.h
#define FRAME_NUM 25//tank1 10//field 25
#define THRE_IMG  0.03// 0


### 1.4.3. feature_tracker/src/feature_tracker.cpp
            double threshold = 5;//AUV mode 50

# 2. Steps conversion from mvi-slam benchmark
## 2.1. estimator.c

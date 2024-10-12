
## AUV mode 
```
        para_Feature[feature_index][0] = 1.0 / 2.1;
        problem.AddParameterBlock(para_Feature[feature_index], 1);

        problem.SetParameterBlockConstant(para_Feature[feature_index]);

```


## TANK1
### vins_estimator/launch/auv_right.launch
auv_enhance1030

 /media/ubuntu/Bingbing-Data2/MVINS-Data/20231020-20231105-idsse-auv-test/scene_2023-10-30-12-05-20-time-16000-17650-modified.bag -r 1"
### vins_estimator/src/estimator.h
#define AUV
### vins_estimator/src/estimator.cpp
        para_Feature[feature_index][0] = 1.0 / 2.15;//2.25
### feature_tracker/src/feature_tracker.cpp
            double threshold = 50;//AUV mode 50
### pose_graph/src/pose_graph.h
#define FRAME_NUM 8
#define THRE_IMG 0.016//field0.032//tank2 0.011
### pose_graph/src/pose_graph.c
    if (flag_detect_loop && (global_index % 3) == 0)
    if (flag_detect_loop && (global_index % 1) == 0)

### run_auv_loop.sh
    sleep 1800;

## TANK2


## FIELD
### vins_estimator/src/estimator.h
//#define AUV
### pose_graph/src/pose_graph.h
#define FRAME_NUM 25//tank1 10//field 25
#define THRE_IMG  0.03// 0


### feature_tracker/src/feature_tracker.cpp
            double threshold = 5;//AUV mode 50

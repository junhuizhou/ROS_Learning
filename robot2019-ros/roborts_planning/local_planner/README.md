### LINKS

[局部轨迹规划](https://robomaster.github.io/RoboRTS-Tutorial/#/sdk_docs/roborts_planning_local_planner)

### TREE

```
local_planner
├── CMakeLists.txt
├── config
│   └── local_planner.prototxt          #局部路径规划算法配置文件
├── include
│   └── local_planner
│       ├── proto 
│       │   ├── local_planner.pb.cc
│       │   ├── local_planner.pb.h
│       │   └── local_planner.proto     # 局部路径规划参数生成文件
│       ├── data_base.h                 # 局部路径规划数据结构,用于g2o构建边以及顶点
│       ├── data_converter.h            # 数据转换,将数据转换为data_base数据结构
│       ├── distance_calculation.h      # 计算二维点、线、几何图形之间的距离
│       ├── line_iterator.h             # 连续线段在离散栅格中的坐标计算
│       ├── local_planner_algorithms.h  # 局部路径规划算法头文件（所有算法头文件都应在此文件中引入）
│       ├── local_planner_base.h        # 局部路径规划算法父类
│       ├── local_planner_node.h        # 局部路径规划入口节点
│       ├── local_visualization.h       # 可视化局部路径规划结果
│       ├── obstacle.h                  # 二维障碍物信息
│       ├── odom_info.h                 # 里程计信息
│       ├── optimal_base.h              # 优化相关算法父类
│       ├── robot_footprint_model.h     # 机器人外形描述
│       ├── robot_position_cost.h       # 机器人所在位置代价计算，用于判断路径是否可行
│       └── utility_tool.h              # 通用函数
├── src
│   ├── local_planner_node.cpp          # ros node文件，负责局部路径规划内的逻辑调度
│   ├── vel_converter.cpp               # ros node文件, 仿真时将局部路径规划速度转换为“cmd_vel”发布
│   ├── teb_test.cpp                    # 算法timed elastic band 测试文件
│   └── ...
└── timed_elastic_band
    ├── CMakeLists.txt
    ├── config
    │   └── timed_elastic_band.prototxt # timed elastic band 配置文件
    ├── include
    │   └── timed_elastic_band
    │       ├── proto
    │       │   ├── timed_elastic_band.pb.cc
    │       │   ├── timed_elastic_band.pb.h
    │       │   └── timed_elastic_band.proto
    │       ├── teb_local_planner.h     #算法timed elastic band实现，继承local_planner_base.h
    │       ├── teb_optimal.h           # g2o优化逻辑，继承optimal_base.h
    │       └── ...                     # 其他g2o的边以及顶点相关文件。
    └── src
        ├── teb_local_planner.cpp       # 算法timed elastic band实现
        ├── teb_optimal.cpp             #g2o优化逻辑
        └── ...

```

### FILES

* /local_planner/src/local_planner_node.cpp                         局部规划节点 
* /local_planner/timed_elastic_band/src/teb_local_planner.cpp       TEB算法实现
* /local_planner/timed_elastic_band/src/teb_optimal.cpp             g2o优化

### /local_planner/src/local_planner_node.cpp

```
local_planner_node
{
    Init();
    ExcuteCB()
    {
        setPlan();
        StartPlanning()
        {
            Loop()
            {
                Initialize();
                while()
                {
                    ComputeVelocityCommands()
                    {
                        //详情看teb_local_planner.cpp
                    }
                    publish(cmd_vel_);
                    IsGoalReached();
                }
            }
        }
    } 
}
```

### /local_planner/timed_elastic_band/src/teb_local_planner.cpp

```
TebLocalPlanner::ComputeVelocityCommands
{
    GetPlan();                      //global_plan_ = temp_plan_

    //信息更新
    UpdateRobotPose();              //from local_cost_
    UpdateRobotVel();               //from odom_info_
    UpdateGlobalToPlanTranform();
    PruneGlobalPlan();
    TransformGlobalPlan();
    UpdateObstacleWithCostmap();    //obst_vector_.push_back()
    UpdateViaPointsContainer();
    
    // g2o优化，详情看teb_optimal.cpp
    Optimal();
    IsTrajectoryFeasible();
    GetVelocity();

    // 获取最终结果
    SaturateVelocity();
    Visualize();
}
```

### /local_planner/timed_elastic_band/src/teb_optimal.cpp

// TODO:待阅，要改的是添加TEB的约束条件中的运动学限制

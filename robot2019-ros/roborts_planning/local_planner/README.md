## local_planner代码阅读笔记

* 结论在最后

### LINKS

[局部轨迹规划](https://robomaster.github.io/RoboRTS-Tutorial/#/sdk_docs/roborts_planning_local_planner)
[趣讲局部规划](https://www.leiphone.com/news/201612/0TCtaBOIcFOIBN69.html)
[TEB流程图](https://blog.csdn.net/xiekaikaibing/article/details/83417223)
[g2o图优化的使用](https://blog.csdn.net/zzyczzyc/article/details/89036143)

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
* /local_planner/timed_elastic_band/src/teb_optimal.cpp             g2o图优化
* /local_planner/config/local_planner.prototxt
* /local_planner/timed_elastic_band/config/timed_elastic_band.prototxt  TEB条件配置
* teb_velocity_eage.h   图的速度约束边
* teb_kinematics_edge.h 图的运动学约束边
* teb_vertex_pose.h     图节点中的姿态部分

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
    Optimal();                      // 建图、图优化、更新代价
    IsTrajectoryFeasible();         // 判断优化轨迹是否可行

    // 获取最终结果
    GetVelocity();          //vx,vy,oemga,ax,ay,acc_omega
    SaturateVelocity();     //对超过实际最大速度值等的优化结果限值

    // 可视化
    Visualize();
}
```

### /local_planner/timed_elastic_band/src/teb_optimal.cpp

* 图优化，节点vertices是状态也是优化量(姿态+时间)，edges是目标优化函数即约束
* 节点vertices具体细节见teb_vertex_xxx.h等文件
    * 节点在teb_vertex_console.h由姿态teb_vertex_pose.h和teb_vertex_timediff.h组成
* 边edge的具体细节见teb_xxx_edge.h等文件

```
{
    Optimal()
    {
        OptimizeTeb()
        {
            BuildGraph()    
            {
                AddTebVertices();       
                AddObstacleLegacyEdges() or AddObstacleEdges();
                AddVelocityEdges();                                             //分为有无横向速度vy
                AddAccelerationEdges();
                AddTimeOptimalEdges();
                AddKinematicsDiffDriveEdges or AddKinematicsCarlikeEdges();     //差速和类车运动学约束
                AddPreferRotDirEdges();
            }
            OptimizeGraph();            //优化结果见vertices即pose变量
            ComputeCurrentCost()
            {
                getError()
                {
                    computeError()
                    {
                        // 具体细节见teb_xxx_edge.h等文件
                        // 涉及各优化函数(此处应为惩罚函数)，由物理量的特性而来
                    }
                }
            }
        }
    }
    IsTrajectoryFeasible();
    GetVelocity()     //vx,vy,oemga,ax,ay,acc_omega
    {
        ExtractVelocity();
    }
}
```

### conclusion

理解：
* TEB是对全局规划的再优化，在g2o优化后局部会得到一条新轨迹，优化结果直接体现是图中间顶点的改变，
* 之后会确定这条轨迹是否可行，可行的话就会根据新轨迹特征求出小车的状态量进行控制，     
* 不同类型车的区别在于g2o优化中的优化函数不一致，优化函数由车的运动特性而来，各优化轨迹区别在于代价不一

修改：
* 修改车的运动学约束即teb_kinematics_edge.h，目前运动学约束有差动轮型和类车型，可根据差动轮修改

// TODO: 确定末状态节点的车朝向要转弯的处理
// TODO: 若要修改输出为vx,vy型，改哪些，需商量一下

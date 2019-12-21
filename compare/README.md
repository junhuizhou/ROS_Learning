# COMPARE roborts_detection

* 注明一些比较重要的函数所在等
* [Detection模块官方教程](https://robomaster.github.io/RoboRTS-Tutorial/#/sdk_docs/roborts_detection)

## TREE

```bash
roborts_detection
├── package.xml
├── CMakeLists.txt
├── armor_detection                  # 装甲识别算法
│   ├── CMakeLists.txt
│   ├── config
│   │   └── armor_detection.prototxt # 装甲识别参数配置文件
│   ├── armor_detection_algorithms.h # 装甲识别算法头文件（所有算法头文件都应在此文件中引入）
│   ├── armor_detection_base.h       # 装甲识别父类
│   ├── armor_detection_client.cpp   # 装甲识别actionlib中的client，在调试中使用。
│   ├── armor_detection_node.cpp     # ros node，负责装甲识别内部逻辑调度
│   ├── armor_detection_node.h       # 装甲识别入口文件
│   ├── gimbal_control.cpp           # 云台控制文件，以弹丸模型建立，得出云台控制的pitch以及yaw
│   ├── gimbal_control.h
│   └── proto
│       ├── armor_detection.pb.cc
│       ├── armor_detection.pb.h
│       └── armor_detection.proto    # 装甲识别参数生成文件
│   ├── constraint_set               # 装甲识别算法，使用装甲特征识别装甲板
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   └── constraint_set.prototxt # 装甲识别可调参数
│   │   ├── constraint_set.cpp
│   │   ├── constraint_set.h
│   │   └── proto
│   │       ├── constraint_set.pb.cc
│   │       ├── constraint_set.pb.h
│   │       └── constraint_set.proto    # 装甲识别参数生成文件
├── cmake_module
│   ├── FindEigen3.cmake
│   └── FindProtoBuf.cmake
└── util
    ├── CMakeLists.txt
    └── cv_toolbox.h # 负责订阅图片以整个detection使用，同时也包含通用图像处理函数
```

## 重点查看文件

* /util/cv_toolbox.h
* /armor_detection/armor_detection_node.cpp
* /armor_detection/constraint_set/constraint_set.cpp

### /util/cv_toolbox.h

1. ros::Subscriber depth_sub_;
    * 节点订阅realsence相机获得的深度信息
2. void DepthCallback(const sensor_msgs::ImageConstPtr &img_msg)
    * 返回realsence相机获得的深度信息
3. cv::Mat DistillationColor(const cv::Mat &src_img, unsigned int color, bool using_hsv)
    * Highlight the blue or red region of the image.

### /armor_detection/

* 几个主要改动的文件

#### /armor_detection/armor_detection_node.cpp

1. 节点流程
```C 
ArmorDetectionNode()
{
    Init();
    ActionCB()
    {
        case 1:
            startThread()
            {
                ExecuteLoop()   
                {
                    DetectArmor();  // constraint_set.cpp中那个
                    /*
                    ...预测算法
                    */
                    GimbalContrl(); // 根据弹丸飞行模型及其他算法修正角度
                    PublishMsgs();  // 发布云台所需转动角度等消息，让robots_base_node节点来订阅
                }
            }
        case others:
            PauseThread()orStopThread();
    }
}
```

#### /armor_detection/constraint_set/constraint_set.cpp

0. ConstraintSet-->LoadParam();
    * 导入两个参数文件，在识别流程中要用相关参数
    * 灯条参数：/armor_detection/constraint_set/config/constraint_set.prototxt
    * SVM模型：/armor_detection/constraint_set/contourHOG_SVM
1. 装甲板识别流程
    * 下方每个函数的实现细节需要一点OpenCV基础
```C
DetectArmor()
{
    DetectLights();
    // FilterLights();
    PossibleArmors();
    FilterArmors();
    detect12FromImage();    //SVM
    Add12Label();
    SlectFinalArmor();
}
```

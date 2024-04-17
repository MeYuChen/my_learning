# udi-carla

## 0. 介绍
本包是基于Carla仿真器的插件包。主要用于话题转发、地图控制、软件接口等。
## 1. 编译安装

### 1.1 carla安装
以下是carla的安装链接:  
[buid from linux](https://carla.readthedocs.io/en/0.9.12/build_linux/)  
### 注意事项：  
* 目前我们安装的版本并不是最新的而是0.9.12,所以安装开始前应该切换至0.9.12版本。随后再跟随步骤进行安装。    
* 在随后的build carla 两个步骤中，在make PythonAPI(我们用的是 make PythonAPI ARGS="--python-version=2.7, 3.6" ) ,可能会出现问题，建议将包更换成本地的。  
* 在从网上下载时如果长时间没有反应，建议终止命令多试几次。

### 1.2 其他软件包的安装
* [ros-bridge](https://gitlab.unity-drive.net/simulation/ros-bridge), 注意切换到feature/udi分支
* planning相关软件包,按需下载
* perception相关软件包

## 2 使用
### 2.1 启动仿真的步骤
* 进入Carla根目录 ```cd carla```
* 执行```make launch```，打开Carla-Unreal编辑器（第一次打开时会比较慢）
* 点击编辑器的play选项（可能会卡顿）
* 启动通讯桥等相关接口```roslaunch udi_carla udi_carla_bridge.launch``` 包括通讯桥、自车生成，注意该接口是默认同步通讯（关于同步和异步的区别，[移步](https://carla.readthedocs.io/en/0.9.12/adv_synchrony_timestep/)
* 启动主要的接口```roslaunch udi_carla udi_carla_single_terminal.launch``` 包括全局规划、行为规划、转发接口、底层控制器、虚拟底盘、监视器，注意该接口是异步通讯
* (一般不选)启动感知的接口```roslaunch udi_carla udi_carla_percep.launch``` 注意该接口是异步通讯
* 启动次要的接口```roslaunch udi_carla udi_iso_local.launch``` 局部规划,注意该接口是异步通讯
* 启动可视化的接口```roslaunch udi_carla udi_test_rviz.launch``` 
* （一般不选）启动CARLA场景器相关功能```roslaunch udi_carla udi_scenario_runner.launch``` 
* （一般不选）生成NPC(Tesla)车辆，切换至udi_carla/src/目录 ```python npc_manual_control.py```
* 剩余的操作与操作实车相同，拉点开启任务。

### 2.2 重启仿真的步骤
由于脚本中有同步和异步的区分，因此重启仿真的步骤非常重要，错误的步骤可能会导致卡死（原因参见同步和异步的区别）
* 先关闭所有异步接口，包括次要接口、感知接口和主要接口
* 关闭同步接口，即桥通讯接口
* 在carla仿真编辑器中选择stop选项（在play选项旁边）
* 按照2.1中操作顺序执行(从点击play选项开始)，不需要关闭carla仿真编辑器。即可重新开启仿真

### 2.3 代码编译
注意服务器上同时存在感知和规控的代码，修改源码之后不要直接执行catkin_make。而是使用workspace下提供的脚本，执行即可```bash build.sh```

### 2.4 多车生成
* 如果要启动多车（目前支持同时生成18辆夸父），请修改udi_carla_bridge.launch 和 udi_carla_single_terminal.launch 将multiple_kuafu变量置为true，并注意修改udi_carla_node.cpp源码，将完美感知激活```enable_perfect_perception```（不支持所有车启动感知程序，负载过大，推荐使用完美感知）。
* 每台车命名为kuafu_id，id从0开始。每台车的话题注意前缀。目前多车主要是给网端提供提调试接口。

## 3 不知原因的一些BUG
* 注意，通讯桥启动后，当其设置为固定的步长和同步模式之后，服务器的帧率并不和客户端的帧率一致（客户端的帧率是脚本所设置的参数大小），服务器会按照默认最大的帧率去运行（比如60HZ，当加上很多LIDAR传感器的负载之后，帧率会掉下来，但不可控）。因此，实际执行的效果会看起来快很多。但并不影响程序执行的正确性，因为CARLA似乎通过某种机制，让ROS整个体系的频率加快了（即比如实际发布频率是10HZ，但是在真实物理世界中，其发布的频率可能达到了30HZ）。
* 这个问题可能是CARLA本身设置上的BUG。解决办法是，在UE编辑器里点击Engine->General Settings->Framerate->Use Fixed Frame Rate,并将Fixed Frame Rate 改为20(与通讯桥脚本一致的帧率)。 通讯桥脚本中，将固定的时间步长改为None，同步模式仍然设为同步。其余的保持不变。

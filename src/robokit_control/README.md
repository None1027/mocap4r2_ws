# robokit_control ROS 2 功能包

适用环境为 Ubuntu 22.04、ROS 2 Humble、Python 3.10。包名使用 `robokit_control`，构建类型为 `ament_python`。

## 放进工作空间

把压缩包内的 `robokit_control` 文件夹解压到 `~/ros2_ws/src/`。最终目录应为

```text
~/ros2_ws/
└── src/
    └── robokit_control/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── robokit_control
        ├── robokit_control/
        │   ├── __init__.py
        │   ├── robokit_tcp.py
        │   └── robokit_cmd_vel.py
        ├── README.md
        └── API_USAGE.md
```

`package.xml` 声明包和依赖，`setup.py` 声明 Python 模块、ament 索引资源和可执行入口，`setup.cfg` 将入口脚本安装到 ROS 2 查找的位置。构建并 source 工作空间后，`ros2 run` 就能找到两个入口。

使用 Ubuntu 系统 Python 及 Humble 环境执行

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 完成解压后执行
rosdep install --from-paths src/robokit_control --ignore-src -r -y
colcon build --symlink-install --packages-select robokit_control
source install/setup.bash

ros2 pkg executables robokit_control
```

最后一条应显示

```text
robokit_control robokit_cmd_vel
robokit_control robokit_tcp
```

若 `colcon` 命令缺失，可安装 `python3-colcon-common-extensions`。若 `rosdep` 命令缺失，可安装 `python3-rosdep`。首次使用 rosdep 且尚未初始化时运行 `sudo rosdep init`，然后执行 `rosdep update`。

## 启动桥接节点

将示例 IP 替换为底盘实际 IP。在终端 1 执行

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run robokit_control robokit_cmd_vel --ros-args \
  -p robot_ip:=192.168.1.200
```

默认协议版本 1 对应 RBK3.4。RBK3.5 使用

```bash
ros2 run robokit_control robokit_cmd_vel --ros-args \
  -p robot_ip:=192.168.1.200 \
  -p protocol_version:=2
```

节点等待 `/cmd_vel`。在终端 2 发布机器人自身坐标系中的速度

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

桥接将 `linear.x`、`linear.y`、`angular.z` 分别转为 API 的 `vx`、`vy`、`w`。按 Ctrl+C 停止发布后，桥接在接收超时后发送零速度。默认接收超时 0.25 秒，单条底盘速度有效时长 300 ms，网络和底盘减速会影响实际停转时间。

桥接通过 TCP 向底盘的 19205 端口发送 API 2010。使用该接口会取消底盘当前导航任务。底盘无需安装 ROS 2 SDK。

## 直接调用 TCP 命令入口

`robokit_tcp` 是同步命令行工具，也注册在包中。用以下命令发送零速度并检查响应

```bash
ros2 run robokit_control robokit_tcp --host 192.168.1.200
```

低速运动约 2 秒后发送零速度

```bash
ros2 run robokit_control robokit_tcp \
  --host 192.168.1.200 --vx 0.05 --seconds 2
```

这个入口使用普通命令行参数，RBK3.5 添加 `--version 2`。测试时一次运行一个控制入口，避免多个程序向同一底盘发送相互覆盖的速度。

完整协议解释和参数表见 `API_USAGE.md`。其中 `python3 文件名.py` 的命令是原始脚本用法，在此 ROS 2 包内使用本页的 `ros2 run` 命令。

## 后续连接动捕

```text
动捕当前位姿 + 目标位姿
          ↓
位姿闭环控制节点
          ↓ /cmd_vel
robokit_cmd_vel 节点
          ↓ TCP / API 2010
底盘
```

未来可以把动捕接收和位置控制分别做成功能包，放在同一个工作空间的 `src/` 下。控制节点输出机器人坐标系速度，即可使用本桥接。

## 验证与元数据

本包保留前一版的通信与控制逻辑，并将桥接中的导入改为包内导入。交付时检查 Python 语法、ROS 包元数据、安装入口、ament 索引资源，以及模拟 TCP 和回调逻辑。

本集成版已在 WSL Ubuntu 22.04 / ROS 2 Humble 下完成构建、ROS 2 运行与模拟 TCP 底盘联调，完整记录见交付包根目录 README。实机运动仍需在用户设备上验证。维护者和许可证字段使用占位内容，后续分发时请填写项目实际信息。

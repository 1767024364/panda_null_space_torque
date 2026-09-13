# Panda Null-Space Torque Control

基于 ROS 2 Control、MuJoCo 和 Pinocchio 的 Franka Emika Panda 七自由度机械臂力矩级零空间控制仿真项目。控制器在保持末端六维位姿的同时，将关节次任务投影到主任务的动力学一致零空间，并可通过动量观测器估计关节外力矩。

> 本项目用于控制算法学习和仿真验证，不应未经额外的安全设计就直接部署到真实机械臂。

## 仿真演示

![Panda 末端位姿力矩级零空间控制演示](docs/images/pose_null_space_torque.gif)

GIF 请保存为：

```text
docs/images/pose_null_space_torque.gif
```

## 主要功能

- Panda 七轴机械臂 MuJoCo 仿真与可视化；
- ROS 2 Control `effort` 力矩指令接口；
- Pinocchio 正运动学、刚体动力学、末端局部雅可比及其导数；
- 完整 SE(3) 末端位姿控制；
- 动力学一致广义逆与力矩零空间投影；
- 基于逆动力学的关节零空间次任务；
- 动量观测器外力矩估计及可选的任务空间外力矩补偿。

## 控制律

### 1. 关节空间动力学

机械臂关节空间动力学写为：

$$
M(q)\ddot q+C(q,\dot q)\dot q+g(q)=\tau_c+\tau_{\mathrm{ext}}
$$

其中，$M$ 为关节空间惯性矩阵，$C\dot q$ 为科氏力和离心力项，$g$ 为重力项，$\tau_c$ 为控制力矩，$\tau_{\mathrm{ext}}$ 为外力对应的广义关节力矩。

### 2. 末端位姿误差

控制器激活时记录末端位姿 $T_d$ 作为期望位姿。当前位姿为 $T$，局部坐标系下的六维误差为：

$$
e=\operatorname{Log}_6\!\left(T^{-1}T_d\right)^\vee
$$

雅可比使用 Pinocchio 的 `LOCAL` 表达：

$$
V=J(q)\dot q
$$

由于期望位姿固定，当前代码采用以下误差速度：

$$
\dot e\approx-J(q)\dot q
$$

这与代码中的 `dpose_error = -J_local * dq` 一致。远离零误差时，严格的 SE(3) 误差导数还需要考虑 $J_{\log 6}$。

### 3. 操作空间动力学

操作空间惯性矩阵为：

$$
\Lambda=\left(JM^{-1}J^T\right)^{-1}
$$

动力学一致广义逆为：

$$
\bar J=M^{-1}J^T\Lambda
$$

力矩零空间投影矩阵为：

$$
N_\tau=I-J^T\bar J^T
$$

任务空间科氏力和重力项分别为：

$$
\mu=\left(\bar J^T C-\Lambda\dot J\right)\dot q
$$

$$
p=\bar J^Tg
$$

### 4. 任务空间控制力

对固定期望位姿，代码中的末端六维控制力为：

$$
F_{\mathrm{task}}=\Lambda\left(K_p e+K_d\dot e\right)+\mu+p
$$

其中，$K_p$ 和 $K_d$ 分别为末端位姿的比例与微分增益。

### 5. 关节零空间次任务

七个关节使用相同的正弦期望轨迹：

$$
q_d=A\sin(\omega t)\mathbf{1}_7
$$

$$
\dot q_d=A\omega\cos(\omega t)\mathbf{1}_7
$$

$$
\ddot q_d=-A\omega^2\sin(\omega t)\mathbf{1}_7
$$

未投影的关节次任务力矩由逆动力学给出：

$$
\tau_0=M\left[\ddot q_d+K_{p,0}(q_d-q)+K_{d,0}(\dot q_d-\dot q)\right]+C\dot q+g
$$

只有 $N_\tau\tau_0$ 会加入总控制力矩，因此在理想模型和非奇异条件下，次任务不会影响六维末端主任务。对七轴机械臂的六维位姿任务，一般约剩一维零空间。

### 6. 动量观测器

定义广义动量：

$$
r=M(q)\dot q
$$

代码采用离散积分更新动量估计：

$$
\hat r_k=\hat r_{k-1}+\Delta t\left(\tau_{c,k-1}-g_{k-1}+C_{k-1}^T\dot q_{k-1}+\hat\tau_{\mathrm{ext},k-1}\right)
$$

外力矩估计为：

$$
\hat\tau_{\mathrm{ext},k}=K_o\left(r_k-\hat r_k\right)
$$

随后对每个关节的估计值限幅：

$$
\hat\tau_{\mathrm{ext},k}^{\mathrm{lim}}
=\operatorname{clip}\!\left(\hat\tau_{\mathrm{ext},k},-\tau_{\max},\tau_{\max}\right)
$$

### 7. 总控制律

关闭观测器补偿时：

$$
\tau_c=J^T F_{\mathrm{task}}+N_\tau\tau_0
$$

开启观测器补偿时，当前代码的总控制律为：

$$
\tau_c=J^T F_{\mathrm{task}}+N_\tau\tau_0-J^T\bar J^T\hat\tau_{\mathrm{ext}}^{\mathrm{lim}}
$$

最后一项将估计外力矩中的任务空间分量映射回关节空间进行补偿。

## 当前参数

参数位于 `panda_bringup/config/controller.yaml`：

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| `update_rate` | `500` | 控制器更新频率，单位 Hz |
| `Kp` | `200.0` | 末端位姿比例增益 |
| `Kd` | `20.0` | 末端位姿微分增益 |
| `Kp_0` | `0.0` | 零空间关节位置增益 |
| `Kd_0` | `20.0` | 零空间关节速度增益 |
| `A` | `0.0` | 零空间正弦关节轨迹幅值 |
| `omega` | `2.0` | 正弦轨迹角频率，单位 rad/s |
| `Ko` | `50.0` | 动量观测器增益 |
| `add_observer` | `true` | 是否将外力矩估计加入补偿回路 |
| `max_tau_ext_hat` | `3000.0` | 单关节外力矩估计限幅 |

当前 `A = 0` 且 `Kp_0 = 0`，因此零空间不执行主动正弦位置轨迹，但 `Kd_0` 仍提供零空间速度阻尼。

## 代码结构

```text
panda_null_space_torque/
├── docs/images/                         # README 演示 GIF
├── panda_bringup/
│   ├── config/controller.yaml          # 控制器参数
│   └── launch/panda.launch.py           # MuJoCo 与控制器启动文件
├── panda_controllers/
│   ├── include/panda_controllers/
│   │   └── null_space_controller.hpp
│   └── src/
│       ├── null_space_controller.cpp    # 生命周期、接口和参数
│       └── pose_null_space_controller_torque.cpp
│                                           # 位姿、零空间和观测器算法
└── panda_description/
    ├── urdf/panda_ros2_control.urdf.xacro
    └── mujoco/
        ├── panda_tau.xml              # Panda MJCF 与力矩执行器
        └── scene_tau.xml              # MuJoCo 场景
```

`NullSpaceController::update()` 当前每个控制周期调用：

```cpp
pose_null_space_controller_torque(time, period);
```

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- ros2_control / controller_manager
- MuJoCo
- mujoco_ros2_control
- Pinocchio
- Eigen3

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/mujoco_ros2_learning/panda_null_space_torque
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

只重新编译控制器：

```bash
colcon build --packages-select panda_controllers --symlink-install
source install/setup.bash
```

## 运行

启动 MuJoCo 图形界面：

```bash
ros2 launch panda_bringup panda.launch.py headless:=false
```

无界面运行：

```bash
ros2 launch panda_bringup panda.launch.py headless:=true
```

控制器激活时会将当前末端位姿记录为 $T_d$。MuJoCo 中末端的红色球表示 `ee_center_body` 控制点。

## 调参说明

- 建议先在 `add_observer: false` 下确认基础位姿和零空间控制稳定，再开启外力矩补偿；
- 提高 `Kp` 会增大末端等效刚度，`Kd` 用于抑制振荡；
- `Ko` 越大，观测器响应越快，但对测量噪声和模型误差也越敏感；
- `max_tau_ext_hat` 用于抑制异常外力矩估计；
- 启用主动零空间轨迹前，应从较小的 `A`、`Kp_0` 和 `Kd_0` 开始调试。

## 实现说明

- 当前 Panda MJCF 模型共有 9 个速度自由度，控制器使用前 7 个机械臂关节；
- 当前代码通过 `leftCols(7)` 取出机械臂雅可比，这依赖七个手臂关节在 Pinocchio 速度向量中排在前七位；
- $\Lambda$ 和 $\bar J$ 的当前实现使用显式矩阵求逆，运行时应避免使机械臂进入奇异构型；
- `max_velocity` 是从速度控制版本保留的参数，当前力矩控制律未使用它。

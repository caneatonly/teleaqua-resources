# Teleaqua 仿真资源仓库

本仓库用于存放 Teleaqua 系列载具在 PX4 + Gazebo 仿真中使用的外部资源，包括模型、世界文件、自定义 Gazebo 插件和 PX4 airframe 配置。

详细的环境部署、Docker 启动、插件编译、PX4/ROS 2 启动流程请参考：

```text
环境搭建指南.md
```

## 仓库内容

```text
external/
├── models/              # Gazebo 模型
├── worlds/              # Gazebo 世界文件
├── plugins/             # 自定义 Gazebo 插件源码
├── teleai_airframes/    # PX4 SITL airframe 配置
└── 环境搭建指南.md       # 新电脑环境搭建和运行说明
```

## models

`models/` 目录存放仿真载具模型：

```text
models/
├── teleh4z/
├── teleaquah8p/
└── teleaquah8p_mono_cam/
```

其中 `teleh4z` 是当前主要使用的空水跨介质载具模型，包含机体、浮力结构、相机、机臂、空中桨、水下推进器以及相关 Gazebo 插件配置。

TeleH4Z 的执行器和模式约定可参考：

```text
models/teleh4z/TELEH4Z_CONTRACT.md
```

## worlds

`worlds/` 目录存放 Gazebo 仿真世界文件：

```text
worlds/
├── playground.sdf
└── pool_apriltagex.sdf
```

其中 `playground.sdf` 是主要使用的水池仿真环境，包含物理系统、传感器系统、浮力系统、水池结构、AprilTag 和坐标配置。

## plugins

`plugins/` 目录存放自定义 Gazebo 插件源码：

```text
plugins/
├── bidir_motor_model/
├── hydrodynamics/
├── buoyancy/
├── hydrodynamics_offical/
└── build_plugin.sh
```

主要插件包括：

- `bidir_motor_model`：双向水下推进器电机模型
- `hydrodynamics`：混合流体动力学插件，用于水动力、附加质量和空气/水过渡效果
- `buoyancy`：浮力相关插件

编译插件可在容器内执行以下脚本或阅读插件仓库Readme文档：

```bash
cd /home/user/external/plugins
./build_plugin.sh
```

## teleai_airframes

`teleai_airframes/` 目录存放 PX4 SITL 使用的自定义 airframe 文件：

```text
teleai_airframes/
├── 4024_gz_teleaquah8
└── 4026_gz_teleh4z
```

这些文件会同步到 PX4 的 airframe 目录中，用于启动对应的仿真机型。

## 使用说明

本仓库通常挂载到容器内：

```text
/home/user/external
```

修改模型、世界、插件或 airframe 后，需要将资源同步到 PX4 的 Gazebo/SITL 目录。具体同步方式和完整运行流程请查看：

```text
环境搭建指南.md
```

## 注意事项

- `plugins/*/build/` 是插件编译生成目录，不建议手动修改。
- `models/` 和 `worlds/` 中的 SDF 文件是仿真资源的主要编辑入口。
- 修改插件源码后需要重新编译插件。
- 修改模型、世界或 airframe 后需要重新同步到 PX4 目录。

# RTK-GNSS
基于 C++ 的 GNSS RTK 相对定位课程实践项目，针对 GPS+BDS，支持解析 NovAtel OEM7 二进制数据（RANGEB/EPHEM/PSRPOS 等），在单点定位与测速（SPP/SPV）基础上实现历元同步、单差/双差观测构建与 GF/MW 周跳检测、广播星历卫星轨道与钟差传播、对流层与高度截止角建模，以及基于 Eigen 的最小二乘 + LAMBDA 与 EKF 两种 RTK 解算模式，提供离线文件和实时 TCP 流两种输入，并配套 MATLAB 结果分析与 ENU 误差可视化。

> 项目实验报告：[项目实验报告.pdf](项目实验报告.pdf)

## 项目简介

本项目实现了一套完整的 GNSS RTK 处理链路，包括：

- 配置解析与数据源管理（文件 / 网络）。
- NovAtel OEM7 二进制观测与星历解码。
- 单点定位（SPP/SPV）、误差建模与观测预处理。
- 单差 / 双差构建、周跳探测与参考星选择。
- 基于最小二乘 + LAMBDA 的 RTK 浮点 / 固定解。
- 基于扩展卡尔曼滤波（EKF）的 RTK 解算模式。
- MATLAB 后处理与 ENU 误差分析 / 绘图。

适用于 GNSS/RTK 算法研究、教学实验以及短基线验证等场景。

## 主要特性

- **多数据源支持**
  - **文件模式**：从 NovAtel OEM7 观测二进制文件读取基站 / 流动站数据。
  - **网络模式**：通过 TCP/IP 接收 OEM7 实时数据流。
- **RTK 解算链路**
  - GPS + BDS 的 SPP/SPV 单点解算。
  - 单差 / 双差观测构建与周跳探测。
  - 最小二乘 RTK 浮点解 + LAMBDA 整周模糊度固定。
  - EKF（扩展卡尔曼滤波）RTK 模式（可由配置选择）。
- **结果输出与可视化**
  - 输出历元级流动站固定 / 浮点解坐标、基线向量、PDOP、Ratio、卫星数等。
  - MATLAB 脚本对结果进行 ENU 误差分析、RMS 统计与可视化。
- **坐标 / 时间工具**
  - ECEF ↔ BLH、ENU 坐标转换。
  - 通用时 / MJD / GPS 时之间的转换。

## 目录结构

项目根目录下的主要文件与子目录如下：

```text
.
├─ README.md                 # 项目说明文档（本文件）
├─ LICENSE                   # MIT 开源许可证
├─ .gitignore                # Git 忽略规则
├─ 项目实验报告.pdf          # 项目实验报告（实验背景、流程与结果）
├─ src/                      # C++ 核心解算代码
│  ├─ main.cpp               # 程序入口，配置读取、数据源打开、历元循环与结果输出
│  ├─ SPP.h                  # 公共数据结构、常量与函数声明（SPP/RTK/EKF 等）
│  ├─ Config.txt             # 运行时配置文件示例（数据源、模式、阈值等）
│  ├─ ReadRTKConfigInfo.cpp  # 解析 Config.txt，填充 ROVERCFGINFO
│  ├─ RTK.cpp                # 单差/双差构建、周跳探测、RTK 浮点/固定解
│  ├─ EKF.cpp                # 扩展卡尔曼滤波 RTK 解算
│  ├─ PVT.cpp                # 单点定位（SPP）与速度估计（SPV）
│  ├─ Lambda.cpp             # LAMBDA 整周模糊度固定及矩阵运算
│  ├─ Decode.cpp             # NovAtel OEM7 二进制消息解码
│  ├─ ErrorCorrection.cpp    # 粗差探测、对流层等误差建模
│  ├─ SatellitePosition.cpp  # GPS/BDS 卫星位置与钟差计算
│  ├─ CoordinateTransformation.cpp  # 坐标变换（ECEF/BLH/ENU 等）
│  ├─ TimeConverter.cpp      # 时间系统转换（UTC/MJD/GPS）
│  ├─ Socket.h               # TCP Socket 跨平台封装头文件
│  └─ Socket.cpp             # TCP Socket 在 Windows 平台的具体实现
├─ plot/                     # MATLAB 后处理与绘图脚本
│  ├─ main.m                 # 示例主脚本，组织读取、分析与绘图流程
│  ├─ ReadRTKResults.m       # 读取 C++ 输出的 RTK 结果文件
│  ├─ AnalyzeRTKData.m       # 计算 ENU 误差与 RMSE
│  ├─ CoordinateTransformation.m # MATLAB 版坐标转换与 dENU 计算
│  ├─ PlotRTKSingleFigures.m # 绘制单组 RTK 结果的 dE/dN/dH、PDOP、Ratio 等曲线
│  ├─ Compare_RTK_SPP.m      # 对比 RTK 与 SPP 性能（误差、PDOP 等）
│  └─ Compare_LSQ_EKF.m      # 对比 LSQ 与 EKF 两种 RTK 模式
```

## 环境依赖

- **操作系统**
  - Windows 10/11（当前主要测试平台）。
  - 其他平台（Linux/macOS）需要对 Socket 与部分平台相关代码进行适配。
- **编译器**
  - 支持 C++11 及以上（如 MSVC / Clang / GCC）。
- **第三方库**
  - [Eigen](https://eigen.tuxfamily.org/)：线性代数库（仅头文件）。
- **可选**
  - MATLAB：用于运行 `plot/` 目录下的后处理脚本。

## 编译方式（CMake 示例）

```bash
git clone https://github.com/<your_name>/<your_repo>.git
cd <your_repo>

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```

也可以在 Visual Studio 中手动创建工程，将 `src/` 下的源文件加入工程，并正确设置：

- Eigen 头文件包含路径；
- 链接 WinSock 相关库（如 `ws2_32.lib`）。

## 配置文件说明（`Config.txt`）

程序通过 `Config.txt` 控制数据源、RTK 模式和若干参数。一个典型示例如下：

```text
FROM FILE OR COM: 1
RTK PROCESSING MODE: 2
ROVER IP ADDRESS AND PORT: 127.0.0.1 9001
BASE IP ADDRESS AND PORT: 127.0.0.1 9002
ROVER COM SETUP: 3 115200
BASE OBSDATA SOURCE FILE: base_oem7.bin
ROVER OBSDATA SOURCE FILE: rover_oem7.bin
POSITION RESULT FILE: rtk_results.txt
POSITION DIFF FILE: rtk_diff.txt
CODE AND CARRIER PHASE NOISE: 0.3 0.01
THRESHOLD FOR ELEVATION MASK: 15
RATIO FOR DD FIXED SOLUTION: 3
```

字段含义示意：

- **FROM FILE OR COM**：`0=串口实时`，`1=文件`，`2=网络`（具体实现以源码为准）。
- **RTK PROCESSING MODE**：`1=EKF`，`2=LSQ+LAMBDA`。
- **ROVER/BASE IP ADDRESS AND PORT**：网络模式下流动站 / 基站的 IP 与端口。
- **ROVER COM SETUP**：串口模式下的串口号与波特率。
- **BASE/ROVER OBSDATA SOURCE FILE**：文件模式下的 OEM7 观测二进制文件路径。
- **POSITION RESULT FILE**：RTK 结果输出文件名。
- **POSITION DIFF FILE**：位置差分输出文件（当前实现中可选 / 备用）。
- **CODE AND CARRIER PHASE NOISE**：码观测与载波观测噪声参数。
- **THRESHOLD FOR ELEVATION MASK**：高度截止角（单位：度）。
- **RATIO FOR DD FIXED SOLUTION**：LAMBDA 固定解 Ratio 阈值。

建议在仓库中提供一个 `Config_example.txt`，用户可以复制后按需修改。

## 运行示例

1. 在可执行程序所在目录（或源码指定目录）放置 `Config.txt`。
2. 准备相应的 OEM7 观测文件（或启动网络数据源）。
3. 运行解算程序，例如：

```bash
./rtk_solver
```

程序会读取 `Config.txt`，从文件或网络获取观测数据，进行逐历元 RTK 解算，并将结果写入配置中指定的结果文件。

## MATLAB 后处理

1. 打开 MATLAB，将当前目录切换到 `plot/`。
2. 编辑 `main.m`，将结果文件路径改为你的 RTK 结果文件名（例如 `rtk_results.txt`）。
3. 在 MATLAB 命令行运行：

```matlab
main
```

脚本将：

- 读取 C++ 输出的 RTK 结果文件；
- 根据给定参考坐标计算 ENU 误差与整体 RMS；
- 绘制 dE/dN/dH、PDOP、Ratio、卫星数、是否固定解等时间序列图。

## 结果文件格式概述

RTK 结果文件通常每行对应一个历元，包含（但不限于）如下字段（以实际实现为准）：

- `GPSWeek`, `GPSWeekSec`
- 固定解：`Fixed_B`, `Fixed_L`, `Fixed_H`, `Fixed_X`, `Fixed_Y`, `Fixed_Z`
- 浮点解：`Float_B`, `Float_L`, `Float_H`, `Float_X`, `Float_Y`, `Float_Z`
- `IsFixed`（是否固定解）
- `SatNum`, `GPS_SatNum`, `BDS_SatNum`
- `PDOP`, `Ratio`

具体列顺序可参考 `plot/ReadRTKResults.m` 中的实现。

## 已知限制与后续工作

- 当前主要在 Windows + NovAtel OEM7 数据上测试，其他平台与接收机类型暂未完整验证。
- 仅支持 OEM7 二进制消息格式，如需支持其他接收机需新增解码模块。
- 数值稳定性与异常情况处理在部分退化几何或数据质量较差场景仍有改进空间。

欢迎在 Issue 中反馈问题或提出改进建议。

## 许可证

本项目基于 **MIT License** 发布，详细条款见仓库根目录下的 `LICENSE` 文件。

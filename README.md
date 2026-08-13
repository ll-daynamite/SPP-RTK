# SPP-RTK 卫星导航解算程序

本项目对 NovAtel OEM7 原始数据进行解码，并提供以下解算流程：

- GPS/BDS 单点定位（SPP）与单点测速（SPV）；
- 基于双差观测值的 RTK 浮点解与固定解；
- 基于扩展卡尔曼滤波（EKF）的 RTK 解算；
- 文件数据后处理和 Socket 实时数据处理。

## 本次重构说明

原程序在 `GNSS_PPP.cpp` 的 `main` 函数中同时处理参数配置、文件、Socket、解码、定位、滤波和结果输出，代码接近 400 行。重构后，`main` 只负责三件事：创建默认配置、解析命令行、启动应用。

核心定位算法的数学计算过程没有改动。重构主要调整调用组织方式，并将原来的编译期宏 `positioning`、`FILEMODE` 替换为运行时参数。

### 模块职责

| 文件 | 职责 |
| --- | --- |
| `GNSS_PPP.cpp` | 精简后的程序入口 |
| `ApplicationConfig.*` | 默认配置、命令行解析、使用帮助 |
| `Application.*` | 根据定位模式和输入模式分发工作流 |
| `SppWorkflow.*` | SPP 文件处理与实时处理 |
| `RtkWorkflow.*` | RTK 文件处理、实时处理和共享历元管线 |
| `WorkflowSupport.*` | 两条工作流共用的文件、时间戳和 Socket 资源管理 |
| `Decode.*` | OEM7 数据解码 |
| `SPP_SPV.*` | SPP/SPV 核心算法 |
| `RTK.*`、`KF.cpp`、`lambda.cpp` | RTK、EKF 和 LAMBDA 核心算法 |
| `Position.*`、`ErrorCorrection.*` | 卫星位置和误差改正 |
| `CoordinateTransformation.*`、`TimeConversion.*` | 坐标与时间转换 |

共享 RTK 历元管线依次完成粗差探测、SPP、SPV、异常观测检查、站间单差、周跳探测、参考星选择和星历检查。文件模式和实时模式通过选项复用同一管线。

## 构建环境

- Windows 10/11；
- Visual Studio 2022（安装“使用 C++ 的桌面开发”）；
- CMake 3.20 或更高版本；
- Eigen 3.3 或更高版本。

Eigen 是仅包含头文件的库。假设解压后的目录中存在 `Eigen/Dense`，可按以下方式构建：

```powershell
cmake -S . -B build -DEIGEN3_INCLUDE_DIR="D:/Libraries/eigen-3.4.0"
cmake --build build --config Release
```

生成的程序通常位于：

```text
build/Release/SPP-RTK.exe
```

查看所有运行参数：

```powershell
./build/Release/SPP-RTK.exe --help
```

## 使用方法

### SPP 文件模式

```powershell
./build/Release/SPP-RTK.exe `
  --positioning spp `
  --input file `
  --spp-file "D:/data/receiver.bin" `
  --spp-output "spp.txt"
```

### RTK 文件模式

普通 RTK 和 EKF 使用两组流动站/基准站文件：

```powershell
./build/Release/SPP-RTK.exe `
  --positioning rtk `
  --input file `
  --rover-file "D:/data/rover.bin" `
  --base-file "D:/data/base.bin" `
  --kf-rover-file "D:/data/rover2.bin" `
  --kf-base-file "D:/data/base2.bin" `
  --rtk-output "rtkout.csv" `
  --kf-output "kfout.csv"
```

### SPP 实时模式

```powershell
./build/Release/SPP-RTK.exe `
  --positioning spp `
  --input realtime `
  --spp-ip 127.0.0.1 `
  --spp-port 7190
```

### RTK 实时模式

```powershell
./build/Release/SPP-RTK.exe `
  --positioning rtk `
  --input realtime `
  --rover-ip 127.0.0.1 `
  --rover-port 5002 `
  --base-ip 127.0.0.1 `
  --base-port 7190
```

实时模式会把收到的原始二进制数据保存为带时间戳的 `.bin` 文件。程序默认配置仍与重构前一致，可在 `ApplicationConfig.cpp` 的 `MakeDefaultConfig` 中统一修改。

## 输出文件

| 文件 | 内容 |
| --- | --- |
| `spp.txt` | GPS 周、周内秒、ECEF 坐标和钟差 |
| `rtkout.csv` | RTK 坐标、固定状态和 Ratio |
| `kfout.csv` | EKF 坐标、固定状态和 Ratio |
| `OBS.txt` | 文件模式下的卫星位置和观测辅助信息 |
| `*_raw_data_*.bin` | 实时模式保存的原始数据 |

运行输出和原始接收机数据已加入 `.gitignore`，避免误提交大文件。输入数据不包含在仓库中。


## 注意事项

- 当前 Socket 实现依赖 WinSock，因此构建目标为 Windows。
- 原始数据必须是程序现有解码器支持的 NovAtel OEM7 消息格式。
- RTK 文件应来自时间能够对齐的流动站和基准站。
- 本次重构没有更改坐标解算、误差改正、LAMBDA 或 EKF 的数学公式。

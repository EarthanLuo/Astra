# GS-LIVO Rewrite: 总体架构设计

## 1. 架构总览

### 1.1 设计原则

1. **单进程多线程** — 避免 IPC 开销，所有模块共享地址空间，通过并发队列传递数据
2. **显式依赖方向** — 依赖从 Orchestrator 向下流向模块，模块间不直接耦合
3. **集中式状态管理** — 所有 EKF 状态经由 EkfStateManager 访问，消除裸指针竞争
4. **模板化八叉树** — 一套 `Octree<T>` 服务 LIO 和 GS 两个用途
5. **持久化 3DGS** — GaussianModel 跨帧保持，避免每帧重建销毁
6. **双流 CUDA** — compute stream + transfer stream 重叠 kernel 执行与数据传输
7. **C++20 原生同步** — `std::jthread` + `std::stop_token` 管理线程生命周期，`std::shared_mutex` 保护状态

### 1.2 模块分解图

```
                        +-------------------+
                        |   ConfigManager   |  (YAML 配置解析，全局单例)
                        +--------+----------+
                                 |
                                 v
+-------------------+    +------+--------+    +-------------------+
|   SensorDriver    |--->| Orchestrator  |<---|     Logger        |
|   (LIDAR/IMU/Cam) |    | (主循环调度器)  |    | (spdlog 包装)     |
+-------------------+    +------+--------+    +-------------------+
                                 |
            +--------------------+-------------------+
            |                    |                   |
            v                    v                   v
    +-------+------+    +-------+-------+    +------+--------+
    | LIOEngine    |    | VIOEngine     |    | MapManager    |
    | (LiDAR-惯性)  |    | (视觉-惯性)    |    | (统一地图管理) |
    +-------+------+    +-------+-------+    +------+--------+
            |                    |                   |
            v                    v                   v
    +-------+------+    +-------+-------+    +------+--------+
    | LidarProcessor|    | Gaussian-     |    | Octree<T>    |
    | (预处理/去畸变) |    | Splatting    |    | (模板八叉树)   |
    +---------------+    | (3DGS渲染引擎) |    +---------------+
                          +-------+-------+
                                  |
                                  v
                          +-------+-------+
                          |  lib3dgs      |
                          | (CUDA 库,     |
                          |  git submodule)|
                          +---------------+

    +-------------------+
    |  IMUPropagator    |  (250Hz jthread, 独立于主循环)
    +-------------------+
            |
            v
    +-------+-----------+
    | EkfStateManager   |  (集中式 EKF 状态管理, shared_mutex 保护)
    +-------------------+
```

### 1.3 对比原版的变化

| 维度 | 原版 | 重写 |
|------|------|------|
| 通信 | ROS 话题 | 内部 ConcurrentQueue + 可选 ZeroMQ 可视化 |
| 状态管理 | LIVMapper 持有裸指针，三上下文竞争 | EkfStateManager 集中管理，读写锁 |
| 八叉树 | 两套独立实现 (VoxelOctoTree + GSVoxelOctree) | Octree\<T\> 模板，LidarOctree/GSOctree 特化 |
| 3DGS 生命周期 | 每帧重建销毁 | 跨帧持久化，增量更新 |
| CUDA 执行 | 全部默认流串行 | 双流 (compute + transfer) |
| 线程管理 | ROS 回调 + OpenMP | std::jthread + std::stop_token |
| 配置 | roslaunch + yaml | 纯 YAML |
| 构建 | catkin + 绝对路径 | 现代 CMake + add_subdirectory |
| 测试 | 零 | Catch2 单元测试 + 轨迹回归 |

---

## 2. 模块职责与接口

### 2.1 SensorDriver

**职责:** 封装传感器硬件接入，提供统一的数据抽象层。

- LiDAR 驱动: 读取 Livox/LiDAR 原始数据包，解析为 `LidarScan`
- IMU 驱动: 读取 IMU 数据流，解析为 `ImuData`
- Camera 驱动: 读取相机帧，解析为 `ImageData`
- 支持回放模式: 从 rosbag 或自定义格式文件读取

**数据产出:** 三个 ConcurrentQueue (LidarScan, ImuData, ImageData)

**依赖方向:** 无（使用 ConfigManager 获取传感器参数）
**被依赖:** Orchestrator 创建并启动

### 2.2 SensorSynchronizer

**职责:** 从三个 ConcurrentQueue 中取出时间对齐的传感器包。

- 维护三个时间排序的 deque
- 实现状态机: WAIT_IMU -> WAIT_LIDAR -> WAIT_IMG -> SYNCED
- 输出 `SyncPacket` 包含对齐的 LidarScan + ImageData + IMU 窗口

**关键设计:** 纯函数式，不持有状态。Orchestrator 持有 deques 作为状态。

### 2.3 EkfStateManager

**职责:** 集中管理 EKF 19 维状态 + 协方差矩阵，提供线程安全访问。

- 读操作: `getState()` / `getLatest()` 使用 `shared_lock`
- 写操作: `predict()` / `applyVioUpdate()` / `applyLioCorrection()` 使用 `unique_lock`
- IMU 传播: `predict()` 方法内部先读后写，一次 `unique_lock` 完成

详见 `rewrite-interfaces.md` 完整类设计。

### 2.4 IMUPropagator

**职责:** 250Hz 独立线程，高频 IMU 传播。

- `std::jthread` 运行，`std::stop_token` 控制停止
- 从 IMU ConcurrentQueue 消费数据
- 调用 `EkfStateManager::predict()` 更新状态
- 传播即时发布 `ImuPose` 到可视化队列

### 2.5 LidarProcessor

**职责:** LiDAR 点云预处理与去畸变。

- 体素降采样 (voxel grid filter)
- 运动畸变补偿（使用 EKF 状态估计运动）
- 输出滤波后的 `ProcessedLidarScan`

### 2.6 LIOEngine

**职责:** LiDAR-惯性里程计。

- 维护局部体素地图 (通过 MapManager)
- scan-to-map 匹配 (ICP / NDT 类)
- 残差构建 + 状态修正
- 调用 `EkfStateManager::applyLioCorrection()`

### 2.7 GaussianSplatting

**职责:** 3DGS 渲染引擎，包装 lib3dgs。

- 持久化 `GaussianModel` 和 `Scene` 对象（跨帧复用）
- `render()`: 给定位姿渲染图像
- `insertPoints()`: 增量添加新的高斯点
- `densifyAndPrune()`: 致密化与剪枝
- 双 CUDA 流执行

**依赖:** lib3dgs (git submodule, CUDA + LibTorch)

### 2.8 VIOEngine

**职责:** 视觉-惯性里程计 (含 3DGS)。

- 调用 GaussianSplatting::render() 渲染预测视图
- 光度残差计算 (rendered vs original)
- 雅可比构建
- EKF 更新: `EkfStateManager::applyVioUpdate()`
- GS 点插入与致密化

### 2.9 MapManager

**职责:** 统一地图管理。

- **LIO 地图**: `Octree<LidarPoint>` + 体素平面拟合
- **GS 地图**: `Octree<GsPoint>` + GS 点列表
- 支持按区域查询、最近邻搜索
- 地图持久化 (保存/加载)

### 2.10 Orchestrator

**职责:** 主循环调度器，驱动整个系统。

- 主线程: 10Hz 循环 (sync -> LIO -> VIO -> publish)
- 创建并管理所有模块的生命周期
- 实现系统状态机 (mode 切换)
- 异常处理与优雅关闭

### 2.11 ConfigManager

**职责:** YAML 配置解析，全局可访问。

- 单例模式
- 解析传感器标定、算法参数、运行模式
- 支持运行时参数重载 (通过 ZeroMQ command socket)

### 2.12 Logger

**职责:** 日志系统。

- 基于 spdlog
- 文件日志 + 控制台日志
- 分级: DEBUG / INFO / WARN / ERROR
- 循环日志文件，自动清理

### 2.13 Visualizer (可选)

**职责:** 提供可视化数据接口。

- ZeroMQ PUB 套接字发布 SLAM 状态
- 数据: 点云、轨迹、图像、3DGS 渲染
- 轻量文本/Protobuf 序列化
- 外部 viewer 可独立订阅

---

## 3. 数据流设计

### 3.1 完整数据流

```
时间轴 --->

[SensorDriver]
    |-- LiDAR --> [LidarQueue]
    |-- IMU   --> [ImuQueue]  --+
    |-- Camera--> [ImgQueue]    |
                                 |
[IMUPropagator jthread] <---------+  (250Hz, 独立于主循环)
    |-- EkfStateManager::predict()
    |
[Orchestrator 主循环]  (10Hz)
    |
    +-- 1. sync_packages()
    |       LidarQueue + ImuQueue + ImgQueue -> SyncPacket
    |
    +-- 2. LidarProcessor::preprocess(scan, state)
    |       -> 去畸变 + 降采样
    |
    +-- 3. LIOEngine::process(processed_scan, state)
    |       -> scan-to-map 匹配
    |       -> EkfStateManager::applyLioCorrection(corrected_state)
    |       -> MapManager::insertLidarPoints(new_points)
    |
    +-- 4. VIOEngine::processFrame(img, state)
    |       -> GaussianSplatting::render(camera_pose)
    |       -> 光度误差计算
    |       -> 雅可比构建
    |       -> EkfStateManager::applyVioUpdate(H, residual, R)
    |       -> GaussianSplatting::insertPoints(new_gs_points)
    |       -> GaussianSplatting::densifyAndPrune()
    |
    +-- 5. 发布结果
            -> Logger (轨迹记录)
            -> Visualizer (ZMQueue 发布)
```

### 3.2 IMU 传播数据流 (独立线程)

```
[ImuQueue] --pop--> [IMUPropagator::run()]
    |
    +-- EkfStateManager::predict(imu, dt)  [unique_lock, 1微秒]
    |
    +-- [VisualizerQueue] 推送最新 pose
```

### 3.3 3DGS CUDA 数据流

```
[VIOEngine::processFrame()]
    |
    +-- GaussianSplatting::prepareRenderData()
    |       CPU: 准备 GS 参数 (positions, opacities, covariances)
    |       H2D Async (transfer stream): 拷贝到 GPU
    |
    +-- GaussianSplatting::render()
    |       Compute Stream: 3DGS forward kernel
    |       D2H Async (transfer stream): 拷贝渲染图像回 CPU
    |
    +-- 光度误差计算 (CPU)
    |
    +-- GaussianSplatting::backward()
    |       Compute Stream: 3DGS backward kernel
    |
    +-- GaussianSplatting::insertPoints()
            H2D Async (transfer stream): 新增点拷贝到 GPU
```

---

## 4. 线程模型

### 4.1 线程清单

| 线程 | 类型 | 频率 | 职责 | 同步方式 |
|------|------|------|------|----------|
| 主线程 | `main()` -> Orchestrator | ~10Hz | sync + LIO + VIO + 发布 | blocking queue wait |
| IMU 传播 | `std::jthread` | 250Hz | predict() 调用 | blocking queue wait |
| LiDAR 采集 | `std::jthread` | 传感器帧率 | 读取硬件 -> 入队 | 无锁入队 |
| Camera 采集 | `std::jthread` | 传感器帧率 | 读取硬件 -> 入队 | 无锁入队 |
| Visualizer | `std::jthread` | ~20Hz | ZMQ 发布 | blocking queue wait |

注: IMU 数据通常与 LiDAR 数据包绑定 (Livox 协议)，IMU 采集由 LiDAR 线程完成。

### 4.2 线程生命周期管理

```cpp
class Application {
    std::jthread imu_propagator_;
    std::jthread lidar_driver_;
    std::jthread camera_driver_;
    std::jthread visualizer_;

    void run() {
        // jthread 构造函数自动传递 stop_token
        imu_propagator_ = std::jthread([this](std::stop_token st) {
            imuPropagateLoop(st);
        });
        lidar_driver_ = std::jthread([this](std::stop_token st) {
            lidarDriverLoop(st);
        });
        camera_driver_ = std::jthread([this](std::stop_token st) {
            cameraDriverLoop(st);
        });
        visualizer_ = std::jthread([this](std::stop_token st) {
            visualizerLoop(st);
        });

        orchestratorLoop();  // 主线程阻塞在此

        // jthread 析构自动 request_stop() + join()
    }
};
```

### 4.3 同步点

| 同步点 | 机制 | 参与者 |
|--------|------|--------|
| EkfStateManager 读 | `shared_lock` | IMU 传播、VIO 构建雅可比时 |
| EkfStateManager 写 | `unique_lock` | Orchestrator(LIO/VIO update)、IMU 传播(predict) |
| IMU 队列消费 | `condition_variable_any` + `stop_token` | IMU 传播线程 |
| 同步包获取 | 带超时的 blocking pop | Orchestrator |
| 可视化数据推送 | blocking pop | Visualizer 线程 |

### 4.4 竞态消除对比

原版存在三个执行上下文竞争同一 `_state`:

```
原版:
  Main Loop (10Hz)   -- 直接修改 _state
  IMU Callback (250Hz) -- 直接读取 _state
  OpenMP 线程          -- 同时读取 _state.rot_end
  -> 无保护，竞态

重写:
  Orchestrator    -- EkfStateManager::applyVioUpdate() (unique_lock)
  IMUPropagator   -- EkfStateManager::predict() (unique_lock, ~1微秒)
  VIO 构建雅可比   -- EkfStateManager::getState() (shared_lock)
  -> 读写锁保护，读可并行，写独占
```

---

## 5. 进程/线程拓扑分析

### 5.1 方案评估

| 方案 | 优点 | 缺点 |
|------|------|------|
| **单进程多线程** | 零 IPC 延迟，共享内存，调试简单 | 一个线程崩溃影响全部 |
| 双进程 (SLAM + Viewer) | 视图进程崩溃不影响 SLAM | IPC 延迟，序列化开销 |
| 多进程 (LIO + VIO + Viewer) | 最强隔离性 | 数据拷贝多倍，同步复杂 |

### 5.2 决策: 单进程多线程 (核心) + 可选 ZeroMQ Viewer (外挂)

**核心 SLAM:** 单进程，所有模块通过方法调用 + 并发队列通信。

**可视化:** ZeroMQ PUB/SUB — SLAM 进程作为 PUB，外部 Viewer 作为 SUB 订阅。Visualizer 线程在 SLAM 进程内将数据序列化后发布。Viewer 进程独立，可崩溃不影响 SLAM。

```
+-----------------------------------+
|  gs-livo (单进程)                  |
|                                    |
|  Orchestrator (主线程)             |
|  IMUPropagator (jthread)           |
|  LiDAR Driver (jthread)            |
|  Camera Driver (jthread)           |
|  Visualizer (jthread)              |
|       |                            |
|       | ZMQ PUB                    |
+------------------------------------+
        |
        | (网络或本地 IPC)
        v
+------------------+
|  external_viewer  |  (独立进程)
|  (ZMQ SUB)        |
+-------------------+
```

### 5.3 为什么不是多进程

1. **SLAM 是延迟敏感系统**: 增加 IPC 序列化/反序列化会引入 1-5ms 延迟
2. **共享地图数据量大**: 点云地图可达数百 MB，跨进程共享需要共享内存机制
3. **单进程已足够隔离**: 使用 jthread + stop_token 可以实现优雅退出。CUDA 错误、EKF 发散等错误通过异常向上传播，Orchestrator 统一处理

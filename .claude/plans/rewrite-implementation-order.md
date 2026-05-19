# GS-LIVO Rewrite: 实施顺序

## 总览

10 个阶段，完全从零构建。每个阶段产出**可编译、可验证**的成果。阶段顺序确保:
1. 基础先于依赖它的上层
2. 每个阶段有明确的验证条件
3. 无 ROS 依赖 — 所有阶段原生可构建
4. 尽早建立测试基础设施

```
Phase 1: 基础设施       Phase 2: 传感器               Phase 3: IMU+EKF
  core/types.h            SensorDriver                  IMUPreintegrator
  ConfigManager           SensorSynchronizer            EkfStateManager
  Logger                  ConcurrentQueue 完善           IMUPropagator (jthread)
  Build system            (可录制/回放数据)
                              |                              |
                              v                              v
Phase 4: LIO             Phase 5: 3DGS               Phase 6: VIO
  LidarProcessor           lib3dgs submodule            VIOEngine
  VoxelMap / Octree<T>     GaussianSplatting            光度误差
  LIOEngine                CUDA 双流                    雅可比构建
  scan-to-map 匹配         持久化 GS 状态                EKF 更新
                              |                              |
                              +--------------+---------------+
                                             |
                                     Phase 7: Orchestrator
                                       主循环调度器
                                       状态机 (LIVO/LIO/LO)
                                       优雅关闭
                                             |
                              +--------------+---------------+
                              |                              |
                    Phase 8: Visualizer            Phase 9: 性能优化
                      ZMQ 发布                       CUDA 流重叠
                      轨迹记录                       内存池
                      外部 Viewer                    GS 增量更新
                              |                              |
                              +--------------+---------------+
                                             |
                                     Phase 10: 测试 + 调优
                                       单元测试覆盖
                                       rosbag 回放回归
                                       轨迹精度验证
                                       延迟分析
```

---

## Phase 1: 基础设施

**目标:** 可编译的空项目 + 消息类型 + 配置 + 日志

### 任务清单

- [x] 创建顶层 CMakeLists.txt + cmake/ 辅助模块
- [x] `src/core/types.h` — 所有消息类型 (ImuData, LidarScan, ImageData, StatesGroup, SyncPacket, etc.)
- [x] `src/core/config.h + .cpp` — ConfigManager + 配置结构体，YAML 解析
- [x] `src/core/concurrent_queue.h` — 线程安全队列模板
- [x] `src/core/logger.h` — spdlog 包装
- [x] `src/main.cpp` — 空的入口，仅加载配置 + 日志
- [x] 安装 clang-format + .clang-format
- [x] 安装 GTest (find_package) + 空测试

### 验证条件

```
$ cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
$ cmake --build build --parallel
$ ./build/gs_livo --config config/default.yaml
  [2026-05-19 10:00:00] [info] GS-LIVO v0.1.0 starting...
$ cd build && ctest
  No tests were found  (测试框架就绪但无测试)
```

### 产出文件

```
src/core/types.h
src/core/config.h + config.cpp
src/core/concurrent_queue.h
src/core/logger.h
src/main.cpp
CMakeLists.txt
cmake/CompilerOptions.cmake
cmake/CudaOptions.cmake
cmake/Dependencies.cmake
cmake/InstallRules.cmake
config/default.yaml
```

---

## Phase 2: 传感器驱动 + 同步

**目标:** 从文件/回放读取传感器数据，输出同步包

### 任务清单

- [ ] `src/sensor/lidar_driver.h + .cpp` — ILidarDriver 实现
  - 文件回放模式: 读取录制的 LiDAR 数据
  - 硬件模式: 读取 Livox UDP 数据包 (后续阶段)
- [ ] `src/sensor/camera_driver.h + .cpp` — ICameraDriver 实现
  - 文件回放模式: 读取图像序列
  - 硬件模式: OpenCV VideoCapture
- [ ] `src/sensor/sensor_synchronizer.h + .cpp`
  - 状态机: WAIT_IMU -> WAIT_LIDAR -> WAIT_IMG -> SYNCED
  - 超时处理与降级策略
  - 输出 SyncPacket
- [ ] 支持 rosbag 回放模式 (通过第三方库或自行解析)
- [ ] 单元测试: 给定时间戳偏移的数据，验证对齐正确性

### 验证条件

```
$ ./build/gs_livo --config config/default.yaml --bag test_data.bag
  [2026-05-19 10:00:00] [info] Sensor sync: 150 packets in, 148 synced, 2 dropped
$ cd build && ctest -R sensor
  All tests passed (4 passed)
```

### 关键设计决策

**回放数据格式:** 使用自定义二进制录制格式而非直接解析 rosbag (rosbag 有 license 依赖)。提供 `record` 工具将 rosbag 转换为自有格式。

---

## Phase 3: IMU 处理 + EKF 状态管理

**目标:** 独立运行的 IMU 传播线程 + 线程安全的 EKF 状态管理

### 任务清单

- [ ] `src/ekf/ekf_state_manager.h + .cpp`
  - StatesGroup 封装
  - shared_mutex 读写锁
  - `predict()`: IMU 传播 (EKF 预测步骤)
  - `applyVioUpdate()`: 视觉 EKF 更新 (桩函数，Phase 6 实现)
  - `applyLioCorrection()`: LiDAR 修正 (桩函数，Phase 4 实现)
- [ ] `src/imu/imu_preintegrator.h + .cpp` — IMU 预积分
- [ ] `src/imu/imu_propagator.h + .cpp` — std::jthread 250Hz 循环
  - 从 ImuQueue 消费
  - 调用 EkfStateManager::predict()
  - 使用 stop_token 控制停止
- [ ] 单元测试:
  - 并发场景: 多线程同时读写，验证无数据竞争
  - EKF predict: 给定已知 IMU 输入，验证状态传播
  - IMU 预积分: 数值积分精度验证

### 验证条件

```
$ ./build/gs_livo --config config/default.yaml --bag test_data.bag
  [info] IMU propagator started (250 Hz)
  [info] State: pos=(0.01, 0.02, -0.01) vel=(0.1, 0.0, 0.0)
  ...
$ cd build && ctest -R ekf
  All tests passed (8 passed)
$ # 压力测试: 100 线程并发读，1 线程写
  No data races detected (TSAN clean)
```

---

## Phase 4: LiDAR 预处理 + LIO

**目标:** 可运行的 LiDAR 惯性里程计，输出修正后的 EKF 状态

### 任务清单

- [ ] `src/lio/lidar_processor.h + .cpp`
  - 体素降采样 (voxel grid filter)
  - 运动畸变补偿（使用 EKF 状态）
- [ ] `src/map/octree.h` — Octree\<T\> 模板 (header-only)
  - 插入、查找、范围查询
  - LidarOctree 特化: 体素平面拟合
  - GSOctree 特化: 叶节点列表维护
- [ ] `src/map/map_manager.h + .cpp` — IMapManager 实现
  - 维护 LidarOctree 和 GSOctree
  - 查询接口
- [ ] `src/lio/voxel_map.h + .cpp` — 体素地图 + 平面拟合
- [ ] `src/lio/lio_engine.h + .cpp` — ILIOEngine 实现
  - scan-to-map 匹配 (点到面 ICP)
  - 残差构建
  - 调用 EkfStateManager::applyLioCorrection()
  - 向 MapManager 插入新点
- [ ] 单元测试:
  - Octree 插入/查询: 验证空间正确性
  - LidarProcessor: 畸变补偿精度
  - LIOEngine: 给定已知输入的匹配结果

### 验证条件

```
$ ./build/gs_livo --config config/default.yaml --bag test_data.bag
  [info] LIO enabled
  [info] LIO: match_count=845 residual=0.023
  [info] State: pos=(1.23, 4.56, -0.78)
$ cd build && ctest -R "lio|map|octree"
  All tests passed (15 passed)
```

### 与原版的差异

| 原版 | 重写 |
|------|------|
| VoxelOctoTree 和 GSVoxelOctree 独立实现 | Octree\<T\> 模板 + 策略特化 |
| 直接访问 LIVMapper._state | 通过 EkfStateManager 更新 |
| 扫描数据直接操作裸指针 | ProcessedLidarScan 值语义 |

---

## Phase 5: 3DGS 引擎集成

**目标:** lib3dgs 作为 submodule 集成，持久化 GaussianModel，CUDA 双流

### 任务清单

- [ ] 添加 lib3dgs git submodule: `git submodule add <url> external/lib3dgs`
- [ ] 修复 lib3dgs CMakeLists.txt 中的 PROJECT_SOURCE_DIR -> CMAKE_CURRENT_SOURCE_DIR
- [ ] 修复 lib3dgs 中的硬编码路径
- [ ] `src/gs/gs_types.h` — 桥接类型 (GsPoint <-> lib3dgs 内部格式)
- [ ] `src/gs/gaussian_splatting.h + .cpp` — IGaussianSplatting 实现
  - 持久化 GaussianModel（不再每帧重建）
  - Create_from_our_format() 追加模式 (append=true)
  - render() 封装 lib3dgs forward
  - backward() 封装 lib3dgs backward
  - insertPoints() / densifyAndPrune() / resetOpacity()
- [ ] CUDA 双流实现:
  - compute_stream: forward/backward kernels
  - transfer_stream: H2D/D2H async copies
  - cudaEvent 同步
- [ ] 验证 3DGS 持久化: 运行 100 帧，GS 点持续增长而非每帧重置

### 验证条件

```
$ cmake -B build ... -DCMAKE_BUILD_TYPE=Release
$ cmake --build build --parallel
$ # lib3dgs 成功作为子项目编译
$ ./build/gs_livo --config config/default.yaml --bag test_data.bag
  [info] 3DGS initialized (CUDA compute cap 8.6)
  [info] GS render: 1280x720, 15000 points, 4.2ms (compute)
  [info] GS render: 1280x720, 15230 points, 4.1ms (compute)
  [info] GS render: 1280x720, 15891 points, 4.3ms (compute)
  # points 持续增长，非每帧重置
$ nvidia-smi
  GPU-Util: 65%  (较原版估计提升 20%+)
```

### CUDA 双流关键代码 (概念验证)

```cpp
// src/gs/gaussian_splatting.cpp
bool GaussianSplatting::render(cudaStream_t compute_stream,
                                cudaStream_t transfer_stream) {
    // Transfer stream: 异步上传 GS 数据到 GPU
    cudaMemcpyAsync(gpu_positions_, cpu_positions_, bytes,
                    cudaMemcpyHostToDevice, transfer_stream);
    cudaMemcpyAsync(gpu_opacities_, cpu_opacities_, bytes,
                    cudaMemcpyHostToDevice, transfer_stream);

    // 同步: 等待传输完成
    cudaEventRecord(transfer_done_, transfer_stream);
    cudaStreamWaitEvent(compute_stream, transfer_done_);

    // Compute stream: forward kernel
    CudaRasterizer::Rasterizer::forward(compute_stream, ...);

    // Compute stream: 记录完成事件
    cudaEventRecord(render_done_, compute_stream);

    // Transfer stream: 等待渲染完成，异步下载结果
    cudaStreamWaitEvent(transfer_stream, render_done_);
    cudaMemcpyAsync(cpu_image_, gpu_image_, image_bytes,
                    cudaMemcpyDeviceToHost, transfer_stream);

    cudaStreamSynchronize(transfer_stream);  // 最终同步
    return true;
}
```

---

## Phase 6: VIO + 3DGS 融合

**目标:** 完整的视觉惯性里程计，使用 3DGS 渲染进行光度跟踪

### 任务清单

- [ ] `src/vio/vio_engine.h + .cpp` — IVIOEngine 实现
  - `processFrame()` 主流程:
    1. 从 EkfStateManager 获取当前状态
    2. 调用 GaussianSplatting::render() 渲染预测视图
    3. 计算渲染图与原图的光度残差
    4. 构建光度雅可比矩阵 (200+ 行 updateState_gs 拆分)
  - `computePhotometricError()` 提取
  - `buildJacobian()` 提取
  - `ekfUpdate()` 提取
- [ ] 3DGS 增量更新:
  - 新点插入: VIO 中判别新区域，调用 insertPoints()
  - 致密化: 每 N 帧调用 densifyAndPrune()
- [ ] `src/vio/vio_gs.h + .cpp` — VIO 特有的 GS 操作
  - GS 点到体素地图的映射
  - GS 点可视化格式转换

### EKF 更新拆分

```cpp
// src/vio/vio_engine.cpp

struct PhotometricResult {
    std::vector<double> residuals;
    int valid_pixel_count;
    double mean_error;
};

PhotometricResult VIOEngine::computePhotometricError(
    const cv::Mat& rendered, const cv::Mat& original, int border) {
    // 从原版 updateState_gs() 提取
    // 逐像素比较，边缘裁剪
    // 输出残差向量
}

struct Jacobian {
    Eigen::MatrixXd H;   // n x 19
    Eigen::VectorXd z;   // n x 1
    Eigen::MatrixXd R;   // n x n (噪声协方差)
};

Jacobian VIOEngine::buildJacobian(
    const PhotometricResult& error, const StatesGroup& state) {
    // 从原版 updateState_gs() 提取
    // 链式法则: 光度误差对位姿的雅可比
    // 输出 H, z, R
}

void VIOEngine::ekfUpdate(
    const Jacobian& jac, EkfStateManager& ekf) {
    // 标准 EKF 更新方程:
    // K = P * H^T * (H * P * H^T + R)^-1
    // x = x + K * z
    // P = (I - K * H) * P

    ekf.applyVioUpdate(jac.H, jac.z, jac.R);
}
```

### 验证条件

```
$ ./build/gs_livo --config config/default.yaml --bag test_data.bag
  [info] VIO enabled
  [info] Frame 10: photometric_error=0.042 gs_points=15230
  [info] Frame 20: photometric_error=0.038 gs_points=16541
  [info] Frame 30: photometric_error=0.031 gs_points=17892
  # 光度误差随跟踪收敛下降
$ cd build && ctest -R vio
  All tests passed (10 passed)
```

---

## Phase 7: Orchestrator 主循环

**目标:** 整合所有模块的完整 SLAM 系统

### 任务清单

- [ ] `src/orchestrator/orchestrator.h + .cpp`
  - 主循环:
    ```
    while (!stop_requested) {
        1. sync_packages() -> SyncPacket
        2. LidarProcessor::process()
        3. LIOEngine::process()
        4. VIOEngine::processFrame()
        5. Logger::record()
        6. Visualizer::publish()  (if enabled)
    }
    ```
  - 系统模式切换: LIVO / LIO / LO / VIO
  - 异常处理: EKF 发散检测 -> 降级模式 -> 记录错误
  - 优雅关闭: SIGINT/SIGTERM -> request_stop() -> join all threads
- [ ] ConfigManager 运行时重载 (ZeroMQ command)
- [ ] 集成测试: 加载 rosbag 数据，运行完整 pipeline

### 模式切换

```cpp
void Orchestrator::runOnce() {
    auto pkt = synchronizer_.syncPackages();

    switch (mode_) {
    case SystemMode::LIVO:
    case SystemMode::LIO: {
        auto processed = lidar_processor_.process(pkt.lidar, ekf_.getState());
        auto lio_result = lio_engine_.process(processed, ekf_.getState());
        if (lio_result.success) {
            ekf_.applyLioCorrection(lio_result.corrected_state);
            map_.insertLidarPoints(processed.points);
        }
        if (mode_ != SystemMode::LIVO) break;
        [[fallthrough]];
    }
    case SystemMode::VIO: {
        auto vio_result = vio_engine_.processFrame(pkt.image, ekf_.getState(), ekf_);
        if (vio_result.success && mode_ == SystemMode::VIVO) {
            map_.insertGsPoints(/* from vio_result */);
        }
        break;
    }
    case SystemMode::LO:
        // LiDAR-only fallback
        break;
    }

    logger_.record(ekf_.getState());
    viz_.publish(ekf_.getState(), map_);
}
```

### 验证条件

```
$ ./build/gs_livo --config config/default.yaml --bag test_data.bag
  # 实时输出:
  [info] Frame   1 | LIO: 845 pts | VIO: err=0.042 | pos=(0.01, 0.02, 0.00)
  [info] Frame  10 | LIO: 912 pts | VIO: err=0.038 | pos=(0.12, 0.45, -0.01)
  ...
  [info] Frame 100 | LIO: 1024 pts | VIO: err=0.031 | pos=(1.23, 4.56, -0.78)
  [info] === Trajectory saved to output/trajectory.txt ===
$ # 验证轨迹输出文件存在
  Test-Path output/trajectory.txt
  True
$ cd build && ctest
  All tests passed (38 passed)
```

---

## Phase 8: 可视化 + 日志记录

**目标:** 可观察系统运行状态

### 任务清单

- [ ] ZeroMQ PUB 套接字: 发布 SLAM 状态、点云、图像、轨迹
- [ ] 序列化: 简单二进制格式 (或 FlatBuffers/Protobuf)
- [ ] 轨迹记录: TUM/ETH 格式输出
- [ ] 点云保存: 按帧保存或关键帧保存
- [ ] 日志: spdlog 文件回滚 + 控制台输出
- [ ] 外部 Viewer 样例 (Python ZMQ SUB 客户端)

### 验证条件

```
# 终端 1: SLAM 进程
$ ./build/gs_livo --config config/default.yaml --bag test_data.bag --viz tcp://*:5555

# 终端 2: 外部 viewer
$ python scripts/viewer.py --endpoint tcp://localhost:5555
  # 实时显示: 轨迹、点云、当前图像
```

---

## Phase 9: 性能优化

**目标:** 达到原版目标性能 (30fps+)

### 任务清单

- [ ] CUDA 双流重叠验证 (Nsight Systems)
  - 确保 compute 和 transfer 流有效重叠
  - GPU 利用率目标: >60%
- [ ] 3DGS 增量更新优化
  - GS 点追加而非重建
  - 训练迭代数从 4 -> 1-2
- [ ] 内存池: GS_point 预分配池，避免每帧 new/delete
  - `GsPointPool` 类，固定大小块分配
- [ ] EkfStateManager 锁优化
  - 测量锁竞争: 250Hz predict 的 unique_lock 开销
  - 如果必要: 引入无锁 PropagationState 缓存
- [ ] MapManager 查询优化
  - Octree 范围查询剪枝
  - 局部地图窗口 (sliding window)

### 验证条件

```
$ # Nsight Systems 结果
  GPU utilization: 65%  (原版约 40%)
  Compute-transfer overlap: 75% of frame time

$ # 端到端延迟
  Frame processing: 28ms avg (35 fps)  (原版约 45ms)
    LIO:       8ms
    VIO+3DGS:  18ms
    Overhead:   2ms

$ # 内存
  Memory allocation: 减少 60% (内存池)
  No per-frame new/delete in hot path
```

---

## Phase 10: 测试 + 调优

**目标:** 回归测试覆盖 + 轨迹精度验证

### 任务清单

- [ ] 单元测试覆盖:
  - EkfStateManager: 并发读写、predict、update
  - Octree\<T\>: 插入、查找、范围查询、边界条件
  - LidarProcessor: 畸变补偿精度
  - SensorSynchronizer: 时间对齐、超时、降级
  - LIOEngine: scan-to-map 匹配
  - VIOEngine: computePhotometricError、buildJacobian、ekfUpdate
  - ConfigManager: YAML 解析、默认值、错误处理
- [ ] 集成测试:
  - rosbag 回放管道: sensor -> sync -> LIO -> VIO -> publish
  - 轨迹精度对比 (ATE): 与原始二进制输出对比
- [ ] 性能回归:
  - 每帧处理时间 (< 33ms)
  - GPU 利用率 (> 60%)
  - 内存增长 (线性，可控)
- [ ] 压力测试:
  - 100x 回放: 无崩溃
  - ASAN: 无内存错误
  - TSAN: 无数据竞争

### 测试目录结构 (最终)

```
tests/
|-- CMakeLists.txt
|-- ekf/
|   |-- test_ekf_state_manager.cpp    # 并发读写测试
|   |-- test_ekf_predict.cpp          # IMU 传播精度
|   +-- test_ekf_update.cpp           # EKF 更新正确性
|-- map/
|   |-- test_octree.cpp               # Octree 空间查询
|   +-- test_map_manager.cpp          # MapManager 接口
|-- sensor/
|   |-- test_synchronizer.cpp          # 时间同步
|   +-- test_drivers.cpp              # 驱动回放
|-- lio/
|   |-- test_lidar_processor.cpp      # 去畸变
|   +-- test_lio_engine.cpp           # scan-to-map
|-- vio/
|   |-- test_photometric_error.cpp    # 光度误差
|   +-- test_jacobian.cpp             # 雅可比构建
|-- integration/
|   +-- test_full_pipeline.cpp        # 端到端回放
+-- benchmarks/
    +-- benchmark_octree.cpp          # 八叉树性能
```

### 验证条件

```
$ cd build && ctest --output-on-failure
  Test #1:  ekf_state_manager ................. PASSED
  Test #2:  ekf_predict ...................... PASSED
  Test #3:  ekf_update ....................... PASSED
  Test #4:  octree_insert_query .............. PASSED
  Test #5:  octree_edge_cases ................ PASSED
  Test #6:  map_manager ...................... PASSED
  Test #7:  sensor_synchronizer .............. PASSED
  Test #8:  lidar_processor .................. PASSED
  Test #9:  lio_engine ....................... PASSED
  Test #10: photometric_error ................ PASSED
  Test #11: jacobian_build ................... PASSED
  Test #12: full_pipeline .................... PASSED
  ---
  12 tests passed, 0 failed

$ # 轨迹精度
$ python scripts/eval_trajectory.py output/traj.txt ground_truth.txt
  ATE: 0.042m (baseline: 0.039m)
  Relative error: +7.7% (acceptable)
```

---

## 各阶段依赖关系

```
Phase 1 (基础设施)
   |
   v
Phase 2 (传感器)
   |
   v
Phase 3 (IMU+EKF)
   |
   +---->----+
   |         |
   v         v
Phase 4   Phase 5
(LIO)     (3DGS)
   |         |
   +---->----+
   |         |
   v         v
Phase 6 (VIO)
   |
   v
Phase 7 (Orchestrator)
   |
   +---->----+
   |         |
   v         v
Phase 8   Phase 9
(Viz)     (Perf)
   |         |
   +---->----+
   |
   v
Phase 10 (Test+Tune)
```

- Phase 8 (可视化) 和 Phase 9 (性能优化) 可并行
- Phase 4 (LIO) 和 Phase 5 (3DGS) 可并行
- 单元测试编写与对应模块开发同步进行 (Phase 2-7 每个阶段包含测试)

## 估算工作量

| 阶段 | 估算 (人天) | 并行可能性 |
|------|------------|-----------|
| Phase 1: 基础设施 | 2 | — |
| Phase 2: 传感器 | 3 | — |
| Phase 3: IMU+EKF | 4 | — |
| Phase 4: LIO | 5 | 与 Phase 5 并行 |
| Phase 5: 3DGS | 5 | 与 Phase 4 并行 |
| Phase 6: VIO | 5 | — |
| Phase 7: Orchestrator | 3 | — |
| Phase 8: Visualizer | 2 | 与 Phase 9 并行 |
| Phase 9: 性能优化 | 4 | 与 Phase 8 并行 |
| Phase 10: 测试+调优 | 5 | — |
| **合计** | **38 人天 (约 8 周)** | 可压缩至 6 周 |

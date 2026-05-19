# GS-LIVO Rewrite: 核心接口定义

## 1. 消息类型定义 (替代 ROS msg)

所有消息类型位于 `src/core/types.h`，C++20 标准布局类型。

```cpp
// ============================================================
// src/core/types.h
// ============================================================
#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cstdint>
#include <array>
#include <opencv2/core.hpp>

namespace gs_livo {

// --- 基础数学类型 ---
using V3D = Eigen::Vector3d;
using V2D = Eigen::Vector2d;
using M3D = Eigen::Matrix3d;
using Q4D = Eigen::Quaterniond;
using VXD = Eigen::VectorXd;
using MXD = Eigen::MatrixXd;

// --- 时间戳 ---
using Timestamp = double;  // 秒，统一浮点时间

// --- IMU 数据 ---
struct ImuData {
    Timestamp timestamp;
    V3D        accel;   // m/s^2
    V3D        gyro;    // rad/s
};

// --- LiDAR 点 ---
struct LidarPoint {
    V3D    position;   // x, y, z
    float  intensity;
    float  time_offset;  // 相对于扫描起始的时间偏移（去畸变用）
    uint16_t line;       // LiDAR 线号
};

// --- LiDAR 扫描 ---
struct LidarScan {
    Timestamp          timestamp;     // 扫描起始时间
    std::vector<LidarPoint> points;
};

// --- 处理后 LiDAR 扫描 ---
struct ProcessedLidarScan {
    Timestamp          timestamp;
    std::vector<LidarPoint> points;
    std::vector<V3D>   point_normals;  // 法向量（用于平面拟合）
};

// --- 图像数据 ---
struct ImageData {
    Timestamp  timestamp;
    cv::Mat    image;    // 灰度图 CV_8UC1
    int        width;
    int        height;
};

// --- 3DGS 点 ---
struct GsPoint {
    V3D     position;
    V3D     scale;
    Q4D     rotation;    // 四元数表示的协方差旋转
    float   opacity;
    std::array<float, 48> sh_coeffs;  // 球谐系数（取决于阶数）
};

// --- EKF 19 维状态 ---
struct StatesGroup {
    // 0-2: position (world)
    // 3-5: velocity (body)
    // 6-9: orientation (quaternion, wxyz)
    // 10-12: gyro bias
    // 13-15: accel bias
    // 16-18: gravity
    V3D    pos   = V3D::Zero();
    V3D    vel   = V3D::Zero();
    Q4D    rot   = Q4D::Identity();
    V3D    bg    = V3D::Zero();  // gyro bias
    V3D    ba    = V3D::Zero();  // accel bias
    V3D    grav  = V3D(0, 0, -9.81);

    // 协方差
    Eigen::Matrix<double, 19, 19> cov = Eigen::Matrix<double, 19, 19>::Identity();
};

// --- 同步传感器包 ---
struct SyncPacket {
    Timestamp              timestamp;
    LidarScan              lidar;
    std::vector<ImuData>   imu_window;  // 上次 LIO/VIO 更新以来的 IMU 数据
    ImageData              image;
};

// --- 外参标定 ---
struct Extrinsics {
    V3D translation = V3D::Zero();
    Q4D rotation    = Q4D::Identity();
};

struct SensorExtrinsics {
    Extrinsics lidar_to_imu;
    Extrinsics camera_to_imu;
};

// --- 系统模式 ---
enum class SystemMode : uint8_t {
    LIVO,     // 全模式: LiDAR + IMU + Visual + 3DGS
    LIO,      // LiDAR + IMU only
    LO,       // LiDAR only
    VIO       // Visual + IMU only
};

// --- LIO 输出 ---
struct LioResult {
    StatesGroup corrected_state;
    bool        success;
    int         match_count;
    double      residual;
};

// --- VIO 输出 ---
struct VioResult {
    StatesGroup corrected_state;
    bool        success;
    cv::Mat     rendered_image;  // 3DGS 渲染图
    double      photometric_error;
    int         gs_point_count;
    int         new_points_added;
};

}  // namespace gs_livo
```

## 2. 并发队列

```cpp
// ============================================================
// src/core/concurrent_queue.h
// ============================================================
#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <stop_token>

namespace gs_livo {

template<typename T>
class ConcurrentQueue {
public:
    ConcurrentQueue() = default;
    ~ConcurrentQueue() { clear(); }

    // 非阻塞入队
    void push(T item) {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            queue_.push(std::move(item));
        }
        cv_.notify_one();
    }

    // 非阻塞出队，返回 false 表示队列空
    bool tryPop(T& out) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (queue_.empty()) return false;
        out = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    // 阻塞出队（响应 stop_token）
    T waitPop(std::stop_token stoken) {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, stoken, [this] { return !queue_.empty(); });
        if (queue_.empty()) return T{};  // 被 stop 唤醒
        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

    // 阻塞出队（普通版本）
    T waitPop() {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this] { return !queue_.empty(); });
        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.size();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mtx_);
        while (!queue_.empty()) queue_.pop();
    }

private:
    std::queue<T>              queue_;
    mutable std::mutex          mtx_;
    std::condition_variable_any cv_;
};

}  // namespace gs_livo
```

## 3. EkfStateManager 完整类设计

```cpp
// ============================================================
// src/ekf/ekf_state_manager.h
// ============================================================
#pragma once
#include "core/types.h"
#include <shared_mutex>

namespace gs_livo {

class EkfStateManager {
public:
    EkfStateManager() = default;

    // --- 读操作 (shared_lock) ---

    // 获取完整 EKF 状态快照（VIO/LIO 处理使用）
    StatesGroup getState() const {
        std::shared_lock lock(mtx_);
        return state_;
    }

    // 获取最新传播状态（IMU 传播用，可能未包含最新 VIO/LIO 修正）
    StatesGroup getLatest() const {
        std::shared_lock lock(mtx_);
        return latest_;
    }

    // 获取协方差
    Eigen::Matrix<double, 19, 19> getCovariance() const {
        std::shared_lock lock(mtx_);
        return state_.cov;
    }

    // --- 写操作 (unique_lock) ---

    // 直接设置状态（初始化时使用）
    void setState(const StatesGroup& state) {
        std::unique_lock lock(mtx_);
        state_ = state;
        latest_ = state;
    }

    // IMU 传播: 更新 state_.pos, vel, rot 等基于 IMU 测量
    // 输入: IMU 测量值 + 时间间隔
    // 内部: unique_lock 保护，微秒级完成
    void predict(const ImuData& imu, double dt);

    // VIO EKF 更新: 使用光度残差修正状态
    // H: 雅可比矩阵 (n x 19)
    // residual: 观测残差 (n x 1)
    // R: 观测噪声协方差 (n x n)
    void applyVioUpdate(const Eigen::MatrixXd& H,
                        const Eigen::VectorXd& residual,
                        const Eigen::MatrixXd& R);

    // LIO 修正: 使用 scan-to-map 匹配结果修正状态
    void applyLioCorrection(const StatesGroup& corrected);

    // 重置状态
    void reset() {
        std::unique_lock lock(mtx_);
        state_ = StatesGroup{};
        latest_ = StatesGroup{};
    }

private:
    mutable std::shared_mutex mtx_;
    StatesGroup state_;    // 最新 EKF 状态（VIO/LIO 更新后）
    StatesGroup latest_;   // IMU 传播后的最新状态

    // --- EKF 核心方法 ---
    void predictState(const ImuData& imu, double dt);
    void updateState(const Eigen::MatrixXd& H,
                     const Eigen::VectorXd& residual,
                     const Eigen::MatrixXd& R);
};

}  // namespace gs_livo
```

### 与原版的关键差异

| 原版 | 重写 |
|------|------|
| `StatesGroup state_` 是 LIVMapper 的 public 成员 | `EkfStateManager` 封装，私有成员 |
| VIOManager 通过 `StatesGroup*` 裸指针访问 | VIOManager 通过 `EkfStateManager&` 引用访问 |
| 无锁竞争 | `std::shared_mutex` 保护 |
| EKF 数学分布在各个模块 | EKF 预测/更新集中在 EkfStateManager |
| 协方差在调用者处维护 | 协方差在 EkfStateManager 内部维护 |

## 4. 模块间抽象接口

```cpp
// ============================================================
// src/interfaces/ 目录下所有纯虚类
// ============================================================
#pragma once
#include "core/types.h"

namespace gs_livo {

// --- ILidarDriver ---
class ILidarDriver {
public:
    virtual ~ILidarDriver() = default;

    // 启动传感器采集（启动内部线程）
    virtual void start() = 0;

    // 停止传感器采集
    virtual void stop() = 0;

    // 获取输出队列引用
    virtual ConcurrentQueue<LidarScan>& lidarQueue() = 0;
    virtual ConcurrentQueue<ImuData>&   imuQueue()   = 0;

    // 传感器参数
    virtual std::string sensorModel() const = 0;
};

// --- ICameraDriver ---
class ICameraDriver {
public:
    virtual ~ICameraDriver() = default;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual ConcurrentQueue<ImageData>& imageQueue() = 0;
};

// --- ILidarProcessor ---
class ILidarProcessor {
public:
    virtual ~ILidarProcessor() = default;

    // 预处理: 去畸变 + 降采样
    virtual ProcessedLidarScan process(
        const LidarScan& raw,
        const StatesGroup& state) = 0;

    // 设置外参
    virtual void setExtrinsics(const Extrinsics& ext) = 0;
};

// --- ILIOEngine ---
class ILIOEngine {
public:
    virtual ~ILIOEngine() = default;

    // 处理一帧 LiDAR 数据
    // 输入: 预处理后的扫描 + 当前 EKF 状态
    // 输出: 修正后的状态 + 匹配统计
    virtual LioResult process(
        const ProcessedLidarScan& scan,
        const StatesGroup& current_state) = 0;

    // 设置地图管理器引用
    virtual void setMapManager(class IMapManager* map) = 0;

    // 配置
    virtual void configure(const LioConfig& config) = 0;
};

// --- IGaussianSplatting ---
class IGaussianSplatting {
public:
    virtual ~IGaussianSplatting() = default;

    // 渲染: 给定位姿渲染图像
    // stream: CUDA 计算流
    virtual cv::Mat render(
        const V3D& camera_pos,
        const Q4D& camera_rot,
        const CameraIntrinsics& intrinsics,
        cudaStream_t stream) = 0;

    // 反向传播: 计算光度残差对 GS 参数的梯度
    virtual void backward(
        const cv::Mat& rendered,
        const cv::Mat& target,
        cudaStream_t stream) = 0;

    // 插入新的 GS 点
    virtual void insertPoints(const std::vector<GsPoint>& points) = 0;

    // 致密化与剪枝
    virtual void densifyAndPrune() = 0;

    // 重置透明度
    virtual void resetOpacity() = 0;

    // 获取当前 GS 点数量
    virtual size_t pointCount() const = 0;

    // 设置 CUDA 流（重写使用的双流模型）
    virtual void setStreams(cudaStream_t compute, cudaStream_t transfer) = 0;
};

// --- IVIOEngine ---
class IVIOEngine {
public:
    virtual ~IVIOEngine() = default;

    // 处理一帧视觉数据
    virtual VioResult processFrame(
        const ImageData& image,
        const StatesGroup& current_state,
        EkfStateManager& ekf) = 0;

    // 设置 3DGS 引擎引用
    virtual void setSplattingEngine(IGaussianSplatting* gs) = 0;

    // 设置外参
    virtual void setExtrinsics(const Extrinsics& camera_to_imu) = 0;
};

// --- IMapManager ---
class IMapManager {
public:
    virtual ~IMapManager() = default;

    // 插入 LiDAR 点（LIO 使用）
    virtual void insertLidarPoints(const std::vector<LidarPoint>& points) = 0;

    // 插入 GS 点（VIO 使用）
    virtual void insertGsPoints(const std::vector<GsPoint>& points) = 0;

    // 查询指定位置附近的 LiDAR 点（scan-to-map 匹配用）
    virtual std::vector<LidarPoint> queryLidarPoints(
        const V3D& center, double radius) const = 0;

    // 查询指定位置附近的 GS 点
    virtual std::vector<GsPoint> queryGsPoints(
        const V3D& center, double radius) const = 0;

    // 保存/加载地图
    virtual bool saveMap(const std::string& path) = 0;
    virtual bool loadMap(const std::string& path) = 0;

    // 清空
    virtual void clear() = 0;
};

// --- IOctree ---
template<typename PointT>
class IOctree {
public:
    virtual ~IOctree() = default;
    virtual void insert(PointT* point) = 0;
    virtual IOctree* find(const V3D& pos) = 0;
    virtual std::vector<PointT*> query(const V3D& center, double radius) = 0;
    virtual bool isLeaf() const = 0;
    virtual int depth() const = 0;
};

}  // namespace gs_livo
```

## 5. 配置结构体

```cpp
// ============================================================
// src/core/config.h
// ============================================================
#pragma once
#include "types.h"
#include <string>
#include <filesystem>

namespace gs_livo {

// --- 相机内参 ---
struct CameraIntrinsics {
    int    width  = 640;
    int    height = 480;
    double fx     = 320.0;
    double fy     = 320.0;
    double cx     = 320.0;
    double cy     = 240.0;
    double k1     = 0.0;  // 径向畸变
    double k2     = 0.0;
    double p1     = 0.0;  // 切向畸变
    double p2     = 0.0;
};

// --- LiDAR 参数 ---
struct LidarConfig {
    std::string model = "livox_avia";
    double  max_range    = 100.0;  // 最大距离(m)
    double  min_range    = 0.5;    // 最小距离(m)
    int     max_points   = 100000;  // 每帧最大点数
    double  time_offset  = 0.0;    // 时间偏移补偿
};

// --- IMU 参数 ---
struct ImuConfig {
    double rate        = 200.0;   // Hz
    double gyro_noise  = 1.7e-4;  // 连续时间噪声密度 (rad/s/rtHz)
    double acc_noise   = 2.0e-3;  // 连续时间噪声密度 (m/s^2/rtHz)
    double gyro_bias_noise = 1.0e-6;
    double acc_bias_noise  = 1.0e-4;
};

// --- LIO 参数 ---
struct LioConfig {
    double voxel_size          = 0.5;   // 体素大小 (m)
    int    max_points_per_voxel = 20;
    double planer_threshold    = 0.1;   // 平面判定阈值
    int    min_plane_points    = 5;     // 最小平面点数
};

// --- VIO / 3DGS 参数 ---
struct VioConfig {
    int    gs_iterations    = 4;     // 每帧 GS 优化迭代数
    int    border           = 10;    // 边缘裁剪像素
    int    img_downsample   = 1;     // 图像降采样因子
    double gs_opacity_thresh = 0.01; // 透明度剪枝阈值
    double gs_scale_thresh   = 0.1;  // 尺度剪枝阈值
    int    gs_max_points     = 50000; // 最大 GS 点数
};

// --- 传感器外参 ---
struct ExtrinsicsConfig {
    // LiDAR -> IMU
    double lidar_to_imu_x = 0.0;
    double lidar_to_imu_y = 0.0;
    double lidar_to_imu_z = 0.0;
    double lidar_to_imu_roll = 0.0;
    double lidar_to_imu_pitch = 0.0;
    double lidar_to_imu_yaw = 0.0;

    // Camera -> IMU
    double camera_to_imu_x = 0.0;
    double camera_to_imu_y = 0.0;
    double camera_to_imu_z = 0.0;
    double camera_to_imu_roll = 0.0;
    double camera_to_imu_pitch = 0.0;
    double camera_to_imu_yaw = 0.0;

    SensorExtrinsics toExtrinsics() const;
};

// --- 系统配置 ---
struct SystemConfig {
    SystemMode mode = SystemMode::LIVO;

    // 各模块开关
    bool use_lio   = true;
    bool use_vio   = true;
    bool use_3dgs  = true;

    // 配置块
    LidarConfig    lidar;
    ImuConfig      imu;
    CameraIntrinsics camera;
    LioConfig      lio;
    VioConfig      vio;
    ExtrinsicsConfig extrinsics;

    // 文件路径
    std::filesystem::path config_path;
    std::filesystem::path output_dir = "./output";
    std::filesystem::param log_dir   = "./logs";

    // 日志级别
    std::string log_level = "info";  // debug, info, warn, error
};

// --- ConfigManager ---
class ConfigManager {
public:
    static ConfigManager& instance();

    bool load(const std::filesystem::path& path);
    bool save(const std::filesystem::path& path) const;

    const SystemConfig& config() const { return config_; }
    SystemConfig&       mutableConfig() { return config_; }

    // 运行时重载: 监听 ZMQ command socket 更新参数
    void startRuntimeReload(const std::string& endpoint);

private:
    ConfigManager() = default;
    SystemConfig config_;
};

}  // namespace gs_livo
```

## 6. 与原有设计的关键差异总结

| 类别 | 原有设计 | 重写设计 |
|------|----------|----------|
| **消息** | ROS msg (sensor_msgs, geometry_msgs) | 自有 POD 类型，Eigen 向量 |
| **状态访问** | 裸指针 `StatesGroup*`，无保护 | `EkfStateManager` with `shared_mutex` |
| **模块通信** | 直接方法调用 + ROS 话题 | 抽象接口 (纯虚类) + ConcurrentQueue |
| **配置** | roslaunch param + yaml 混合 | 纯 YAML，ConfigManager 单例 |
| **八叉树** | 两套代码重复 | `IOctree\<T\>` 接口 + 模板实现 |
| **CUDA 流** | 默认流 | 双流接口参数 |
| **3DGS** | `GaussianModel` 局部变量每帧重建 | `IGaussianSplatting` 持久化成员 |
| **线程** | ROS 回调 (不可控) | `std::jthread` + `std::stop_token` |
| **测试** | 无 | 接口设计支持 mock 注入 |
| **异常处理** | 无 | 异常向上传播到 Orchestrator |

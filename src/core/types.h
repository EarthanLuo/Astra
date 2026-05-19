#pragma once
#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#ifdef GS_LIVO_HAS_OPENCV
#include <opencv2/core.hpp>
#endif

namespace gs_livo {

    // --- Math aliases ---
    using V3D = Eigen::Vector3d;
    using V2D = Eigen::Vector2d;
    using M3D = Eigen::Matrix3d;
    using Q4D = Eigen::Quaterniond;
    using VXD = Eigen::VectorXd;
    using MXD = Eigen::MatrixXd;

    // --- Timestamp ---
    using Timestamp = double;

    // --- IMU measurement ---
    struct ImuData
    {
        Timestamp timestamp = 0.0;
        V3D accel = V3D::Zero();
        V3D gyro = V3D::Zero();
    };

    // --- LiDAR point ---
    struct LidarPoint
    {
        V3D position = V3D::Zero();
        float intensity = 0.0f;
        float time_offset = 0.0f;
        uint16_t line = 0;
    };

    // --- LiDAR scan ---
    struct LidarScan
    {
        Timestamp timestamp = 0.0;
        std::vector<LidarPoint> points;
    };

    // --- Processed LiDAR scan ---
    struct ProcessedLidarScan
    {
        Timestamp timestamp = 0.0;
        std::vector<LidarPoint> points;
        std::vector<V3D> point_normals;
    };

    // --- Image ---
    struct ImageData
    {
        Timestamp timestamp = 0.0;
#ifdef GS_LIVO_HAS_OPENCV
        cv::Mat image;
#endif
        int width = 0;
        int height = 0;
    };

    // --- 3DGS point ---
    struct GsPoint
    {
        V3D position = V3D::Zero();
        V3D scale = V3D::Ones();
        Q4D rotation = Q4D::Identity();
        float opacity = 0.0f;
        std::array<float, 48> sh_coeffs = {};
    };

    // --- EKF 19D state ---
    struct StatesGroup
    {
        V3D pos = V3D::Zero();
        V3D vel = V3D::Zero();
        Q4D rot = Q4D::Identity();
        V3D bg = V3D::Zero();
        V3D ba = V3D::Zero();
        V3D grav = V3D(0, 0, -9.81);

        Eigen::Matrix<double, 19, 19> cov = Eigen::Matrix<double, 19, 19>::Identity() * 0.01;
    };

    // --- Synchronized sensor packet ---
    struct SyncPacket
    {
        Timestamp timestamp = 0.0;
        LidarScan lidar;
        std::vector<ImuData> imu_window;
        ImageData image;
    };

    // --- Extrinsics ---
    struct Extrinsics
    {
        V3D translation = V3D::Zero();
        Q4D rotation = Q4D::Identity();
    };

    struct SensorExtrinsics
    {
        Extrinsics lidar_to_imu;
        Extrinsics camera_to_imu;
    };

    // --- System mode ---
    enum class SystemMode : uint8_t { LIVO, LIO, LO, VIO };

    // --- LIO output ---
    struct LioResult
    {
        StatesGroup corrected_state;
        bool success = false;
        int match_count = 0;
        double residual = 0.0;
    };

    // --- VIO output ---
    struct VioResult
    {
        StatesGroup corrected_state;
        bool success = false;
#ifdef GS_LIVO_HAS_OPENCV
        cv::Mat rendered_image;
#endif
        double photometric_error = 0.0;
        int gs_point_count = 0;
        int new_points_added = 0;
    };

}  // namespace gs_livo

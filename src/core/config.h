#pragma once
#include <filesystem>
#include <string>

#include "types.h"

namespace gs_livo {

    // --- Camera intrinsics ---
    struct CameraIntrinsics
    {
        int width = 640;
        int height = 480;
        double fx = 320.0;
        double fy = 320.0;
        double cx = 320.0;
        double cy = 240.0;
        double k1 = 0.0;
        double k2 = 0.0;
        double p1 = 0.0;
        double p2 = 0.0;
    };

    // --- LiDAR config ---
    struct LidarConfig
    {
        std::string model = "livox_avia";
        double max_range = 100.0;
        double min_range = 0.5;
        int max_points = 100000;
        double time_offset = 0.0;
    };

    // --- IMU config ---
    struct ImuConfig
    {
        double rate = 200.0;
        double gyro_noise = 1.7e-4;
        double acc_noise = 2.0e-3;
        double gyro_bias_noise = 1.0e-6;
        double acc_bias_noise = 1.0e-4;
    };

    // --- LIO config ---
    struct LioConfig
    {
        double voxel_size = 0.5;
        int max_points_per_voxel = 20;
        double planer_threshold = 0.1;
        int min_plane_points = 5;
    };

    // --- VIO / 3DGS config ---
    struct VioConfig
    {
        int gs_iterations = 4;
        int border = 10;
        int img_downsample = 1;
        double gs_opacity_thresh = 0.01;
        double gs_scale_thresh = 0.1;
        int gs_max_points = 50000;
    };

    // --- Extrinsics config (raw YAML form) ---
    struct ExtrinsicsConfig
    {
        double lidar_to_imu_x = 0.0, lidar_to_imu_y = 0.0, lidar_to_imu_z = 0.0;
        double lidar_to_imu_roll = 0.0, lidar_to_imu_pitch = 0.0, lidar_to_imu_yaw = 0.0;
        double camera_to_imu_x = 0.0, camera_to_imu_y = 0.0, camera_to_imu_z = 0.0;
        double camera_to_imu_roll = 0.0, camera_to_imu_pitch = 0.0, camera_to_imu_yaw = 0.0;

        SensorExtrinsics toExtrinsics() const;
    };

    // --- System config ---
    struct SystemConfig
    {
        SystemMode mode = SystemMode::LIVO;

        bool use_lio = true;
        bool use_vio = true;
        bool use_3dgs = true;

        LidarConfig lidar;
        ImuConfig imu;
        CameraIntrinsics camera;
        LioConfig lio;
        VioConfig vio;
        ExtrinsicsConfig extrinsics;

        std::filesystem::path config_path;
        std::filesystem::path output_dir = "./output";
        std::filesystem::path log_dir = "./logs";

        std::string log_level = "info";
    };

    // --- ConfigManager singleton ---
    class ConfigManager
    {
    public:
        static ConfigManager& instance();

        bool load(const std::filesystem::path& path);
        bool save(const std::filesystem::path& path) const;

        const SystemConfig& config() const
        {
            return config_;
        }
        SystemConfig& mutableConfig()
        {
            return config_;
        }

    private:
        ConfigManager() = default;
        SystemConfig config_;
    };

}  // namespace gs_livo

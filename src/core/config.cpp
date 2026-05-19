#include "config.h"
#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>
#include <fstream>

namespace gs_livo {

SensorExtrinsics ExtrinsicsConfig::toExtrinsics() const {
    SensorExtrinsics ext;
    ext.lidar_to_imu.translation = V3D(lidar_to_imu_x, lidar_to_imu_y, lidar_to_imu_z);
    Eigen::AngleAxisd roll(lidar_to_imu_roll, V3D::UnitX());
    Eigen::AngleAxisd pitch(lidar_to_imu_pitch, V3D::UnitY());
    Eigen::AngleAxisd yaw(lidar_to_imu_yaw, V3D::UnitZ());
    ext.lidar_to_imu.rotation = Q4D(yaw * pitch * roll);

    ext.camera_to_imu.translation = V3D(camera_to_imu_x, camera_to_imu_y, camera_to_imu_z);
    Eigen::AngleAxisd c_roll(camera_to_imu_roll, V3D::UnitX());
    Eigen::AngleAxisd c_pitch(camera_to_imu_pitch, V3D::UnitY());
    Eigen::AngleAxisd c_yaw(camera_to_imu_yaw, V3D::UnitZ());
    ext.camera_to_imu.rotation = Q4D(c_yaw * c_pitch * c_roll);

    return ext;
}

ConfigManager& ConfigManager::instance() {
    static ConfigManager mgr;
    return mgr;
}

namespace {

template<typename T>
T get_or(YAML::Node const& node, std::string const& key, T const& fallback) {
    if (!node[key]) return fallback;
    return node[key].as<T>();
}

}  // namespace

bool ConfigManager::load(std::filesystem::path const& path) {
    config_.config_path = path;
    try {
        YAML::Node root = YAML::LoadFile(path.string());

        config_.log_level = get_or<std::string>(root, "log_level", "info");

        auto const mode_str = get_or<std::string>(root, "mode", "LIVO");
        if (mode_str == "LIO") config_.mode = SystemMode::LIO;
        else if (mode_str == "LO") config_.mode = SystemMode::LO;
        else if (mode_str == "VIO") config_.mode = SystemMode::VIO;
        else config_.mode = SystemMode::LIVO;

        config_.use_lio  = get_or<bool>(root, "use_lio", true);
        config_.use_vio  = get_or<bool>(root, "use_vio", true);
        config_.use_3dgs = get_or<bool>(root, "use_3dgs", true);

        // LiDAR
        if (auto lidar = root["lidar"]) {
            config_.lidar.model       = get_or<std::string>(lidar, "model", "livox_avia");
            config_.lidar.max_range   = get_or<double>(lidar, "max_range", 100.0);
            config_.lidar.min_range   = get_or<double>(lidar, "min_range", 0.5);
            config_.lidar.max_points  = get_or<int>(lidar, "max_points", 100000);
            config_.lidar.time_offset = get_or<double>(lidar, "time_offset", 0.0);
        }

        // IMU
        if (auto imu = root["imu"]) {
            config_.imu.rate = get_or<double>(imu, "rate", 200.0);
            config_.imu.gyro_noise = get_or<double>(imu, "gyro_noise", 1.7e-4);
            config_.imu.acc_noise  = get_or<double>(imu, "acc_noise", 2.0e-3);
            config_.imu.gyro_bias_noise = get_or<double>(imu, "gyro_bias_noise", 1.0e-6);
            config_.imu.acc_bias_noise  = get_or<double>(imu, "acc_bias_noise", 1.0e-4);
        }

        // Camera
        if (auto cam = root["camera"]) {
            config_.camera.width  = get_or<int>(cam, "width", 640);
            config_.camera.height = get_or<int>(cam, "height", 480);
            config_.camera.fx = get_or<double>(cam, "fx", 320.0);
            config_.camera.fy = get_or<double>(cam, "fy", 320.0);
            config_.camera.cx = get_or<double>(cam, "cx", 320.0);
            config_.camera.cy = get_or<double>(cam, "cy", 240.0);
            config_.camera.k1 = get_or<double>(cam, "k1", 0.0);
            config_.camera.k2 = get_or<double>(cam, "k2", 0.0);
            config_.camera.p1 = get_or<double>(cam, "p1", 0.0);
            config_.camera.p2 = get_or<double>(cam, "p2", 0.0);
        }

        // LIO
        if (auto lio = root["lio"]) {
            config_.lio.voxel_size          = get_or<double>(lio, "voxel_size", 0.5);
            config_.lio.max_points_per_voxel = get_or<int>(lio, "max_points_per_voxel", 20);
            config_.lio.planer_threshold     = get_or<double>(lio, "planer_threshold", 0.1);
            config_.lio.min_plane_points     = get_or<int>(lio, "min_plane_points", 5);
        }

        // VIO
        if (auto vio = root["vio"]) {
            config_.vio.gs_iterations     = get_or<int>(vio, "gs_iterations", 4);
            config_.vio.border            = get_or<int>(vio, "border", 10);
            config_.vio.img_downsample    = get_or<int>(vio, "img_downsample", 1);
            config_.vio.gs_opacity_thresh = get_or<double>(vio, "gs_opacity_thresh", 0.01);
            config_.vio.gs_scale_thresh   = get_or<double>(vio, "gs_scale_thresh", 0.1);
            config_.vio.gs_max_points     = get_or<int>(vio, "gs_max_points", 50000);
        }

        // Extrinsics
        if (auto ext = root["extrinsics"]) {
            auto le = ext["lidar_to_imu"];
            if (le) {
                config_.extrinsics.lidar_to_imu_x = get_or<double>(le, "x", 0.0);
                config_.extrinsics.lidar_to_imu_y = get_or<double>(le, "y", 0.0);
                config_.extrinsics.lidar_to_imu_z = get_or<double>(le, "z", 0.0);
                config_.extrinsics.lidar_to_imu_roll  = get_or<double>(le, "roll", 0.0);
                config_.extrinsics.lidar_to_imu_pitch = get_or<double>(le, "pitch", 0.0);
                config_.extrinsics.lidar_to_imu_yaw   = get_or<double>(le, "yaw", 0.0);
            }
            auto ce = ext["camera_to_imu"];
            if (ce) {
                config_.extrinsics.camera_to_imu_x = get_or<double>(ce, "x", 0.0);
                config_.extrinsics.camera_to_imu_y = get_or<double>(ce, "y", 0.0);
                config_.extrinsics.camera_to_imu_z = get_or<double>(ce, "z", 0.0);
                config_.extrinsics.camera_to_imu_roll  = get_or<double>(ce, "roll", 0.0);
                config_.extrinsics.camera_to_imu_pitch = get_or<double>(ce, "pitch", 0.0);
                config_.extrinsics.camera_to_imu_yaw   = get_or<double>(ce, "yaw", 0.0);
            }
        }

        if (auto out = root["output"]) {
            config_.output_dir = get_or<std::string>(out, "dir", "./output");
            config_.log_dir    = get_or<std::string>(out, "log_dir", "./logs");
        }

        spdlog::info("Config loaded from {}", path.string());
        return true;
    } catch (YAML::Exception const& e) {
        spdlog::error("Failed to parse config: {}", e.what());
        return false;
    }
}

bool ConfigManager::save(std::filesystem::path const& path) const {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "log_level" << YAML::Value << config_.log_level;
    out << YAML::Key << "use_lio" << YAML::Value << config_.use_lio;
    out << YAML::Key << "use_vio" << YAML::Value << config_.use_vio;
    out << YAML::Key << "use_3dgs" << YAML::Value << config_.use_3dgs;
    out << YAML::EndMap;

    std::ofstream f(path);
    if (!f) return false;
    f << out.c_str();
    return true;
}

}  // namespace gs_livo

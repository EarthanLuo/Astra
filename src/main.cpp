#include <spdlog/spdlog.h>

#include <cstdlib>

#include "core/config.h"
#include "core/logger.h"

int main(int argc, char* argv[])
{
    auto logger = gs_livo::setupLogger("gs_livo", "info");

    std::filesystem::path config_path = "config/default.yaml";
    for (int i = 1; i < argc; ++i) {
        std::string_view arg(argv[i]);
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        }
    }

    auto& cfg = gs_livo::ConfigManager::instance();
    bool loaded = cfg.load(config_path);
    if (!loaded) {
        spdlog::warn("Could not load config from {}, using defaults", config_path.string());
    }

    spdlog::info("GS-LIVO v0.1.0 starting...");
    spdlog::info("Mode: {}", static_cast<int>(cfg.config().mode));
    spdlog::info("LIO: {}, VIO: {}, 3DGS: {}", cfg.config().use_lio, cfg.config().use_vio,
                 cfg.config().use_3dgs);

    return EXIT_SUCCESS;
}

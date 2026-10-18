#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace simulator_config {
namespace fs = std::filesystem;

inline fs::path packageShare() {
    try {
        return ament_index_cpp::get_package_share_directory("sensor_simulator");
    } catch (const std::exception&) {
        return fs::path(SIMULATOR_SOURCE_DIR);
    }
}

inline std::string defaultConfig() {
    return (packageShare() / "config" / "config.yaml").string();
}

inline std::string resourcePath(const std::string& filename, const fs::path& config_file) {
    const fs::path path(filename);
    if (path.is_absolute()) {
        return path.string();
    }
    fs::path package_relative = path;
    auto part = path.begin();
    // Original configurations address package assets as src/pointcloud/....
    if (part != path.end() && *part == "src") {
        package_relative.clear();
        for (++part; part != path.end(); ++part) {
            package_relative /= *part;
        }
    }
    const std::vector<fs::path> candidates = {
        config_file.parent_path() / path,
        config_file.parent_path().parent_path() / package_relative,
        packageShare() / package_relative,
        fs::path(SIMULATOR_SOURCE_DIR) / package_relative,
    };
    for (const auto& candidate : candidates) {
        if (fs::exists(candidate)) {
            return fs::absolute(candidate).lexically_normal().string();
        }
    }
    throw std::runtime_error("Cannot find simulator resource: " + filename);
}

inline YAML::Node load(const std::string& filename) {
    const fs::path config_file = fs::absolute(filename);
    YAML::Node config = YAML::LoadFile(config_file.string());
    if (config["tree_file"] && config["maze_type"].as<int>() == 5) {
        config["tree_file"] = resourcePath(config["tree_file"].as<std::string>(), config_file);
    }
    if (config["ply_file"] && !config["random_map"].as<bool>()) {
        config["ply_file"] = resourcePath(config["ply_file"].as<std::string>(), config_file);
    }
    return config;
}

inline std::string datasetPath(const YAML::Node& config, const std::string& filename) {
    fs::path path(config["save_path"].as<std::string>());
    if (!path.is_absolute()) {
        const fs::path config_file = fs::absolute(filename).lexically_normal();
        // Keep the original default ../dataset location relative to Simulator/.
        const bool default_config = config_file == fs::absolute(defaultConfig()).lexically_normal()
            || config_file == (fs::path(SIMULATOR_SOURCE_DIR) / "config/config.yaml").lexically_normal();
        const fs::path base = default_config
            ? fs::path(SIMULATOR_SOURCE_DIR).parent_path()
            : config_file.parent_path();
        path = base / path;
    }
    return fs::absolute(path).lexically_normal().string() + "/";
}
}  // namespace simulator_config

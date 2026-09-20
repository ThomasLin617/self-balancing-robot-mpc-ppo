#include <mujoco_backend/known_map_world.hpp>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <unistd.h>
#include <vector>

namespace mujoco_backend
{

KnownMapWorld::KnownMapWorld(const std::string& robot_model, const navigation::KnownMap& scenario) {
    std::ifstream input(robot_model);
    if (!input) throw std::runtime_error("Cannot read the robot model.");
    std::ostringstream contents;
    contents << input.rdbuf();
    std::string xml = contents.str();
    const auto world = xml.find("<worldbody>");
    if (world == std::string::npos) throw std::runtime_error("Robot model has no worldbody.");
    std::ostringstream obstacles;
    obstacles << std::setprecision(17);
    for (const auto& obstacle : scenario.obstacles) {
        obstacles << "\n<geom name=\"known_" << obstacle.name << "\" type=\"box\" pos=\""
                  << 0.5*(obstacle.min_x+obstacle.max_x) << ' '
                  << 0.5*(obstacle.min_y+obstacle.max_y) << ' ' << 0.5*obstacle.height
                  << "\" size=\"" << 0.5*(obstacle.max_x-obstacle.min_x) << ' '
                  << 0.5*(obstacle.max_y-obstacle.min_y) << ' ' << 0.5*obstacle.height
                  << "\" rgba=\"0.65 0.18 0.15 1\"/>\n";
    }
    xml.insert(world + std::string("<worldbody>").size(), obstacles.str());
    const std::string pattern = (std::filesystem::temp_directory_path() / "known-map-XXXXXX.xml").string();
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back('\0');
    const int descriptor = mkstemps(name.data(), 4);
    if (descriptor < 0) throw std::runtime_error("Cannot create the known-map model file.");
    close(descriptor);
    path_ = name.data();
    std::ofstream output(path_);
    output << xml;
    output.close();
    if (!output) {
        std::remove(path_.c_str());
        throw std::runtime_error("Cannot write the known-map model file.");
    }
}

KnownMapWorld::~KnownMapWorld() {
    if (!path_.empty()) std::remove(path_.c_str());
}

const std::string& KnownMapWorld::path() const { return path_; }

}

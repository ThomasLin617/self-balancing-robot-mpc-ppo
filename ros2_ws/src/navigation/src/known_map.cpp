#include <navigation/known_map.hpp>
#include <cmath>
#include <fstream>
#include <set>
#include <stdexcept>

namespace navigation
{

KnownMap KnownMap::load(const std::string& path) {
    std::ifstream input(path);
    std::string tag;
    int width, height;
    double resolution, origin_x, origin_y;
    if (!(input >> tag >> width >> height >> resolution >> origin_x >> origin_y) || tag != "grid") {
        throw std::runtime_error("Expected grid width height resolution origin_x origin_y.");
    }
    KnownMap result{OccupancyGrid(width, height, resolution, origin_x, origin_y), {}, {}};
    if (!(input >> tag >> result.goal[0] >> result.goal[1]) || tag != "goal") {
        throw std::runtime_error("Expected goal x y.");
    }
    result.grid.world_to_grid(result.goal);
    std::set<std::string> names;
    while (input >> tag) {
        MapObstacle obstacle;
        if (tag != "obstacle" || !(input >> obstacle.name >> obstacle.min_x >> obstacle.min_y
                                        >> obstacle.max_x >> obstacle.max_y >> obstacle.height)
            || obstacle.name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != std::string::npos
            || !names.insert(obstacle.name).second || !std::isfinite(obstacle.height) || obstacle.height <= 0.0) {
            throw std::runtime_error("Invalid or duplicate map obstacle.");
        }
        result.grid.world_to_grid({obstacle.min_x, obstacle.min_y});
        result.grid.world_to_grid({obstacle.max_x, obstacle.max_y});
        result.grid.add_rectangle(obstacle.min_x, obstacle.min_y, obstacle.max_x, obstacle.max_y);
        result.obstacles.push_back(obstacle);
    }
    if (!input.eof() || result.obstacles.empty()) throw std::runtime_error("Known map has no obstacle layout.");
    return result;
}

}

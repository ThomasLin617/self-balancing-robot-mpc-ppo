#include <navigation/global_planner.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace navigation
{

std::vector<PathPoint> AStarPlanner::plan(const OccupancyGrid& map, PathPoint start, PathPoint goal) const {
    const auto first = map.world_to_grid(start);
    const auto last = map.world_to_grid(goal);
    if (!map.segment_free(start, start) || !map.segment_free(goal, goal)) {
        throw std::invalid_argument("Planner start or goal is occupied.");
    }
    if (first == last) {
        if (!map.segment_free(start, goal)) throw std::runtime_error("No path in endpoint cell.");
        if (start == goal) return {start};
        return {start, goal};
    }
    const int count = map.width() * map.height();
    const int start_id = first.row * map.width() + first.column;
    const int goal_id = last.row * map.width() + last.column;
    std::vector<double> cost(count, std::numeric_limits<double>::infinity());
    std::vector<int> parent(count, -1);
    std::vector<bool> closed(count, false);
    auto heuristic = [&](GridCell cell) {
        const int dx = std::abs(cell.column - last.column);
        const int dy = std::abs(cell.row - last.row);
        return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
    };
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    cost[start_id] = 0.0;
    open.push({heuristic(first), start_id});
    while (!open.empty()) {
        const int current = open.top().second;
        open.pop();
        if (closed[current]) continue;
        closed[current] = true;
        if (current == goal_id) break;
        const GridCell cell{current / map.width(), current % map.width()};
        for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
                if (dr == 0 && dc == 0) continue;
                const GridCell next{cell.row + dr, cell.column + dc};
                if (map.occupied(next)) continue;
                if (dr != 0 && dc != 0
                    && (map.occupied({cell.row + dr, cell.column})
                        || map.occupied({cell.row, cell.column + dc}))) continue;
                const int id = next.row * map.width() + next.column;
                const double candidate = cost[current] + std::hypot(dr, dc);
                if (candidate < cost[id]) {
                    cost[id] = candidate;
                    parent[id] = current;
                    open.push({candidate + heuristic(next), id});
                }
            }
        }
    }
    if (!closed[goal_id]) throw std::runtime_error("No complete A* path exists.");
    std::vector<PathPoint> path;
    for (int id = goal_id; id != -1; id = parent[id]) {
        path.push_back(map.grid_to_world({id / map.width(), id % map.width()}));
    }
    std::reverse(path.begin(), path.end());
    if (path.front() != start) path.insert(path.begin(), start);
    if (path.back() != goal) path.push_back(goal);
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (!map.segment_free(path[i - 1], path[i])) throw std::runtime_error("Invalid A* endpoint connection.");
    }
    return path;
}

std::vector<PathPoint> simplify_path(const OccupancyGrid& map, const std::vector<PathPoint>& path) {
    if (path.empty()) throw std::invalid_argument("Cannot simplify an empty path.");
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (!map.segment_free(path[i], path[i == 0 ? 0 : i - 1])) {
            throw std::invalid_argument("Input path is not collision-free.");
        }
    }
    std::vector<PathPoint> result{path.front()};
    std::size_t current = 0;
    while (current + 1 < path.size()) {
        std::size_t next = path.size() - 1;
        while (next > current + 1 && !map.segment_free(path[current], path[next])) --next;
        result.push_back(path[next]);
        current = next;
    }
    return result;
}

}

#include <navigation/occupancy_grid.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace navigation
{

OccupancyGrid::OccupancyGrid(int width, int height, double resolution, double origin_x, double origin_y)
    : width_(width), height_(height), resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y)
{
    if (width <= 0 || height <= 0 || !std::isfinite(resolution) || resolution <= 0.0
        || !std::isfinite(origin_x) || !std::isfinite(origin_y)) {
        throw std::invalid_argument("Invalid occupancy grid dimensions or origin.");
    }
    cells_.resize(static_cast<std::size_t>(width) * height, 0);
}

int OccupancyGrid::width() const { return width_; }
int OccupancyGrid::height() const { return height_; }
double OccupancyGrid::resolution() const { return resolution_; }

bool OccupancyGrid::contains(GridCell cell) const {
    return cell.row >= 0 && cell.row < height_ && cell.column >= 0 && cell.column < width_;
}

bool OccupancyGrid::occupied(GridCell cell) const {
    return !contains(cell) || cells_[cell.row * width_ + cell.column] != 0;
}

void OccupancyGrid::set_occupied(GridCell cell) {
    if (!contains(cell)) throw std::out_of_range("Grid cell is outside the map.");
    cells_[cell.row * width_ + cell.column] = 1;
}

GridCell OccupancyGrid::world_to_grid(PathPoint point) const {
    const double column = (point[0] - origin_x_) / resolution_;
    const double row = (point[1] - origin_y_) / resolution_;
    if (!std::isfinite(column) || !std::isfinite(row)
        || column < 0.0 || column >= width_ || row < 0.0 || row >= height_) {
        throw std::out_of_range("World point is outside the map.");
    }
    return {static_cast<int>(std::floor(row)), static_cast<int>(std::floor(column))};
}

PathPoint OccupancyGrid::grid_to_world(GridCell cell) const {
    if (!contains(cell)) throw std::out_of_range("Grid cell is outside the map.");
    return {origin_x_ + (cell.column + 0.5) * resolution_,
            origin_y_ + (cell.row + 0.5) * resolution_};
}

void OccupancyGrid::add_rectangle(double min_x, double min_y, double max_x, double max_y) {
    if (!std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(max_x)
        || !std::isfinite(max_y) || min_x >= max_x || min_y >= max_y) {
        throw std::invalid_argument("Invalid obstacle rectangle.");
    }
    for (int row = 0; row < height_; ++row) {
        for (int column = 0; column < width_; ++column) {
            const double x = origin_x_ + column * resolution_;
            const double y = origin_y_ + row * resolution_;
            if (x <= max_x && x + resolution_ >= min_x
                && y <= max_y && y + resolution_ >= min_y) {
                set_occupied({row, column});
            }
        }
    }
}

OccupancyGrid OccupancyGrid::inflated(double clearance) const {
    if (!std::isfinite(clearance) || clearance < 0.0) {
        throw std::invalid_argument("Inflation clearance must be nonnegative.");
    }
    OccupancyGrid result = *this;
    if (clearance == 0.0) return result;
    const int reach = static_cast<int>(std::ceil(clearance / resolution_)) + 1;
    for (int row = 0; row < height_; ++row) {
        for (int column = 0; column < width_; ++column) {
            const double boundary_distance = resolution_ * std::min({row, column, height_ - row - 1, width_ - column - 1});
            if (boundary_distance <= clearance) result.set_occupied({row, column});
            if (!occupied({row, column})) continue;
            for (int dr = -reach; dr <= reach; ++dr) {
                for (int dc = -reach; dc <= reach; ++dc) {
                    const GridCell neighbor{row + dr, column + dc};
                    const double dx = std::max(0, std::abs(dc) - 1) * resolution_;
                    const double dy = std::max(0, std::abs(dr) - 1) * resolution_;
                    if (result.contains(neighbor) && std::hypot(dx, dy) <= clearance) {
                        result.set_occupied(neighbor);
                    }
                }
            }
        }
    }
    return result;
}

bool OccupancyGrid::point_free(PathPoint point) const {
    try { return !occupied(world_to_grid(point)); }
    catch (const std::out_of_range&) { return false; }
}

bool OccupancyGrid::segment_free(PathPoint from, PathPoint to) const {
    if (!point_free(from) || !point_free(to)) return false;
    // Closed-cell intersection also rejects touching an occupied corner.
    const auto a = world_to_grid(from);
    const auto b = world_to_grid(to);
    for (int row = std::max(0, std::min(a.row, b.row) - 1);
         row <= std::min(height_ - 1, std::max(a.row, b.row) + 1); ++row) {
        for (int column = std::max(0, std::min(a.column, b.column) - 1);
             column <= std::min(width_ - 1, std::max(a.column, b.column) + 1); ++column) {
            if (!occupied({row, column})) continue;
            const double low[2] = {origin_x_ + column * resolution_, origin_y_ + row * resolution_};
            double enter = 0.0;
            double leave = 1.0;
            for (int axis = 0; axis < 2; ++axis) {
                const double delta = to[axis] - from[axis];
                if (std::abs(delta) < 1e-14) {
                    if (from[axis] < low[axis] || from[axis] > low[axis] + resolution_) {
                        enter = 2.0;
                        break;
                    }
                }
                else {
                    double t0 = (low[axis] - from[axis]) / delta;
                    double t1 = (low[axis] + resolution_ - from[axis]) / delta;
                    if (t0 > t1) std::swap(t0, t1);
                    enter = std::max(enter, t0);
                    leave = std::min(leave, t1);
                }
            }
            if (enter <= leave) return false;
        }
    }
    return true;
}

}

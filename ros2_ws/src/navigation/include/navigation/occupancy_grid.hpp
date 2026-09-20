#pragma once

#include <array>
#include <vector>

namespace navigation
{

struct GridCell
{
    int row = 0;
    int column = 0;
    bool operator==(const GridCell& other) const {
        return row == other.row && column == other.column;
    }
};

using PathPoint = std::array<double, 2>;

class OccupancyGrid
{
public:
    OccupancyGrid(int width, int height, double resolution, double origin_x, double origin_y);
    int width() const;
    int height() const;
    double resolution() const;
    bool contains(GridCell cell) const;
    bool occupied(GridCell cell) const;
    void set_occupied(GridCell cell);
    GridCell world_to_grid(PathPoint point) const;
    PathPoint grid_to_world(GridCell cell) const;
    void add_rectangle(double min_x, double min_y, double max_x, double max_y);
    OccupancyGrid inflated(double clearance) const;
    bool point_free(PathPoint point) const;
    bool segment_free(PathPoint from, PathPoint to) const;

private:
    int width_;
    int height_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    std::vector<unsigned char> cells_;
};

}

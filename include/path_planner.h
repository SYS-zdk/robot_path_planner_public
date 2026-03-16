#pragma once

#include "grid_map.h"
#include "astar.h"
#include <string>

/**
 * @brief High-level robot path planner.
 *
 * Provides a simple interface for configuring a grid map, planning a
 * collision-free path, and visualizing the result as ASCII art.
 */
class PathPlanner {
public:
    /**
     * @brief Construct a planner with a grid of the given dimensions.
     * @throws std::invalid_argument if width or height is not positive.
     */
    PathPlanner(int width, int height);

    /** @brief Access the underlying grid map. */
    GridMap& map() { return map_; }
    const GridMap& map() const { return map_; }

    /**
     * @brief Allow diagonal (8-directional) movement.
     * Default is false (4-directional only).
     */
    void setAllowDiagonal(bool allow);

    /**
     * @brief Plan a path from @p start to @p goal.
     * @return The path (including start and goal), or an empty vector
     *         if no path exists.
     */
    Path plan(Point start, Point goal);

    /**
     * @brief Return an ASCII visualization of the map and path.
     *
     * Legend:
     *   S  – start position
     *   G  – goal position
     *   #  – obstacle
     *   .  – path cell
     *   (space) – free cell
     */
    std::string visualize(const Path& path, Point start, Point goal) const;

private:
    GridMap map_;
    bool allow_diagonal_ = false;
};

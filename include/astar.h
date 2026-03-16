#pragma once

#include "grid_map.h"
#include <vector>
#include <utility>

/** @brief A 2D grid coordinate. */
using Point = std::pair<int, int>;

/** @brief An ordered sequence of grid coordinates forming a path. */
using Path = std::vector<Point>;

/**
 * @brief A* shortest-path algorithm on a 2D grid.
 *
 * Finds the shortest collision-free path from a start cell to a goal
 * cell using the A* search algorithm.  Supports both 4-directional
 * (Manhattan) and 8-directional (diagonal) movement.
 */
class AStar {
public:
    explicit AStar(const GridMap& map);

    /**
     * @brief Find the shortest path from @p start to @p goal.
     * @return The path (including start and goal), or an empty vector
     *         if no path exists or the start/goal are invalid/occupied.
     */
    Path findPath(Point start, Point goal);

    /**
     * @brief Allow diagonal (8-directional) movement.
     * Default is false (4-directional).
     */
    void setAllowDiagonal(bool allow) { allow_diagonal_ = allow; }

private:
    double heuristic(int x1, int y1, int x2, int y2) const;

    const GridMap& map_;
    bool allow_diagonal_ = false;
};

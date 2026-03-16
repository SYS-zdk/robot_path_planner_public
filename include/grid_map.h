#pragma once

#include <vector>
#include <stdexcept>

/**
 * @brief A 2D grid map for robot path planning.
 *
 * The map is a rectangular grid of cells, each of which is either
 * FREE or an OBSTACLE. Coordinates use (x, y) with x increasing to
 * the right and y increasing downward.
 */
class GridMap {
public:
    enum class CellType {
        FREE = 0,
        OBSTACLE = 1
    };

    /**
     * @brief Construct a grid map with the given dimensions.
     * @throws std::invalid_argument if width or height is not positive.
     */
    GridMap(int width, int height);

    int width() const { return width_; }
    int height() const { return height_; }

    /** @brief Returns true if (x, y) is within the map boundaries. */
    bool isValid(int x, int y) const;

    /**
     * @brief Returns true if (x, y) is an obstacle or out of bounds.
     */
    bool isObstacle(int x, int y) const;

    /** @throws std::out_of_range if the position is out of bounds. */
    void setObstacle(int x, int y);

    /** @throws std::out_of_range if the position is out of bounds. */
    void clearObstacle(int x, int y);

    /** @brief Remove all obstacles from the map. */
    void clear();

private:
    int width_;
    int height_;
    std::vector<std::vector<CellType>> grid_;
};

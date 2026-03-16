#include "grid_map.h"
#include <stdexcept>
#include <algorithm>

GridMap::GridMap(int width, int height)
    : width_(width), height_(height),
      grid_(height, std::vector<CellType>(width, CellType::FREE)) {
    if (width <= 0 || height <= 0) {
        throw std::invalid_argument("Grid dimensions must be positive");
    }
}

bool GridMap::isValid(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

bool GridMap::isObstacle(int x, int y) const {
    if (!isValid(x, y)) {
        return true;  // treat out-of-bounds as obstacle
    }
    return grid_[y][x] == CellType::OBSTACLE;
}

void GridMap::setObstacle(int x, int y) {
    if (!isValid(x, y)) {
        throw std::out_of_range("Position out of grid bounds");
    }
    grid_[y][x] = CellType::OBSTACLE;
}

void GridMap::clearObstacle(int x, int y) {
    if (!isValid(x, y)) {
        throw std::out_of_range("Position out of grid bounds");
    }
    grid_[y][x] = CellType::FREE;
}

void GridMap::clear() {
    for (auto& row : grid_) {
        std::fill(row.begin(), row.end(), CellType::FREE);
    }
}

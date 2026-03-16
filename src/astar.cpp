#include "astar.h"
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tuple>

AStar::AStar(const GridMap& map) : map_(map) {}

double AStar::heuristic(int x1, int y1, int x2, int y2) const {
    if (allow_diagonal_) {
        // Octile distance – optimal heuristic for 8-directional movement
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
    } else {
        // Manhattan distance – optimal heuristic for 4-directional movement
        return static_cast<double>(std::abs(x2 - x1) + std::abs(y2 - y1));
    }
}

Path AStar::findPath(Point start, Point goal) {
    // Validate inputs
    if (!map_.isValid(start.first, start.second) ||
        !map_.isValid(goal.first, goal.second)) {
        return {};
    }
    if (map_.isObstacle(start.first, start.second) ||
        map_.isObstacle(goal.first, goal.second)) {
        return {};
    }
    if (start == goal) {
        return {start};
    }

    const int w = map_.width();
    const int h = map_.height();

    constexpr double kInf = std::numeric_limits<double>::infinity();

    std::vector<std::vector<double>> g(h, std::vector<double>(w, kInf));
    std::vector<std::vector<Point>> parent(h, std::vector<Point>(w, {-1, -1}));
    std::vector<std::vector<bool>> closed(h, std::vector<bool>(w, false));

    // Min-heap: (f_cost, x, y)
    using QNode = std::tuple<double, int, int>;
    std::priority_queue<QNode, std::vector<QNode>, std::greater<QNode>> open_set;

    g[start.second][start.first] = 0.0;
    double h0 = heuristic(start.first, start.second, goal.first, goal.second);
    open_set.push({h0, start.first, start.second});

    // Movement directions: 4- or 8-directional
    static const std::pair<int, int> dirs4[] = {{0,1},{0,-1},{1,0},{-1,0}};
    static const std::pair<int, int> dirs8[] = {
        {0,1},{0,-1},{1,0},{-1,0},{1,1},{1,-1},{-1,1},{-1,-1}
    };
    const auto* dirs = allow_diagonal_ ? dirs8 : dirs4;
    const int num_dirs = allow_diagonal_ ? 8 : 4;

    while (!open_set.empty()) {
        auto [f, cx, cy] = open_set.top();
        open_set.pop();

        if (closed[cy][cx]) {
            continue;
        }
        closed[cy][cx] = true;

        if (cx == goal.first && cy == goal.second) {
            // Reconstruct path by following parent pointers
            Path path;
            int px = goal.first;
            int py = goal.second;
            while (px != -1 && py != -1) {
                path.push_back({px, py});
                auto [npx, npy] = parent[py][px];
                px = npx;
                py = npy;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < num_dirs; ++i) {
            int nx = cx + dirs[i].first;
            int ny = cy + dirs[i].second;

            if (!map_.isValid(nx, ny) || map_.isObstacle(nx, ny) || closed[ny][nx]) {
                continue;
            }

            double move_cost = (dirs[i].first != 0 && dirs[i].second != 0)
                                   ? std::sqrt(2.0)
                                   : 1.0;
            double new_g = g[cy][cx] + move_cost;

            if (new_g < g[ny][nx]) {
                g[ny][nx] = new_g;
                parent[ny][nx] = {cx, cy};
                double h_val = heuristic(nx, ny, goal.first, goal.second);
                open_set.push({new_g + h_val, nx, ny});
            }
        }
    }

    return {};  // No path found
}

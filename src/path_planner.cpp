#include "path_planner.h"
#include <set>
#include <sstream>

PathPlanner::PathPlanner(int width, int height) : map_(width, height) {}

void PathPlanner::setAllowDiagonal(bool allow) {
    allow_diagonal_ = allow;
}

Path PathPlanner::plan(Point start, Point goal) {
    AStar astar(map_);
    astar.setAllowDiagonal(allow_diagonal_);
    return astar.findPath(start, goal);
}

std::string PathPlanner::visualize(const Path& path, Point start, Point goal) const {
    std::set<Point> path_set(path.begin(), path.end());

    std::ostringstream oss;
    for (int y = 0; y < map_.height(); ++y) {
        for (int x = 0; x < map_.width(); ++x) {
            Point p{x, y};
            if (p == start) {
                oss << 'S';
            } else if (p == goal) {
                oss << 'G';
            } else if (map_.isObstacle(x, y)) {
                oss << '#';
            } else if (path_set.count(p) != 0) {
                oss << '.';
            } else {
                oss << ' ';
            }
        }
        oss << '\n';
    }
    return oss.str();
}

#include "path_planner.h"
#include <iostream>

int main() {
    // -----------------------------------------------------------------------
    // Example 1 – simple open grid, 4-directional movement
    // -----------------------------------------------------------------------
    std::cout << "=== Example 1: Open 10x10 grid (4-directional) ===\n";
    {
        PathPlanner planner(10, 10);

        // Add some obstacles
        for (int y = 2; y <= 7; ++y) {
            planner.map().setObstacle(4, y);
        }
        for (int x = 3; x <= 6; ++x) {
            planner.map().setObstacle(x, 5);
        }

        Point start{0, 0};
        Point goal{9, 9};

        Path path = planner.plan(start, goal);

        if (path.empty()) {
            std::cout << "No path found!\n";
        } else {
            std::cout << "Path length: " << path.size() << " cells\n";
            std::cout << planner.visualize(path, start, goal);
        }
    }

    // -----------------------------------------------------------------------
    // Example 2 – narrow corridor, diagonal movement allowed
    // -----------------------------------------------------------------------
    std::cout << "\n=== Example 2: Diagonal movement on 8x8 grid ===\n";
    {
        PathPlanner planner(8, 8);
        planner.setAllowDiagonal(true);

        // Vertical wall at x=3 with gap at y=7
        for (int y = 0; y <= 6; ++y) {
            planner.map().setObstacle(3, y);
        }

        Point start{1, 1};
        Point goal{6, 6};

        Path path = planner.plan(start, goal);

        if (path.empty()) {
            std::cout << "No path found!\n";
        } else {
            std::cout << "Path length: " << path.size() << " cells\n";
            std::cout << planner.visualize(path, start, goal);
        }
    }

    // -----------------------------------------------------------------------
    // Example 3 – unreachable goal
    // -----------------------------------------------------------------------
    std::cout << "\n=== Example 3: Completely enclosed goal ===\n";
    {
        PathPlanner planner(5, 5);
        planner.map().setObstacle(2, 1);
        planner.map().setObstacle(1, 2);
        planner.map().setObstacle(2, 3);
        planner.map().setObstacle(3, 2);

        Point start{0, 0};
        Point goal{2, 2};

        Path path = planner.plan(start, goal);

        if (path.empty()) {
            std::cout << "No path found (goal is unreachable – correct!)\n";
        } else {
            std::cout << "Path length: " << path.size() << " cells\n";
            std::cout << planner.visualize(path, start, goal);
        }
    }

    return 0;
}

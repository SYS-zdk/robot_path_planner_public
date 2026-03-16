#include "path_planner.h"
#include <cassert>
#include <iostream>
#include <string>

// ---------------------------------------------------------------------------
// Minimal test framework
// ---------------------------------------------------------------------------

static int tests_run = 0;
static int tests_failed = 0;

#define EXPECT_TRUE(cond)                                                 \
    do {                                                                   \
        ++tests_run;                                                       \
        if (!(cond)) {                                                     \
            ++tests_failed;                                                \
            std::cerr << "FAIL  " << __FILE__ << ":" << __LINE__          \
                      << "  " << #cond << "\n";                           \
        }                                                                  \
    } while (false)

#define EXPECT_EQ(a, b)                                                   \
    do {                                                                   \
        ++tests_run;                                                       \
        if ((a) != (b)) {                                                  \
            ++tests_failed;                                                \
            std::cerr << "FAIL  " << __FILE__ << ":" << __LINE__          \
                      << "  " << #a << " != " << #b << "\n";             \
        }                                                                  \
    } while (false)

// ---------------------------------------------------------------------------
// GridMap tests
// ---------------------------------------------------------------------------

void test_grid_map_dimensions() {
    GridMap g(5, 10);
    EXPECT_EQ(g.width(), 5);
    EXPECT_EQ(g.height(), 10);
}

void test_grid_map_valid() {
    GridMap g(4, 4);
    EXPECT_TRUE(g.isValid(0, 0));
    EXPECT_TRUE(g.isValid(3, 3));
    EXPECT_TRUE(!g.isValid(-1, 0));
    EXPECT_TRUE(!g.isValid(4, 0));
    EXPECT_TRUE(!g.isValid(0, 4));
}

void test_grid_map_obstacle() {
    GridMap g(5, 5);
    EXPECT_TRUE(!g.isObstacle(2, 2));
    g.setObstacle(2, 2);
    EXPECT_TRUE(g.isObstacle(2, 2));
    g.clearObstacle(2, 2);
    EXPECT_TRUE(!g.isObstacle(2, 2));
}

void test_grid_map_out_of_bounds_is_obstacle() {
    GridMap g(3, 3);
    EXPECT_TRUE(g.isObstacle(-1, 0));
    EXPECT_TRUE(g.isObstacle(3, 0));
    EXPECT_TRUE(g.isObstacle(0, -1));
    EXPECT_TRUE(g.isObstacle(0, 3));
}

void test_grid_map_clear() {
    GridMap g(4, 4);
    g.setObstacle(1, 1);
    g.setObstacle(2, 2);
    g.clear();
    EXPECT_TRUE(!g.isObstacle(1, 1));
    EXPECT_TRUE(!g.isObstacle(2, 2));
}

void test_grid_map_invalid_dimensions() {
    bool threw = false;
    try {
        GridMap bad(0, 5);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    EXPECT_TRUE(threw);
}

// ---------------------------------------------------------------------------
// A* / PathPlanner tests
// ---------------------------------------------------------------------------

void test_astar_straight_path() {
    // Open 5x1 corridor: start=(0,0), goal=(4,0)
    PathPlanner planner(5, 1);
    Path path = planner.plan({0, 0}, {4, 0});
    EXPECT_TRUE(!path.empty());
    EXPECT_EQ(path.front(), Point(0, 0));
    EXPECT_EQ(path.back(), Point(4, 0));
    EXPECT_EQ(static_cast<int>(path.size()), 5);
}

void test_astar_same_start_goal() {
    PathPlanner planner(5, 5);
    Path path = planner.plan({2, 2}, {2, 2});
    EXPECT_EQ(static_cast<int>(path.size()), 1);
    EXPECT_EQ(path.front(), Point(2, 2));
}

void test_astar_simple_detour() {
    // 5x3 grid with a vertical wall at x=2, except the top row
    //   . . # . .
    //   . . # . .
    //   . . . . .  <- gap at y=2
    // Start=(0,1), Goal=(4,1)
    PathPlanner planner(5, 3);
    planner.map().setObstacle(2, 0);
    planner.map().setObstacle(2, 1);
    Path path = planner.plan({0, 1}, {4, 1});
    EXPECT_TRUE(!path.empty());
    EXPECT_EQ(path.front(), Point(0, 1));
    EXPECT_EQ(path.back(), Point(4, 1));
    // Path must not pass through an obstacle
    for (const auto& p : path) {
        EXPECT_TRUE(!planner.map().isObstacle(p.first, p.second));
    }
}

void test_astar_blocked_path() {
    // Complete vertical wall blocking all passage
    PathPlanner planner(5, 3);
    planner.map().setObstacle(2, 0);
    planner.map().setObstacle(2, 1);
    planner.map().setObstacle(2, 2);
    Path path = planner.plan({0, 1}, {4, 1});
    EXPECT_TRUE(path.empty());
}

void test_astar_start_is_obstacle() {
    PathPlanner planner(5, 5);
    planner.map().setObstacle(0, 0);
    Path path = planner.plan({0, 0}, {4, 4});
    EXPECT_TRUE(path.empty());
}

void test_astar_goal_is_obstacle() {
    PathPlanner planner(5, 5);
    planner.map().setObstacle(4, 4);
    Path path = planner.plan({0, 0}, {4, 4});
    EXPECT_TRUE(path.empty());
}

void test_astar_diagonal_movement() {
    // With diagonal movement, path from (0,0) to (3,3) should have 4 cells
    PathPlanner planner(4, 4);
    planner.setAllowDiagonal(true);
    Path path = planner.plan({0, 0}, {3, 3});
    EXPECT_TRUE(!path.empty());
    EXPECT_EQ(path.front(), Point(0, 0));
    EXPECT_EQ(path.back(), Point(3, 3));
    EXPECT_EQ(static_cast<int>(path.size()), 4);
}

void test_astar_out_of_bounds_inputs() {
    PathPlanner planner(5, 5);
    Path path = planner.plan({-1, 0}, {4, 4});
    EXPECT_TRUE(path.empty());
    path = planner.plan({0, 0}, {5, 5});
    EXPECT_TRUE(path.empty());
}

void test_visualize_contains_start_and_goal() {
    PathPlanner planner(5, 5);
    Path path = planner.plan({0, 0}, {4, 4});
    std::string vis = planner.visualize(path, {0, 0}, {4, 4});
    EXPECT_TRUE(vis.find('S') != std::string::npos);
    EXPECT_TRUE(vis.find('G') != std::string::npos);
}

void test_visualize_obstacles_shown() {
    PathPlanner planner(5, 5);
    planner.map().setObstacle(2, 2);
    Path path = planner.plan({0, 0}, {4, 4});
    std::string vis = planner.visualize(path, {0, 0}, {4, 4});
    EXPECT_TRUE(vis.find('#') != std::string::npos);
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

int main() {
    test_grid_map_dimensions();
    test_grid_map_valid();
    test_grid_map_obstacle();
    test_grid_map_out_of_bounds_is_obstacle();
    test_grid_map_clear();
    test_grid_map_invalid_dimensions();

    test_astar_straight_path();
    test_astar_same_start_goal();
    test_astar_simple_detour();
    test_astar_blocked_path();
    test_astar_start_is_obstacle();
    test_astar_goal_is_obstacle();
    test_astar_diagonal_movement();
    test_astar_out_of_bounds_inputs();

    test_visualize_contains_start_and_goal();
    test_visualize_obstacles_shown();

    if (tests_failed == 0) {
        std::cout << "All " << tests_run << " tests passed.\n";
        return 0;
    } else {
        std::cerr << tests_failed << "/" << tests_run << " tests FAILED.\n";
        return 1;
    }
}

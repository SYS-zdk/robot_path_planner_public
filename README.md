# robot_path_planner_public

A lightweight 2D grid-based robot path planner written in C++17.
It implements the **A\* (A-star)** shortest-path algorithm and ships with
a simple ASCII visualiser so you can inspect planned routes at a glance.

---

## Features

| Feature | Details |
|---|---|
| Grid map | Rectangular 2D grid with configurable obstacles |
| Algorithm | A\* search with admissible heuristics |
| Movement | 4-directional (default) or 8-directional (diagonal) |
| Collision check | Obstacles and out-of-bounds cells are avoided |
| Visualiser | ASCII art showing the map, path, start and goal |

---

## Project layout

```
robot_path_planner_public/
├── CMakeLists.txt          # Root build configuration
├── include/
│   ├── grid_map.h          # 2D grid map with obstacle support
│   ├── astar.h             # A* algorithm
│   └── path_planner.h      # High-level planner interface
├── src/
│   ├── grid_map.cpp
│   ├── astar.cpp
│   └── path_planner.cpp
├── test/
│   ├── CMakeLists.txt
│   └── test_path_planner.cpp   # Unit tests (no external dependencies)
└── examples/
    └── main.cpp            # Demo application
```

---

## Requirements

* C++17-capable compiler (GCC ≥ 7, Clang ≥ 6, MSVC 2017+)
* CMake ≥ 3.14

---

## Building

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

---

## Running the tests

```bash
cd build
ctest --output-on-failure
```

---

## Running the example

```bash
./build/path_planner_example
```

Sample output:

```
=== Example 1: Open 10x10 grid (4-directional) ===
Path length: 19 cells
S
.
.   #
.   #
.   #
.  ####
.   #
.   #
.
.........G
```

---

## API overview

```cpp
#include "path_planner.h"

// Create a 10×10 grid planner
PathPlanner planner(10, 10);

// Place obstacles
planner.map().setObstacle(3, 5);

// Allow diagonal movement (optional, default = false)
planner.setAllowDiagonal(true);

// Plan a path
Path path = planner.plan({0, 0}, {9, 9});

if (path.empty()) {
    std::cout << "No path found\n";
} else {
    std::cout << planner.visualize(path, {0, 0}, {9, 9});
}
```

---

## Visualiser legend

| Symbol | Meaning |
|--------|---------|
| `S` | Start position |
| `G` | Goal position |
| `.` | Path cell |
| `#` | Obstacle |
| ` ` | Free cell |

---

## License

This project is released under the [GNU General Public License v3](LICENSE).


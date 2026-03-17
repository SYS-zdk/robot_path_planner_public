
/**
 * *********************************************************
 *
 * @file: test_convex_safety_corridor.cpp
 * @brief: Convex safety corridor test file
 * @author: Yang Haodong
 * @date: 2024-11-17
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <iostream>
#include <fstream>
#include <filesystem>

#include <gtest/gtest.h>
#include <matplotlibcpp.h>

#include "common/util/log.h"
#include "common/geometry/point.h"
#include "common/geometry/polygon2d.h"
#include "common/safety_corridor/convex_safety_corridor.h"

namespace plt = matplotlibcpp;
using Points3d = rpp::common::geometry::Points3d;
using Polygon2d = rpp::common::geometry::Polygon2d;
using ConvexSafetyCorridor = rpp::common::safety_corridor::ConvexSafetyCorridor;

std::vector<std::vector<int>> readPGM(const std::string& filename)
{
  std::ifstream file(filename, std::ios::binary);
  std::vector<std::vector<int>> image;

  if (file.is_open())
  {
    std::string format;
    int width, height, maxGrayValue;
    file >> format >> width >> height >> maxGrayValue;
    image.resize(height, std::vector<int>(width));
    for (int i = 0; i < height; i++)
    {
      for (int j = 0; j < width; j++)
      {
        file >> image[i][j];
      }
    }
    file.close();
  }
  return image;
}

TEST(TestConvexSafetyCorridor, RealMap)
{
  // real map data
  auto current_path = std::filesystem::current_path();
  auto pgm_path = std::filesystem::path("../../../src/core/common/test/warehouse.pgm");
  std::string file_name = std::filesystem::canonical(current_path / pgm_path).c_str();
  std::vector<std::vector<int>> image = readPGM(file_name);
  int rows = image.size();
  int cols = image[0].size();
  std::vector<unsigned char> z;
  Points3d obstacles;
  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      z.push_back(image[r][c] == 254 ? 255 : 0);
      if (image[r][c] != 254)
      {
        obstacles.emplace_back(c, r);
      }
    }
  }

  // safety corridor
  Points3d line_points = { { 200, 200 }, { 240, 200 }, { 280, 250 }, { 285, 280 }, { 300, 400 }, { 400, 390 } };
  std::vector<Polygon2d> safety_polygons;
  auto safety_corridor = std::make_unique<ConvexSafetyCorridor>(obstacles, 20);
  safety_corridor->decompose(line_points, safety_polygons);
  for (const auto& polygon : safety_polygons)
  {
    std::vector<double> x, y;
    for (int i = 0; i < polygon.num_points(); i++)
    {
      const auto& pt = polygon.points()[i];
      const auto& next_pt = polygon.points()[polygon.next(i)];
      x.push_back(pt.x());
      x.push_back(next_pt.x());
      y.push_back(pt.y());
      y.push_back(next_pt.y());

      plt::plot(x, y, { { "color", "r" }, { "linewidth", "2.0" }, { "linestyle", "-" } });
    }
  }
  plt::plot({ safety_polygons[0].points()[0].x(), safety_polygons[0].points()[1].x() },
            { safety_polygons[0].points()[0].y(), safety_polygons[0].points()[1].y() },
            { { "label", "safety corridor" }, { "color", "r" }, { "linewidth", "2.0" }, { "linestyle", "-" } });

  // waypoints
  std::vector<double> line_x, line_y;
  for (const auto& pt : line_points)
  {
    line_x.push_back(pt.x());
    line_y.push_back(pt.y());
  }
  plt::plot(line_x, line_y,
            { { "label", "path" },
              { "color", "b" },
              { "linewidth", "2.0" },
              { "linestyle", "--" },
              { "marker", "s" },
              { "markersize", "4.0" } });

  // show plots
  plt::xlabel("x");
  plt::ylabel("y");
  const unsigned char* img_ptr = &(z[0]);
  plt::title("Real map test");
  plt::imshow(img_ptr, rows, cols, 1, { { "cmap", "gray" } });
  plt::legend();
  plt::show();
}

TEST(TestConvexSafetyCorridor, Numerical)
{
  // prepare data
  Points3d obstacles = {
    { 4.0, 2.0 },   { 6.0, 3.0 },    { 2.0, 1.5 },   { 0.0, 1.0 }, { 1.0, 0.0 },  { 1.8, 0.0 },
    { 3.8, 2.0 },   { 0.5, 1.2 },    { 4.3, 0.0 },   { 8.0, 0.9 }, { 2.8, -0.3 }, { 6.0, -0.9 },
    { -0.5, -0.5 }, { -0.75, -0.5 }, { -1.0, -0.5 }, { -1, 0.8 },
  };
  Points3d line_points = { { -1.5, 0.0 }, { 0.0, 0.8 }, { 1.5, 0.3 }, { 5.0, 0.6 }, { 6.0, 1.2 }, { 7.6, 2.2 } };

  // safety corridor
  std::vector<Polygon2d> safety_polygons;
  auto safety_corridor = std::make_unique<ConvexSafetyCorridor>(obstacles, 2.0);
  safety_corridor->decompose(line_points, safety_polygons);
  for (const auto& polygon : safety_polygons)
  {
    std::vector<double> x, y;
    for (int i = 0; i < polygon.num_points(); i++)
    {
      const auto& pt = polygon.points()[i];
      const auto& next_pt = polygon.points()[polygon.next(i)];
      x.push_back(pt.x());
      x.push_back(next_pt.x());
      y.push_back(pt.y());
      y.push_back(next_pt.y());

      plt::plot(x, y, { { "color", "r" }, { "linewidth", "2.0" }, { "linestyle", "-" } });
    }
  }

  plt::plot({ safety_polygons[0].points()[0].x(), safety_polygons[0].points()[1].x() },
            { safety_polygons[0].points()[0].y(), safety_polygons[0].points()[1].y() },
            { { "label", "safety corridor" }, { "color", "r" }, { "linewidth", "2.0" }, { "linestyle", "-" } });

  // obstacle
  std::vector<double> obs_x, obs_y;
  for (const auto& obs : obstacles)
  {
    obs_x.push_back(obs.x());
    obs_y.push_back(obs.y());
  }
  plt::scatter(obs_x, obs_y, 40.0, { { "color", "k" }, { "marker", "s" }, { "label", "obstacle" } });

  // show plots
  plt::xlabel("x");
  plt::ylabel("y");
  plt::legend();
  plt::grid(true);
  plt::title("Numerical test using Convex safety corridor");
  plt::show();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

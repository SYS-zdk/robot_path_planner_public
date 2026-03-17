
/**
 * *********************************************************
 *
 * @file: euclidean_distance_field.cpp
 * @brief: euclidean distance transform for 1D/2D matrix
 * @author: Yang Haodong
 * @date: 2024-06-27
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
#include "costmap_2d/cost_values.h"
#include "euclidean_distance_field.h"

namespace euclidean_distance_field
{
EuclideanDistanceField::EuclideanDistanceField() : map_(nullptr), nx_(0), ny_(0){};

EuclideanDistanceField::EuclideanDistanceField(const unsigned char* map, int nx, int ny) : map_(map), nx_(nx), ny_(ny)
{
  edf_ = std::vector<std::vector<double>>(ny, std::vector<double>(nx));
}

void EuclideanDistanceField::setGridMap(const unsigned char* map, int nx, int ny)
{
  map_ = map;
  nx_ = nx;
  ny_ = ny;
  edf_ = std::vector<std::vector<double>>(ny, std::vector<double>(nx));
}

const std::vector<std::vector<double>>& EuclideanDistanceField::getEDF() const
{
  return edf_;
}

void EuclideanDistanceField::truncate(double delta)
{
  for (int y = 0; y < ny_; y++)
  {
    for (int x = 0; x < nx_; x++)
    {
      double val = edf_[y][x];
      if (val >= -delta && val <= delta)
        edf_[y][x] /= delta;
      else if (val > delta)
        edf_[y][x] = 1.0;
      else
        edf_[y][x] = -1.0;
    }
  }
}

double EuclideanDistanceField::getDistance(double x, double y)
{
  x = std::max(std::min(static_cast<double>(nx_ - 1), x), 0.0);
  y = std::max(std::min(static_cast<double>(ny_ - 1), y), 0.0);
  int xi = static_cast<int>(x);
  int yi = static_cast<int>(y);
  double dx = x - xi;
  double dy = y - yi;

  // yp | tl    tr
  //    |
  // yi | bl    br
  //    |_____________
  //      xi    xp

  xi = std::max(std::min(nx_ - 1, xi), 0);
  yi = std::max(std::min(ny_ - 1, yi), 0);
  int xp = std::max(std::min(nx_ - 1, xi + 1), 0);
  int yp = std::max(std::min(ny_ - 1, yi - 1), 0);

  double bl = edf_[yi][xi];
  double br = edf_[yi][xp];
  double tl = edf_[yp][xi];
  double tr = edf_[yp][xp];

  return ((1 - dx) * bl + dx * br) * (1 - dy) + ((1 - dx) * tl + dx * tr) * dy;
}

void EuclideanDistanceField::getGradient(double x, double y, double& gx, double& gy)
{
  x = std::max(std::min(static_cast<double>(nx_ - 1), x), 0.0);
  y = std::max(std::min(static_cast<double>(ny_ - 1), y), 0.0);
  int xi = static_cast<int>(x);
  int yi = static_cast<int>(y);
  double dx = x - xi;
  double dy = y - yi;

  // yp | tl    tr
  //    |
  // yi | bl    br
  //    |_____________
  //      xi    xp

  xi = std::max(std::min(nx_ - 1, xi), 0);
  yi = std::max(std::min(ny_ - 1, yi), 0);
  int xp = std::max(std::min(nx_ - 1, xi + 1), 0);
  int yp = std::max(std::min(ny_ - 1, yi - 1), 0);

  double bl = edf_[yi][xi];
  double br = edf_[yi][xp];
  double tl = edf_[yp][xi];
  double tr = edf_[yp][xp];

  gx = (1 - dy) * (br - bl) + dy * (tr - tl);
  gy = -((1 - dx) * (tl - bl) + dx * (tr - br));
}

void EuclideanDistanceField::compute2d()
{
  // x-dim
  for (int y = 0; y < ny_; y++)
  {
    compute(
        [&](int x) {
          return map_[grid2Index(x, y)] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE ? std::numeric_limits<double>::max() :
                                                                                    0.0f;
        },
        [&](int x, double val) { edf_[y][x] = val; }, 1);
  }

  // y-dim
  for (int x = 0; x < nx_; x++)
  {
    compute([&](int y) { return edf_[y][x]; }, [&](int y, double val) { edf_[y][x] = val; }, 0);
  }
}

int EuclideanDistanceField::grid2Index(int x, int y)
{
  return x + static_cast<int>(nx_ * y);
}
}  // namespace euclidean_distance_field

/**
 * *********************************************************
 *
 * @file: euclidean_distance_field.h
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
#ifndef EUCLIDEAN_DISTANCE_FIELD_H
#define EUCLIDEAN_DISTANCE_FIELD_H

#include <memory>
#include <vector>
#include <limits>
#include <cmath>

namespace euclidean_distance_field
{
namespace
{
constexpr int kFreeCellState = 0;
constexpr int kOccupiedCellState = 1;
};  // namespace

class EuclideanDistanceField
{
public:
  EuclideanDistanceField();
  EuclideanDistanceField(const unsigned char* map, int nx, int ny);

  ~EuclideanDistanceField() = default;

  void setGridMap(const unsigned char* map, int nx, int ny);
  void compute2d();
  void truncate(double delta);
  double getDistance(double x, double y);
  void getGradient(double x, double y, double& gx, double& gy);
  const std::vector<std::vector<double>>& getEDF() const;

private:
  int grid2Index(int x, int y);

  template <typename F_get_val, typename F_set_val>
  void compute(F_get_val f_get, F_set_val f_set, int dim)
  {
    // initialzation
    int k = 0;
    int n = dim == 0 ? ny_ : nx_;
    std::vector<int> v(n);
    std::vector<double> z(n + 1);
    z[0] = -std::numeric_limits<double>::max();
    z[1] = std::numeric_limits<double>::max();

    // envelope
    for (int q = 1; q < n; q++)
    {
      double s =
          ((f_get(v[k]) + std::pow(static_cast<double>(v[k]), 2)) - (f_get(q) + std::pow(static_cast<double>(q), 2))) /
          (2 * (v[k] - q));
      while (s <= z[k])
      {
        k -= 1;
        s = ((f_get(v[k]) + std::pow(static_cast<double>(v[k]), 2)) -
             (f_get(q) + std::pow(static_cast<double>(q), 2))) /
            (2 * (v[k] - q));
      }
      k += 1;
      v[k] = q;
      z[k] = s;
      z[k + 1] = std::numeric_limits<double>::max();
    }

    // distance calculation
    k = 0;
    for (int q = 0; q < n; q++)
    {
      while (z[k + 1] < q)
        k += 1;
      f_set(q, std::pow(static_cast<double>(q - v[k]), 2) + f_get(v[k]));
    }
  }

private:
  const unsigned char* map_;
  int nx_, ny_;
  std::vector<std::vector<double>> edf_;
};

}  // namespace euclidean_distance_field
#endif
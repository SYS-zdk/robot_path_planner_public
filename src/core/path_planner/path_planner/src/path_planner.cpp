
/**
 * @file: global_PathPlanner.cpp
 * @brief: Contains the abstract global PathPlanner class
 * @author: ZhangDingkun
 * @date: 2024.06.08
 * @version: 1.0
 */
#include <costmap_2d/cost_values.h>

#include "path_planner/path_planner.h"

namespace rpp
{
namespace path_planner
{
/**
 * @brief Construct a new Global PathPlanner object
 * @param costmap   the environment for path planning
 */
PathPlanner::PathPlanner(costmap_2d::Costmap2DROS* costmap_ros)
  : factor_(0.5f)
  , nx_(static_cast<int>(costmap_ros->getCostmap()->getSizeInCellsX()))
  , ny_(static_cast<int>(costmap_ros->getCostmap()->getSizeInCellsY()))
  , map_size_(nx_ * ny_)
  , costmap_ros_(costmap_ros)
  , costmap_(costmap_ros->getCostmap())
{
}

PathPlanner::PathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double factor) //new
  : factor_(factor)
  , nx_(static_cast<int>(costmap_ros->getCostmap()->getSizeInCellsX()))
  , ny_(static_cast<int>(costmap_ros->getCostmap()->getSizeInCellsY()))
  , map_size_(nx_ * ny_)
  , costmap_ros_(costmap_ros)
  , costmap_(costmap_ros->getCostmap())
  , collision_checker_(std::make_shared<rpp::common::geometry::CollisionChecker>(costmap_ros, factor))
{
}

/**
 * @brief Set or reset obstacle factor
 * @param factor obstacle factor
 */
void PathPlanner::setFactor(float factor)
{
  factor_ = factor;
}

/**
 * @brief get the costmap
 * @return costmap costmap2d pointer
 */
costmap_2d::Costmap2D* PathPlanner::getCostMap() const
{
  return costmap_;
}

/**
 * @brief get the size of costmap
 * @return map_size the size of costmap
 */
int PathPlanner::getMapSize() const
{
  return map_size_;
}

/**
 * @brief Transform from grid map(x, y) to grid index(i)
 * @param x grid map x
 * @param y grid map y
 * @return index
 */
int PathPlanner::grid2Index(int x, int y) const
{
  return x + nx_ * y;
}

/**
 * @brief Transform from grid index(i) to grid map(x, y)
 * @param i grid index i
 * @param x grid map x
 * @param y grid map y
 */
void PathPlanner::index2Grid(int i, int& x, int& y) const
{
  x = static_cast<int>(i % nx_);
  y = static_cast<int>(i / nx_);
}

/**
 * @brief Tranform from world map(x, y) to costmap(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 * @return true if successfull, else false
 */
bool PathPlanner::world2Map(double wx, double wy, double& mx, double& my) const
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY())
    return false;

  mx = (wx - costmap_->getOriginX()) / costmap_->getResolution();
  my = (wy - costmap_->getOriginY()) / costmap_->getResolution();

  if (mx < nx_ && my < ny_)
    return true;

  return false;
}

/**
 * @brief Tranform from costmap(x, y) to world map(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 */
void PathPlanner::map2World(double mx, double my, double& wx, double& wy) const
{
  wx = costmap_->getOriginX() + (mx + 0.5) * costmap_->getResolution();
  wy = costmap_->getOriginY() + (my + 0.5) * costmap_->getResolution();
}

/**
 * @brief Inflate the boundary of costmap into obstacles to prevent cross planning
 */
void PathPlanner::outlineMap()
{
  auto pc = costmap_->getCharMap();
  for (int i = 0; i < nx_; i++)
    *pc++ = costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap() + (ny_ - 1) * nx_;
  for (int i = 0; i < nx_; i++)
    *pc++ = costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap();
  for (int i = 0; i < ny_; i++, pc += nx_)
    *pc = costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap() + nx_ - 1;
  for (int i = 0; i < ny_; i++, pc += nx_)
    *pc = costmap_2d::LETHAL_OBSTACLE;
}

bool PathPlanner::validityCheck(double wx, double wy, double& mx, double& my) const
{
  if (!world2Map(wx, wy, mx, my))
  {
    R_WARN << "The robot's position is off the global costmap. Planning will always fail, are you sure the robot "
              "has been properly localized?";
    return false;
  }
  return true;
}
}  // namespace path_planner
}  // namespace rpp
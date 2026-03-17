/**
 * @file social_cost_function.cpp
 * @brief Implementation of the SocialCostFunction.
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#include <cmath>
#include <social_cost_function.h>
#include <social_layer/social_layer.h>

namespace hlpmpccorridor_local_planner
{

SocialCostFunction::SocialCostFunction()
  : layered_costmap_(nullptr), scale_(0.0), enabled_(false)
{
}

bool SocialCostFunction::prepare()
{
  // Check if SocialLayer exists and is accessible
  if (!layered_costmap_)
  {
    // Gracefully disable if costmap is not wired yet.
    enabled_ = false;
    return true;
  }

  // Try to find SocialLayer in the layered costmap
  std::vector<boost::shared_ptr<costmap_2d::Layer>>* plugins = layered_costmap_->getPlugins();
  for (auto& plugin : *plugins)
  {
    if (plugin->getName().find("social") != std::string::npos)
    {
      enabled_ = true;
      return true;
    }
  }

  // If social layer not found, gracefully disable this cost function
  enabled_ = false;
  return true;
}

double SocialCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj)
{
  if (scale_ <= 0.0 || !enabled_ || !layered_costmap_)
    return 0.0;

  // Get direct access to SocialLayer using plugin name matching
  // This follows the same pattern as ReachabilityDWA accessing ReachabilityLayer
  social_layer::SocialLayer* social_layer_ptr = nullptr;

  std::vector<boost::shared_ptr<costmap_2d::Layer>>* plugins = layered_costmap_->getPlugins();
  for (auto& plugin : *plugins)
  {
    if (plugin->getName().find("social") != std::string::npos)
    {
      social_layer_ptr = dynamic_cast<social_layer::SocialLayer*>(plugin.get());
      break;
    }
  }

  if (!social_layer_ptr)
    return 0.0;

  double max_cost = 0.0;

  // Integrate social costs along the trajectory
  // Similar to how obstacle costs are evaluated, but sampling social layer directly
  for (unsigned int i = 0; i < traj.getPointsSize(); ++i)
  {
    double px, py, pth;
    traj.getPoint(i, px, py, pth);

    // Query social cost at this trajectory point via the public interface
    unsigned char social_cost = social_layer_ptr->getSocialCostAt(px, py);

    // Normalize cost (0-254) to range [0, 1]
    double normalized_cost = static_cast<double>(social_cost) / 254.0;

    if (normalized_cost > max_cost)
      max_cost = normalized_cost;
  }

  return scale_ * max_cost;
}

}  // namespace hlpmpccorridor_local_planner

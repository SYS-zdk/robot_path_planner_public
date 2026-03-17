
/**
 * @file: path_processor.h
 * @brief: Path pre-processing or post-processing
 * @author: ZhangDingkun
 * @date: 2024.06.08
 * @version: 1.0
 */
#include "path_planner/path_processor/path_processor.h"

namespace rpp
{
namespace path_planner
{
void PathProcessor::setObstacles(const Points3d& obstacles)
{
  obstacles_.clear();
  for (const auto& obstacle : obstacles)
  {
    obstacles_.emplace_back(obstacle.x(), obstacle.y());
  }
}

void PathProcessor::setObstacles(Points3d&& obstacles)
{
  obstacles_.clear();
  obstacles_ = std::forward<Points3d&&>(obstacles);
}
}  // namespace path_planner
}  // namespace rpp
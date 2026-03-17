
/**
 * @file ackermann_config.h
 * @brief Ackermann steering geometry parameters.
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#pragma once

namespace rpp
{
struct AckermannConfig
{
    // 轴距 [m]
    double wheelbase{ 1.0 };
    // 最大转向角 [rad]
    double max_steer_angle{ 0.83 };
    // 轮距 [m]
    double track_width{ 0.30 };

    double minTurnRadius() const
    {
        return wheelbase / std::tan(max_steer_angle);
    }

    double steeringCenterOffset(double current_steer) const
    {
        return wheelbase / (2.0 * std::sin(current_steer));
    }
};
}  // namespace rpp
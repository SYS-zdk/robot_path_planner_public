/**
 * @file: localreachability_layer.h
 * @brief: Costmap layer that computes a local (robot-conditioned) reachability field
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <boost/thread.hpp>

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <euclidean_distance_field.h>

namespace costmap_2d
{

class LocalReachabilityLayer : public Layer
{
public:
	LocalReachabilityLayer() = default;
	~LocalReachabilityLayer() override = default;

	// Thread-safe snapshot of cached robot-conditioned reachability field (widest-path maximin).
	// Returns false if cache is not yet valid.
	bool getReachabilityCache(std::vector<double>& reach_field, unsigned int& size_x, unsigned int& size_y) const;

	void onInitialize() override;
	void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
						 double* max_y) override;
	void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

private:
	struct RayDir
	{
		int dx;
		int dy;
		double step_len_m;
	};

	void reconfigureCB(const costmap_2d::GenericPluginConfig& config, uint32_t level);
	bool isOccupiedCost(unsigned char cost) const;
	double clip01(double v) const;
	double scoreFromThresholds(double v, double v0, double v1) const;

	double computeClearanceMeters(unsigned int mx, unsigned int my) const;

	double castUntilObstacleMeters(const costmap_2d::Costmap2D& grid, int mx, int my, const RayDir& dir,
										int max_steps) const;
	double computePassageWidthMetersAxisAligned(const costmap_2d::Costmap2D& grid, unsigned int mx, unsigned int my) const;
	double computePassageWidthMetersAdaptive(const costmap_2d::Costmap2D& grid, unsigned int mx, unsigned int my) const;

	double computeReachability(unsigned int mx, unsigned int my, const costmap_2d::Costmap2D& grid) const;
	unsigned char reachabilityToCost(double reachability) const;

	double visualizeValue(double reachability_raw) const;
	void jetColor(double t, uint8_t& r, uint8_t& g, uint8_t& b) const;
	float packRGBFloat(uint8_t r, uint8_t g, uint8_t b) const;
	void publishColorCloud(const costmap_2d::Costmap2D& master_grid);
	void publishColorCloudFromField(const costmap_2d::Costmap2D& master_grid, const std::vector<double>& field);

	// last robot pose (in global frame) from updateBounds
	double last_robot_x_{ 0.0 };
	double last_robot_y_{ 0.0 };
	bool have_robot_pose_{ false };

	ros::NodeHandle nh_;

	// NOTE: do NOT declare another "enabled_" here.
	bool write_to_master_{ false };
	bool publish_color_cloud_{ true };
	bool treat_unknown_as_obstacle_{ true };
	bool include_diagonal_rays_{ true };
	bool adaptive_width_{ true };
	// Scheme-A scales (meters). Larger => less sensitive (more spread, slower to approach 1).
	double risk_clearance_scale_m_{ 0.30 };
	double risk_width_scale_m_{ 0.60 };

	// Visualization-only mapping V(R): keeps planning semantics on raw R.
	std::string viz_mode_{ "raw" };  // raw | clamp | clamp_gamma | sigmoid
	double viz_clamp_min_{ 0.0 };
	double viz_clamp_max_{ 1.0 };
	double viz_gamma_{ 1.0 };
	double viz_sigmoid_k_{ 10.0 };
	double viz_sigmoid_center_{ 0.7 };

	int color_cloud_stride_{ 3 };
	double color_cloud_z_{ 0.02 };

	double update_interval_s_{ 1.0 };
	ros::Time last_update_{};
	// ✨ 自适应频率优化: 保存前一个occupancy状态用于变化检测
	std::vector<unsigned char> prev_occupancy_bin_;
	bool cache_valid_{ false };
	unsigned int cache_size_x_{ 0 };
	unsigned int cache_size_y_{ 0 };
	std::vector<double> cache_reach_field_;

	double robot_radius_{ 0.25 };
	double safety_distance_min_{ 0.05 };
	double safety_distance_good_{ 0.25 };

	double width_min_{ -1.0 };
	double width_good_{ -1.0 };

	double max_raytrace_range_m_{ 3.0 };

	double alpha_clearance_{ 1.0 };
	double alpha_width_{ 1.0 };

	int occupied_cost_threshold_{ static_cast<int>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE) };
	int max_cost_{ 252 };

	ros::Publisher color_cloud_pub_;

	std::unique_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>> dsrv_;

	mutable boost::mutex mutex_;
	std::unique_ptr<euclidean_distance_field::EuclideanDistanceField> edf_manager_;
	std::vector<unsigned char> occupancy_bin_;
	std::vector<std::vector<double>> edf_sq_cells_;
	bool edf_has_obstacles_{ true };

	std::vector<RayDir> rays_;
};

}  // namespace costmap_2d

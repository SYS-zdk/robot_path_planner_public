/**
 * @file: globalreachability_layer.h
 * @brief: Costmap layer that computes a global reachability/risk field (ESDF-based)
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
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include <euclidean_distance_field.h>

namespace costmap_2d
{

class GlobalReachabilityLayer : public Layer
{
public:
	GlobalReachabilityLayer() = default;
	~GlobalReachabilityLayer() override = default;

	// Thread-safe snapshot of cached planning reachability field.
	// Returns false if cache is not yet valid.
	bool getEnvScoreCache(std::vector<double>& env_score, unsigned int& size_x, unsigned int& size_y) const;

	void onInitialize() override;
	void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
						 double* max_y) override;
	void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

private:
	void reconfigureCB(const costmap_2d::GenericPluginConfig& config, uint32_t level);
	bool isOccupiedCost(unsigned char cost) const;
	double clip01(double v) const;

	double computeClearanceMeters(unsigned int mx, unsigned int my) const;
	double computeReachabilitySmooth(unsigned int mx, unsigned int my, const costmap_2d::Costmap2D& grid) const;
	unsigned char reachabilityToCost(double reachability) const;

	double visualizeValue(double reachability_raw) const;
	void jetColor(double t, uint8_t& r, uint8_t& g, uint8_t& b) const;
	void gradientColor(double t, uint8_t& r, uint8_t& g, uint8_t& b) const;
	float packRGBFloat(uint8_t r, uint8_t g, uint8_t b) const;
	float packRGBAFloat(uint8_t r, uint8_t g, uint8_t b, uint8_t a) const;

	ros::NodeHandle nh_;

	bool write_to_master_{ false };
	bool publish_color_cloud_{ true };
	bool publish_grid_{ true };
	bool treat_unknown_as_obstacle_{ true };
	// Scheme-A scales (meters). Larger => less sensitive.
	double risk_clearance_scale_m_{ 0.30 };
	double risk_width_scale_m_{ 0.60 };

	// Visualization-only mapping V(R): keeps planning semantics on raw R.
	std::string viz_mode_{ "raw" };  // raw | clamp | clamp_gamma | sigmoid
	std::string viz_colormap_{ "gradient" };  // gradient | jet
	double viz_clamp_min_{ 0.0 };
	double viz_clamp_max_{ 1.0 };
	double viz_gamma_{ 1.0 };
	double viz_sigmoid_k_{ 10.0 };
	double viz_sigmoid_center_{ 0.7 };
	// ESDF-like visualization scale (meters): v_esdf = d/(d+scale)
	double viz_esdf_scale_m_{ 1.0 };
	// PointCloud alpha (opacity): alpha = min + (max-min)*viz_value.
	double cloud_alpha_min_{ 0.25 };
	double cloud_alpha_max_{ 1.0 };

	int color_cloud_stride_{ 1 };
	double color_cloud_z_{ 0.02 };

	// Optional smoothing (box filter radius in cells). 0 disables.
	int smooth_radius_{ 1 };

	// Voronoi/medial-axis inspired skeleton scoring (derived from ESDF local maxima)
	bool enable_skeleton_{ true };
	int ridge_window_radius_cells_{ 1 };     // local-max window radius (cells)
	double ridge_min_clearance_m_{ 0.20 };   // only consider ridges if clearance >= this
	double ridge_attract_scale_m_{ 0.50 };   // exp(-dist/scale)
	double alpha_ridge_{ 1.0 };              // weight of ridge attraction term
	int skeleton_seed_dilate_radius_cells_{ 1 };  // densify skeleton seeds (0 disables)

	double update_interval_s_{ 2.0 };
	ros::Time last_update_{};
	// Cache last computed env field to avoid periodic "on/off" contribution when update_interval_s is large.
	bool cache_valid_{ false };
	unsigned int cache_size_x_{ 0 };
	unsigned int cache_size_y_{ 0 };
	std::vector<double> cache_env_score_;
	std::vector<double> cache_clearance_m_;
	std::vector<double> cache_dist_to_ridge_m_;

	double robot_radius_{ 0.25 };
	double safety_distance_min_{ 0.05 };
	double safety_distance_good_{ 0.25 };

	double width_min_{ -1.0 };
	double width_good_{ -1.0 };

	double alpha_clearance_{ 1.0 };
	double alpha_width_{ 1.0 };

	int occupied_cost_threshold_{ 254 };
	int max_cost_{ 252 };

	ros::Publisher color_cloud_pub_;
	ros::Publisher grid_pub_;

	std::unique_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>> dsrv_;

	mutable boost::mutex mutex_;
	std::unique_ptr<euclidean_distance_field::EuclideanDistanceField> edf_manager_;
	std::vector<unsigned char> occupancy_bin_;
	std::vector<std::vector<double>> edf_sq_cells_;
};

}  // namespace costmap_2d

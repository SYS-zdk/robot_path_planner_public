/**
 * @file: localreachability_layer.cpp
 * @brief: Implementation of LocalReachabilityLayer
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#include "localreachability_layer/localreachability_layer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <string>

#include <cstring>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointField.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::LocalReachabilityLayer, costmap_2d::Layer)

namespace costmap_2d
{

namespace
{
struct PQNode
{
	double score;
	unsigned int idx;
};

struct PQGreater
{
	bool operator()(const PQNode& a, const PQNode& b) const { return a.score < b.score; }
};
}

void LocalReachabilityLayer::onInitialize()
{
	nh_ = ros::NodeHandle("~/" + name_);

	nh_.param("enabled", enabled_, true);
	nh_.param("write_to_master", write_to_master_, false);
	nh_.param("publish_color_cloud", publish_color_cloud_, true);
	nh_.param("treat_unknown_as_obstacle", treat_unknown_as_obstacle_, true);
	nh_.param("include_diagonal_rays", include_diagonal_rays_, true);
	nh_.param("adaptive_width", adaptive_width_, true);

	nh_.param("risk_clearance_scale_m", risk_clearance_scale_m_, risk_clearance_scale_m_);
	nh_.param("risk_width_scale_m", risk_width_scale_m_, risk_width_scale_m_);

	// Visualization-only mapping V(R): used only for heatmap coloring.
	nh_.param<std::string>("viz_mode", viz_mode_, viz_mode_);
	nh_.param("viz_clamp_min", viz_clamp_min_, viz_clamp_min_);
	nh_.param("viz_clamp_max", viz_clamp_max_, viz_clamp_max_);
	nh_.param("viz_gamma", viz_gamma_, viz_gamma_);
	nh_.param("viz_sigmoid_k", viz_sigmoid_k_, viz_sigmoid_k_);
	nh_.param("viz_sigmoid_center", viz_sigmoid_center_, viz_sigmoid_center_);

	nh_.param("color_cloud_stride", color_cloud_stride_, 3);
	nh_.param("color_cloud_z", color_cloud_z_, 0.02);

	ROS_INFO_STREAM("[localreachability_layer] viz_mode=" << viz_mode_ << " clamp=[" << viz_clamp_min_ << "," << viz_clamp_max_
										<< "] gamma=" << viz_gamma_ << " sigmoid(k=" << viz_sigmoid_k_ << ",c=" << viz_sigmoid_center_ << ")"
										<< "; cloud intensity publishes r_viz (0..1), RGB uses jet(r_viz)");

	if (publish_color_cloud_)
	{
		color_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("reachability_cloud", 1, true);
		ROS_INFO_STREAM("[localreachability_layer] advertise: " << nh_.resolveName("reachability_cloud"));
	}

	nh_.param("update_interval_s", update_interval_s_, 1.0);

	dsrv_ = std::make_unique<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>(nh_);
	dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb =
			boost::bind(&LocalReachabilityLayer::reconfigureCB, this, _1, _2);
	dsrv_->setCallback(cb);

	edf_manager_ = std::make_unique<euclidean_distance_field::EuclideanDistanceField>();

	const double res = layered_costmap_->getCostmap()->getResolution();
	rays_.clear();
	rays_.push_back({ 1, 0, res });
	rays_.push_back({ 0, 1, res });
	if (include_diagonal_rays_)
	{
		rays_.push_back({ 1, 1, res * std::sqrt(2.0) });
		rays_.push_back({ 1, -1, res * std::sqrt(2.0) });
	}

	current_ = true;
}

void LocalReachabilityLayer::reconfigureCB(const costmap_2d::GenericPluginConfig& config, uint32_t level)
{
	enabled_ = config.enabled;
}

bool LocalReachabilityLayer::getReachabilityCache(std::vector<double>& reach_field, unsigned int& size_x,
									 unsigned int& size_y) const
{
	boost::unique_lock<boost::mutex> lock(mutex_);
	if (!cache_valid_)
	{
		return false;
	}
	reach_field = cache_reach_field_;
	size_x = cache_size_x_;
	size_y = cache_size_y_;
	return reach_field.size() == static_cast<size_t>(size_x) * static_cast<size_t>(size_y);
}

void LocalReachabilityLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
										  double* max_x, double* max_y)
{
	ROS_INFO_ONCE("[localreachability_layer] updateBounds() entered");
	last_robot_x_ = robot_x;
	last_robot_y_ = robot_y;
	have_robot_pose_ = true;
	if (!enabled_)
	{
		ROS_WARN_ONCE("[localreachability_layer] disabled: enabled=false");
		return;
	}

	// Ensure periodic update/publish by expanding bounds to full map.
	const auto* costmap = layered_costmap_->getCostmap();
	if (!costmap)
	{
		return;
	}
	const double ox = costmap->getOriginX();
	const double oy = costmap->getOriginY();
	const double res = costmap->getResolution();
	const double wx = ox + static_cast<double>(costmap->getSizeInCellsX()) * res;
	const double wy = oy + static_cast<double>(costmap->getSizeInCellsY()) * res;

	*min_x = std::min(*min_x, ox);
	*min_y = std::min(*min_y, oy);
	*max_x = std::max(*max_x, wx);
	*max_y = std::max(*max_y, wy);
}

bool LocalReachabilityLayer::isOccupiedCost(unsigned char cost) const
{
	if (cost == costmap_2d::NO_INFORMATION)
	{
		return treat_unknown_as_obstacle_;
	}
	return cost >= static_cast<unsigned char>(occupied_cost_threshold_);
}

double LocalReachabilityLayer::clip01(double v) const
{
	if (!std::isfinite(v))
		return 0.0;
	if (v < 0.0)
		return 0.0;
	if (v > 1.0)
		return 1.0;
	return v;
}

double LocalReachabilityLayer::visualizeValue(double reachability_raw) const
{
	double v = clip01(reachability_raw);

	const double vmin = viz_clamp_min_;
	const double vmax = viz_clamp_max_;
	if (std::isfinite(vmin) && std::isfinite(vmax) && vmax > vmin)
	{
		v = clip01((v - vmin) / (vmax - vmin));
	}

	if (viz_mode_ == "raw" || viz_mode_ == "clamp")
	{
		return v;
	}
	if (viz_mode_ == "clamp_gamma")
	{
		const double g = viz_gamma_;
		if (std::isfinite(g) && g > 0.0)
		{
			return clip01(std::pow(v, g));
		}
		return v;
	}
	if (viz_mode_ == "sigmoid")
	{
		const double k = viz_sigmoid_k_;
		const double c = viz_sigmoid_center_;
		if (std::isfinite(k) && std::isfinite(c) && std::fabs(k) > 1e-9)
		{
			const double z = -k * (v - c);
			if (z > 60.0)
				return 0.0;
			if (z < -60.0)
				return 1.0;
			return 1.0 / (1.0 + std::exp(z));
		}
		return v;
	}

	return v;
}

double LocalReachabilityLayer::scoreFromThresholds(double v, double v0, double v1) const
{
	if (!(std::isfinite(v0) && std::isfinite(v1)) || v1 <= v0)
	{
		return (v > v0) ? 1.0 : 0.0;
	}
	return clip01((v - v0) / (v1 - v0));
}

double LocalReachabilityLayer::computeClearanceMeters(unsigned int mx, unsigned int my) const
{
	if (!edf_has_obstacles_)
	{
		return std::numeric_limits<double>::infinity();
	}
	if (edf_sq_cells_.empty() || my >= edf_sq_cells_.size() || mx >= edf_sq_cells_[my].size())
	{
		return std::numeric_limits<double>::infinity();
	}
	const double d2 = edf_sq_cells_[my][mx];
	if (!std::isfinite(d2))
	{
		return std::numeric_limits<double>::infinity();
	}
	const double res = layered_costmap_->getCostmap()->getResolution();
	return std::sqrt(std::max(0.0, d2)) * res;
}

double LocalReachabilityLayer::castUntilObstacleMeters(const costmap_2d::Costmap2D& grid, int mx, int my, const RayDir& dir,
															int max_steps) const
{
	const int size_x = static_cast<int>(grid.getSizeInCellsX());
	const int size_y = static_cast<int>(grid.getSizeInCellsY());
	double traveled = 0.0;
	int x = mx;
	int y = my;
	for (int step = 1; step <= max_steps; ++step)
	{
		x += dir.dx;
		y += dir.dy;
		traveled += dir.step_len_m;

		if (x < 0 || y < 0 || x >= size_x || y >= size_y)
		{
			break;
		}
		if (isOccupiedCost(grid.getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y))))
		{
			break;
		}
	}
	return traveled;
}

double LocalReachabilityLayer::computePassageWidthMetersAxisAligned(const costmap_2d::Costmap2D& grid, unsigned int mx,
															unsigned int my) const
{
	const double res = grid.getResolution();
	const int max_steps = std::max(1, static_cast<int>(std::ceil(max_raytrace_range_m_ / res)));

	double min_width = std::numeric_limits<double>::infinity();
	const int x = static_cast<int>(mx);
	const int y = static_cast<int>(my);

	for (const auto& dir : rays_)
	{
		const double plus = castUntilObstacleMeters(grid, x, y, dir, max_steps);
		RayDir inv{ -dir.dx, -dir.dy, dir.step_len_m };
		const double minus = castUntilObstacleMeters(grid, x, y, inv, max_steps);
		min_width = std::min(min_width, plus + minus);
	}
	return min_width;
}

double LocalReachabilityLayer::computePassageWidthMetersAdaptive(const costmap_2d::Costmap2D& grid, unsigned int mx,
															 unsigned int my) const
{
	if (!edf_has_obstacles_)
	{
		return computePassageWidthMetersAxisAligned(grid, mx, my);
	}

	const double res = grid.getResolution();
	const int max_steps = std::max(1, static_cast<int>(std::ceil(max_raytrace_range_m_ / res)));

	double gx = 0.0, gy = 0.0;
	edf_manager_->getGradient(static_cast<double>(mx), static_cast<double>(my), gx, gy);
	const double n = std::sqrt(gx * gx + gy * gy);
	if (!(n > 1e-6))
	{
		return computePassageWidthMetersAxisAligned(grid, mx, my);
	}

	const double tx = -gy / n;
	const double ty = gx / n;

	auto stepSign = [](double v) -> int {
		if (v > 0.35)
			return 1;
		if (v < -0.35)
			return -1;
		return 0;
	};

	const int dx = stepSign(tx);
	const int dy = stepSign(ty);
	if (dx == 0 && dy == 0)
	{
		return computePassageWidthMetersAxisAligned(grid, mx, my);
	}

	const double step_len = res * ((dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0);
	const RayDir dir{ dx, dy, step_len };
	const RayDir inv{ -dx, -dy, step_len };

	const int x = static_cast<int>(mx);
	const int y = static_cast<int>(my);
	const double plus = castUntilObstacleMeters(grid, x, y, dir, max_steps);
	const double minus = castUntilObstacleMeters(grid, x, y, inv, max_steps);

	const double w_adapt = plus + minus;
	const double w_axis = computePassageWidthMetersAxisAligned(grid, mx, my);
	return std::min(w_adapt, w_axis);
}

double LocalReachabilityLayer::computeReachability(unsigned int mx, unsigned int my, const costmap_2d::Costmap2D& grid) const
{
	const unsigned char c = grid.getCost(mx, my);
	
	// 如果是完全占用，返回0
	if (isOccupiedCost(c))
	{
		return 0.0;
	}

	// 考虑社交成本：即使成本小于occupation_threshold，高社交成本也应该降低可达性
	// social_layer添加的成本通常是50-150，应该被考虑进来
	double cost_penalty = 1.0;
	if (c > 0 && c < static_cast<unsigned char>(occupied_cost_threshold_))
	{
		// 将costmap值（0-254）映射到降低因子（1.0-0.0）
		// 成本越高，降低因子越小，可达性越低
		cost_penalty = 1.0 - (static_cast<double>(c) / 254.0);  // 成本254→penalty 0.0, 成本0→penalty 1.0
		// 调试：打印有社交成本的栅格
		if (c > 50)  // 只在明显的社交成本处打印
		{
			static unsigned int debug_count = 0;
			if (debug_count++ % 100 == 0)
			{
				ROS_DEBUG("[localreachability_layer] HIGH COST: mx=%u my=%u cost=%u penalty=%.3f", mx, my, c, cost_penalty);
			}
		}
	}

	const double clearance_m = computeClearanceMeters(mx, my);
	const double c0 = robot_radius_ + safety_distance_min_;

	const double w0 = (width_min_ > 0.0) ? width_min_ : 2.0 * c0;
	const double width_m = adaptive_width_ ? computePassageWidthMetersAdaptive(grid, mx, my)
						   : computePassageWidthMetersAxisAligned(grid, mx, my);

	if (!(clearance_m > c0) || !(width_m > w0))
	{
		return 0.0;
	}

	auto continuous01 = [this](double delta, double scale) -> double {
		if (!std::isfinite(delta))
		{
			return 1.0;
		}
		delta = std::max(0.0, delta);
		scale = std::max(1e-6, std::fabs(scale));
		return clip01(delta / (delta + scale));
	};

	const double clearance_score = continuous01(clearance_m - c0, risk_clearance_scale_m_);
	const double width_score = continuous01(width_m - w0, risk_width_scale_m_);

	const double a_c = std::max(0.0, alpha_clearance_);
	const double a_w = std::max(0.0, alpha_width_);

	double r = 1.0;
	if (a_c > 0.0)
		r *= std::pow(clearance_score, a_c);
	if (a_w > 0.0)
		r *= std::pow(width_score, a_w);

	// 应用社交成本惩罚
	double r_before_penalty = r;
	r *= cost_penalty;
	
	// 调试：如果penalty有效果，打印对比
	if (cost_penalty < 0.99 && r < r_before_penalty)
	{
		static unsigned int debug_count2 = 0;
		if (debug_count2++ % 100 == 0)
		{
			ROS_DEBUG("[localreachability_layer] PENALTY APPLIED: r_before=%.3f r_after=%.3f penalty=%.3f cost=%u",
					  r_before_penalty, r, cost_penalty, static_cast<unsigned int>(c));
		}
	}

	return clip01(r);
}

unsigned char LocalReachabilityLayer::reachabilityToCost(double reachability) const
{
	reachability = clip01(reachability);
	const int c = static_cast<int>(std::round((1.0 - reachability) * static_cast<double>(max_cost_)));
	return static_cast<unsigned char>(std::max(0, std::min(max_cost_, c)));
}

void LocalReachabilityLayer::jetColor(double t, uint8_t& r, uint8_t& g, uint8_t& b) const
{
	t = clip01(t);
	const double four_t = 4.0 * t;
	const double rt = std::min(std::max(std::min(four_t - 1.5, -four_t + 4.5), 0.0), 1.0);
	const double gt = std::min(std::max(std::min(four_t - 0.5, -four_t + 3.5), 0.0), 1.0);
	const double bt = std::min(std::max(std::min(four_t + 0.5, -four_t + 2.5), 0.0), 1.0);
	r = static_cast<uint8_t>(std::round(rt * 255.0));
	g = static_cast<uint8_t>(std::round(gt * 255.0));
	b = static_cast<uint8_t>(std::round(bt * 255.0));
}

float LocalReachabilityLayer::packRGBFloat(uint8_t r, uint8_t g, uint8_t b) const
{
	const uint32_t rgb = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
	float out;
	static_assert(sizeof(float) == sizeof(uint32_t), "float/uint32 size mismatch");
	std::memcpy(&out, &rgb, sizeof(float));
	return out;
}

void LocalReachabilityLayer::publishColorCloud(const costmap_2d::Costmap2D& master_grid)
{
	if (!publish_color_cloud_ || !color_cloud_pub_)
	{
		return;
	}

	// Default behavior: publish point-wise traversability
	const unsigned int size_x = master_grid.getSizeInCellsX();
	const unsigned int size_y = master_grid.getSizeInCellsY();
	std::vector<double> field(size_x * size_y, 0.0);
	for (unsigned int y = 0; y < size_y; ++y)
	{
		for (unsigned int x = 0; x < size_x; ++x)
		{
			field[x + y * size_x] = computeReachability(x, y, master_grid);
		}
	}
	publishColorCloudFromField(master_grid, field);
}

void LocalReachabilityLayer::publishColorCloudFromField(const costmap_2d::Costmap2D& master_grid, const std::vector<double>& field)
{
	if (!publish_color_cloud_ || !color_cloud_pub_)
	{
		return;
	}

	const unsigned int size_x = master_grid.getSizeInCellsX();
	const unsigned int size_y = master_grid.getSizeInCellsY();
	const int stride = std::max(1, color_cloud_stride_);

	sensor_msgs::PointCloud2 cloud;
	cloud.header.stamp = ros::Time::now();
	cloud.header.frame_id = layered_costmap_->getGlobalFrameID();
	cloud.height = 1;
	cloud.is_bigendian = false;
	cloud.is_dense = true;

	cloud.fields.resize(5);
	cloud.fields[0].name = "x";
	cloud.fields[0].offset = 0;
	cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	cloud.fields[0].count = 1;
	cloud.fields[1].name = "y";
	cloud.fields[1].offset = 4;
	cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
	cloud.fields[1].count = 1;
	cloud.fields[2].name = "z";
	cloud.fields[2].offset = 8;
	cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
	cloud.fields[2].count = 1;
	cloud.fields[3].name = "rgb";
	cloud.fields[3].offset = 12;
	cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
	cloud.fields[3].count = 1;
	cloud.fields[4].name = "intensity";
	cloud.fields[4].offset = 16;
	cloud.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
	cloud.fields[4].count = 1;

	cloud.point_step = 20;

	const unsigned int approx_nx = (size_x + static_cast<unsigned int>(stride) - 1U) / static_cast<unsigned int>(stride);
	const unsigned int approx_ny = (size_y + static_cast<unsigned int>(stride) - 1U) / static_cast<unsigned int>(stride);
	cloud.data.reserve(static_cast<size_t>(approx_nx) * approx_ny * cloud.point_step);

	double min_raw = std::numeric_limits<double>::infinity();
	double max_raw = -std::numeric_limits<double>::infinity();
	double min_viz = std::numeric_limits<double>::infinity();
	double max_viz = -std::numeric_limits<double>::infinity();

	for (unsigned int y = 0; y < size_y; y += static_cast<unsigned int>(stride))
	{
		for (unsigned int x = 0; x < size_x; x += static_cast<unsigned int>(stride))
		{
			const double r_raw = field[x + y * size_x];
			const double r_viz = visualizeValue(r_raw);

			min_raw = std::min(min_raw, r_raw);
			max_raw = std::max(max_raw, r_raw);
			min_viz = std::min(min_viz, r_viz);
			max_viz = std::max(max_viz, r_viz);

			double wx, wy;
			master_grid.mapToWorld(x, y, wx, wy);
			const float fx = static_cast<float>(wx);
			const float fy = static_cast<float>(wy);
			const float fz = static_cast<float>(color_cloud_z_);

			uint8_t rr, gg, bb;
			jetColor(r_viz, rr, gg, bb);
			const float frgb = packRGBFloat(rr, gg, bb);
			const float fintensity = static_cast<float>(r_viz);

			const size_t base = cloud.data.size();
			cloud.data.resize(base + cloud.point_step);
			std::memcpy(&cloud.data[base + 0], &fx, sizeof(float));
			std::memcpy(&cloud.data[base + 4], &fy, sizeof(float));
			std::memcpy(&cloud.data[base + 8], &fz, sizeof(float));
			std::memcpy(&cloud.data[base + 12], &frgb, sizeof(float));
			std::memcpy(&cloud.data[base + 16], &fintensity, sizeof(float));
		}
	}

	ROS_INFO_THROTTLE(2.0,
						"[localreachability_layer] cloud stats: r_raw[min=%.3f,max=%.3f], r_viz[min=%.3f,max=%.3f] (if r_viz~1 -> Intensity looks red)",
						min_raw, max_raw, min_viz, max_viz);

	cloud.width = static_cast<uint32_t>(cloud.data.size() / cloud.point_step);
	cloud.row_step = cloud.width * cloud.point_step;
	color_cloud_pub_.publish(cloud);
}

void LocalReachabilityLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
	ROS_INFO_ONCE("[localreachability_layer] updateCosts() entered");
	if (!enabled_)
	{
		ROS_WARN_ONCE("[localreachability_layer] updateCosts skipped: enabled=false");
		return;
	}
	if (!have_robot_pose_)
	{
		ROS_WARN_THROTTLE(2.0, "[localreachability_layer] no robot pose yet; skip update");
		return;
	}

	// 调试：检查costmap中的成本分布
	{
		unsigned int size_x = master_grid.getSizeInCellsX();
		unsigned int size_y = master_grid.getSizeInCellsY();
		size_t nonzero_cost_cells = 0, high_cost_cells = 0;
		for (unsigned int y = 0; y < size_y; ++y) {
			for (unsigned int x = 0; x < size_x; ++x) {
				unsigned char c = master_grid.getCost(x, y);
				if (c > 0) ++nonzero_cost_cells;
				if (c > 50) ++high_cost_cells;
			}
		}
		ROS_INFO_THROTTLE(1.0, "[localreachability_layer] costmap stats: nonzero=%zu high_cost(>50)=%zu total=%u",
						  nonzero_cost_cells, high_cost_cells, size_x * size_y);
	}

	boost::unique_lock<boost::mutex> lock(mutex_);

	const unsigned int size_x = master_grid.getSizeInCellsX();
	const unsigned int size_y = master_grid.getSizeInCellsY();
	const double map_res = master_grid.getResolution();

	// Important: rays_ step length must match current map resolution.
	// If the costmap resolution was 0/undefined during onInitialize(), rays_ could have step_len_m=0,
	// which would force width_m=0 everywhere and thus reachability=0.
	if (!rays_.empty() && std::isfinite(map_res) && map_res > 1e-9)
	{
		for (auto& r : rays_)
		{
			r.step_len_m = map_res * ((r.dx != 0 && r.dy != 0) ? std::sqrt(2.0) : 1.0);
		}
	}

	// ✨ 优化：快速检测occupancy变化 + 同时进行诊断统计 (避免O(n)重复遍历)
	std::vector<unsigned char> curr_occupancy(size_x * size_y);
	size_t n_unknown = 0, n_occ = 0, n_free = 0, n_ge_253 = 0, n_ge_254 = 0;
	unsigned int max_c = 0;
	
	for (unsigned int y = 0; y < size_y; ++y)
	{
		for (unsigned int x = 0; x < size_x; ++x)
		{
			const unsigned char c = master_grid.getCost(x, y);
			max_c = std::max<unsigned int>(max_c, static_cast<unsigned int>(c));
			
			// Occupancy detection
			const bool occ = isOccupiedCost(c);
			curr_occupancy[x + y * size_x] = occ ? 1 : 0;
			
			// Cost statistics (同步计算，避免O(n)重复)
			if (c >= static_cast<unsigned char>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
				++n_ge_253;
			if (c >= static_cast<unsigned char>(costmap_2d::LETHAL_OBSTACLE))
				++n_ge_254;
			if (c == costmap_2d::NO_INFORMATION)
			{
				++n_unknown;
				if (treat_unknown_as_obstacle_) ++n_occ;
				else ++n_free;
			}
			else if (occ) ++n_occ;
			else ++n_free;
		}
	}

	bool occupancy_changed = false;
	if (!prev_occupancy_bin_.empty() && prev_occupancy_bin_.size() == curr_occupancy.size())
	{
		occupancy_changed = !std::equal(prev_occupancy_bin_.begin(), prev_occupancy_bin_.end(), 
										curr_occupancy.begin());
	}
	else
	{
		occupancy_changed = true;  // 首次或大小不匹配
	}
	prev_occupancy_bin_ = curr_occupancy;

	// ✨ 改进的时间检查：自适应阈值
	const ros::Time now = ros::Time::now();
	bool skip_computation = false;
	if (!last_update_.isZero() && update_interval_s_ > 1e-6)
	{
		double dt = (now - last_update_).toSec();
		
		// 关键：如果occupancy变了，立即更新（阈值=20ms）
		// 如果occupancy没变，使用配置的update_interval_s
		double threshold = occupancy_changed ? 0.02 : update_interval_s_;
		
		if (dt < threshold)
		{
			skip_computation = true;
			if (!occupancy_changed)
			{
				ROS_DEBUG_THROTTLE(1.0, "[localreachability_layer] occupancy unchanged, using cache (dt=%.3fs < threshold=%.3fs)", 
								   dt, threshold);
			}
			// 关键修复：即使跳过计算，也要发布缓存的可视化
			if (cache_valid_ && cache_reach_field_.size() == size_x * size_y && publish_color_cloud_)
			{
				publishColorCloudFromField(master_grid, cache_reach_field_);
			}
			// 记录诊断信息（throttle，避免频繁输出）
			const double denom = std::max(1.0, static_cast<double>(size_x) * size_y);
			const double c0 = robot_radius_ + safety_distance_min_;
			const double w0 = (width_min_ > 0.0) ? width_min_ : 2.0 * c0;
			ROS_INFO_THROTTLE(2.0,
							"[localreachability_layer] CACHED: size=%ux%u max_cost=%u ge253=%.1f%% ge254=%.1f%% unknown=%.1f%% occ=%.1f%% free=%.1f%% [cache_hit]",
							size_x, size_y, max_c,
							100.0 * (static_cast<double>(n_ge_253) / denom),
							100.0 * (static_cast<double>(n_ge_254) / denom),
							100.0 * (static_cast<double>(n_unknown) / denom),
							100.0 * (static_cast<double>(n_occ) / denom),
							100.0 * (static_cast<double>(n_free) / denom));
			return;
		}
	}
	last_update_ = now;

	// 诊断信息：统计已在occupancy检测中完成，此处直接使用
	// (避免重复O(n)遍历，提高性能)
	{
		const double c0 = robot_radius_ + safety_distance_min_;
		const double w0 = (width_min_ > 0.0) ? width_min_ : 2.0 * c0;
		const double denom = std::max(1.0, static_cast<double>(size_x) * size_y);
		ROS_INFO_THROTTLE(2.0,
						"[localreachability_layer] FULL_UPDATE: size=%ux%u max_cost=%u ge253=%.1f%% ge254=%.1f%% unknown=%.1f%% occ=%.1f%% free=%.1f%%; occ_thr=%d; treat_unknown_as_obstacle=%s; c0=%.2f w0=%.2f",
						size_x, size_y, max_c,
						100.0 * (static_cast<double>(n_ge_253) / denom),
						100.0 * (static_cast<double>(n_ge_254) / denom),
						100.0 * (static_cast<double>(n_unknown) / denom),
						100.0 * (static_cast<double>(n_occ) / denom),
						100.0 * (static_cast<double>(n_free) / denom),
						occupied_cost_threshold_,
						(treat_unknown_as_obstacle_ ? "true" : "false"),
						c0, w0);
	}

	occupancy_bin_.assign(size_x * size_y, costmap_2d::FREE_SPACE);
	size_t occ_cells = 0;
	for (unsigned int y = 0; y < size_y; ++y)
	{
		for (unsigned int x = 0; x < size_x; ++x)
		{
			const bool occ = (curr_occupancy[x + y * size_x] != 0);
			if (occ)
			{
				++occ_cells;
			}
			occupancy_bin_[x + y * size_x] = occ ? costmap_2d::INSCRIBED_INFLATED_OBSTACLE : costmap_2d::FREE_SPACE;
		}
	}

	// ✨ 只在occupancy真的变化时才重新计算EDF (避免不必要的计算)
	if (occupancy_changed)
	{
		edf_has_obstacles_ = (occ_cells > 0);
		if (edf_has_obstacles_)
		{
			edf_manager_->setGridMap(occupancy_bin_.data(), static_cast<int>(size_x), static_cast<int>(size_y));
			edf_manager_->compute2d();
			edf_sq_cells_ = edf_manager_->getEDF();
		}
		else
		{
			// Important edge case:
			// If there are no obstacles in the local window, some EDF implementations return 0 everywhere,
			// which would make clearance_m<=c0 and force reachability to 0. In this case, we treat clearance as +inf.
			edf_sq_cells_.assign(size_y, std::vector<double>(size_x, std::numeric_limits<double>::infinity()));
		}
		ROS_DEBUG_THROTTLE(1.0, "[localreachability_layer] EDF recomputed (occupancy changed, %.1f%% occ)", 
						   100.0 * occ_cells / std::max(1.0, static_cast<double>(size_x * size_y)));
	}
	else
	{
		ROS_DEBUG_THROTTLE(2.0, "[localreachability_layer] Using cached EDF (occupancy stable)");
	}

	// 1) Point-wise traversability
	std::vector<double> local_score(size_x * size_y, 0.0);
	for (unsigned int y = 0; y < size_y; ++y)
	{
		for (unsigned int x = 0; x < size_x; ++x)
		{
			local_score[x + y * size_x] = computeReachability(x, y, master_grid);
		}
	}

	// 2) Robot-conditioned reachability via widest-path (maximin) propagation
	unsigned int sx = 0, sy = 0;
	if (!master_grid.worldToMap(last_robot_x_, last_robot_y_, sx, sy))
	{
		ROS_WARN_THROTTLE(2.0, "[localreachability_layer] robot pose out of map; skip update");
		return;
	}
	const unsigned int sidx = sx + sy * size_x;
	const double s0 = local_score[sidx];
	{
		const double clearance_m = computeClearanceMeters(sx, sy);
		const double width_m = adaptive_width_ ? computePassageWidthMetersAdaptive(master_grid, sx, sy)
									 : computePassageWidthMetersAxisAligned(master_grid, sx, sy);
		ROS_INFO_THROTTLE(2.0,
						"[localreachability_layer] robot cell debug: res=%.3f rays=%zu clearance=%.3f width=%.3f reach=%.3f",
						map_res, rays_.size(),
						(std::isfinite(clearance_m) ? clearance_m : 1e9),
						(std::isfinite(width_m) ? width_m : 1e9),
						s0);
	}

	std::vector<double> reach_field(size_x * size_y, 0.0);
	std::priority_queue<PQNode, std::vector<PQNode>, PQGreater> pq;
	reach_field[sidx] = s0;
	pq.push(PQNode{ s0, sidx });

	auto relax = [&](unsigned int cur_idx, unsigned int nx, unsigned int ny) {
		const unsigned int nidx = nx + ny * size_x;
		const double candidate = std::min(reach_field[cur_idx], local_score[nidx]);
		if (candidate > reach_field[nidx] + 1e-12)
		{
			reach_field[nidx] = candidate;
			pq.push(PQNode{ candidate, nidx });
		}
	};

	while (!pq.empty())
	{
		const PQNode top = pq.top();
		pq.pop();
		if (top.score + 1e-12 < reach_field[top.idx])
		{
			continue;
		}
		const unsigned int cx = top.idx % size_x;
		const unsigned int cy = top.idx / size_x;

		if (cx > 0)
			relax(top.idx, cx - 1, cy);
		if (cx + 1 < size_x)
			relax(top.idx, cx + 1, cy);
		if (cy > 0)
			relax(top.idx, cx, cy - 1);
		if (cy + 1 < size_y)
			relax(top.idx, cx, cy + 1);
		if (cx > 0 && cy > 0)
			relax(top.idx, cx - 1, cy - 1);
		if (cx + 1 < size_x && cy > 0)
			relax(top.idx, cx + 1, cy - 1);
		if (cx > 0 && cy + 1 < size_y)
			relax(top.idx, cx - 1, cy + 1);
		if (cx + 1 < size_x && cy + 1 < size_y)
			relax(top.idx, cx + 1, cy + 1);
	}

	if (write_to_master_)
	{
		min_i = std::max(0, min_i);
		min_j = std::max(0, min_j);
		max_i = std::min(static_cast<int>(size_x), max_i);
		max_j = std::min(static_cast<int>(size_y), max_j);

		for (int j = min_j; j < max_j; ++j)
		{
			for (int i = min_i; i < max_i; ++i)
			{
				const unsigned int x = static_cast<unsigned int>(i);
				const unsigned int y = static_cast<unsigned int>(j);
				const unsigned char old_cost = master_grid.getCost(x, y);
				if (isOccupiedCost(old_cost))
				{
					continue;
				}
				const double r = reach_field[x + y * size_x];
				const unsigned char new_cost = reachabilityToCost(r);
				if (new_cost > old_cost)
				{
					master_grid.setCost(x, y, new_cost);
				}
			}
		}
	}

	// Update cache (for reachability_controller and other consumers)
	cache_valid_ = true;
	cache_size_x_ = size_x;
	cache_size_y_ = size_y;
	cache_reach_field_ = reach_field;

	// ✨ 始终发布可视化数据（条件检查）
	if (publish_color_cloud_)
	{
		publishColorCloudFromField(master_grid, reach_field);
	}
}

}  // namespace costmap_2d

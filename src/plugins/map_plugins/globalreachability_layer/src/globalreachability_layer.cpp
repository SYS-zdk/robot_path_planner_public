/**
 * @file: globalreachability_layer.cpp
 * @brief: Implementation of GlobalReachabilityLayer
 * @author Zhang Dingkun
 * @date 2026-03-17
 * @version 1.0
 */

#include "globalreachability_layer/globalreachability_layer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <string>

#include <cstring>

#include <costmap_2d/cost_values.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointField.h>

#include <dynamicvoronoi.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::GlobalReachabilityLayer, costmap_2d::Layer)

namespace costmap_2d
{

void GlobalReachabilityLayer::onInitialize()
{
	nh_ = ros::NodeHandle("~/" + name_);

	nh_.param("enabled", enabled_, true);
	nh_.param("write_to_master", write_to_master_, false);
	nh_.param("publish_color_cloud", publish_color_cloud_, true);
	nh_.param("publish_grid", publish_grid_, true);
	nh_.param("treat_unknown_as_obstacle", treat_unknown_as_obstacle_, true);

	nh_.param("risk_clearance_scale_m", risk_clearance_scale_m_, risk_clearance_scale_m_);
	nh_.param("risk_width_scale_m", risk_width_scale_m_, risk_width_scale_m_);

	// Visualization-only mapping V(R): used only for heatmap coloring.
	nh_.param<std::string>("viz_mode", viz_mode_, viz_mode_);
	nh_.param<std::string>("viz_colormap", viz_colormap_, viz_colormap_);
	nh_.param("viz_clamp_min", viz_clamp_min_, viz_clamp_min_);
	nh_.param("viz_clamp_max", viz_clamp_max_, viz_clamp_max_);
	nh_.param("viz_gamma", viz_gamma_, viz_gamma_);
	nh_.param("viz_sigmoid_k", viz_sigmoid_k_, viz_sigmoid_k_);
	nh_.param("viz_sigmoid_center", viz_sigmoid_center_, viz_sigmoid_center_);
	nh_.param("viz_esdf_scale_m", viz_esdf_scale_m_, viz_esdf_scale_m_);

	// PointCloud opacity
	nh_.param("cloud_alpha_min", cloud_alpha_min_, cloud_alpha_min_);
	nh_.param("cloud_alpha_max", cloud_alpha_max_, cloud_alpha_max_);

	nh_.param("color_cloud_stride", color_cloud_stride_, 1);
	nh_.param("color_cloud_z", color_cloud_z_, 0.02);
	nh_.param("smooth_radius", smooth_radius_, 1);

	nh_.param("enable_skeleton", enable_skeleton_, true);
	nh_.param("ridge_window_radius_cells", ridge_window_radius_cells_, 1);
	nh_.param("ridge_min_clearance_m", ridge_min_clearance_m_, 0.20);
	nh_.param("ridge_attract_scale_m", ridge_attract_scale_m_, 0.50);
	nh_.param("alpha_ridge", alpha_ridge_, 1.0);
	nh_.param("skeleton_seed_dilate_radius_cells", skeleton_seed_dilate_radius_cells_, 1);

	nh_.param("update_interval_s", update_interval_s_, 2.0);

	nh_.param("robot_radius", robot_radius_, 0.25);
	nh_.param("safety_distance_min", safety_distance_min_, 0.05);
	nh_.param("safety_distance_good", safety_distance_good_, 0.25);
	nh_.param("width_min", width_min_, -1.0);
	nh_.param("width_good", width_good_, -1.0);

	nh_.param("alpha_clearance", alpha_clearance_, 1.0);
	nh_.param("alpha_width", alpha_width_, 1.0);

	nh_.param("occupied_cost_threshold", occupied_cost_threshold_, static_cast<int>(costmap_2d::LETHAL_OBSTACLE));
	nh_.param("max_cost", max_cost_, 252);

	if (publish_color_cloud_)
	{
		color_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("reachability_cloud", 1, true);
		ROS_INFO_STREAM("[globalreachability_layer] advertise: " << nh_.resolveName("reachability_cloud"));
	}
	if (publish_grid_)
	{
		grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("reachability_grid", 1, true);
		ROS_INFO_STREAM("[globalreachability_layer] advertise: " << nh_.resolveName("reachability_grid"));
	}

	ROS_INFO_STREAM("[globalreachability_layer] enable_skeleton=" << enable_skeleton_
					<< " ridge_min_clearance_m=" << ridge_min_clearance_m_
					<< " skeleton_seed_dilate_radius_cells=" << skeleton_seed_dilate_radius_cells_
					<< " ridge_attract_scale_m=" << ridge_attract_scale_m_
					<< " alpha_ridge=" << alpha_ridge_);

	dsrv_ = std::make_unique<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>(nh_);
	dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb =
			boost::bind(&GlobalReachabilityLayer::reconfigureCB, this, _1, _2);
	dsrv_->setCallback(cb);

	edf_manager_ = std::make_unique<euclidean_distance_field::EuclideanDistanceField>();
	current_ = true;
}

void GlobalReachabilityLayer::reconfigureCB(const costmap_2d::GenericPluginConfig& config, uint32_t level)
{
	enabled_ = config.enabled;
}

bool GlobalReachabilityLayer::getEnvScoreCache(std::vector<double>& env_score, unsigned int& size_x,
								 unsigned int& size_y) const
{
	boost::unique_lock<boost::mutex> lock(mutex_);
	if (!cache_valid_)
	{
		return false;
	}
	env_score = cache_env_score_;
	size_x = cache_size_x_;
	size_y = cache_size_y_;
	return env_score.size() == static_cast<size_t>(size_x) * static_cast<size_t>(size_y);
}

void GlobalReachabilityLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
										   double* max_x, double* max_y)
{
	ROS_INFO_ONCE("[globalreachability_layer] updateBounds() entered");
	if (!enabled_)
	{
		ROS_WARN_ONCE("[globalreachability_layer] disabled: enabled=false");
		return;
	}

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

bool GlobalReachabilityLayer::isOccupiedCost(unsigned char cost) const
{
	if (cost == costmap_2d::NO_INFORMATION)
	{
		return treat_unknown_as_obstacle_;
	}
	return cost >= static_cast<unsigned char>(occupied_cost_threshold_);
}

double GlobalReachabilityLayer::clip01(double v) const
{
	if (v < 0.0)
		return 0.0;
	if (v > 1.0)
		return 1.0;
	return v;
}

double GlobalReachabilityLayer::visualizeValue(double reachability_raw) const
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

double GlobalReachabilityLayer::computeClearanceMeters(unsigned int mx, unsigned int my) const
{
	const double d2 = edf_sq_cells_[my][mx];
	if (!std::isfinite(d2))
	{
		return std::numeric_limits<double>::infinity();
	}
	const double res = layered_costmap_->getCostmap()->getResolution();
	return std::sqrt(std::max(0.0, d2)) * res;
}

double GlobalReachabilityLayer::computeReachabilitySmooth(unsigned int mx, unsigned int my, const costmap_2d::Costmap2D& grid) const
{
	const unsigned char c = grid.getCost(mx, my);
	if (isOccupiedCost(c))
	{
		return 0.0;
	}

	// Smooth local traversability: use clearance-derived width to avoid raycast aliasing/speckles.
	const double clearance_m = computeClearanceMeters(mx, my);
	const double c0 = robot_radius_ + safety_distance_min_;
	const double w0 = (width_min_ > 0.0) ? width_min_ : 2.0 * c0;

	// width approx: diameter of maximal inscribed circle at this cell
	const double width_m = 2.0 * clearance_m;

	if (!(clearance_m > c0) || !(width_m > w0))
	{
		return 0.0;
	}

	auto continuous01 = [this](double delta, double scale) -> double {
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

	return clip01(r);
}

unsigned char GlobalReachabilityLayer::reachabilityToCost(double reachability) const
{
	reachability = clip01(reachability);
	const int c = static_cast<int>(std::round((1.0 - reachability) * static_cast<double>(max_cost_)));
	return static_cast<unsigned char>(std::max(0, std::min(max_cost_, c)));
}

void GlobalReachabilityLayer::jetColor(double t, uint8_t& r, uint8_t& g, uint8_t& b) const
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

void GlobalReachabilityLayer::gradientColor(double t, uint8_t& r, uint8_t& g, uint8_t& b) const
{
	// Simple perceptual-ish gradient: navy -> blue -> cyan -> yellow -> red
	// (avoids jet's harsh banding, looks closer to common costmap heatmaps)
	t = clip01(t);
	struct RGB
	{
		double r;
		double g;
		double b;
	};
	static const RGB anchors[] = {
		{ 0.05, 0.05, 0.25 },  // dark navy
		{ 0.10, 0.35, 0.95 },  // blue
		{ 0.00, 0.85, 0.85 },  // cyan
		{ 0.95, 0.95, 0.15 },  // yellow
		{ 0.95, 0.15, 0.10 },  // red
	};
	static const int n = static_cast<int>(sizeof(anchors) / sizeof(anchors[0]));
	const double x = t * (n - 1);
	const int i0 = std::min(std::max(static_cast<int>(std::floor(x)), 0), n - 2);
	const int i1 = i0 + 1;
	const double a = x - static_cast<double>(i0);
	const double rr = (1.0 - a) * anchors[i0].r + a * anchors[i1].r;
	const double gg = (1.0 - a) * anchors[i0].g + a * anchors[i1].g;
	const double bb = (1.0 - a) * anchors[i0].b + a * anchors[i1].b;
	r = static_cast<uint8_t>(std::round(clip01(rr) * 255.0));
	g = static_cast<uint8_t>(std::round(clip01(gg) * 255.0));
	b = static_cast<uint8_t>(std::round(clip01(bb) * 255.0));
}

float GlobalReachabilityLayer::packRGBFloat(uint8_t r, uint8_t g, uint8_t b) const
{
	const uint32_t rgb = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
	float out;
	static_assert(sizeof(float) == sizeof(uint32_t), "float/uint32 size mismatch");
	std::memcpy(&out, &rgb, sizeof(float));
	return out;
}

float GlobalReachabilityLayer::packRGBAFloat(uint8_t r, uint8_t g, uint8_t b, uint8_t a) const
{
	// PCL convention: rgba packed into a float via uint32.
	const uint32_t rgba = (static_cast<uint32_t>(a) << 24) | (static_cast<uint32_t>(r) << 16) |
					  (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
	float out;
	static_assert(sizeof(float) == sizeof(uint32_t), "float/uint32 size mismatch");
	std::memcpy(&out, &rgba, sizeof(float));
	return out;
}

void GlobalReachabilityLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
	ROS_INFO_ONCE("[globalreachability_layer] updateCosts() entered");
	if (!enabled_)
	{
		ROS_WARN_ONCE("[globalreachability_layer] updateCosts skipped: enabled=false");
		return;
	}

	const ros::Time now = ros::Time::now();
	boost::unique_lock<boost::mutex> lock(mutex_);

	const unsigned int size_x = master_grid.getSizeInCellsX();
	const unsigned int size_y = master_grid.getSizeInCellsY();
	const double res = master_grid.getResolution();
	const double ridge_scale = std::max(1e-6, ridge_attract_scale_m_);

	const bool size_changed = (!cache_valid_) || (cache_size_x_ != size_x) || (cache_size_y_ != size_y);
	bool need_recompute = size_changed;
	if (!need_recompute)
	{
		if (update_interval_s_ <= 1e-6)
		{
			need_recompute = true;
		}
		else if (!last_update_.isZero())
		{
			need_recompute = ((now - last_update_).toSec() >= update_interval_s_);
		}
		else
		{
			need_recompute = true;
		}
	}
	if (need_recompute)
		last_update_ = now;

	std::vector<uint8_t> is_free(size_x * size_y, 0);
	std::vector<double> clearance_m;
	std::vector<double> dist_to_ridge_m;
	std::vector<double> ridge_clearance_m;
	std::vector<uint8_t> is_ridge;
	std::vector<double> env_score;

	if (need_recompute)
	{
		// 1) EDF on current master grid
		occupancy_bin_.assign(size_x * size_y, costmap_2d::FREE_SPACE);
		for (unsigned int y = 0; y < size_y; ++y)
		{
			for (unsigned int x = 0; x < size_x; ++x)
			{
				const unsigned char c = master_grid.getCost(x, y);
				const bool occ = isOccupiedCost(c);
				occupancy_bin_[x + y * size_x] = occ ? costmap_2d::INSCRIBED_INFLATED_OBSTACLE : costmap_2d::FREE_SPACE;
			}
		}

		edf_manager_->setGridMap(occupancy_bin_.data(), static_cast<int>(size_x), static_cast<int>(size_y));
		edf_manager_->compute2d();
		edf_sq_cells_ = edf_manager_->getEDF();

		// 2) Build ESDF clearance array (meters) + occupancy mask
		clearance_m.assign(size_x * size_y, 0.0);
		for (unsigned int y = 0; y < size_y; ++y)
		{
			for (unsigned int x = 0; x < size_x; ++x)
			{
				const unsigned int idx = x + y * size_x;
				const unsigned char c = master_grid.getCost(x, y);
				if (isOccupiedCost(c))
				{
					clearance_m[idx] = 0.0;
					is_free[idx] = 0;
					continue;
				}
				is_free[idx] = 1;
				clearance_m[idx] = computeClearanceMeters(x, y);
			}
		}

		// 3) Voronoi skeleton + distance-to-skeleton
		dist_to_ridge_m.assign(size_x * size_y, std::numeric_limits<double>::infinity());
		ridge_clearance_m.assign(size_x * size_y, 0.0);
		is_ridge.assign(size_x * size_y, 0);

		if (enable_skeleton_)
		{
		// Use the same DynamicVoronoi implementation as voronoi_layer to avoid "scattered ridge pixels".
		DynamicVoronoi vor;
		std::vector<std::unique_ptr<bool[]>> grid_rows(size_y);
		std::vector<bool*> grid_ptrs(size_y);
		for (unsigned int y = 0; y < size_y; ++y)
		{
			grid_rows[y] = std::make_unique<bool[]>(size_x);
			grid_ptrs[y] = grid_rows[y].get();
			for (unsigned int x = 0; x < size_x; ++x)
			{
				const unsigned char c = master_grid.getCost(x, y);
				grid_ptrs[y][x] = isOccupiedCost(c);
			}
		}
		vor.initializeMap(static_cast<int>(size_x), static_cast<int>(size_y), grid_ptrs.data());
		vor.update(true);
		vor.prune();
		// Alternative pruned diagram can be sparser/cleaner; keep as internal improvement.
		vor.updateAlternativePrunedDiagram();

		const double min_c = std::max(0.0, ridge_min_clearance_m_);
		std::vector<uint8_t> is_skel(size_x * size_y, 0);
		for (unsigned int y = 0; y < size_y; ++y)
		{
			for (unsigned int x = 0; x < size_x; ++x)
			{
				const unsigned int idx = x + y * size_x;
				if (!is_free[idx])
					continue;
				if (!(clearance_m[idx] >= min_c))
					continue;
				if (vor.isVoronoiAlternative(static_cast<int>(x), static_cast<int>(y)) || vor.isVoronoi(static_cast<int>(x), static_cast<int>(y)))
				{
					is_skel[idx] = 1;
				}
			}
		}
		// Densify skeleton seeds by dilation (helps reduce "too sparse" appearance).
		const int dil = std::max(0, skeleton_seed_dilate_radius_cells_);
		if (dil > 0)
		{
			std::vector<uint8_t> dilated = is_skel;
			for (unsigned int y = 0; y < size_y; ++y)
			{
				for (unsigned int x = 0; x < size_x; ++x)
				{
					const unsigned int idx = x + y * size_x;
					if (!is_skel[idx])
						continue;
					for (int dy = -dil; dy <= dil; ++dy)
					{
						for (int dx = -dil; dx <= dil; ++dx)
						{
							const int nx = static_cast<int>(x) + dx;
							const int ny = static_cast<int>(y) + dy;
							if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x) || ny >= static_cast<int>(size_y))
								continue;
							const unsigned int nidx = static_cast<unsigned int>(nx) + static_cast<unsigned int>(ny) * size_x;
							if (is_free[nidx])
								dilated[nidx] = 1;
						}
					}
				}
			}
			is_skel.swap(dilated);
		}
		// Remove isolated single pixels (bad-looking dots)
		for (unsigned int y = 0; y < size_y; ++y)
		{
			for (unsigned int x = 0; x < size_x; ++x)
			{
				const unsigned int idx = x + y * size_x;
				if (!is_skel[idx])
					continue;
				int neigh = 0;
				for (int dy = -1; dy <= 1; ++dy)
				{
					for (int dx = -1; dx <= 1; ++dx)
					{
						if (dx == 0 && dy == 0)
							continue;
						const int nx = static_cast<int>(x) + dx;
						const int ny = static_cast<int>(y) + dy;
						if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x) || ny >= static_cast<int>(size_y))
							continue;
						const unsigned int nidx = static_cast<unsigned int>(nx) + static_cast<unsigned int>(ny) * size_x;
						neigh += static_cast<int>(is_skel[nidx] != 0);
					}
				}
				if (neigh == 0)
					is_skel[idx] = 0;
			}
		}

		std::vector<unsigned int> seeds;
		seeds.reserve(size_x * size_y / 64);
		for (unsigned int idx = 0; idx < is_skel.size(); ++idx)
		{
			if (is_skel[idx])
			{
				is_ridge[idx] = 1;
				seeds.push_back(idx);
			}
		}

		ROS_INFO_THROTTLE(2.0,
						"[globalreachability_layer] skeleton seeds=%zu (min_clearance=%.3f m, dilate=%d). If this is small, skeleton will look sparse.",
						seeds.size(), min_c, std::max(0, skeleton_seed_dilate_radius_cells_));

		if (seeds.empty())
		{
			ROS_WARN_THROTTLE(2.0,
						"[globalreachability_layer] enable_skeleton=true but Voronoi skeleton is empty; fallback to clearance-only corridor proxy.");
			for (unsigned int y = 0; y < size_y; ++y)
			{
				for (unsigned int x = 0; x < size_x; ++x)
				{
					const unsigned int idx = x + y * size_x;
					if (!is_free[idx])
						continue;
					dist_to_ridge_m[idx] = 0.0;
					ridge_clearance_m[idx] = clearance_m[idx];
				}
			}
		}
		else
		{
			// Dijkstra over 8-connected grid to approximate Euclidean distance to skeleton.
			struct Node
			{
				double dist;
				unsigned int idx;
			};
			struct Greater
			{
				bool operator()(const Node& a, const Node& b) const { return a.dist > b.dist; }
			};
			std::priority_queue<Node, std::vector<Node>, Greater> pq;

			for (const auto sidx : seeds)
			{
				dist_to_ridge_m[sidx] = 0.0;
				ridge_clearance_m[sidx] = clearance_m[sidx];
				pq.push(Node{ 0.0, sidx });
			}

			const double diag = res * std::sqrt(2.0);
			while (!pq.empty())
			{
				const Node cur = pq.top();
				pq.pop();
				if (cur.dist > dist_to_ridge_m[cur.idx] + 1e-12)
					continue;
				const unsigned int cx = cur.idx % size_x;
				const unsigned int cy = cur.idx / size_x;

				auto relax = [&](int nx, int ny, double w) {
					if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x) || ny >= static_cast<int>(size_y))
						return;
					const unsigned int nidx = static_cast<unsigned int>(nx) + static_cast<unsigned int>(ny) * size_x;
					if (!is_free[nidx])
						return;
					const double cand = cur.dist + w;
					if (cand + 1e-12 < dist_to_ridge_m[nidx])
					{
						dist_to_ridge_m[nidx] = cand;
						ridge_clearance_m[nidx] = ridge_clearance_m[cur.idx];
						pq.push(Node{ cand, nidx });
					}
				};

				relax(static_cast<int>(cx) - 1, static_cast<int>(cy), res);
				relax(static_cast<int>(cx) + 1, static_cast<int>(cy), res);
				relax(static_cast<int>(cx), static_cast<int>(cy) - 1, res);
				relax(static_cast<int>(cx), static_cast<int>(cy) + 1, res);
				relax(static_cast<int>(cx) - 1, static_cast<int>(cy) - 1, diag);
				relax(static_cast<int>(cx) + 1, static_cast<int>(cy) - 1, diag);
				relax(static_cast<int>(cx) - 1, static_cast<int>(cy) + 1, diag);
				relax(static_cast<int>(cx) + 1, static_cast<int>(cy) + 1, diag);
			}
		}
	}
	else
	{
		// No skeleton: treat each cell's own clearance as corridor clearance.
		for (unsigned int y = 0; y < size_y; ++y)
		{
			for (unsigned int x = 0; x < size_x; ++x)
			{
				const unsigned int idx = x + y * size_x;
				if (!is_free[idx])
					continue;
				dist_to_ridge_m[idx] = 0.0;
				ridge_clearance_m[idx] = clearance_m[idx];
			}
		}
	}

	// 4) Compute env_score using ESDF + corridor thickness + (optional) ridge attraction
	env_score.assign(size_x * size_y, 0.0);
	const double c_min = robot_radius_ + safety_distance_min_;
	const double a_c = std::max(0.0, alpha_clearance_);
	const double a_w = std::max(0.0, alpha_width_);
	const double a_r = std::max(0.0, alpha_ridge_);
	const double ridge_scale = std::max(1e-6, ridge_attract_scale_m_);

	auto continuous01 = [this](double delta, double scale) -> double {
		if (!std::isfinite(delta))
			return 1.0;
		delta = std::max(0.0, delta);
		scale = std::max(1e-6, std::fabs(scale));
		const double denom = delta + scale;
		if (!std::isfinite(denom) || denom <= 0.0)
			return 0.0;
		return clip01(delta / denom);
	};

	for (unsigned int y = 0; y < size_y; ++y)
	{
		for (unsigned int x = 0; x < size_x; ++x)
		{
			const unsigned int idx = x + y * size_x;
			if (!is_free[idx])
			{
				env_score[idx] = 0.0;
				continue;
			}

			const double c = clearance_m[idx];
			// corridor radius (half-width proxy) taken from nearest ridge's clearance
			const double corridor_r = ridge_clearance_m[idx];
			if (!(c > c_min) || !(corridor_r > c_min))
			{
				env_score[idx] = 0.0;
				continue;
			}

			const double s_c = continuous01(c - c_min, risk_clearance_scale_m_);
			const double s_w = continuous01(corridor_r - c_min, risk_width_scale_m_);
			const double s_r = enable_skeleton_ ? std::exp(-dist_to_ridge_m[idx] / ridge_scale) : 1.0;

			double r = 1.0;
			if (a_c > 0.0)
				r *= std::pow(s_c, a_c);
			if (a_w > 0.0)
				r *= std::pow(s_w, a_w);
			if (a_r > 0.0)
				r *= std::pow(clip01(s_r), a_r);

			env_score[idx] = clip01(r);
		}
	}

	// 5) Optional smoothing to reduce checkerboard/speckle artifacts in visualization.
	const int sr = std::max(0, smooth_radius_);
	if (sr > 0)
	{
		std::vector<double> tmp = env_score;
		for (unsigned int y = 0; y < size_y; ++y)
		{
			for (unsigned int x = 0; x < size_x; ++x)
			{
				double sum = 0.0;
				unsigned int cnt = 0;
				const int xmin = std::max(0, static_cast<int>(x) - sr);
				const int xmax = std::min(static_cast<int>(size_x) - 1, static_cast<int>(x) + sr);
				const int ymin = std::max(0, static_cast<int>(y) - sr);
				const int ymax = std::min(static_cast<int>(size_y) - 1, static_cast<int>(y) + sr);
				for (int yy = ymin; yy <= ymax; ++yy)
				{
					for (int xx = xmin; xx <= xmax; ++xx)
					{
						sum += tmp[static_cast<unsigned int>(xx) + static_cast<unsigned int>(yy) * size_x];
						++cnt;
					}
				}
				env_score[x + y * size_x] = (cnt > 0) ? (sum / static_cast<double>(cnt)) : env_score[x + y * size_x];
			}
		}
	}

		// Update cache
		cache_valid_ = true;
		cache_size_x_ = size_x;
		cache_size_y_ = size_y;
		cache_env_score_ = env_score;
		cache_clearance_m_ = clearance_m;
		cache_dist_to_ridge_m_ = dist_to_ridge_m;
	}
	else
	{
		// Use cached results (still write to master to avoid periodic on/off effects)
		env_score = cache_env_score_;
		clearance_m = cache_clearance_m_;
		dist_to_ridge_m = cache_dist_to_ridge_m_;
	}

	// 6) Write to master (optional) using env_score
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
				const unsigned int idx = x + y * size_x;

				const unsigned char old_cost = master_grid.getCost(x, y);
				if (isOccupiedCost(old_cost))
				{
					continue;
				}

				const unsigned char new_cost = reachabilityToCost(env_score[idx]);
				if (new_cost > old_cost)
				{
					master_grid.setCost(x, y, new_cost);
				}
			}
		}
	}

	// 7) Publish PointCloud2 heatmap (optional)
	if (publish_color_cloud_ && color_cloud_pub_)
	{
		const int stride = std::max(1, color_cloud_stride_);
		sensor_msgs::PointCloud2 cloud;
		cloud.header.stamp = ros::Time::now();
		cloud.header.frame_id = layered_costmap_->getGlobalFrameID();
		cloud.height = 1;
		cloud.is_bigendian = false;
		cloud.is_dense = true;

		cloud.fields.resize(6);
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
		cloud.fields[4].name = "rgba";
		cloud.fields[4].offset = 16;
		cloud.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
		cloud.fields[4].count = 1;
		cloud.fields[5].name = "intensity";
		cloud.fields[5].offset = 20;
		cloud.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
		cloud.fields[5].count = 1;
		cloud.point_step = 24;

		const unsigned int approx_nx = (size_x + static_cast<unsigned int>(stride) - 1U) / static_cast<unsigned int>(stride);
		const unsigned int approx_ny = (size_y + static_cast<unsigned int>(stride) - 1U) / static_cast<unsigned int>(stride);
		cloud.data.reserve(static_cast<size_t>(approx_nx) * approx_ny * cloud.point_step);

		for (unsigned int y = 0; y < size_y; y += static_cast<unsigned int>(stride))
		{
			for (unsigned int x = 0; x < size_x; x += static_cast<unsigned int>(stride))
			{
				const unsigned int idx = x + y * size_x;
				// Make visualization closer to ESDF/Voronoi:
				// - ESDF distance gradient: v_esdf = d/(d+scale)
				// - Voronoi-like centerline emphasis: v_ridge = exp(-dist_to_ridge/scale)
				// - Combine multiplicatively: bright on centerline, fades to walls
				const double d_obs = clearance_m[idx];
				const double esdf_scale = std::max(1e-6, viz_esdf_scale_m_);
				double v_esdf = 0.0;
				if (std::isfinite(d_obs))
				{
					v_esdf = clip01(d_obs / (d_obs + esdf_scale));
				}
				else
				{
					v_esdf = 1.0;
				}
				double v_ridge = 1.0;
				if (enable_skeleton_)
				{
					const double dr = dist_to_ridge_m[idx];
					if (std::isfinite(dr))	
					{
						v_ridge = std::exp(-dr / std::max(1e-6, ridge_scale));
						// use alpha_ridge_ to sharpen the centerline in visualization
						v_ridge = std::pow(clip01(v_ridge), std::max(1.0, alpha_ridge_));
					}
					else
					{
						v_ridge = 0.0;
					}
				}
				// Unknown/occupied are already excluded upstream; this is purely visual.
				const double r_raw = clip01(v_esdf * v_ridge);
				const double r_viz = visualizeValue(r_raw);

				double wx, wy;
				master_grid.mapToWorld(x, y, wx, wy);
				const float fx = static_cast<float>(wx);
				const float fy = static_cast<float>(wy);
				const float fz = static_cast<float>(color_cloud_z_);

				uint8_t rr, gg, bb;
				if (viz_colormap_ == "jet")
				{
					jetColor(r_viz, rr, gg, bb);
				}
				else
				{
					gradientColor(r_viz, rr, gg, bb);
				}
				const double a01 = clip01(cloud_alpha_min_ + (cloud_alpha_max_ - cloud_alpha_min_) * r_viz);
				const uint8_t aa = static_cast<uint8_t>(std::round(a01 * 255.0));
				const float frgb = packRGBFloat(rr, gg, bb);
				const float frgba = packRGBAFloat(rr, gg, bb, aa);
				const float fintensity = static_cast<float>(r_viz);

				const size_t base = cloud.data.size();
				cloud.data.resize(base + cloud.point_step);
				std::memcpy(&cloud.data[base + 0], &fx, sizeof(float));
				std::memcpy(&cloud.data[base + 4], &fy, sizeof(float));
				std::memcpy(&cloud.data[base + 8], &fz, sizeof(float));
				std::memcpy(&cloud.data[base + 12], &frgb, sizeof(float));
				std::memcpy(&cloud.data[base + 16], &frgba, sizeof(float));
				std::memcpy(&cloud.data[base + 20], &fintensity, sizeof(float));
			}
		}

		cloud.width = static_cast<uint32_t>(cloud.data.size() / cloud.point_step);
		cloud.row_step = cloud.width * cloud.point_step;
		color_cloud_pub_.publish(cloud);
	}

	// 8) Publish OccupancyGrid for RViz Map visualization (ESDF/Voronoi-style)
	if (publish_grid_ && grid_pub_)
	{
		nav_msgs::OccupancyGrid grid;
		grid.header.stamp = ros::Time::now();
		grid.header.frame_id = layered_costmap_->getGlobalFrameID();
		grid.info.resolution = static_cast<float>(res);
		grid.info.width = size_x;
		grid.info.height = size_y;
		grid.info.origin.position.x = master_grid.getOriginX();
		grid.info.origin.position.y = master_grid.getOriginY();
		grid.info.origin.position.z = 0.0;
		grid.info.origin.orientation.w = 1.0;
		grid.data.assign(size_x * size_y, 0);

		for (unsigned int y = 0; y < size_y; ++y)
		{
			for (unsigned int x = 0; x < size_x; ++x)
			{
				const unsigned int idx = x + y * size_x;
				const unsigned char c = master_grid.getCost(x, y);
				if (c == costmap_2d::NO_INFORMATION)
				{
					grid.data[idx] = treat_unknown_as_obstacle_ ? 0 : -1;
					continue;
				}
				if (isOccupiedCost(c))
				{
					grid.data[idx] = 0;
					continue;
				}

				// Same visual scalar as cloud: ESDF gradient * ridge emphasis, then run viz mapping.
				const double d_obs = clearance_m[idx];
				const double esdf_scale = std::max(1e-6, viz_esdf_scale_m_);
				double v_esdf = 0.0;
				if (std::isfinite(d_obs))
				{
					v_esdf = clip01(d_obs / (d_obs + esdf_scale));
				}
				else
				{
					v_esdf = 1.0;
				}
				double v_ridge = 1.0;
				if (enable_skeleton_)
				{
					const double dr = dist_to_ridge_m[idx];
					if (std::isfinite(dr))
					{
						v_ridge = std::exp(-dr / std::max(1e-6, ridge_scale));
						v_ridge = std::pow(clip01(v_ridge), std::max(1.0, alpha_ridge_));
					}
					else
					{
						v_ridge = 0.0;
					}
				}
				const double v_raw = clip01(v_esdf * v_ridge);
				const double v = visualizeValue(v_raw);
				grid.data[idx] = static_cast<int8_t>(std::max(0, std::min(100, static_cast<int>(std::round(v * 100.0)))));
			}
		}

		grid_pub_.publish(grid);
	}
}

}  // namespace costmap_2d

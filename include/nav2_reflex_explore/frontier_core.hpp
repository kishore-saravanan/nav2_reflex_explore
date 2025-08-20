#pragma once
#include <vector>
#include <utility>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace nav2_reflex_explore {

struct Frontier {
  double x{0.0}, y{0.0};
  int size{0};
};

struct FrontierParams {
  int    min_frontier_cells{12};
  double sensor_range{3.0};
};

class FrontierCore {
public:
  explicit FrontierCore(const FrontierParams &p) : p_(p) {}
  std::vector<Frontier> extract(const nav_msgs::msg::OccupancyGrid &map);

private:
  FrontierParams p_;
};

}

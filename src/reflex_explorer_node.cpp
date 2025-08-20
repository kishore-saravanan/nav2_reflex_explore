#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/qos.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <optional>
#include <vector>
#include <deque>
#include <queue>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <memory>
#include <limits>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <string>

#include "nav2_reflex_explore/frontier_core.hpp"

// ---- Candidate goal near a frontier
struct Candidate {
  double x{0.0}, y{0.0};
  int info_gain{0};          // unknown cells visible nearby
  double score{0.0};         // here: -distance (nearest-first)
  int attempts{0};
  rclcpp::Time last_attempt; // same clock as node
};

static inline double sqr(double v){ return v*v; }

class FrontierExplorerNode : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  FrontierExplorerNode()
  : Node("reflex_explorer"),
    tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {

    // ---- core params (distance-first)
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_base_frame", "base_link");

    declare_parameter<double>("sensor_range", 3.5);
    declare_parameter<int>("min_frontier_cells", 12);
    declare_parameter<int>("min_info_gain_cells", 10);

    declare_parameter<double>("edge_ignore_margin_m", 0.25);
    declare_parameter<double>("dedupe_radius_m", 0.5);
    declare_parameter<double>("retry_cooldown_sec", 5.0);
    declare_parameter<int>("max_attempts_per_goal", 3);

    declare_parameter<double>("min_unknown_ratio", 0.02);   // finish when interior unknown <= this
    declare_parameter<double>("refresh_period_sec", 2.0);

    declare_parameter<double>("min_goal_runtime_sec", 6.0);
    declare_parameter<double>("progress_timeout_sec", 10.0);
    declare_parameter<double>("progress_dist_thresh_m", 0.05);
    declare_parameter<double>("cancel_cooldown_sec", 1.5);

    declare_parameter<double>("near_goal_epsilon_m", 0.35);
    declare_parameter<double>("visited_suppress_radius_m", 0.8);
    declare_parameter<double>("visited_suppress_sec", 30.0);

    // Empty default → we compute a timestamped default in the constructor
    declare_parameter<std::string>("map_save_path", "");
    declare_parameter<bool>("save_when_done", true);

    // ---- plateau-of-progress (with guards)
    declare_parameter<double>("plateau_sample_sec", 4.0);
    declare_parameter<double>("plateau_window_sec", 60.0);
    declare_parameter<double>("plateau_min_improve", 0.01);
    declare_parameter<double>("plateau_warmup_sec", 20.0);
    declare_parameter<double>("plateau_transit_dist_m", 2.0);
    declare_parameter<bool>("plateau_autoscale_window", true);
    declare_parameter<double>("plateau_min_window_sec", 45.0);
    declare_parameter<double>("plateau_max_window_sec", 240.0);
    declare_parameter<double>("plan_speed_guess_mps", 0.22);
    // Only enable plateau when map is mostly explored
    declare_parameter<double>("plateau_enable_unknown_ratio", 0.25);

    // read params
    get_parameter("global_frame", global_frame_);
    get_parameter("robot_base_frame", robot_base_frame_);
    sensor_range_           = get_parameter("sensor_range").as_double();
    min_frontier_cells_     = get_parameter("min_frontier_cells").as_int();
    min_info_gain_cells_    = get_parameter("min_info_gain_cells").as_int();
    edge_ignore_margin_m_   = get_parameter("edge_ignore_margin_m").as_double();
    dedupe_radius_m_        = get_parameter("dedupe_radius_m").as_double();
    retry_cooldown_sec_     = get_parameter("retry_cooldown_sec").as_double();
    max_attempts_           = get_parameter("max_attempts_per_goal").as_int();
    min_unknown_ratio_      = get_parameter("min_unknown_ratio").as_double();
    refresh_period_sec_     = get_parameter("refresh_period_sec").as_double();
    min_goal_runtime_sec_   = get_parameter("min_goal_runtime_sec").as_double();
    progress_timeout_sec_   = get_parameter("progress_timeout_sec").as_double();
    progress_dist_thresh_m_ = get_parameter("progress_dist_thresh_m").as_double();
    cancel_cooldown_sec_    = get_parameter("cancel_cooldown_sec").as_double();
    near_goal_epsilon_m_    = get_parameter("near_goal_epsilon_m").as_double();
    visited_suppress_radius_m_ = get_parameter("visited_suppress_radius_m").as_double();
    visited_suppress_sec_   = get_parameter("visited_suppress_sec").as_double();
    map_save_path_          = get_parameter("map_save_path").as_string();
    save_when_done_         = get_parameter("save_when_done").as_bool();

    plateau_sample_sec_      = get_parameter("plateau_sample_sec").as_double();
    plateau_window_sec_      = get_parameter("plateau_window_sec").as_double();
    plateau_min_improve_     = get_parameter("plateau_min_improve").as_double();
    plateau_warmup_sec_      = get_parameter("plateau_warmup_sec").as_double();
    plateau_transit_dist_m_  = get_parameter("plateau_transit_dist_m").as_double();
    plateau_autoscale_window_= get_parameter("plateau_autoscale_window").as_bool();
    plateau_min_window_sec_  = get_parameter("plateau_min_window_sec").as_double();
    plateau_max_window_sec_  = get_parameter("plateau_max_window_sec").as_double();
    plan_speed_guess_mps_    = get_parameter("plan_speed_guess_mps").as_double();
    plateau_enable_unknown_ratio_ = get_parameter("plateau_enable_unknown_ratio").as_double();

    // compute default map save path if not provided
    if (map_save_path_.empty()) {
      map_save_path_ = defaultStampedSavePrefix();
      RCLCPP_INFO(get_logger(), "Map save path (default): %s.[yaml|pgm]", map_save_path_.c_str());
    } else {
      std::filesystem::path p(map_save_path_);
      if (!p.is_absolute()) {
        const char* home = std::getenv("HOME");
        std::filesystem::path base = (home ? std::filesystem::path(home) : std::filesystem::path("/tmp"));
        map_save_path_ = (base / p).string();
      }
      RCLCPP_INFO(get_logger(), "Map save path (override): %s.[yaml|pgm]", map_save_path_.c_str());
    }

    // frontier core
    nav2_reflex_explore::FrontierParams fp;
    fp.min_frontier_cells = min_frontier_cells_;
    fp.sensor_range       = sensor_range_;
    core_ = std::make_unique<nav2_reflex_explore::FrontierCore>(fp);

    // I/O
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1),
      std::bind(&FrontierExplorerNode::mapCb, this, std::placeholders::_1));

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
      std::bind(&FrontierExplorerNode::cmdCb, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server...");
    nav_client_->wait_for_action_server();
    RCLCPP_INFO(get_logger(), "Nav2 ready.");

    const auto t0 = this->now();
    last_refresh_       = t0;
    last_cmd_time_      = t0;
    last_cancel_time_   = t0;
    last_progress_time_ = t0;
    plateau_last_sample_= t0;

    timer_ = create_wall_timer(std::chrono::milliseconds(200),
                               std::bind(&FrontierExplorerNode::tick, this));
  }

private:
  // compute default "src/nav2_reflex_explore/maps/<YYYYmmdd-HHMMSS>/map" or fallback to share/maps
  std::string defaultStampedSavePrefix() {
    // package share dir: .../install/nav2_reflex_explore/share/nav2_reflex_explore
    const std::string share = ament_index_cpp::get_package_share_directory("nav2_reflex_explore");
    std::filesystem::path share_dir(share);

    // climb up to workspace root: share_dir -> share -> pkg install -> install -> ws_root
    std::filesystem::path ws_root = share_dir;
    for (int i=0; i<4; ++i) ws_root = ws_root.parent_path();

    // prefer the source tree maps dir if it exists
    std::filesystem::path src_maps = ws_root / "src" / "nav2_reflex_explore" / "maps";
    std::filesystem::path root = std::filesystem::exists(src_maps) ? src_maps
                                                                   : (share_dir / "maps");

    // timestamp folder
    char buf[32]{0};
    std::time_t tt = std::time(nullptr);
    std::tm tm{};
    #if defined(_WIN32)
      localtime_s(&tm, &tt);
    #else
      localtime_r(&tt, &tm);
    #endif
    std::strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", &tm);
    std::filesystem::path outdir = root / buf;

    // ensure dir exists
    std::error_code ec;
    std::filesystem::create_directories(outdir, ec);

    return (outdir / "map").string();
  }

  // ROS
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  // state
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::deque<Candidate> backlog_;
  Candidate active_;
  bool navigating_{false}, finished_{false};

  // watchdog
  rclcpp::Time goal_start_time_;
  rclcpp::Time last_progress_time_;
  double last_feedback_dist_{std::numeric_limits<double>::infinity()};
  double last_distance_remaining_{std::numeric_limits<double>::infinity()};
  std::pair<double,double> last_progress_pose_{0.0,0.0};
  rclcpp::Time last_cancel_time_;
  rclcpp::Time last_refresh_;
  rclcpp::Time last_cmd_time_;
  double last_cmd_speed_{0.0};

  // visited suppression
  struct Visited { double x, y; rclcpp::Time t; };
  std::deque<Visited> visited_;
  double visited_suppress_radius_m_{0.8};
  double visited_suppress_sec_{30.0};
  double near_goal_epsilon_m_{0.35};

  // plateau-of-progress tracking
  struct Sample { rclcpp::Time t; double unknown_ratio; };
  std::deque<Sample> plateau_samples_;
  rclcpp::Time plateau_last_sample_;
  double plateau_sample_sec_{4.0};
  double plateau_window_sec_{60.0};
  double plateau_min_improve_{0.01};
  double plateau_warmup_sec_{20.0};
  double plateau_transit_dist_m_{2.0};
  bool   plateau_autoscale_window_{true};
  double plateau_min_window_sec_{45.0};
  double plateau_max_window_sec_{240.0};
  double plan_speed_guess_mps_{0.22};
  double plateau_enable_unknown_ratio_{0.25};
  int    goals_succeeded_{0};

  // params
  std::string global_frame_, robot_base_frame_, map_save_path_;
  double sensor_range_{3.5};
  int    min_frontier_cells_{12};
  int    min_info_gain_cells_{10};
  double edge_ignore_margin_m_{0.25};
  double dedupe_radius_m_{0.5};
  double retry_cooldown_sec_{5.0};
  int    max_attempts_{3};
  double min_unknown_ratio_{0.02};
  double refresh_period_sec_{2.0};
  double min_goal_runtime_sec_{6.0};
  double progress_timeout_sec_{10.0};
  double progress_dist_thresh_m_{0.05};
  double cancel_cooldown_sec_{1.5};
  bool   save_when_done_{true};

  // frontier core
  std::unique_ptr<nav2_reflex_explore::FrontierCore> core_;

  // ---- Callbacks
  void mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){ map_ = msg; }
  void cmdCb(const geometry_msgs::msg::Twist::SharedPtr msg){
    last_cmd_time_ = this->now();
    last_cmd_speed_ = std::hypot(msg->linear.x, msg->linear.y);
  }

  // ---- Helpers
  inline bool validRC(int r, int c) const {
    return (r>=0 && (size_t)r<map_->info.height && c>=0 && (size_t)c<map_->info.width);
  }
  inline bool isFree(int r, int c) const { return map_->data[r*map_->info.width + c] == 0; }
  inline bool isUnknown(int r, int c) const { return map_->data[r*map_->info.width + c] == -1; }

  inline std::pair<int,int> toCell(double x, double y) const {
    const double res = map_->info.resolution;
    const double ox  = map_->info.origin.position.x;
    const double oy  = map_->info.origin.position.y;
    int c = (int)std::floor((x-ox)/res);
    int r = (int)std::floor((y-oy)/res);
    return {r,c};
  }
  bool isInInterior(int r, int c) const {
    const int H = map_->info.height, W = map_->info.width;
    const int margin = std::max(1, (int)std::round(edge_ignore_margin_m_ / map_->info.resolution));
    return r >= margin && r < H - margin && c >= margin && c < W - margin;
  }
  double interiorUnknownRatio() const {
    if (!map_ || map_->data.empty()) return 1.0;
    const int H = map_->info.height, W = map_->info.width;
    size_t unk=0, tot=0;
    for (int r=0; r<H; ++r){
      for (int c=0; c<W; ++c){
        if (!isInInterior(r,c)) continue;
        ++tot;
        if (map_->data[r*W+c] == -1) ++unk;
      }
    }
    return tot ? (double)unk / (double)tot : 1.0;
  }
  int infoGainAt(double x, double y, double radius_m) const {
    const int H=map_->info.height, W=map_->info.width;
    const double res = map_->info.resolution;
    auto [cr,cc]=toCell(x,y);
    int rad = std::max(1,(int)std::round(radius_m/res));
    int cnt=0;
    for (int dr=-rad; dr<=rad; ++dr){
      for (int dc=-rad; dc<=rad; ++dc){
        int rr=cr+dr, cc2=cc+dc;
        if (rr<0||rr>=H||cc2<0||cc2>=W) continue;
        if (dr*dr + dc*dc > rad*rad) continue;
        if (isInInterior(rr,cc2) && isUnknown(rr,cc2)) ++cnt;
      }
    }
    return cnt;
  }

  // treat UNKNOWN as traversable for feasibility (only occupied is blocked)
  double bfsPathLengthMeters(double sx, double sy, double gx, double gy){
    if (!map_) return -1.0;
    const int H=map_->info.height, W=map_->info.width;
    const double res=map_->info.resolution;
    const double ox=map_->info.origin.position.x, oy=map_->info.origin.position.y;

    auto toCellFn=[&](double x,double y){
      int c = (int)std::floor((x-ox)/res);
      int r = (int)std::floor((y-oy)/res);
      return std::pair<int,int>(r,c);
    };
    auto ok=[&](int r,int c){
      if (r<0 || r>=H || c<0 || c>=W) return false;
      int8_t v = map_->data[r*W+c];
      return (v == 0 || v == -1); // free OR unknown
    };

    auto [sr,sc] = toCellFn(sx,sy);
    auto [gr,gc] = toCellFn(gx,gy);
    if (!ok(sr,sc) || !ok(gr,gc)) return -1.0;

    std::vector<int> dist(W*H, -1);
    std::queue<std::pair<int,int>> q;
    dist[sr*W+sc]=0; q.emplace(sr,sc);
    const int d4[4][2]={{1,0},{-1,0},{0,1},{0,-1}};
    while(!q.empty()){
      auto [r,c]=q.front(); q.pop();
      if (r==gr && c==gc) break;
      for (auto &d: d4){
        int nr=r+d[0], nc=c+d[1];
        if (!ok(nr,nc)) continue;
        int id=nr*W+nc;
        if (dist[id]!=-1) continue;
        dist[id]=dist[r*W+c]+1;
        q.emplace(nr,nc);
      }
    }
    int steps = dist[gr*W+gc];
    if (steps <= 0) return -1.0;
    return steps * res;
  }

  void pushUnique(const Candidate& c){
    for (auto &e: backlog_){
      if (sqr(e.x-c.x)+sqr(e.y-c.y) < sqr(dedupe_radius_m_)) {
        if (c.score>e.score) e=c;
        return;
      }
    }
    backlog_.push_back(c);
  }

  void pruneVisited(){
    const auto now = this->now();
    while(!visited_.empty()){
      if ((now - visited_.front().t).seconds() > visited_suppress_sec_) visited_.pop_front();
      else break;
    }
  }
  bool suppressedByVisited(double x, double y) const {
    for (const auto& v : visited_){
      if (sqr(v.x - x) + sqr(v.y - y) <= sqr(visited_suppress_radius_m_)) return true;
    }
    return false;
  }

  void refreshBacklog(const std::pair<double,double>& robot){
    if (!map_) return;

    // fresh frontiers
    for (auto &f : core_->extract(*map_)){
      int ig = infoGainAt(f.x, f.y, sensor_range_);
      if (ig < min_info_gain_cells_) continue;
      if (suppressedByVisited(f.x, f.y)) continue;
      double d = std::hypot(f.x-robot.first, f.y-robot.second);
      Candidate c{f.x,f.y,ig, -d, 0, rclcpp::Time(0,0,this->get_clock()->get_clock_type())};
      pushUnique(c);
    }
    // keep + rescore
    std::deque<Candidate> kept;
    for (auto &e : backlog_){
      int ig = infoGainAt(e.x, e.y, sensor_range_);
      if (ig < min_info_gain_cells_) continue;
      if (suppressedByVisited(e.x, e.y)) continue;
      e.info_gain = ig;
      double d = std::hypot(e.x-robot.first, e.y-robot.second);
      e.score = -d;
      kept.push_back(e);
    }
    backlog_.swap(kept);
    std::sort(backlog_.begin(), backlog_.end(),
              [](auto &A, auto &B){ return A.score > B.score; }); // nearest first
  }

  bool eligible(Candidate& c, const std::pair<double,double>& r){
    if (c.attempts >= max_attempts_) return false;
    if (c.last_attempt.nanoseconds()>0 &&
        (this->now() - c.last_attempt).seconds() < retry_cooldown_sec_) return false;
    if (suppressedByVisited(c.x, c.y)) return false;

    int ig = infoGainAt(c.x, c.y, sensor_range_);
    if (ig < min_info_gain_cells_) return false;

    // Early-stage relaxation: if map is mostly unknown, skip BFS feasibility
    double ur = interiorUnknownRatio();
    if (ur > 0.50) {
      return std::hypot(c.x - r.first, c.y - r.second) > 0.10;
    }

    double L = bfsPathLengthMeters(r.first, r.second, c.x, c.y);
    return L > 0.10;
  }
  bool anyEligible(const std::pair<double,double>& robot){
    for (auto c : backlog_) if (eligible(c, robot)) return true;
    return false;
  }

  std::optional<Candidate> pickNext(const std::pair<double,double>& robot){
    // nearest eligible
    size_t best = backlog_.size(); double best_d=1e9;
    for (size_t i=0;i<backlog_.size();++i){
      if (!eligible(backlog_[i], robot)) continue;
      double d = std::hypot(backlog_[i].x-robot.first, backlog_[i].y-robot.second);
      if (d < best_d){ best_d=d; best=i; }
    }
    if (best < backlog_.size()){
      Candidate c = backlog_[best];
      backlog_.erase(backlog_.begin()+best);
      return c;
    }
    // relax info-gain
    int relaxed = std::max(1, min_info_gain_cells_ / 2);
    best = backlog_.size(); best_d=1e9;
    for (size_t i=0;i<backlog_.size();++i){
      auto cand = backlog_[i];
      if (cand.attempts >= max_attempts_) continue;
      if (cand.last_attempt.nanoseconds()>0 &&
          (this->now()-cand.last_attempt).seconds() < retry_cooldown_sec_) continue;
      if (suppressedByVisited(cand.x, cand.y)) continue;
      int ig = infoGainAt(cand.x,cand.y,sensor_range_);
      if (ig < relaxed) continue;
      double ur = interiorUnknownRatio();
      bool ok = (ur > 0.50) ? true
                            : (bfsPathLengthMeters(robot.first,robot.second,cand.x,cand.y) > 0.10);
      if (!ok) continue;
      double d = std::hypot(cand.x-robot.first, cand.y-robot.second);
      if (d < best_d){ best_d=d; best=i; }
    }
    if (best < backlog_.size()){
      Candidate c = backlog_[best];
      backlog_.erase(backlog_.begin()+best);
      return c;
    }
    return std::nullopt;
  }

  void requeueFailed(const Candidate& c){
    Candidate n = c;
    n.attempts = std::min(max_attempts_, c.attempts+1);
    n.last_attempt = this->now();
    n.score -= 0.1;
    pushUnique(n);
  }

  std::optional<std::pair<double,double>> robotXY(){
    try {
      auto tf = tf_buffer_.lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
      return std::make_pair(tf.transform.translation.x, tf.transform.translation.y);
    } catch (...) { return std::nullopt; }
  }

  // ---- Plateau (with guards so it won't end immediately)
  void updatePlateau(){
    const auto now = this->now();

    // Only evaluate plateau when a significant portion is already explored AND we've succeeded at least one goal
    if (interiorUnknownRatio() > plateau_enable_unknown_ratio_) return;
    if (goals_succeeded_ < 1) return;

    // Warm-up & long-transit guards
    bool in_warmup = navigating_ && (now - goal_start_time_).seconds() < plateau_warmup_sec_;
    bool long_transit = std::isfinite(last_distance_remaining_) &&
                        last_distance_remaining_ > plateau_transit_dist_m_;
    if (in_warmup || long_transit) return;

    // Auto-scale window by rough ETA if enabled
    double window_sec = plateau_window_sec_;
    if (plateau_autoscale_window_ && std::isfinite(last_distance_remaining_)) {
      double eta = last_distance_remaining_ / std::max(0.05, plan_speed_guess_mps_);
      window_sec = std::clamp(eta, plateau_min_window_sec_, plateau_max_window_sec_);
    }

    // sample cadence
    if ((now - plateau_last_sample_).seconds() < plateau_sample_sec_) return;

    double ur = interiorUnknownRatio();
    plateau_samples_.push_back(Sample{now, ur});
    plateau_last_sample_ = now;

    // trim window
    while (!plateau_samples_.empty() &&
           (now - plateau_samples_.front().t).seconds() > window_sec) {
      plateau_samples_.pop_front();
    }

    if (plateau_samples_.size() >= 2) {
      double oldest = plateau_samples_.front().unknown_ratio;
      double newest = plateau_samples_.back().unknown_ratio;
      double denom  = std::max(1e-6, oldest);
      double improve = (oldest - newest) / denom; // relative improvement

      if (improve < plateau_min_improve_) {
        RCLCPP_WARN(get_logger(),
          "Plateau: Δunknown=%.4f over ~%.0fs (thr=%.4f). Finalizing.",
          improve, window_sec, plateau_min_improve_);
        finished_ = true;
        if (save_when_done_) saveMap();
      }
    }
  }

  // ---- Tick
  void tick(){
    if (finished_ || !map_) return;

    pruneVisited();

    auto robot = robotXY(); if (!robot) return;

    if ((this->now() - last_refresh_).seconds() >= refresh_period_sec_){
      refreshBacklog(*robot);
      last_refresh_ = this->now();
      RCLCPP_INFO(get_logger(), "Backlog=%zu (unknown_ratio=%.3f)",
                  backlog_.size(), interiorUnknownRatio());
    }

    // Plateau stop (planner-agnostic)
    if (!finished_) updatePlateau();
    if (finished_) return;

    // Optional classic stop (interior unknown + no eligible)
    const double ur = interiorUnknownRatio();
    if (!navigating_ && ur <= min_unknown_ratio_ && !anyEligible(*robot)){
      RCLCPP_INFO(get_logger(), "Done (unknown=%.3f). Saving map...", ur);
      finished_ = true;
      if (save_when_done_) saveMap();
      return;
    }

    // Watchdog while navigating
    if (navigating_){
      if ((this->now() - last_cancel_time_).seconds() < cancel_cooldown_sec_) return;

      double since_start = (this->now() - goal_start_time_).seconds();
      double since_prog  = (this->now() - last_progress_time_).seconds();

      if (auto p = robotXY()){
        double d = std::hypot(p->first - last_progress_pose_.first,
                              p->second - last_progress_pose_.second);
        if (d > progress_dist_thresh_m_/2.0){
          last_progress_pose_ = *p;
          last_progress_time_ = this->now();
        }
      }

      if (since_start >= min_goal_runtime_sec_ && since_prog >= progress_timeout_sec_){
        RCLCPP_WARN(get_logger(), "No progress for %.1fs (after %.1fs). Cancel & requeue.",
                    since_prog, since_start);
        nav_client_->async_cancel_all_goals();
        requeueFailed(active_);
        navigating_ = false;
        last_cancel_time_ = this->now();
      }
      return;
    }

    // pick & send next (skip goals already under robot; mark them visited so we don't loop)
    int tries = 0;
    while (tries++ < 3){
      if (backlog_.empty()) { refreshBacklog(*robot); if (backlog_.empty()) break; }
      auto cand = pickNext(*robot);
      if (!cand) { refreshBacklog(*robot); cand = pickNext(*robot); }
      if (!cand) break;

      const double d = std::hypot(cand->x - robot->first, cand->y - robot->second);
      if (d < near_goal_epsilon_m_){
        RCLCPP_INFO(get_logger(), "Goal is already under robot (%.2fm). Skipping send, refreshing.", d);
        visited_.push_back(Visited{cand->x, cand->y, this->now()});
        refreshBacklog(*robot);
        continue;
      }

      sendGoal(*cand, *robot);
      break;
    }
  }

  void sendGoal(Candidate c, const std::pair<double,double>& r){
    double yaw = std::atan2(c.y - r.second, c.x - r.first);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = this->now();
    pose.pose.position.x = c.x;
    pose.pose.position.y = c.y;
    tf2::Quaternion q; q.setRPY(0,0,yaw);
    pose.pose.orientation = tf2::toMsg(q);

    NavigateToPose::Goal goal; goal.pose = pose;

    navigating_ = true;
    active_ = c;
    active_.last_attempt = this->now();

    // progress init
    goal_start_time_    = this->now();
    last_progress_time_ = this->now();
    last_feedback_dist_ = std::numeric_limits<double>::infinity();
    last_distance_remaining_ = std::numeric_limits<double>::infinity();
    if (auto p = robotXY()) last_progress_pose_ = *p;

    // reset plateau samples on new move
    plateau_samples_.clear();
    plateau_last_sample_ = this->now();

    RCLCPP_INFO(get_logger(), "Goal -> (%.2f, %.2f) ig=%d",
                c.x, c.y, c.info_gain);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    opts.feedback_callback = [this](GoalHandle::SharedPtr,
                                    const std::shared_ptr<const NavigateToPose::Feedback> fb){
      if (!fb) return;
      double d = fb->distance_remaining;
      if (!std::isfinite(d)) return;
      last_distance_remaining_ = d;
      if (d + 1e-3 < last_feedback_dist_ - progress_dist_thresh_m_){
        last_feedback_dist_ = d;
        last_progress_time_ = this->now();
      }
    };

    opts.result_callback = [this](const GoalHandle::WrappedResult &res){
      navigating_ = false;
      visited_.push_back(Visited{active_.x, active_.y, this->now()});

      if (res.code == rclcpp_action::ResultCode::SUCCEEDED){
        goals_succeeded_++;
        RCLCPP_INFO(this->get_logger(), "Reached.");
        if (auto rxy = robotXY()){
          refreshBacklog(*rxy);
          RCLCPP_INFO(this->get_logger(), "Post-reach refresh: backlog=%zu (unknown=%.3f)",
                      backlog_.size(), interiorUnknownRatio());
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Nav result code=%d; requeue.", (int)res.code);
        requeueFailed(active_);
        last_cancel_time_ = this->now();
      }
    };

    nav_client_->async_send_goal(goal, opts);
  }

  void saveMap(){
    // 1) Normalize path and create parent directory
    std::filesystem::path base(map_save_path_);
    if (!base.is_absolute()) {
      RCLCPP_WARN(get_logger(),
        "map_save_path is not absolute ('%s'). Prepending HOME.", map_save_path_.c_str());
      const char* home = std::getenv("HOME");
      base = (home ? std::filesystem::path(home) : std::filesystem::path("/tmp")) / base;
      map_save_path_ = base.string();
    }
    std::filesystem::path parent = base.parent_path();
    if (!parent.empty()) {
      std::error_code ec;
      std::filesystem::create_directories(parent, ec);
      if (ec) {
        RCLCPP_ERROR(get_logger(), "Failed to create directory '%s': %s",
                    parent.string().c_str(), ec.message().c_str());
      }
    }

    // 2) Try commands compatible with different Nav2 versions
    std::vector<std::string> cmds = {
      "ros2 run nav2_map_server map_saver_cli -f " + map_save_path_ + " --occ 0.65 --free 0.25",
      "ros2 run nav2_map_server map_saver_cli -f " + map_save_path_ + " --occ 0.65 --free 0.25 --image-format pgm",
      "ros2 run nav2_map_server map_saver_cli -f " + map_save_path_
    };

    RCLCPP_INFO(get_logger(), "Saving map to %s.[yaml|pgm]", map_save_path_.c_str());
    int ret = 1;
    std::string tried;
    for (auto &cmd : cmds) {
      tried += "  " + cmd + "\n";
      ret = std::system(cmd.c_str());
      if (ret == 0) {
        RCLCPP_INFO(get_logger(), "Map saved: %s.[yaml|pgm]", map_save_path_.c_str());
        return;
      }
    }
    RCLCPP_ERROR(get_logger(), "Map save failed (exit %d). Tried:\n%s", ret, tried.c_str());
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierExplorerNode>());
  rclcpp::shutdown();
  return 0;
}

REFLEX Explorer — Reactive Frontier Exploration with Local Exemptions

REFLEX Explorer is a lightweight ROS 2 node that drives autonomous map exploration on Nav2 by selecting frontier-adjacent goals using a reactive, nearest-first policy with local exemptions to avoid loops and thrashing. It’s designed to be practical, debuggable, and drop-in for small indoor robots (e.g., TurtleBot3) and Gazebo sims.

REFLEX = Reactive Frontier Exploration with Local Exemptions

Why another explorer?

Most “classical” frontier explorers pick frontiers, cluster them, and plan to centroids. In practice this can cause:

goals re-issued under the robot,

oscillation between nearby frontiers,

long detours to distant, high-gain clusters,

premature “done” when unknown space remains but the candidate selector stalls.

REFLEX keeps things simple and robust:

Nearest-first with BFS feasibility (unknown treated as traversable for feasibility only).

Local exemptions: recently visited spots are temporarily suppressed (radius+time) so we don’t re-pick the same goal after arrival or cancellation.

Underfoot guard: if the chosen goal is already under the robot (within near_goal_epsilon_m), it’s skipped and suppressed, not re-sent.

Plateau-of-progress stop: exploration ends only when unknown ratio improves too slowly over a window (and only after at least one success), avoiding instant “done.”

Small-step bootstrap: allows very short, reachable moves early so the robot starts exploring without manual teleop.

Architecture at a glance

Subscribes: /map (nav_msgs/OccupancyGrid), /cmd_vel (for progress heuristics).

Action client: /navigate_to_pose (nav2_msgs/action/NavigateToPose).

Frontier extraction: thin core that returns frontier points; REFLEX scores and filters them.

Goal selection:

refresh backlog from latest map,

filter by info gain and visited suppression,

nearest eligible by Euclidean distance (ties broken by score),

verify short BFS reachability (occupied blocked, unknown allowed),

send NavigateToPose once; never re-send same goal underfoot.

Differences vs. “classical” frontier exploration
Aspect	Classical	REFLEX
Frontier choice	Clustered, often global scoring	Nearest-first (reactive)
Loop avoidance	Heuristics, sometimes absent	Visited-suppression + underfoot guard
Feasibility test	Sometimes none or costmap probe	BFS in grid (unknown ok, occupied blocked)
Finish condition	Unknown threshold or queue empty	Plateau-of-progress + unknown threshold fallback
Behavior after goal reach	May re-pick same spot	Suppresses just-visited area for a cooldown

Trade-offs:
REFLEX is intentionally simple; it won’t “globally optimize” exploration routes. In highly structured environments with large loops, global sequencing may reduce path length more than REFLEX. The flip side is that REFLEX is stable, reactive, and tends to avoid local oscillations with minimal tuning.

Install & Build (ROS 2 Humble)
# Workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
# Add this repo into src (git clone your fork here)
git clone <your_fork_url> nav2_explore
cd ..

# Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Overlay
source install/setup.bash

Launch

REFLEX installs a single launcher:

ros2 launch nav2_explore explore.launch.py params_file:=/path/to/explorer_params.yaml \
    map_save_path:=/absolute/path/prefix   # optional override


params_file is your YAML with the parameters below.

map_save_path (CLI, optional): absolute prefix to save map (…/map.yaml + …/map.pgm).
If omitted or empty, REFLEX auto-creates:

<pkg_share>/maps/<YYYYMMDD-HHMMSS>/map.{yaml,pgm}


Note: REFLEX is an explorer, not a full Nav2 bringup. Ensure your Nav2 stack and map server are running and publishing/consuming the expected topics/frames.

Parameters

These match the node exactly (no extras). Names are under the node key reflex_explorer (or frontier_explorer if you didn’t rename the node).

Name	Type	Default	Notes
global_frame	string	map	Planning/global frame.
robot_base_frame	string	base_link	Robot base frame for TF lookup.
use_sim_time	bool	true	Typical in sim; optional.
sensor_range	double	3.5	Info-gain radius (meters).
min_frontier_cells	int	12	Minimum cells to accept a frontier.
min_info_gain_cells	int	10	Minimum unknown cells near a candidate.
edge_ignore_margin_m	double	0.25	Ignore band near map edges.
dedupe_radius_m	double	0.5	Merge near-duplicate candidates.
refresh_period_sec	double	2.0	Backlog refresh cadence.
retry_cooldown_sec	double	5.0	Wait before retrying a failed goal.
max_attempts_per_goal	int	3	Cap attempts before discarding a goal.
cancel_cooldown_sec	double	1.5	Delay after cancel before re-planning.
min_goal_runtime_sec	double	6.0	Don’t cancel too quickly.
progress_timeout_sec	double	10.0	Cancel if no progress for this long.
progress_dist_thresh_m	double	0.05	Movement needed to count as “progress.”
min_unknown_ratio	double	0.02	Fallback completion criterion.
near_goal_epsilon_m	double	0.35	If goal is inside this, skip & suppress.
visited_suppress_radius_m	double	0.8	Local exemption radius (meters).
visited_suppress_sec	double	30.0	Local exemption time (seconds).
plateau_enable_unknown_ratio	double	0.25	Only consider plateau when mostly explored.
plateau_sample_sec	double	4.0	Sampling cadence for plateau check.
plateau_window_sec	double	60.0	Rolling window duration.
plateau_min_improve	double	0.01	Relative improvement threshold to continue.
plateau_warmup_sec	double	20.0	Ignore plateau early in a goal.
plateau_transit_dist_m	double	2.0	Ignore plateau while far from goal.
plateau_autoscale_window	bool	true	Tie window to ETA by distance remaining.
plateau_min_window_sec	double	45.0	Lower bound on window duration.
plateau_max_window_sec	double	240.0	Upper bound on window duration.
plan_speed_guess_mps	double	0.22	Used to estimate ETA for autoscaling.
save_when_done	bool	true	Save map when exploration finishes.
map_save_path	string	""	Empty ⇒ auto timestamped path in <pkg_share>/maps. CLI absolute overrides.

A tuned example YAML (what we’ve been using in the screenshots) is included at config/explorer_params.yaml.

Typical Workflow

Bring up sim + Nav2 (e.g., TB3 Gazebo + Nav2).

Launch REFLEX:

ros2 launch nav2_explore explore.launch.py \
    params_file:=<your_ws>/src/nav2_explore/config/explorer_params.yaml


(If the robot sits on a “goal underfoot”, you’ll see
Goal is already under robot ... Skipping send.) This is normal—either:

wait a few seconds for another frontier to appear,

or gently nudge the robot with teleop once; REFLEX will take over.

Map Saving

Default (no map_save_path):
…/install/nav2_explore/share/nav2_explore/maps/<YYYYMMDD-HHMMSS>/map.yaml|pgm

Override on CLI:

ros2 launch nav2_explore explore.launch.py ... map_save_path:=/home/wolf/map1

Benchmarks (how to measure)

We recommend evaluating with:

Coverage vs. time: fraction of unknown cells declining over time (record map and compute offline).

Number of re-issued goals: REFLEX should keep this near zero.

Exploration time to a given unknown threshold (e.g., 5%).

Path length (optional): accumulate odometry or use Nav2 feedback.

Record:

ros2 bag record /map /tf /tf_static /odom /amcl_pose /cmd_vel /goal_pose /navigate_to_pose/_action/feedback

Troubleshooting

“Backlog=0 (unknown~0.79) and no motion”
Lower min_frontier_cells / min_info_gain_cells, increase sensor_range, or reduce edge_ignore_margin_m. Make sure map is publishing and frames are correct.

Keeps sending the same goal
You should see Goal is already under robot ... Skipping in logs; if not, reduce near_goal_epsilon_m or increase visited_suppress_radius_m/visited_suppress_sec.

Immediate finish at start
Increase plateau_enable_unknown_ratio (e.g., 0.5) so plateau isn’t checked too early; raise plateau_window_sec/plateau_min_improve.

Limitations

Not globally optimal; prioritizes stability and reactivity over tour-length minimization.

BFS feasibility treats unknown as traversable to avoid over-pruning early; Nav2 may still reject difficult goals (which REFLEX retries or suppresses).

Heavily cluttered environments may require tuning min_frontier_cells and suppression radii.

Use Cases

Apartment/office sweep: Quickly map small indoor spaces with doors and tight corners; nearest-first goals prevent dithering at thresholds and keep progress steady.

Warehouse aisle scouting: On SBC-powered bases, reactively clear unknown pockets along aisles without heavy compute or global clustering.

Pop-up lab/demo setups: Bring up a robot in a new room and get a usable map fast; tolerant startup frontiers reduce the need for teleop nudges.

Contributing

Issues and PRs are welcome. Please include:

environment (ROS distro, sim/real),

your params file,

logs that show REFLEX messages,

optional bag snippets.

License

MIT (see LICENSE in the repository).

Citation

If you use REFLEX in academic work, please cite the repository and refer to this approach as:

REFLEX — Reactive Frontier Exploration with Local Exemptions
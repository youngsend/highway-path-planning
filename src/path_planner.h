//
// Created by sen on 2021/07/18.
//

#ifndef PATH_PLANNING_SRC_PATH_PLANNER_H_
#define PATH_PLANNING_SRC_PATH_PLANNER_H_

#include <utility>
#include <vector>
#include <cmath>
#include <iostream>

#include "spline.h"

constexpr double pi() { return M_PI; };
constexpr int PATH_WAYPOINT_SIZE = 50;
constexpr double TIME_INTERVAL = 0.02;
constexpr double MAX_VEL = 49.5 / 2.237; // 49.5mph to m/s.
constexpr double MAX_ACC = 9.5;
constexpr double DELTA_VEL = MAX_ACC * TIME_INTERVAL; // velocity change within 0.02s.

class PathPlanner {
 public:
  PathPlanner(std::vector<double> map_waypoints_x,
              std::vector<double> map_waypoints_y,
              std::vector<double> map_waypoints_s,
              std::vector<double> map_waypoints_dx,
              std::vector<double> map_waypoints_dy) :
      _map_waypoints_x(std::move(map_waypoints_x)),
      _map_waypoints_y(std::move(map_waypoints_y)),
      _map_waypoints_s(std::move(map_waypoints_s)),
      _map_waypoints_dx(std::move(map_waypoints_dx)),
      _map_waypoints_dy(std::move(map_waypoints_dy)) {};
  ~PathPlanner() = default;
  void FillNextPath(double car_x,
                           double car_y,
                           double car_s,
                           double car_d,
                           double car_yaw,
                           double car_speed,
                           const std::vector<double>& previous_path_x,
                           const std::vector<double>& previous_path_y,
                           double end_path_s,
                           double end_path_d,
                           const std::vector<std::vector<double>>& sensor_fusion,
                           std::vector<double>& next_x_vals,
                           std::vector<double>& next_y_vals);

 private:
  // Calculate closest waypoint to current x, y position
  int ClosestWaypoint(double x, double y);
  // Returns next waypoint of the closest waypoint
  int NextWaypoint(double x, double y, double theta);
  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  std::vector<double> getFrenet(double x, double y, double theta);
  // Transform from Frenet s,d coordinates to Cartesian x,y
  std::vector<double> getXY(double s, double d);
  // For converting back and forth between radians and degrees.
  static double deg2rad(double x) { return x * pi() / 180; };
  static double rad2deg(double x) { return x * 180 / pi(); };

  // Calculate distance between two points
  static double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
  };
  static int CurrentLaneIndex(double d);

 private:
  // map info
  std::vector<double> _map_waypoints_x;
  std::vector<double> _map_waypoints_y;
  std::vector<double> _map_waypoints_s;
  // 1. dx and dy are given in the frame of the map and not in the Frenet frame and
  //    if we rotate it 90° we will get the direction of the road at (x,y).
  // 2. If for any reason we would like to convert an point given in the Frenet frame to map frame,
  //    we need the information about the direction of the road.
  std::vector<double> _map_waypoints_dx;
  std::vector<double> _map_waypoints_dy;

  double _ref_vel {MAX_VEL};
};

#endif //PATH_PLANNING_SRC_PATH_PLANNER_H_

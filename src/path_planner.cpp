//
// Created by sen on 2021/07/18.
//

#include "path_planner.h"

// Calculate closest waypoint to current x, y position
int PathPlanner::ClosestWaypoint(double x,
                                 double y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < _map_waypoints_x.size(); ++i) {
    double map_x = _map_waypoints_x[i];
    double map_y = _map_waypoints_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int PathPlanner::NextWaypoint(double x,
                              double y,
                              double theta) {
  int closestWaypoint = ClosestWaypoint(x,y);

  double map_x = _map_waypoints_x[closestWaypoint];
  double map_y = _map_waypoints_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == _map_waypoints_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> PathPlanner::getFrenet(double x,
                                           double y,
                                           double theta) {
  int next_wp = NextWaypoint(x,y, theta);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = _map_waypoints_x.size()-1;
  }

  double n_x = _map_waypoints_x[next_wp]-_map_waypoints_x[prev_wp];
  double n_y = _map_waypoints_y[next_wp]-_map_waypoints_y[prev_wp];
  double x_x = x - _map_waypoints_x[prev_wp];
  double x_y = y - _map_waypoints_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-_map_waypoints_x[prev_wp];
  double center_y = 2000-_map_waypoints_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(_map_waypoints_x[i],_map_waypoints_y[i],
                         _map_waypoints_x[i+1],_map_waypoints_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}


std::vector<double> PathPlanner::getXY(double s,
                                       double d) {
  int prev_wp = -1;

  while (s > _map_waypoints_s[prev_wp+1] && (prev_wp < (int)(_map_waypoints_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%_map_waypoints_x.size();

  double heading = atan2((_map_waypoints_y[wp2]-_map_waypoints_y[prev_wp]),
                         (_map_waypoints_x[wp2]-_map_waypoints_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-_map_waypoints_s[prev_wp]);

  double seg_x = _map_waypoints_x[prev_wp]+seg_s*cos(heading);
  double seg_y = _map_waypoints_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}
void PathPlanner::FillNextPath(double car_x,
                               double car_y,
                               double car_s,
                               double car_d,
                               double car_yaw,
                               double car_speed,
                               const std::vector<double> &previous_path_x,
                               const std::vector<double> &previous_path_y,
                               double end_path_s,
                               double end_path_d,
                               const std::vector<std::vector<double>> &sensor_fusion,
                               std::vector<double> &next_x_vals,
                               std::vector<double> &next_y_vals) {
  // ToDo: 1. lane keeping; 2. follow the preceding vehicle; 3. lane change (cost function);
  // ToDo: 4. trajectory generation.

}

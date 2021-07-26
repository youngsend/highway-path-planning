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

// In order for the passenger to have an enjoyable ride
// both the jerk and the total acceleration should not exceed 10 m/s^2.
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
  // ToDo: 4. trajectory generation (use x, y coordinates).
  // calculate current lane index, left: 0, middle: 1, right: 2.
  int current_lane = CurrentLaneIndex(car_d);
  int target_lane = current_lane; // used for lane change and trajectory generation.
  double last_vel = car_speed; // used as the init vel when generating velocity profile (distance interval).

  // from here are code from project Q&A. to implement lane keeping and preceding vehicle following.

  int prev_size = previous_path_x.size();

  // this means generating trajectory from the last point of previous path.
  if (prev_size > 0) {
    car_s = end_path_s;
  }

  bool too_close = false;

  // find ref_v to use
  double preceding_car_dist = CLOSE_DIST;
  for(const auto & vehicle_data : sensor_fusion) {
    double d = vehicle_data[6];
    if (d < 4+4*current_lane && d > 4*current_lane) {
      // car is in my lane
      double vx = vehicle_data[3];
      double vy = vehicle_data[4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = vehicle_data[5];

      check_car_s += (double)prev_size * TIME_INTERVAL * check_speed;

      if (check_car_s > car_s && check_car_s - car_s < CLOSE_DIST) {
        // this vehicle is in (0, 30) front of ego when previous path is over.
        // ToDo: try lane change. only need to change current_lane.
        //  cost function: I only consider at which lane ego car should be.
        too_close = true;
        if (check_car_s - car_s < preceding_car_dist) {
          // record the preceding vehicle.
          // if there's car right in front of ego, run after it.
          _ref_vel = check_speed;
          preceding_car_dist = check_car_s - car_s;
        }
      }
    }
  }

  if (!too_close) {
    // if there's no preceding vehicle within 30m, run at max velocity.
    _ref_vel = MAX_VEL;
  }

  // create a list of widely spaced (x, y) waypoints, evenly spaced at 30m.
  // later interpolate these waypoints with a spline and fill it with more points that control speed.
  std::vector<double> ptsx, ptsy;

  // reference x, y, yaw states.
  // either reference the starting point as where the car is or at the previous paths and point
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw); // car_yaw is in degree!

  // if previous size is almost empty, use the car as starting reference.
  if (prev_size < 2) {
    // use two points that make the path tangent to the car.
    double prev_car_x = car_x - cos(ref_yaw);
    double prev_car_y = car_y - sin(ref_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  } else {
    // redefine reference state as previous path end point
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // use two points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);

    // if more than 2 points remain, use last 2 points to calculate velocity between them.
    last_vel = distance(ref_x_prev, ref_y_prev, ref_x, ref_y) / TIME_INTERVAL;
  }

  // in Frenet add evenly 30m spaced points ahead of the starting reference.
  // car_s has become the last point of previous path now.
  // when changing lane, this means ego vehicle needs to complete lane change within 30m. Is this OK?
  std::vector<double> next_wp0 = getXY(car_s+30, 2+4*target_lane);
  std::vector<double> next_wp1 = getXY(car_s+60, 2+4*target_lane);
  std::vector<double> next_wp2 = getXY(car_s+90, 2+4*target_lane);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // the reason for transforming to ego car coordinate system may be to avoid that
  // two points in spline correspond to the same x coordinate.
  for(int i = 0; i < ptsx.size(); i++) {
    // shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
    ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
  }

  // create a spline
  tk::spline s;

  // set (x, y) points to the spline
  s.set_points(ptsx, ptsy);

  // start with all of the previous path points from last time
  // ToDo: should not use all points from last path, because this leads to less responsiveness.
  //  this can be mitigated by decreasing path size.
  for(int i=0; i<previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  double x_add_on = 0;
  double x_dist_rate = target_x / target_dist;

  double delta_dist;
  // fill up the rest of our path planner after filling it with previous points, always output 50 points
  for(int i=0; i<PATH_WAYPOINT_SIZE-previous_path_x.size(); i++) {
    // make sure max acceleration is not violated.
    if (last_vel < _ref_vel - DELTA_VEL) {
      // need to accelerate
      delta_dist = last_vel * TIME_INTERVAL + 0.5 * MAX_ACC * TIME_INTERVAL * TIME_INTERVAL;
      last_vel += DELTA_VEL;
    } else if (last_vel > _ref_vel + DELTA_VEL) {
      // need to decelerate
      delta_dist = last_vel * TIME_INTERVAL - 0.5 * MAX_ACC * TIME_INTERVAL * TIME_INTERVAL;
      last_vel -= DELTA_VEL;
    } else {
      // can reach reference velocity after 0.02s.
      delta_dist = TIME_INTERVAL * _ref_vel;
      last_vel = _ref_vel;
    }
    double delta_x = x_dist_rate * delta_dist;

    double x_point = x_add_on + delta_x;
    double y_point = s(x_point);
    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal after rotating it earlier
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}

// calculate current lane index, left: 0, middle: 1, right: 2.
int PathPlanner::CurrentLaneIndex(double d) {
  if (d <= 0 || d > 12) {
    std::cout << "d is invalid!\n";
    return -1;
  } else if (d <= 4) {
    return 0;
  } else if (d <= 8) {
    return 1;
  } else {
    return 2;
  }
}

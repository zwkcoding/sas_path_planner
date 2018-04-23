/*
 * sas_explore.cpp
 *
 *  Created on: Oct 22, 2017
 *      Author: kevin
 */

#include <cmath>
#include <limits>
#include <algorithm>

#include <ros/console.h>
#include <opt_utils/logger.hpp>
#include <opt_utils/time.hpp>
#include "sas_space_explore/sas_explore.hpp"

namespace hmpl {
SasExplore::SasExplore()
    : display_cv_(false),
      start_(),
      goal_(),
      coeff_distance_to_obstacle_(0.8),
      coeff_distance_to_goal_(0.6),
      window_name_("sas_display"),
      reversed_search_(false),
      enable_drive_backward_(false),
      wheelbase_(2.8448),
      wide_(2.5),
      track_(1.5748),
      max_steer_(35 / 180.0 * M_PI),
      direction_fraction_(64),
      steer_fraction_(32),
      end_to_center_(1.09),
      front_to_center_(3.81),
      effective_zone_scale_factor_(0.2),
      display_(),
      overlay_(),
      path_(),
      car_body_(),
      f_goal_(std::numeric_limits<double>::infinity()),
      guarantee_heading_(false),
      effective_threshold_dis_(1),
      max_primitive_length_(14),
      min_primitive_length_(0.5),
      search_scale_map_(nullptr),
      distance_mask(nullptr),
      best_open_(),
      width_(),
      height_(),
      closed_() {

  // initialize cost factor in path
  cost_factor_.reverse_cost = 3.0;
  cost_factor_.steer_change_cost = 2.0;
  cost_factor_.collision_free = 3;
  cost_factor_.heuristic_cost = 5.0;

  ros::NodeHandle private_nh_("~");
  int angle_size, heading_size;
  private_nh_.param<int>("angle_size", angle_size, 32);
  private_nh_.param<int>("heading_size", heading_size, 32);

    // Initialize sas planner
  InitPlanner(heading_size, angle_size);

}

SasExplore::~SasExplore() {
  // clean up memory of heap
  // First delete
  if (search_scale_map_ != nullptr) {
    for (int z = 0; z < direction_fraction_; z++) {
      for (int j = 0; j < height_; j++) {
        delete[] search_scale_map_[z][j];
        search_scale_map_[z] = nullptr;
      }
      delete[] search_scale_map_[z];
    }
    delete[] search_scale_map_;
    search_scale_map_ = nullptr;
  }

  int distance_mask_size = 2 * (static_cast<int>(max_primitive_length_ + 1)) + 1;
  // First delete
  if (distance_mask != nullptr) {
    for (int i = 0; i < distance_mask_size; i++) {
      delete[] distance_mask[i];
    }
    delete[] distance_mask;
    distance_mask = nullptr;
  }
}

void SasExplore::InitPlanner(int directions, int steers) {
  car_body_.max_steer = max_steer_;
  car_body_.minimum_turning_radius = wheelbase_ / tan(max_steer_);
  car_body_.wheelbase = wheelbase_;
  car_body_.track = track_;
  car_body_.end_to_center = end_to_center_;
  car_body_.front_to_center = front_to_center_;
  car_body_.wide = wide_;
  // initialize car geometry
  InitCarGeometry(car_body_);
  // initialize motion primitive
  direction_fraction_ = directions;
  motion_primitives_.Init(car_body_, directions, steers);
  ROS_DEBUG( "motion primitive initialize finish" );
  ROS_INFO_STREAM( " MAX_ARC_LENGTH: " << motion_primitives_.GetMaximumStepLength());
  ROS_INFO_STREAM( " min radius : " << car_body_.minimum_turning_radius << '\n'
            << "max_radius : " << wheelbase_ / tan(motion_primitives_.m_min_steer_step));

  max_primitive_length_ = motion_primitives_.GetMaximumStepLength();
    ROS_DEBUG("search map initialize finish");
}

void SasExplore::InitCarGeometry(sas_element::CarParameter &car_body) {
  Vector2D<double> r_r{}, f_r{}, f_l{}, r_l{};
  r_r = Vector2D<double>(-end_to_center_, -wide_ / 2.0);
  f_r = Vector2D<double>(front_to_center_, -wide_ / 2.0);
  f_l = Vector2D<double>(front_to_center_, wide_ / 2.0);
  r_l = Vector2D<double>(-end_to_center_, wide_ / 2.0);
  car_body.circle_fill_car.setWheebase(car_body.wheelbase);
  car_body.circle_fill_car.setTrack(car_body.track);
  car_body.circle_fill_car.setFootprint(r_r, f_r, f_l, r_l);
  // update fill circle
  car_body.circle_fill_car.buildCirclesFromFootprint();
  ROS_DEBUG("car geometry initialize complete :");
  for (const auto &iter : car_body.circle_fill_car.getCenters()) {
    std::cout << iter.position.x << ", "
              << iter.position.y << ", "
              << iter.r << '\n';
  }
}

void SasExplore::InitSearchMap() {
  // maye resolution loss, convert into gridmap
  width_ = static_cast<int>(internal_grid_.maps.getLength().x());
  height_ = static_cast<int>(internal_grid_.maps.getLength().y());

  search_scale_map_ = new double **[direction_fraction_ * sizeof(double **)];
  for (int z = 0; z < direction_fraction_; z++) {
    search_scale_map_[z] = new double *[height_ * sizeof(double *)];
    for (int y = 0; y < height_; y++) {
      search_scale_map_[z][y] = new double[width_];
      for (int j = 0; j < width_; j++) {
        search_scale_map_[z][y][j] = std::numeric_limits<double>::infinity();
      }
    }
  }

  // Create distance look-up table
  // speed up find effectine zone scope
  int distance_mask_size = 2 * (static_cast<int>(max_primitive_length_ + 1)) + 1;
  distance_mask = new double *[distance_mask_size * sizeof(double *)];
  for (int y = 0; y < distance_mask_size; y++) {
    distance_mask[y] = new double[distance_mask_size * sizeof(double)];
    for (int x = 0; x < distance_mask_size; x++) {
      double deltax = x - distance_mask_size / 2;
      double deltay = y - distance_mask_size / 2;
      distance_mask[y][x] = sqrt(deltax * deltax + deltay * deltay);
    }
  }
}

void SasExplore::StartSearchFromMap(hmpl::InternalGridMap &gridMap,
                                    const Pose2D &start,
                                    const Pose2D &goal) {
  static bool initialized_flag = false;

  // update the grid reference
    // todo copy
  internal_grid_ = gridMap;

    if(!initialized_flag) {
        InitSearchMap();
        effective_zone_scale_factor_ = this->internal_grid_.maps.getResolution();  // adapt to different map resolution
        initialized_flag = true;
    }

  // copy the new start pose
  start_ = start;
  // save the next goal pose
  goal_ = goal;


  grid_map::Position start_pos(start.position.x, start.position.y);
  grid_map::Position goal_pos(goal.position.x, goal.position.y);
  if(!internal_grid_.maps.isInside(start_pos) ||
     !internal_grid_.maps.isInside(goal_pos)) {
        ROS_ERROR("start or goal position is out of map !!!");
        return;
    } else {
      if(0 == internal_grid_.getObstacleDistance(start_pos) ||
         0 == internal_grid_.getObstacleDistance(goal_pos)) {
            ROS_ERROR("start or goal position is within obstacle !!!");
            return;
        }
    }

    // allocate the image
    if (this->display_cv_) {

        int rows = this->internal_grid_.maps.getSize()(0);
        int cols = this->internal_grid_.maps.getSize()(1);

        this->display_ = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
        for (int i = 0; i < display_.cols; i++) {
            for (int j = 0; j < display_.rows; j++) {
                if (0 == internal_grid_.maps.at("obstacle", grid_map::Index(j, i))) {
                    cv::Vec3b intensity(0, 0, 0);
                    this->display_.at<cv::Vec3b>(j, i) = intensity;
                }
            }
        }

    }

  if (!SasSearch()) {
    ROS_WARN("Could no find a feasible path ");
    return;
  }
}

bool SasExplore::SasSearch() {
  Vector2D<double> start_pos{}, end_pos{};
  // initialize start and goal pose
  if (!reversed_search_) {
    start_pos = start_.position;
    end_pos = goal_.position;
  } else {
    start_pos = goal_.position;
    end_pos = start_.position;
  }
  sas_element::PrimitiveNode start_node{}, goal_node{};
  start_node.vehicle_center_position = start_pos;
  goal_node.vehicle_center_position = end_pos;
  if (!reversed_search_) {
    start_node.movement = 1;
    start_node.primitive_node_state.beginning_heading_index = cvFloor(start_.orientation / motion_primitives_.m_min_orientation_angle);
    goal_node.primitive_node_state.beginning_heading_index = cvFloor(goal_.orientation / motion_primitives_.m_min_orientation_angle);

  } else {
    start_node.movement = -1;
    // use cvFloor not cvceil, avoid orientation_index >= 32
    start_node.primitive_node_state.beginning_heading_index = cvFloor(goal_.orientation / motion_primitives_.m_min_orientation_angle);
    goal_node.primitive_node_state.beginning_heading_index = cvFloor(start_.orientation / motion_primitives_.m_min_orientation_angle);
  }

  ROS_INFO("Start node: (%f, %f, %d) ", start_pos.x, start_pos.y, start_node.primitive_node_state.beginning_heading_index);
  ROS_INFO("Goal node: (%f, %f, %d) ", end_pos.x, end_pos.y, goal_node.primitive_node_state.beginning_heading_index);


    // show start and goal
  if (this->display_cv_) {
    display_.copyTo(overlay_);
    DrawStartAndEnd(&overlay_, start_pos, end_pos);
    // create window
    cv::namedWindow(window_name_, 0);
    cv::imshow(window_name_, overlay_);
    cv::waitKey(1);
  }

  // clear
  path_.clear();
  // reset search space
  ResetSearchMap();

  double goal2start_dis = start_.position.Distance(goal_.position);
  start_node.f_cost = goal2start_dis;

   double effective_zone = SetZoomFactorWithEffectiveZone(&start_node, goal2start_dis);

  this->best_open_.push(start_node);
  // record search cost time
  auto start = std::chrono::steady_clock::now();
  while (!best_open_.empty()) {

    auto end = std::chrono::steady_clock::now();
    double elapsed_secondes = getDurationInSecs(start, end);
    if (!display_cv_) {
      ROS_WARN_STREAM_COND(elapsed_secondes > 2,
                           "fail! cost time exceed 2 s");
      if (elapsed_secondes > 2) {
        break;
      }
    } else {
      ROS_WARN_STREAM_COND(elapsed_secondes > 10,
                           "fail! cost time exceed 10 s");
      if (elapsed_secondes > 10) {
        break;
      }
    }
    // pop the min element
    start_node = best_open_.top();

    // remove the top element from the queue
    // need not to consider memory leak
    best_open_.pop();

    // process the current motion primitive node
    // if it's a valid path to the goal it will return true
    // otherwise we got a false value
    if (ExpandNodeProcess(start_node, goal_node, &overlay_)) {
      RebuildPath(&overlay_, goal_node);
      RemoveAllCNodes();
      return (this->f_goal_ < std::numeric_limits<double>::infinity());
    }
  }

  RemoveAllCNodes();
  return false;
}

bool SasExplore::NodeExistInSearchedEfficientZone(const sas_element::PrimitiveNode &pn) {
  //
  grid_map::Position cur_pos(pn.vehicle_center_position.x, pn.vehicle_center_position.y);
  if (internal_grid_.maps.isInside(cur_pos)) {
    int i = pn.primitive_node_state.beginning_heading_index % direction_fraction_;
    int j = MAX(0,MIN(pn.vehicle_center_position.x + width_ / 2, width_ - 1));
    int k = MAX(0, MIN(pn.vehicle_center_position.y + height_ / 2, height_ - 1));
    // update: weighted map with lower g_cost
    if (pn.g_cost <= search_scale_map_[i][k][j]) {
      return false;
    } else {
      return true;
    }
  } else {
    return true;
  }

}

// npn: next primitive node
// cpn: current primitive node
// gpn: goal primitive node
bool SasExplore::ExpandNodeProcess(sas_element::PrimitiveNode &cpn,
                                   sas_element::PrimitiveNode &gpn,
                                   cv::Mat *image) {

  sas_element::PrimitiveNode *cpn1 = new sas_element::PrimitiveNode(cpn);
  sas_element::PrimitiveNode *npn1 = new sas_element::PrimitiveNode();
  npn1->parentnode = cpn1;

  // temp_counter = 1: consider 32 fractions, disable reverse movement
  // temP_counter = 2: consider 64 fractions, enable reverse movement
  int temp_counter = 1;
  if (enable_drive_backward_) {
    temp_counter = 2;
  }

  sas_element::MotionPrimitiveState *primitive_state_ptr = nullptr;

  CvScalar color = CV_RGB(rand() % 100 + 100, rand() % 100 + 100, rand() % 100 + 100);

  for (int steering_index = 0; steering_index < temp_counter * motion_primitives_.GetSteerNum(); steering_index++) {

    ROS_DEBUG_STREAM("STEER_INDEX : " << steering_index );
    // forward motion primitives
    if (steering_index < motion_primitives_.GetSteerNum()) {
      primitive_state_ptr = motion_primitives_.GetStateOfPrimitive(cpn1->primitive_node_state.beginning_heading_index, steering_index);
      npn1->movement = 1;
      npn1->primitive_node_state.beginning_heading_index = cpn1->primitive_node_state.ending_heading_index
              = primitive_state_ptr->ending_heading_index;
      npn1->primitive_node_state.steering_index = steering_index;
    }
    // backward motion primitives
    else {
      primitive_state_ptr = motion_primitives_.GetStateOfPrimitive(
              (cpn1->primitive_node_state.beginning_heading_index + direction_fraction_ / 2) % direction_fraction_,
              steering_index - motion_primitives_.GetSteerNum());
      npn1->movement = -1;
      npn1->primitive_node_state.beginning_heading_index = cpn1->primitive_node_state.ending_heading_index
              = (primitive_state_ptr->ending_heading_index + direction_fraction_ / 2) % direction_fraction_;
      npn1->primitive_node_state.steering_index = steering_index - motion_primitives_.GetSteerNum();
    }

    npn1->index = cpn1->index + 1;
    npn1->current_zoom = cpn1->sub_node_zoom;
    // steering center of curve linking current node(cpn1) and next node(npn1)
    npn1->primitive_node_state.steering_center_position = primitive_state_ptr->steering_center_position * npn1->current_zoom;

    double radius = npn1->primitive_node_state.steering_radius =  primitive_state_ptr->steering_radius * npn1->current_zoom;
    // remove node that violate nonholonomic vehicle constraint
    if (radius < car_body_.minimum_turning_radius &&
        primitive_state_ptr->steering_radius != 0) {
      if (steering_index < motion_primitives_.GetSteerNum()) {
        steering_index = motion_primitives_.GetSteerNum() - 1;
        ROS_DEBUG_STREAM("current node violate nonholinomic constraint");
        continue; // continue to consider reverse steering movement
      } else {
        break;
      }
    }
    // decide steering direction when move forwards or backwards
    if (primitive_state_ptr->steering_angle_radian > 0) {
      npn1->primitive_node_state.steering_angle_radian = atan(car_body_.wheelbase /
          (primitive_state_ptr->steering_radius * npn1->current_zoom));
    } else if (primitive_state_ptr->steering_angle_radian < 0) {
      npn1->primitive_node_state.steering_angle_radian = -atan(
              car_body_.wheelbase / (primitive_state_ptr->steering_radius * npn1->current_zoom));
    }
    npn1->primitive_node_state.steering_angle_radian *= npn1->movement;

    // when shrink original motion primitive curve,
    // keep same changed heading angle, but expand corresponding steering angle
    npn1->vehicle_center_position.x = cpn1->vehicle_center_position.x + primitive_state_ptr->ending_position.x *
        npn1->current_zoom;
    npn1->vehicle_center_position.y = cpn1->vehicle_center_position.y + primitive_state_ptr->ending_position.y *
        npn1->current_zoom;

    // test the improved time
    auto start = hmpl::now();
    // check vehicle body safety
    double clear_cost = ClearanceCostInPathUseOverlayCirles(*primitive_state_ptr, *npn1, image);
//    double clear_cost = ClearanceCostInPathUseCircumcircle(*primitive_state_ptr, *npn1, image);
    auto end = hmpl::now();
    double time_duration_improved = hmpl::getDurationInSecs(start, end);

    if (-1 == clear_cost) {
      ROS_DEBUG("Clearance checking failed or checking states is out of map");
      continue;
    } else {
      clear_cost  = cost_factor_.collision_free * clear_cost;
//      npn1->g_cost -=  clear_cost;
    }

    double dist2goal = npn1->vehicle_center_position.Distance(gpn.vehicle_center_position);

    double length = primitive_state_ptr->arc_length * npn1->current_zoom;
    // calculate arc_length cost
//    double move_cost = length * ((1 == npn1->movement) ? cost_factor_.forward_cost :
//                                   cost_factor_.backward_cost);
//    npn1->g_cost += move_cost;
//
    double delta_steer = fabs(primitive_state_ptr->delta_heading_radian);
    double delta_heading_cost = cost_factor_.steer_change_cost * delta_steer;
//    npn1->g_cost += delta_heading_cost;
//
//    double reverse_cost = 0;
//    if (npn1->movement != cpn1->movement) {
//        reverse_cost = cost_factor_.reverse_cost * length;
//        npn1->g_cost += reverse_cost;
//    }

      double move_cost = length;
      if(abs(delta_steer) > 20 * M_PI / 180)
      move_cost *=  cost_factor_.steer_change_cost;
      if (npn1->movement != cpn1->movement) {
          move_cost *= cost_factor_.reverse_cost;
      }


    npn1->g_cost = cpn1->g_cost + move_cost;
    double heuristic_cost = cost_factor_.heuristic_cost * dist2goal;
    npn1->f_cost = npn1->g_cost + heuristic_cost;

    ROS_WARN_STREAM_THROTTLE( 1, "movement cost : " <<  move_cost << '\n' <<
                                                    "heuristic cost : " << heuristic_cost);

    // decide next node's scale factor
    double effective_zone = SetZoomFactorWithEffectiveZone(npn1, dist2goal);

    // important for effective zone notion
    if (NodeExistInSearchedEfficientZone(*npn1)) {
      ROS_DEBUG("current node has been searched!");
      continue;
    } else {
      // label the effective-zone of current node  with g-cost
      SetNodeInEffectiveZone(*npn1, effective_zone);
    }

    // show
    if (display_cv_) {
      double factor = npn1->current_zoom;
      double k = 1 / npn1->current_zoom;
      // todo fix it
      for (int i = 0; i < primitive_state_ptr->primitive_path.size(); i += k) {
        double x = cpn1->vehicle_center_position.x + primitive_state_ptr->primitive_path[i].x * factor;
        double y = cpn1->vehicle_center_position.y + primitive_state_ptr->primitive_path[i].y * factor;
        // value deviation from default transform double -> int
        Vector2D<double> gridmap(x, y);
        cv::Point opencv(sas_element::GridmapToOpencv(gridmap, width_, height_).x / this->internal_grid_.maps.getResolution() ,
                         sas_element::GridmapToOpencv(gridmap, width_, height_).y / this->internal_grid_.maps.getResolution());
        cv::circle(*image, opencv, 0, color);
      }
      cv::imshow(window_name_, *image);
      cv::waitKey(1);
    }

    // check whether reach goal pose
    if (dist2goal < effective_threshold_dis_) {
      if (guarantee_heading_ == true) {
        if (abs(npn1->primitive_node_state.beginning_heading_index -
                        gpn.primitive_node_state.beginning_heading_index) < 5) {
          gpn.parentnode = npn1;
          gpn.index = npn1->index + 1;
          this->f_goal_ = std::min(this->f_goal_, npn1->f_cost);
          ROS_INFO("reach goal pose!");
          return true;
        }
      } else {
        gpn.parentnode = npn1;
        gpn.index = npn1->index + 1;
        this->f_goal_ = std::min(this->f_goal_, npn1->f_cost);
        ROS_INFO("reach goal position!");
        // remove the nodes
        ROS_DEBUG_STREAM("next top_open : " << best_open_.size() << "  closed : "
                                            << closed_.size() << '\n');
        return true;
      }
    }
    // copy container
    best_open_.push(*npn1);
  }
  closed_.push_back(*cpn1);

  return false;
}

double SasExplore::ClearanceCostInPathUseOverlayCirles(const sas_element::MotionPrimitiveState &primitive_state,
                                                       const sas_element::PrimitiveNode &node, cv::Mat *image) {
  double rad_step = primitive_state.delta_heading_radian / primitive_state.primitive_path.size();
  double start_rad = primitive_state.beginning_heading_index * motion_primitives_.m_min_orientation_angle;
  double k = 1 / node.current_zoom;  // unit arc length sampling
  double clear_cost = 0;
  bool is_inside_map = true;

  // collision detection for every state(unit arc length) in primitive_path of a motion primitive
  for (int i = primitive_state.primitive_path.size() - 1; i >= 0; i -= k) {
    double x = node.parentnode->vehicle_center_position.x + primitive_state.primitive_path[i].x * node.current_zoom;
    double y = node.parentnode->vehicle_center_position.y + primitive_state.primitive_path[i].y * node.current_zoom;
    double rad = start_rad + rad_step * i;
    if (-1 == node.movement) {
      rad += M_PI;
    }
    if (rad > 2 * M_PI) {
      rad -= 2 * M_PI;
    } else if (rad < 0) {
      rad += 2 * M_PI;
    }

    hmpl::State current_state;
    current_state.x = x;
    current_state.y = y;
    current_state.z = rad;

    std::vector<hmpl::Circle> footprint;
    footprint = car_body_.circle_fill_car.getCurrentCenters(current_state);

    if (!internal_grid_.maps.isInside(grid_map::Position(x, y))) {
      is_inside_map = false;
      return -1;
    }

    for (const auto &iter : footprint) {
      if (!internal_grid_.maps.isInside(grid_map::Position(iter.position.x, iter.position.y))) {
        is_inside_map = false;
        return -1;
      } else {
        is_inside_map = true;
      }
    }

    // collision result show
    if (0) {
      cv::Mat temp_mat;
      (*image).copyTo(temp_mat);
      double resolution = this->internal_grid_.maps.getResolution();
      for (const auto &iter : footprint) {
          Vector2D<double> gridmap(iter.position.x, iter.position.y);
          cv::Point opencv(sas_element::GridmapToOpencv(gridmap, width_, height_).x / resolution,
                           sas_element::GridmapToOpencv(gridmap, width_, height_).y / resolution);
            cv::circle(temp_mat, opencv, 0, cv::Scalar(0, 255, 0));
      }
      cv::imshow(window_name_, temp_mat);
      cv::waitKey(-1);
    }

    // only consider valid car state
    // collision check for every circle range in current state
    // notice : getObstaceDistance -> out of range
    for (int j = 0; (j < footprint.size()) && (true == is_inside_map); j++) {
      auto clear = internal_grid_.getObstacleDistance(
              grid_map::Position(footprint[j].position.x, footprint[j].position.y));
      if (clear < footprint[j].r) return -1;
      else {
        clear_cost += clear - footprint[j].r;
      }
    }
  }
  return clear_cost;
}

double SasExplore::ClearanceCostInPathUseCircumcircle(const sas_element::MotionPrimitiveState &primitive_state,
                                                      const sas_element::PrimitiveNode &node, cv::Mat *image) {
  double rad_step = primitive_state.delta_heading_radian / primitive_state.primitive_path.size();
  double start_rad = primitive_state.beginning_heading_index * motion_primitives_.m_min_orientation_angle;
  double k = 1 / node.current_zoom;  // unit arc length sampling
  double clear_cost = 0;

  // collision detection for every state(unit arc length) in primitive_path of a motion primitive
  for (int i = primitive_state.primitive_path.size() - 1; i >= 0; i -= k) {
    double x = node.parentnode->vehicle_center_position.x + primitive_state.primitive_path[i].x * node.current_zoom;
    double y = node.parentnode->vehicle_center_position.y + primitive_state.primitive_path[i].y * node.current_zoom;
    double rad = start_rad + rad_step * i;
    if (-1 == node.movement) {
      rad += M_PI;
    }
    if (rad > 2 * M_PI) {
      rad -= 2 * M_PI;
    } else if (rad < 0) {
      rad += 2 * M_PI;
    }

    hmpl::State current_state;
    current_state.x = x;
    current_state.y = y;
    current_state.z = rad;

    hmpl::Circle bounding_circle;
    bounding_circle = car_body_.circle_fill_car.getBoundingCircle(current_state);

    if (!internal_grid_.maps.isInside(grid_map::Position(x, y))) {
      return -1;
    }
    grid_map::Position pos(bounding_circle.position.x, bounding_circle.position.y);

    double clearance;
    std::vector<hmpl::Circle> footprint;
    if (!internal_grid_.maps.isInside(pos)) {
      return -1;
    } else {
      clearance = internal_grid_.getObstacleDistance(pos);
      if(clearance < bounding_circle.r) {
        // circumcircle checking result isn't collision-free, then do overlay
        // muti-circles collision checking
          footprint = car_body_.circle_fill_car.getCurrentCenters(current_state);
          for (const auto &iter : footprint) {
            grid_map::Position pos(iter.position.x, iter.position.y);
            if (internal_grid_.maps.isInside(pos)) {
                auto clear = internal_grid_.getObstacleDistance(pos);
                if (clear < iter.r) return -1;
                else {
                  clear_cost += clear - iter.r;
                }
              } else { // out of map
              return -1;
            }
          }
      } else {  // collision-free state
        clear_cost += clearance - bounding_circle.r;
      }
    }
    // collision result show
    if (0) {
      double resolution = this->internal_grid_.maps.getResolution();
      if(clearance < bounding_circle.r) { // show overlay multi-circle
        for (const auto &iter : footprint) {
          Vector2D<double> gridmap(iter.position.x, iter.position.y);
          cv::Point opencv(sas_element::GridmapToOpencv(gridmap, width_, height_).x / resolution,
                           sas_element::GridmapToOpencv(gridmap, width_, height_).y / resolution);
          cv::circle(*image, opencv, 0, cv::Scalar(0, 255, 0));
        }
      } else { // only show circumcircle
        Vector2D<double> gridmap(bounding_circle.position.x, bounding_circle.position.y);
        cv::Point opencv(sas_element::GridmapToOpencv(gridmap, width_, height_).x / resolution,
                         sas_element::GridmapToOpencv(gridmap, width_, height_).y / resolution);
        cv::circle(*image, opencv, bounding_circle.r / resolution , cv::Scalar(255, 0, 0));
      }
      cv::imshow(window_name_, *image);
      cv::waitKey(-1);
    }
  }
  return clear_cost;
}

void SasExplore::SetNodeInEffectiveZone(const sas_element::PrimitiveNode &node,
                                        double effective_zone) {
  int m = node.vehicle_center_position.x;
  int n = node.vehicle_center_position.y;
  if (effective_zone < 0) {
    effective_zone = 0;
  }
  // make search space scale into smaller one,fasten to find cost g or rank cost g
  for (int i = m - effective_zone; i <= m + effective_zone; i++) {
    for (int j = n - effective_zone; j <= n + effective_zone; j++) {
      if (!internal_grid_.maps.isInside(grid_map::Position(i, j))) {
        continue;
      }
      // calculate (x,y)position in distance look-up table
      int x = i - m + max_primitive_length_;
      int y = j - n + max_primitive_length_;
      //  calculate (a,b)position in searched_scale_map
      int a = MAX(0,MIN(i + width_ / 2, width_ - 1));
      int b = MAX(0,MIN(j + height_ / 2, height_ - 1));
      if (distance_mask[y][x] <= effective_zone) {
        if (node.g_cost < search_scale_map_[node.primitive_node_state.beginning_heading_index % direction_fraction_][b][a]) {
          search_scale_map_[node.primitive_node_state.beginning_heading_index % direction_fraction_][b][a] = node.g_cost;
        }
      }
    }
  }
}

//
double SasExplore::SetZoomFactorWithEffectiveZone(sas_element::PrimitiveNode *node, double dis2goal) {
  // obtain dis to obstacle from clearance map
  grid_map::Position temp_pos(node->vehicle_center_position.x, node->vehicle_center_position.y);
  double clearance = internal_grid_.getObstacleDistance(temp_pos);

  double effective_zone = clearance * coeff_distance_to_obstacle_;
  effective_zone = MIN(effective_zone, dis2goal * coeff_distance_to_goal_);
  //
  double length = max_primitive_length_ - effective_zone_scale_factor_;
  // limit the max
  effective_zone = MIN(effective_zone, length);
  node->sub_node_zoom = (effective_zone + effective_zone_scale_factor_) / max_primitive_length_;
  double min_zoom = min_primitive_length_ / max_primitive_length_;
  if(node->sub_node_zoom < min_zoom)  node->sub_node_zoom = min_zoom;
  ROS_DEBUG_STREAM("SetScale:current node:  " << clearance << " x: "
                                              << node->vehicle_center_position.x << " y: "
                                              << node->vehicle_center_position.y << " g: "
                                              << node->g_cost << " zone: "
                                              << effective_zone << " zoom: "
                                              << node->sub_node_zoom);
  return effective_zone;
}

void SasExplore::RemoveAllCNodes() {

  // clear the current open set
  while (!best_open_.empty()) {
    best_open_.pop();
  }
  while (!closed_.empty()) {
    // remove from the vector
    closed_.pop_back();
  }
}

void SasExplore::ResetSearchMap() {
  for (int z = 0; z < direction_fraction_; z++) {
    for (int y = 0; y < height_; y++) {
      for (int j = 0; j < width_; j++) {
        search_scale_map_[z][y][j] = std::numeric_limits<double>::infinity();
      }
    }
  }
}


void SasExplore::DrawStartAndEnd(cv::Mat *image,
                                 const Vector2D<double> &start,
                                 const Vector2D<double> &end) {
  if (this->display_cv_) {
    // start point
    cv::Point start_pt(sas_element::GridmapToOpencv(start, width_, height_).x / this->internal_grid_.maps.getResolution() ,
                       sas_element::GridmapToOpencv(start, width_, height_).y / this->internal_grid_.maps.getResolution());
    cv::circle(*image, start_pt, 5, cv::Scalar(0, 255, 255), -1, 8);
    // end point
    cv::Point end_pt(sas_element::GridmapToOpencv(end, width_, height_).x / this->internal_grid_.maps.getResolution() ,
                     sas_element::GridmapToOpencv(end, width_, height_).y / this->internal_grid_.maps.getResolution());
    cv::circle(*image, end_pt, 5, cv::Scalar(255, 255, 0), -1, 8);
  }
}

void SasExplore::RebuildPath(cv::Mat *image, const sas_element::PrimitiveNode &pn) {
  // ending node is goal node, straight line links goal node and last node
  path_.clear();
  sas_element::PrimitiveNode tmp = *pn.parentnode;
  //    CvScalar color = cv::Scalar(rand() % 200, rand() % 200, rand() % 200);
  CvScalar color = cv::Scalar(0, 0, 0);

  sas_element::MotionPrimitiveState *primitive_state_ptr = nullptr;
  State2D temp_state2d;
  while (nullptr != tmp.parentnode) {
    if (1 == tmp.movement) {
      // notice end segament is a line not a curve
      primitive_state_ptr = motion_primitives_.GetStateOfPrimitive(
              tmp.parentnode->primitive_node_state.beginning_heading_index, tmp.primitive_node_state.steering_index);
    } else {
      primitive_state_ptr = motion_primitives_.GetStateOfPrimitive(
              (tmp.parentnode->primitive_node_state.beginning_heading_index + direction_fraction_ / 2) %
              direction_fraction_, tmp.primitive_node_state.steering_index);
    }

    double k = 1 / tmp.current_zoom;
    for (int i = primitive_state_ptr->primitive_path.size() - 1; i > 0; i -= k) {
      temp_state2d.position.x =
              tmp.parentnode->vehicle_center_position.x + primitive_state_ptr->primitive_path[i].x * tmp.current_zoom;
      temp_state2d.position.y =
              tmp.parentnode->vehicle_center_position.y + primitive_state_ptr->primitive_path[i].y * tmp.current_zoom;
      // x axle right dir : Zero  counter clockwise : increase [0,2*pi]
      temp_state2d.orientation = tmp.parentnode->primitive_node_state.beginning_heading_index *
                                 motion_primitives_.m_min_orientation_angle +
                                 tmp.parentnode->primitive_node_state.delta_heading_radian *
                                 (primitive_state_ptr->primitive_path.size() - 1 - i) /
                                 (primitive_state_ptr->primitive_path.size() - 1);
      // positive : left steer
      // negitive : right steer
      temp_state2d.phi = tmp.primitive_node_state.steering_angle_radian;

      path_.push_back(temp_state2d);
      if (display_cv_) {
        Vector2D<double> gridmap(temp_state2d.position.x, temp_state2d.position.y);
        cv::Point state_pt(
                sas_element::GridmapToOpencv(gridmap, width_, height_).x / this->internal_grid_.maps.getResolution(),
                sas_element::GridmapToOpencv(gridmap, width_, height_).y / this->internal_grid_.maps.getResolution());
        if (i == primitive_state_ptr->primitive_path.size() - 1) {
          int font = cv::FONT_HERSHEY_SIMPLEX;
          cv::circle(*image, state_pt, 4, cv::Scalar(0, 255, 0));
          cv::putText(*image, std::to_string(tmp.index), state_pt, font, 0.4, (0, 255, 0), 2, cv::LINE_AA);
        } else {
          cv::circle(*image, state_pt, 1, color);
        }
      }
    }
    tmp = *(tmp.parentnode);
  }
  std::cout << "PATH SIZE:" << path_.size() << '\n';
  if (display_cv_) {
    cv::imshow(window_name_, *image);
    cv::waitKey(-1);
  }
}

void SasExplore::UpdateParams(sas_space_explore::SearchParameters parameters) {
  cost_factor_.steer_change_cost = parameters.steer_change_cost;
  cost_factor_.collision_free = parameters.collision_free;
  cost_factor_.reverse_cost = parameters.reverse_cost;
  cost_factor_.heuristic_cost = parameters.heuristic_cost;
  coeff_distance_to_goal_ = parameters.coeff_distance_to_goal;
  coeff_distance_to_obstacle_ = parameters.coeff_distance_to_obstacle;
  enable_drive_backward_ = parameters.enable_drive_backward;
  guarantee_heading_ = parameters.guarantee_heading;
  // whether to show search process by picture or not
  if(false == parameters.show_flag) {
      if (true == display_cv_) {
          cv::destroyWindow(window_name_);
          display_cv_ = false;
      }
  } else {
    if(false == display_cv_) {
      display_cv_ = true;
      cv::namedWindow(window_name_, 0);

    }
  }


}

} // namespace hmpl




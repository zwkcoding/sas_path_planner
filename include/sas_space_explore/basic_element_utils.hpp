/*
 * sas_explore.hpp
 *
 *  Created on: Oct 22, 2017
 *      Author: kevin
 */

#ifndef INCLUDE_SAS_SPACE_EXPLORE_BASIC_ELEMENT_UTILS_HPP_
#define INCLUDE_SAS_SPACE_EXPLORE_BASIC_ELEMENT_UTILS_HPP_

#include <iostream>
#include <list>
#include <vector>
#include <cmath>

#include <opt_utils/pose2d.hpp>
#include <opt_utils/circle.hpp>
#include <opt_utils/vector2d.hpp>
#include <opt_utils/base.hpp>
#include <car_model/car_geometry.hpp>

namespace hmpl {

/**
 * basic element and methods used in sas algorithm
 *
 */
namespace sas_element {

/**
 * 2D counter clockwise angle degrees rotation of point around center point
 * @param angle rotation angle
 * @param point point to rotate
 * @param center rotate pivot point
 * @return translated point
 */
 // note: key inline [ODR]
 // todo replace with RotateZ in <vector2d.hpp>
inline Vector2D<double> RotateWithCenter(double angle, const Vector2D<double> &point,
                        const Vector2D<double> &center) {
    return Vector2D<double>(center.x + (point.x - center.x) * cos(angle) - (point.y - center.y) * sin(angle),
                            center.y + (point.y - center.y) * cos(angle) + (point.x - center.x) * sin(angle));
}

/** coordination convertion
 *
 * @param gridmap coordinate of point in gridmap
 * @param cols cols of gridmap by getLength().x()
 * @param rows rows of gridmap by getLength().y()
 * @return coordinate of point in opencv format
 */
inline Vector2D<double> GridmapToOpencv(const Vector2D<double> &gridmap,
                                         int cols, int rows) {
    State gridmap_coordinate,cvimage_coordinate, cvimage_origin;
    cvimage_origin.x = cols / 2;
    cvimage_origin.y = rows / 2;
    // rotation angle
    cvimage_origin.z = -M_PI / 2;
    // point in gridmap
    gridmap_coordinate.x = gridmap.x;
    gridmap_coordinate.y = gridmap.y;
    // 2D coordinate system rotation and translation operation
    cvimage_coordinate = globalToLocal(cvimage_origin, gridmap_coordinate);
    return Vector2D<double>(cvimage_coordinate.x, -cvimage_coordinate.y);
}

struct CarParameter {
    double wheelbase{};
    double track{};
    double max_steer{};
    double end_to_center{};
    double front_to_center{};
    double wide{};
    double minimum_turning_radius{};
    Vector2D<double> base_point{};
    CarGeometry circle_fill_car{};
};

//
struct CostFactorOfPath {
    // collision cost
    double collision_free{};
    // cost of changed steer angle
    double steer_change_cost{};
    // cost of current steer angle
    double steer_cost{};
    // cost of backward path length
    double backward_cost{};
    // cost of forward path length
    double forward_cost{};
    // cost of reversed driving direction
    double reverse_cost{};
    // cost of heuristic
    double heuristic_cost{};
};

// this struct represents state of the motion primitive
struct MotionPrimitiveState {
    // steering radius of the related motion primitive
    double steering_radius{};
    // steering angel of the related motion primitive
    double steering_angle_radian{};
    // arc length of motion primitive_path
    double arc_length{};
    // changed heading angle of the related motion primitive
    double delta_heading_radian{};
    // index of heading angle of starting point in the corresponding motion primitive
    int beginning_heading_index{};
    // index of steering angle of corresponding motion primitive
    int steering_index{};
    // index of heading angle of ending point in the related motion primitive
    int ending_heading_index{};
    // steering center position of the related motion primitive
    Vector2D<double> steering_center_position{};
    // ending point position of the related motion primitive
    Vector2D<double> ending_position{};
    // curve of motion primitive
    std::vector<Vector2D<double> > primitive_path{};
};

// defined node to be used in A star search algorithm
// node consists of costs, car_state, ID, scalar...
class PrimitiveNode {
 public:
    // vehicle center position of current node
    Vector2D<double> vehicle_center_position{};
    // 1 : forward -1 : backward
    char movement;
    // ID number of a node in the search tree
    int index;
    double g_cost;
    double f_cost;
    // zoom coefficient of primitive curve linking next node and current node
    double sub_node_zoom;
    // zoom coefficient of primitive curve linking current node and last node
    double current_zoom;
    MotionPrimitiveState primitive_node_state;
    // parent node
    PrimitiveNode *parentnode;

    // (default) constructor
    PrimitiveNode()
            : index(0),
              movement(1),
              g_cost(0),
              f_cost(0),
              sub_node_zoom(0),
              current_zoom(0),
              vehicle_center_position{},
              primitive_node_state{},
              parentnode(nullptr) {
    }
    // parameterized  constructor
    PrimitiveNode(const MotionPrimitiveState &node_state,
                  const Vector2D<double> &node_position,
                  double g, double f)
            : index(0),
              sub_node_zoom(1.0),
              current_zoom(1.0),
              g_cost(g),
              f_cost(f),
              vehicle_center_position(node_position),
              parentnode(nullptr){
        primitive_node_state = node_state;
    }
    // the copy constructor
    PrimitiveNode(const PrimitiveNode &cn)
            : index(cn.index),
              movement(cn.movement),
              vehicle_center_position(cn.vehicle_center_position),
              sub_node_zoom(cn.sub_node_zoom),
              current_zoom(cn.current_zoom),
              g_cost(cn.g_cost),
              f_cost(cn.f_cost) {
        primitive_node_state = cn.primitive_node_state;
        // take a "depy" copy of the pointer
        if(nullptr != cn.parentnode) {
            parentnode = new PrimitiveNode(*cn.parentnode);
        } else {
            parentnode = nullptr;
        }
    }

    // destructor
    ~PrimitiveNode() {
        if(nullptr != parentnode) {
            delete parentnode;
        }
    }

    // no throwing
    void swap(PrimitiveNode& first, PrimitiveNode& second) {

        // swap won't work with my non pointer class
        MotionPrimitiveState temp = second.primitive_node_state;
        first.primitive_node_state = second.primitive_node_state;
        second.primitive_node_state = temp;
        //
        std::swap(first.movement, second.movement);
        std::swap(first.vehicle_center_position, second.vehicle_center_position);
        std::swap(first.index, second.index);
        std::swap(first.f_cost, second.f_cost);
        std::swap(first.g_cost, second.g_cost);
        std::swap(first.current_zoom, second.current_zoom);
        std::swap(first.sub_node_zoom, second.sub_node_zoom);
        std::swap(first.parentnode, second.parentnode);
    }

    // obey copy-and-swap idom
    // custome assignment operator
    PrimitiveNode& operator=(PrimitiveNode other) {
        swap(*this, other);
        return *this;
    }
};

} // namespace sas_element
} // namespace hmpl
#endif /* INCLUDE_SAS_SPACE_EXPLORE_BASIC_ELEMENT_UTILS_HPP_ */

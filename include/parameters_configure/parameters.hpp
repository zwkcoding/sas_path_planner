//
// Created by kevin on 12/14/17.
//

#ifndef SAS_SPACE_EXPLORE_PARAMETERS_HPP
#define SAS_SPACE_EXPLORE_PARAMETERS_HPP

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include "sas_space_explore/exploreParameters.h"

namespace sas_space_explore {
struct SearchParameters {
    // numbers of heading solution angle
    int direction_num{};
    // numbers of steering solution angle
    int steer_num{};
    // collision cost
    double collision_free{};
    // cost of changed steer angle
    double steer_change_cost{};
    // cost of backward path length
    double backward_cost{};
    // cost of forward path length
    double forward_cost{};
    // cost of reversed driving direction
    double reverse_cost{};
    // cost of heuristic item
    double heuristic_cost{};
    // coefficients about distance with closest obstacle
    double coeff_distance_to_obstacle{};
    // coefficients about distance with goal
    double coeff_distance_to_goal{};
    // enable reverse motion primitive
    bool enable_drive_backward{false};
    // show flag
    bool show_flag{false};
    // enable goal heading search
    bool guarantee_heading{false};
    double start_pose_x{};
    double start_pose_y{};
    double start_pose_theta{};
    double goal_pose_x{};
    double goal_pose_y{};
    double goal_pose_theta{};

};
class ParametersTransfer {
public:
    ParametersTransfer(ros::NodeHandle);
    SearchParameters GetSearchParameter();
    void ResetUpdateFlag();
    bool CheckUpdateFlag();

private:

    SearchParameters search_factor_;
    exploreParameters params_;
    bool update_flag_;

    dynamic_reconfigure::Server<exploreConfig> reconfigSrv_; // Dynamic reconfiguration service

    void reconfigureRequest(exploreConfig &, uint32_t);
    void updateParams();
};

} // namespace sas_space_explore


#endif //SAS_SPACE_EXPLORE_PARAMETERS_HPP

//
// Created by kevin on 12/14/17.
//

#include "parameters_configure/parameters.hpp"

namespace sas_space_explore {

ParametersTransfer::ParametersTransfer(ros::NodeHandle private_node_handle)
        : reconfigSrv_{private_node_handle},
          params_{private_node_handle},
          update_flag_(false) {

    /**
     * Initialization
     */
    params_.fromParamServer();

    /**
     * Set up  dynamic reconfigure server
     */
    reconfigSrv_.setCallback(boost::bind(&ParametersTransfer::reconfigureRequest, this, _1, _2));

    // initialize params at first
    updateParams();

}

/**
* This callback is called whenever a change was made in the dynamic_reconfigure window
*/
void ParametersTransfer::reconfigureRequest(exploreConfig &config, uint32_t level) {
    params_.fromConfig(config);
    ROS_WARN_STREAM("Parameter update:\n" << params_);
    updateParams();

}

void ParametersTransfer::updateParams() {
    // guarantee once update call once value transfer.
    if(false == update_flag_) {
        search_factor_.steer_change_cost = params_.steer_change_cost;
        search_factor_.collision_free = params_.collision_free;
        search_factor_.reverse_cost = params_.reverse_cost;
        search_factor_.backward_cost = params_.backward_cost;
        search_factor_.forward_cost = params_.forward_cost;
        search_factor_.coeff_distance_to_goal = params_.coeff_distance_to_goal;
        search_factor_.coeff_distance_to_obstacle = params_.coeff_distance_to_obstacle;
        search_factor_.enable_drive_backward = params_.enable_drive_backward;
        search_factor_.guarantee_heading = params_.guarantee_heading;
        search_factor_.show_flag = params_.show_flag;
        search_factor_.heuristic_cost = params_.heuristic_cost;
        search_factor_.start_pose_x = params_.start_pose_x;
        search_factor_.start_pose_y = params_.start_pose_y;
        search_factor_.start_pose_theta = params_.start_pose_theta;
        search_factor_.goal_pose_x = params_.goal_pose_x;
        search_factor_.goal_pose_y = params_.goal_pose_y;
        search_factor_.goal_pose_theta = params_.goal_pose_theta;
        search_factor_.direction_num = params_.direction_num;
        search_factor_.steer_num = params_.steer_num;
        update_flag_ = true;
    }

}

SearchParameters ParametersTransfer::GetSearchParameter() {
    return search_factor_;
}

void ParametersTransfer::ResetUpdateFlag() {
    update_flag_ = false;
}

bool ParametersTransfer::CheckUpdateFlag() {
    return update_flag_;
}

} // namespace sas_space_explore


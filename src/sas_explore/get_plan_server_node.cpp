//
// Created by kevin on 4/23/18.
//


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opt_utils/opt_utils.hpp>
#include <iv_explore_msgs/GetAstarPlan.h>
#include <control_msgs/Traj_Node.h>
#include <control_msgs/Trajectory.h>
#include "parameters_configure/parameters.hpp"
#include <opt_utils/opt_utils.hpp>

#include "sas_space_explore/sas_explore.hpp"
using namespace ros;

boost::shared_ptr<hmpl::SasExplore> sas_planner_ptr;
boost::shared_ptr<sas_space_explore::ParametersTransfer> parameters_transfer_ptr;
boost::shared_ptr<hmpl::InternalGridMap> igm_ptr;
boost::shared_ptr<tf::TransformListener> tf_listener_ptr;

ros::Publisher path_pub, debug_pose_pub, footprint_pub;
std::string receive_planner_map_topic_name, global_map_frame_name, local_map_frame_name, abso_global_map_frame_name;
nav_msgs::OccupancyGrid planner_map;
bool map_set_ = false;

void  mapCallback(const nav_msgs::OccupancyGridConstPtr &msg) {
    planner_map = *msg;
    map_set_ = true;

}

bool localPlanCallback(iv_explore_msgs::GetAstarPlan::Request &req, iv_explore_msgs::GetAstarPlan::Response &res) {
    if(true == map_set_) {
        if(parameters_transfer_ptr->CheckUpdateFlag()) {
            // start update search algorithm's parameters
            sas_planner_ptr->UpdateParams(parameters_transfer_ptr->GetSearchParameter());
            parameters_transfer_ptr->ResetUpdateFlag();
        }

        // Initialize gridmap with ogm (all is same)
        grid_map::GridMapRosConverter::fromOccupancyGrid(planner_map, igm_ptr->obs, igm_ptr->maps);
        // value replacement
        grid_map::Matrix& grid_data = igm_ptr->maps[igm_ptr->obs];
        size_t size_x = igm_ptr->maps.getSize()(0);
        size_t size_y = igm_ptr->maps.getSize()(1);
        // pre-process
        // 0 : obstacle
        // 255 : free/unknown
        for (size_t idx_x = 0; idx_x < size_x; ++idx_x){
            for (size_t idx_y = 0; idx_y < size_y; ++idx_y) {
                if (0.0 == grid_data(idx_x, idx_y)) {
                    grid_data(idx_x, idx_y) = igm_ptr->FREE;
                } else if(100.0 == grid_data(idx_x, idx_y)) {
                    grid_data(idx_x, idx_y) = igm_ptr->OCCUPY;
                } else {
//                    grid_data(idx_x, idx_y) = igm_.OCCUPY;
                    // warn : view unknown as free
                    grid_data(idx_x, idx_y) = igm_ptr->FREE;
                }
            }
        }
        igm_ptr->updateDistanceLayerCV();

        // search through the environment to get a path
        auto start = std::chrono::steady_clock::now();
        hmpl::Pose2D start_pose, goal_pose;
        start_pose.position.x = req.start.position.x;
        start_pose.position.y = req.start.position.y;
        start_pose.orientation = tf::getYaw(req.start.orientation);
        goal_pose.position.x = req.goal.position.x;
        goal_pose.position.y = req.goal.position.y;
        goal_pose.orientation = tf::getYaw(req.goal.orientation);
        bool search_result =  sas_planner_ptr->StartSearchFromMap(*igm_ptr, start_pose, goal_pose);

        auto end = std::chrono::steady_clock::now();
        double elapsed_secondes =
                std::chrono::duration_cast<std::chrono::duration<double> >(
                        end - start)
                        .count();

        std::cout << "search time:" << elapsed_secondes << std::endl;
        if (search_result) {
            nav_msgs::Path path_msg;
            geometry_msgs::PoseStamped pose;
            path_msg.header.frame_id = igm_ptr->maps.getFrameId();
            path_msg.header.stamp = ros::Time::now();
            pose.header = path_msg.header;
            std::vector<hmpl::State2D> path = sas_planner_ptr->GetPath();
            for (auto &point_itr : path) {
                pose.pose.position.x = point_itr.position.x;
                pose.pose.position.y = point_itr.position.y;
                if(hmpl::ForwardGear == point_itr.gear)
                pose.pose.position.z = 1;
                else
                pose.pose.position.z = -1;

                path_msg.poses.push_back(pose);
            }
            path_pub.publish(path_msg);
            control_msgs::Traj_Node path_node;
            control_msgs::Trajectory trajectory;

            tf::StampedTransform world2map;
            try
            {
                tf_listener_ptr->lookupTransform(abso_global_map_frame_name, global_map_frame_name, ros::Time(0), world2map);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
//                    return;
            }

            trajectory.header.frame_id = global_map_frame_name;
            trajectory.header.stamp = ros::Time::now();
            for (int i = 0; i < path_msg.poses.size(); i++) {
                // decide forward before world2map
                if (path_msg.poses[i].pose.position.z == -1) {
                    path_node.forward = 0;  // 0 --> back
                } else {
                    path_node.forward = 1; // 1 --> forward
                }
                path_msg.poses[i].pose = hmpl::transformPose(path_msg.poses[i].pose, world2map);
                path_node.position.x = path_msg.poses[i].pose.position.x;
                path_node.position.y = path_msg.poses[i].pose.position.y;
                path_node.velocity.linear.x = 2.5;

                trajectory.points.push_back(path_node);
            }

            ROS_INFO_THROTTLE(3, "Path node size is %d", trajectory.points.size());
            res.path = trajectory;
            res.status_code = 0;

        } else {
            ROS_INFO("Fail to find GOAL!");
            // 0 --> success; 1 --> invaid pose(out of map); 2 --> invaid pose(within obstacle); 3 --> time exceed
            res.status_code = sas_planner_ptr->getStatusCode();
            ROS_INFO("Planner Fail status code : %d", res.status_code);

        }
        return true;
    } else {
        ROS_WARN("No planning maps received!");
        return false;
    }
}


int main(int argc, char **argv) {
    init(argc, argv, "get_plan_server");
    ros::NodeHandle private_nh_("~");
    ros::NodeHandle nh_;

//    pListener = new (tf::TransformListener);
    sas_planner_ptr.reset(new hmpl::SasExplore());
    parameters_transfer_ptr.reset((new sas_space_explore::ParametersTransfer(private_nh_)));
    igm_ptr.reset(new hmpl::InternalGridMap());
    tf_listener_ptr.reset(new tf::TransformListener);

    private_nh_.param<std::string>("receive_planner_map_topic_name", receive_planner_map_topic_name, "/global_map");
    private_nh_.param<std::string>("global_map_frame_name", global_map_frame_name, "/odom");
    private_nh_.param<std::string>("local_map_frame_name", local_map_frame_name, "base_link");
    private_nh_.param<std::string>("abso_global_map_frame_name", abso_global_map_frame_name, "/abso_odom");

    // ROS subscribers
    ros::Subscriber map_sub = nh_.subscribe(receive_planner_map_topic_name, 1, mapCallback);
    ServiceServer local_map_srv = nh_.advertiseService("get_plan", localPlanCallback);
//
    path_pub = nh_.advertise<nav_msgs::Path>(nh_.getNamespace() + "global_path_show", 1, false);
//    debug_pose_pub = nh_.advertise<geometry_msgs::PoseArray>(nh_.getNamespace() + "debug_pose_array", 1, false);
//    footprint_pub = nh_.advertise<visualization_msgs::MarkerArray>(nh_.getNamespace() + "astar_footprint", 1, false);

    spin();
    return 0;

}

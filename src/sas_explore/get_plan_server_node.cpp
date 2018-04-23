//
// Created by kevin on 4/23/18.
//


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opt_utils/opt_utils.hpp>

#include "sas_space_explore/sas_explore.hpp"
using namespace ros;

boost::shared_ptr<hmpl::SasExplore> sas_planner_ptr;

ros::Publisher path_pub, debug_pose_pub, footprint_pub;
std::string receive_planner_map_topic_name, global_map_frame_name, local_map_frame_name, abso_global_map_frame_name;

void  mapCallback(const nav_msgs::OccupancyGridConstPtr &msg) {

}



int main(int argc, char **argv) {
    init(argc, argv, "get_plan_server");
    ros::NodeHandle private_nh_("~");
    ros::NodeHandle nh_;

//    pListener = new (tf::TransformListener);
    sas_planner_ptr.reset(new hmpl::SasExplore());

    private_nh_.param<std::string>("receive_planner_map_topic_name", receive_planner_map_topic_name, "/local_map");
    private_nh_.param<std::string>("global_map_frame_name", global_map_frame_name, "/odom");
    private_nh_.param<std::string>("local_map_frame_name", local_map_frame_name, "base_link");
    private_nh_.param<std::string>("abso_global_map_frame_name", abso_global_map_frame_name, "/abso_odom");

    // ROS subscribers
    ros::Subscriber map_sub = nh_.subscribe(receive_planner_map_topic_name, 1, mapCallback);
//    ServiceServer local_map_srv = nh_.advertiseService("get_plan", localPlanCallback);
//
//    path_pub = nh_.advertise<nav_msgs::Path>(nh_.getNamespace() + "global_path_show", 1, false);
//    debug_pose_pub = nh_.advertise<geometry_msgs::PoseArray>(nh_.getNamespace() + "debug_pose_array", 1, false);
//    footprint_pub = nh_.advertise<visualization_msgs::MarkerArray>(nh_.getNamespace() + "astar_footprint", 1, false);

    spin();
    return 0;

}

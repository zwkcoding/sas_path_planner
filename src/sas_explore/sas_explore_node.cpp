/** @file sas_explore_node.cpp
 *  @brief ros framework consist of publisher and subscriber
 *
 *  This contains the image format imput, img format converse into
 *  gridmap,then converse into occupied grid that comply to ros,
 *  define two publisher node,one is to show map,the other show
 *  sas result path; and two subscriber node to receive
 *  initial and goal pose respectively;
 *
 *  @author Wenkai Zhou
 *  @bug
 *
 *
 */
#include "sas_space_explore/sas_explore.hpp"
#include "parameters_configure/parameters.hpp"

#include <ros/ros.h>
#include <chrono>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include <opt_utils/utils.hpp>

hmpl::Pose2D start_point(-14.6, -9.7, 0);
hmpl::Pose2D end_point(14.8, -8.9, 0.28);

void startCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start) {
    start_point.position.x = start->pose.pose.position.x;
    start_point.position.y = start->pose.pose.position.y;
    start_point.orientation = tf::getYaw(start->pose.pose.orientation);
    if(start_point.orientation < 0) {
        start_point.orientation += 2 * M_PI;
    }
}

void goalCb(const geometry_msgs::PoseStampedConstPtr &goal) {
    end_point.position.x = goal->pose.position.x;
    end_point.position.y = goal->pose.position.y;
    end_point.orientation = tf::getYaw(goal->pose.orientation);
    if(end_point.orientation < 0) {
        end_point.orientation += 2 * M_PI;
    }
}

int main(int argc, char **argv) {
    // Initialize the node, publishers and subscribers.
    ros::init(argc, argv, "sas_space_explore_node");
    ros::NodeHandle nh("~");
    std::string package_dir = ros::package::getPath("sas_space_explore");
    std::string base_dir = "/benchmark_file/";
    std::string img_dir = "scene_map";
    int scene_num = 4;
    nh.param("scene", scene_num, scene_num);
    cv::Mat img_src = cv::imread(package_dir + base_dir + img_dir + std::to_string(scene_num) + ".jpg", CV_8UC1);

    // new dynamic_parameter client
    sas_space_explore::ParametersTransfer parameters_transfer(nh);

    double resolution = 0.2;  // in meter
    static int last_direction_num = 32;
    static int last_steer_num = 32;
    nh.param("map_resolution", resolution, resolution);
    hmpl::InternalGridMap in_gm;
    in_gm.initializeFromImage(img_src, resolution, grid_map::Position::Zero());
    in_gm.addObstacleLayerFromImage(img_src, 0.5);
    in_gm.updateDistanceLayer();

    bool display_flag = false;
    hmpl::SasExplore sas_planner(in_gm, display_flag);
    // Initialize sas planner
    sas_planner.InitPlanner(last_direction_num, last_steer_num);

    in_gm.maps.setFrameId("/map");
    ROS_INFO("Created map with size %f x %f m (%i x %i cells), map resolution is %f",
             in_gm.maps.getLength().x(), in_gm.maps.getLength().y(),
             in_gm.maps.getSize()(0), in_gm.maps.getSize()(1), in_gm.maps.getResolution());
    // Create publishers and subscribers
    ros::Publisher path_publisher =
            nh.advertise<nav_msgs::Path>("result_path", 1, true);
    ros::Publisher publisher =
            nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);

    // callback function for start and goal
    ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, startCb);
    ros::Subscriber end_sub = nh.subscribe("/move_base_simple/goal", 1, goalCb);

    // Publisher in a loop.
    ros::Rate rate(10.0);
    while (nh.ok()) {
        // Wait for next cycle.
        ros::spinOnce();
        // Add data to grid map.
        ros::Time time = ros::Time::now();

        // publish the grid_map
        in_gm.maps.setTimestamp(time.toNSec());
        nav_msgs::OccupancyGrid message;
        grid_map::GridMapRosConverter::toOccupancyGrid(
               in_gm.maps, in_gm.obs, in_gm.FREE, in_gm.OCCUPY, message);

        if(parameters_transfer.CheckUpdateFlag()) {
            // start update search algorithm's parameters
            sas_planner.UpdateParams(parameters_transfer.GetSearchParameter());
            parameters_transfer.ResetUpdateFlag();
            start_point.position.x = parameters_transfer.GetSearchParameter().start_pose_x;
            start_point.position.y = parameters_transfer.GetSearchParameter().start_pose_y;
            start_point.orientation = parameters_transfer.GetSearchParameter().start_pose_theta;
            end_point.position.x = parameters_transfer.GetSearchParameter().goal_pose_x;
            end_point.position.y = parameters_transfer.GetSearchParameter().goal_pose_y;
            end_point.orientation = parameters_transfer.GetSearchParameter().goal_pose_theta;
            int updated_direction_num = parameters_transfer.GetSearchParameter().direction_num;
            int updated_steer_num = parameters_transfer.GetSearchParameter().steer_num;
            if ( last_direction_num != updated_direction_num || last_steer_num != updated_steer_num) {
                last_direction_num = updated_direction_num;
                last_steer_num = updated_steer_num;
                sas_planner.InitPlanner(last_direction_num, last_steer_num);
            }
        }

        // search through the environment to get a path
        auto start = std::chrono::steady_clock::now();
        sas_planner.StartSearchFromMap(in_gm, start_point, end_point);

        auto end = std::chrono::steady_clock::now();
        double elapsed_secondes =
                std::chrono::duration_cast<std::chrono::duration<double> >(
                        end - start)
                        .count();

        std::cout << "search time:" << elapsed_secondes << std::endl;

        nav_msgs::Path path_msg;
        geometry_msgs::PoseStamped pose;
        path_msg.header.frame_id = in_gm.maps.getFrameId();
        path_msg.header.stamp = ros::Time::now();
        pose.header = path_msg.header;
        std::vector<hmpl::State2D> path = sas_planner.GetPath();

        // this is the space explore result path
        hmpl::CSVFile sas_path("sas_path.csv");
        sas_path << "s" << "x" << "y" << "z" << "k" << "dk" << hmpl::endrow;
        double wheelbase = 2.845;
        for (auto &point_itr : path) {
            pose.pose.position.x = point_itr.position.x;
            pose.pose.position.y = point_itr.position.y;
            pose.pose.position.z = 0;
            path_msg.poses.push_back(pose);
            double k = tan(point_itr.phi) / wheelbase ;
            sas_path << 0 << pose.pose.position.x
                     << pose.pose.position.y
                     << point_itr.orientation
                     << k << 0 << hmpl::endrow;
        }

        path_publisher.publish(path_msg);
        publisher.publish(message);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
                          message.header.stamp.toSec());
        rate.sleep();
    }
    return 0;
}

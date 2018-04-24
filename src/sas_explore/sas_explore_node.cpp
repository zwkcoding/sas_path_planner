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

#include <ros/package.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include <opt_utils/utils.hpp>

//#define USE_IMAGE

hmpl::Pose2D start_point(-14.6, -9.7, 0);
hmpl::Pose2D end_point(14.8, -8.9, 0.28);
nav_msgs::OccupancyGrid planner_map;
bool map_set_ = false, goal_set_ = false, start_set_ = false;

void startCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start) {
    start_point.position.x = start->pose.pose.position.x;
    start_point.position.y = start->pose.pose.position.y;
    start_point.orientation = tf::getYaw(start->pose.pose.orientation);
    if(start_point.orientation < 0) {
        start_point.orientation += 2 * M_PI;
    }
    start_set_ = true;

}

void goalCb(const geometry_msgs::PoseStampedConstPtr &goal) {
    end_point.position.x = goal->pose.position.x;
    end_point.position.y = goal->pose.position.y;
    end_point.orientation = tf::getYaw(goal->pose.orientation);
    if(end_point.orientation < 0) {
        end_point.orientation += 2 * M_PI;
    }
    goal_set_ = true;

}

void  mapCallback(const nav_msgs::OccupancyGridConstPtr &msg) {
    planner_map = *msg;
    map_set_ = true;
}


int main(int argc, char **argv) {
    // Initialize the node, publishers and subscribers.
    ros::init(argc, argv, "sas_space_explore_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
#ifdef USE_IMAGE
    std::string package_dir = ros::package::getPath("sas_space_explore");
    std::string base_dir = "/benchmark_file/";
    std::string img_dir = "scene_map";
    int scene_num = 4;
    nh.param("scene", scene_num, scene_num);
    cv::Mat img_src = cv::imread(package_dir + base_dir + img_dir + std::to_string(scene_num) + ".jpg", CV_8UC1);

    double resolution = 0.2;  // in meter
    nh.param("map_resolution", resolution, resolution);
    hmpl::InternalGridMap in_gm;
    in_gm.initializeFromImage(img_src, resolution, grid_map::Position::Zero());
    in_gm.addObstacleLayerFromImage(img_src, 0.5);
    in_gm.updateDistanceLayer();

    in_gm.maps.setFrameId("/map");
    ROS_INFO("Created map with size %f x %f m (%i x %i cells), map resolution is %f",
             in_gm.maps.getLength().x(), in_gm.maps.getLength().y(),
             in_gm.maps.getSize()(0), in_gm.maps.getSize()(1), in_gm.maps.getResolution());
    ros::Publisher publisher =
            n.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);
#endif
    // Create publishers and subscribers
    ros::Publisher path_publisher =
            n.advertise<nav_msgs::Path>("result_path", 1, true);
    ros::Subscriber map_sub = n.subscribe("/global_map", 1, mapCallback);
    // callback function for start and goal
    ros::Subscriber start_sub = n.subscribe("/initialpose", 1, startCb);
    ros::Subscriber end_sub = n.subscribe("/move_base_simple/goal", 1, goalCb);

    // new dynamic_parameter client
    sas_space_explore::ParametersTransfer parameters_transfer(nh);
    hmpl::SasExplore sas_planner;
    hmpl::InternalGridMap igm_;

    // Publisher in a loop.
    ros::Rate loop_rate(100.0);
    while (nh.ok()) {
        // Wait for next cycle.
        ros::spinOnce();
#ifdef USE_IMAGE
        // Add data to grid map.
        ros::Time time = ros::Time::now();
        // publish the grid_map
        in_gm.maps.setTimestamp(time.toNSec());
        nav_msgs::OccupancyGrid message;
        grid_map::GridMapRosConverter::toOccupancyGrid(
               in_gm.maps, in_gm.obs, in_gm.FREE, in_gm.OCCUPY, message);
        publisher.publish(message);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
                          message.header.stamp.toSec());
#endif

        if(parameters_transfer.CheckUpdateFlag()) {
            // start update search algorithm's parameters
            sas_planner.UpdateParams(parameters_transfer.GetSearchParameter());
            parameters_transfer.ResetUpdateFlag();
        }

        if (!map_set_ || !start_set_ || !goal_set_) {
            loop_rate.sleep();
            continue;
        }

        // Initialize gridmap with ogm (all is same)
        grid_map::GridMapRosConverter::fromOccupancyGrid(planner_map, igm_.obs, igm_.maps);
        // value replacement
        grid_map::Matrix& grid_data = igm_.maps[igm_.obs];
        size_t size_x = igm_.maps.getSize()(0);
        size_t size_y = igm_.maps.getSize()(1);
        // pre-process
        // 0 : obstacle
        // 255 : free/unknown
        for (size_t idx_x = 0; idx_x < size_x; ++idx_x){
            for (size_t idx_y = 0; idx_y < size_y; ++idx_y) {
                if (0.0 == grid_data(idx_x, idx_y)) {
                    grid_data(idx_x, idx_y) = igm_.FREE;
                } else if(100.0 == grid_data(idx_x, idx_y)) {
                    grid_data(idx_x, idx_y) = igm_.OCCUPY;
                } else {
//                    grid_data(idx_x, idx_y) = igm_.OCCUPY;
                    // warn : view unknown as free
                    grid_data(idx_x, idx_y) = igm_.FREE;
                }
            }
        }
        igm_.updateDistanceLayerCV();

        // search through the environment to get a path
        auto start = std::chrono::steady_clock::now();
        sas_planner.StartSearchFromMap(igm_, start_point, end_point);

        auto end = std::chrono::steady_clock::now();
        double elapsed_secondes =
                std::chrono::duration_cast<std::chrono::duration<double> >(
                        end - start)
                        .count();

        std::cout << "search time:" << elapsed_secondes << std::endl;

        nav_msgs::Path path_msg;
        geometry_msgs::PoseStamped pose;
        path_msg.header.frame_id = igm_.maps.getFrameId();
        path_msg.header.stamp = ros::Time::now();
        pose.header = path_msg.header;
        std::vector<hmpl::State2D> path = sas_planner.GetPath();

        // this is the space explore result path
//        hmpl::CSVFile sas_path("sas_path.csv");
//        sas_path << "s" << "x" << "y" << "z" << "k" << "dk" << hmpl::endrow;
        double wheelbase = 2.845;
        for (auto &point_itr : path) {
            pose.pose.position.x = point_itr.position.x;
            pose.pose.position.y = point_itr.position.y;
            pose.pose.position.z = 0;
            path_msg.poses.push_back(pose);
            double k = tan(point_itr.phi) / wheelbase ;
//            sas_path << 0 << pose.pose.position.x
//                     << pose.pose.position.y
//                     << point_itr.orientation
//                     << k << 0 << hmpl::endrow;
        }

        path_publisher.publish(path_msg);

        loop_rate.sleep();
    }
    return 0;
}

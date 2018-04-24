/*
 * sas_explore.hpp
 *
 *  Created on: Oct 22, 2017
 *      Author: kevin
 */

#ifndef INCLUDE_SAS_SPACE_EXPLORE_SAS_EXPLORE_HPP_
#define INCLUDE_SAS_SPACE_EXPLORE_SAS_EXPLORE_HPP_

#include "sas_space_explore/motion_primitive.hpp"
#include "sas_space_explore/basic_element_utils.hpp"

#include <list>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <opt_utils/circle.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include <opt_utils/utils.hpp>
#include <parameters_configure/parameters.hpp>


namespace hmpl {
    inline geometry_msgs::Pose transformPose(geometry_msgs::Pose &pose, tf::Transform &tf)
    {
        // Convert ROS pose to TF pose
        tf::Pose tf_pose;
        tf::poseMsgToTF(pose, tf_pose);

        // Transform pose
        tf_pose = tf * tf_pose;

        // normalize quaternion
        tf::Quaternion q = tf_pose.getRotation().normalize();
        tf_pose.setRotation(q);

        // Convert TF pose to ROS pose
        geometry_msgs::Pose ros_pose;
        tf::poseTFToMsg(tf_pose, ros_pose);
        return ros_pose;
    }

/*!
 * Space Adaptive Search(SAS) Approach for Nonholonomic Mobile
 * Robots Path Planning.
 */
class SasExplore {
 public:
    // basic constructor for eigen grid map
    SasExplore();
    // basic destructor
    ~SasExplore();

    /**
     * Entry of search algorithm with a new grid map, start and goal pose
     * @param gridmap grid map
     * @param start start pose
     * @param goal  goal pose
     */
    bool StartSearchFromMap(hmpl::InternalGridMap &gridmap, const Pose2D &start,
                            const Pose2D &goal);

    // return unit arc length sampling curves
    std::vector<hmpl::State2D> &GetPath() {
        return this->path_;
    };

    int getStatusCode() {return status_code_;}

    /**
     * update sas search algorithm parameters from parameters dynamic reconfigure module
     * @param parameters come from parameters dynamic reconfigure module
     */
    void UpdateParams(sas_space_explore::SearchParameters parameters);

private:
    // instance first
    hmpl::InternalGridMap internal_grid_;
    // planner status
    int status_code_;
    // whether display internal image
    bool display_cv_;
    // map resolution
    double effective_zone_scale_factor_;
    // the eigen grid map reference
    // the current start pose
    Pose2D start_;
    // the current goal pose
    Pose2D goal_;
    // map width(m)
    int width_;
    // map length(m)
    int height_;
    // coefficients about distance with closest obstacle
    double coeff_distance_to_obstacle_;
    // coefficients about distance with goal
    double coeff_distance_to_goal_;
    // window name of showing img
    std::string window_name_;
    // the value hold the best cost to reach goal
    double f_goal_;
    // whether search begin from goal pose
    // 0 :disable  1 : enable
    bool reversed_search_;
    // whether consider drive backward
    // 0: 1:
    bool enable_drive_backward_;
    // whether consider heading of goal
    bool guarantee_heading_;
    // search halt condition: distance from node to goal position
    double effective_threshold_dis_;
    // distance from car end to rear axle center
    double end_to_center_;
    // distance from car front to rear axle center
    double front_to_center_;
    // wheelbase
    double wheelbase_;
    // vehicle wide
    double wide_;
    // distance between front wheels
    double track_;
    // maximum steer angle (radian) of vehicle
    double max_steer_;
    // numbers of heading solution angles
    int direction_fraction_;
    // numbers of steering solution angles
    int steer_fraction_;
    // original mat data type of img
    cv::Mat display_;
    // data structure to show
    cv::Mat overlay_;
    // class representing physics parameters of vehicle and occupied circle pie
    // to used as collision checking
    sas_element::CarParameter car_body_;
    // structure of kinds of cost coefficients
    sas_element::CostFactorOfPath cost_factor_;
    // original/template motion primitive sets
    MotionPrimitiveSet motion_primitives_;
    // resulted path
    std::vector<hmpl::State2D> path_;
    // last resulted path
    std::vector<hmpl::State2D> last_path_;
    // define a comparator based on value
    struct PrimitiveNodeValueComparator {
        // operator overloading
        bool operator()(const sas_element::PrimitiveNode& lhs,
                        const sas_element::PrimitiveNode& rhs) {
            // the default c++ stl is a max heap, so we need to invert here
            return lhs.f_cost > rhs.f_cost;
        }
    };
    // the value sort open queue
    std::priority_queue<sas_element::PrimitiveNode,
            std::vector<sas_element::PrimitiveNode>,
            PrimitiveNodeValueComparator>
            best_open_;
    // the closed set
    std::vector<sas_element::PrimitiveNode> closed_;
    // 3D search map consists of g_cost
    double ***search_scale_map_;
    // distance look-up table, easy to find position of effective zone
    double **distance_mask;
    // max length of primitive curve: Zero steer primitive
    double max_primitive_length_;
    // min length of primitive curve: Zero steer
    double min_primitive_length_;
    /**
    * Initialize car parameter, motion primitive and so no
    * @param directions numbers of heading solution angle
    * @param steers  numbers of steering solution angle
    */
    void InitPlanner(int directions, int steers);
    /**
     * sas search algorithm
     * @return true: search success false: search fail
     */
    bool SasSearch();
    /**
     * check node whether is searched in efficient zone or not
     * @param pn current node
     * @return false : node is not searched, true: node is searched
     */
    bool NodeExistInSearchedEfficientZone(const sas_element::PrimitiveNode &pn);
    /**
     * Expand node
     * @param cpn current primitive node
     * @param gpn goal primitive node
     * @param image showing image
     * @return search results
     */
    bool ExpandNodeProcess(sas_element::PrimitiveNode &cpn,
                           sas_element::PrimitiveNode &gpn,
                           cv::Mat *image);
    /**
     * Clearance checking help function using overlay multi-circle method
     * @param primitive_state expanded trajectory
     * @param node expanded node
     * @param image showing image
     * @return expected expanded trajectory cost of path. if -1, then this curve is invaild.
     */
    double ClearanceCostInPathUseOverlayCirles(const sas_element::MotionPrimitiveState &primitive_state,
                                               const sas_element::PrimitiveNode &node, cv::Mat *image);
    /**
     * Clearance checking help function using circumcircle method
     * @param primitive_state expanded trajectory
     * @param node expanded node
     * @param image showing image
     * @return expected expanded trajectory cost of path. if -1, then this curve is invaild.
     */
    double ClearanceCostInPathUseCircumcircle(const sas_element::MotionPrimitiveState &primitive_state,
                                               const sas_element::PrimitiveNode &node, cv::Mat *image);

    /**
     * Label cirlce of effective zone with g_cost of
     * current vaild node in search cost map
     * @param node current vaild node
     * @param effective_zone  decided by distance to obstacle and goal posotion
     */
    void SetNodeInEffectiveZone(const sas_element::PrimitiveNode &node,
                                double effective_zone);
    /**
     * decide radius of effective zone and zoom coefficient of next node
     * @param node set node
     * @param dis2goal distance to goal
     * @return  radius of effective zone
     */
    double SetZoomFactorWithEffectiveZone(sas_element::PrimitiveNode *node, double dis2goal);

    // Initialize search cost map with infinity
    void InitSearchMap();

    // reset open list and close lise
    void RemoveAllCNodes();
    /**
     * Initialize car geometry parameters
     * @param car_body car geometry parameters
     */
    void InitCarGeometry(sas_element::CarParameter &car_body);

    /**
     * Show search start position and goal position
     * @param image image
     * @param start start position
     * @param end goal position
     */
    void DrawStartAndEnd(cv::Mat *image,
                         const Vector2D<double> &start,
                         const Vector2D<double> &end);
    /**
     * backtrace path linking start and goal
     * @param image showing image
     * @param pn goal pose used to traceback
     */
    void RebuildPath(cv::Mat *image, const sas_element::PrimitiveNode &pn);
    // reset search cost map
    void ResetSearchMap();
    bool isSingleStateCollisionFree(const hmpl::State &current);
    bool isSingleStateCollisionFreeImproved(const hmpl::State &current);
    bool isSinglePathCollisionFreeImproved(std::vector<hmpl::State2D> &curve);
    bool isNearLastPath(const hmpl::State2D &pose);
    void showFootPrint(cv::Mat *image);

    //  path replan parameters
    double offset_distance_;
    bool allow_use_last_path_;


};

} // namespace hmpl

#endif /* INCLUDE_SAS_SPACE_EXPLORE_SAS_EXPLORE_HPP_ */

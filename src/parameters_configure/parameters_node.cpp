//
// Created by kevin on 12/14/17.
//

#include "parameters_configure/parameters.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "parameters_transfer_node");

    sas_space_explore::ParametersTransfer parameters_transfer(ros::NodeHandle("~"));

    ros::spin();
    return 0;
}

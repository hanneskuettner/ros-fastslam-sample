#include <ros/ros.h>
#include "FastSlam.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "fast_slam");

    ros::NodeHandle n("~");

    FastSlam slam(n);

    ros::spin();

    return EXIT_SUCCESS;
}
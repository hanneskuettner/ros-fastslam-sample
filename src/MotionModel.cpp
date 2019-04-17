#include "MotionModel.h"

std::default_random_engine MotionModel::rng;


float MotionModel::std_x;
float MotionModel::std_y;
float MotionModel::std_theta;

void setStandardDeviations(float std_x, float std_y, float std_theta) {
    MotionModel::std_x = std_x;
    MotionModel::std_y = std_y;
    MotionModel::std_theta = std_theta;
}

Pose MotionModel::updatePose(const Pose& old_pose, const Control& control, float delta_time) {
    Pose new_pose = old_pose;

    float dx = std::normal_distribution<float>(control.vx, std_x)(rng) * delta_time;
    float dy = std::normal_distribution<float>(control.vy, std_y)(rng) * delta_time;

    new_pose.pos.x() += cosf(old_pose.theta) * dx - sinf(old_pose.theta) * dy;
    new_pose.pos.y() += sinf(old_pose.theta) * dx + cosf(old_pose.theta) * dy;
    new_pose.theta += std::normal_distribution<float>(control.dtheta, std_theta)(rng) * delta_time;

    // clamp angle between -pi and pi
    new_pose.theta = clamp_angle_pi_pi(new_pose.theta);
    return new_pose;
}
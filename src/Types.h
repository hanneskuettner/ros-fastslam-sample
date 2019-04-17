#pragma once

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <hphlib/pcl.h>


enum SLAM_PHASE {
    SLAM_PHASE_MAP_BUILDING,
    SLAM_PHASE_LOCALIZATION
};

struct Control {
    float vx = 0;
    float vy = 0;
    float dtheta = 0;

    Control() {}
    Control(const Eigen::Vector3f &v) : vx(v.x()), vy(v.y()), dtheta(v.z()) {};
};

struct Pose {
    Eigen::Vector2f pos = Eigen::Vector2f(0, 0);
    float theta = 0;
};

struct Observation {
    Eigen::Vector2f z = Eigen::Vector2f(0, 0);
    uint32_t color = 0;

    Observation() {}
    Observation(float x, float y, uint32_t c) : z(x, y), color(c) {}
};

enum LANDMARK_STATE {
    LANDMARK_IN_VIEW,
    LANDMARK_LEFT_VIEW,
    LANDMARK_RETURNED
};

enum LANDMARK_SIDE {
    LANDMARK_SIDE_UNKNOWN,
    LANDMARK_SIDE_LEFT,
    LANDMARK_SIDE_RIGHT
};

struct Landmark {
    Eigen::Vector2f mu;    // landmark position mean
    Eigen::Matrix2f sigma; // landmark position covariance

    uint32_t color = 0;

    Eigen::Vector4i colors = Eigen::Vector4i(0, 0, 0, 0);

    int num_observed = 0;
    int16_t travel_idx = -1;
    uint8_t side = LANDMARK_SIDE_UNKNOWN;
    uint8_t state = LANDMARK_IN_VIEW;

    float first_observed_heading = 0;

    void updateColor(uint32_t color) {
        switch (color) {
            case hphlib::REF_COLOR_RED:
                colors.x()++;
                break;
            case hphlib::REF_COLOR_FINISH:
                colors.y()++;
                break;
            case hphlib::REF_COLOR_YELLOW:
                colors.z()++;
                break;
            case hphlib::REF_COLOR_BLUE:
                colors.w()++;
                break;
            default:break;
        }

        int max_idx = 0;
        for (int i = 1; i < 4; i++) {
            if (colors(i) > colors(max_idx))
                max_idx = i;
        }

        this->color = hphlib::REF_COLORS[max_idx];
    }

    bool couldBeColor(uint32_t color) {
        // assume this cone can be any color if we have less than 20 observations
        if (colors.sum() <  20)
            return true;
        switch (color) {
            case hphlib::REF_COLOR_RED:
            case hphlib::REF_COLOR_FINISH:
                return colors.x() > 0 || colors.y() > 0;
            case hphlib::REF_COLOR_YELLOW:
                return colors.z() > 0;
            case hphlib::REF_COLOR_BLUE:
                return colors.w() > 0;
            default:break;
        }
        return false;
    }
};

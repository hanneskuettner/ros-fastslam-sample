#pragma once

#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <ros/node_handle.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <hphlib/pcl.h>
#include <hphlib/misc/FovHelper.h>
#include <hphlib/vehicle/StatusMonitor.h>
#include <telemetry/Runner.h>

#include "Particle.h"
#include "MotionModel.h"

struct OdoMeasurement {
    Eigen::Vector3f val;
    ros::Time timestamp;
};

class FastSlam {
private:
    std::default_random_engine rng;

    std::vector<Particle> particles;
    Particle most_likely_particle;

    bool first_update = true;
    ros::Time last_update_time;

    /* pubs/subs */
    ros::NodeHandle &node_handle;
    ros::Subscriber odo_sub_;
    std::vector<ros::Subscriber> cone_subs_;
    ros::Publisher cones_pub_;
    ros::Publisher ff_cones_pub_;
    ros::Publisher rel_cones_pub_;
    ros::Publisher rel_measurements_pub_;
    ros::Publisher particles_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher midway_pub_;
    ros::Publisher status_pub_;

    /* SLAM config parameters */
    std::string map_frame_id_;
    size_t particle_count;
    bool delete_cones;
    bool associate_same_color_only;
    int observation_increment;
    float particle_resample_factor;
    float loop_closure_particle_factor;
    float cone_radius;
    FovHelper fov;
    /* parameters for map loading */
    bool load_map_from_file;
    bool load_skid_pad;
    std::string map_location;
    std::string load_map_filename;

    tf::TransformListener tf_listener;

    telemetry::Runner tele_;

    Control last_control;

    ros::Time last_pose_time;
    Eigen::Vector3f last_pose;
    SLAM_PHASE current_slam_phase = SLAM_PHASE_MAP_BUILDING;

    bool first_odo_update = true;
    ros::Time last_odo_time;
    std::vector<OdoMeasurement> odo_measurements;

    boost::asio::io_service io_service;
    boost::thread_group worker_threads;
    boost::asio::io_service::work io_service_lock;

    hphlib::vehicle::StatusMonitor vehicle_mon_;

    bool midway_published = false;

    void setupThreadPool(size_t count);

    void update(std::vector<Observation> observations, ros::Time observation_time);
    std::vector<Particle> resampleParticles();

    void init(uint8_t mission);
    void initParticles();
    void loadSkipPad();

    void particlePoses(Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta);
    void tempParticlePoses(Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta);
    void particlesPoseMeanAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std, Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta);
    void particlesPoseMeanAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std);
    void tempParticlesPoseMeanAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std);

    void particlesPoseStdDeviation(float &x_std, float &y_std, float &theta_std);
    void odometryCallback(nav_msgs::Odometry);

    void conesCallback(pcl::PointCloud<pcl::PointXYZRGBA> input_cones);
    void publishCones();
    void publishFrontFacingCones();
    void publishParticles();
    void publishRelativeLandmarksAndMeasurements(std::vector<Observation> &observations);
    void publishOdometry(ros::Time time);

    void publishMidway(const std::vector<Eigen::Vector2f> &midway);
    void calcDriveline();
    void publishStatus();
    void onVehicleReady(uint8_t mission);
public:
    explicit FastSlam(ros::NodeHandle& n);
};
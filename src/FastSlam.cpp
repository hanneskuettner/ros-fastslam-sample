#include <algorithm>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl_ros/transforms.h>
#include <hphlib/util.h>
#include <hphlib/vehicle/StatusMonitor.h>
#include <hphlib/SlamStatus.h>
#include <hphlib/misc/map/Map.h>
#include <hphlib/misc/map/MapLoader.h>

#include "FastSlam.h"

using namespace Eigen;

FastSlam::FastSlam(ros::NodeHandle &n)
    : node_handle(n)
    , odo_sub_(n.subscribe(getRequiredRosParam<std::string>(n, "topic_odometry"), 1, &FastSlam::odometryCallback, this))
    , cones_pub_(n.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(getRequiredRosParam<std::string>(n, "topic_out"), 1))
    , ff_cones_pub_(n.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>(getRequiredRosParam<std::string>(n, "front_only_topic_out"), 1))
    , rel_cones_pub_(n.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>("/slam/relative_cones", 1))
    , rel_measurements_pub_(n.advertise<pcl::PointCloud<pcl::PointXYZRGBA>>("/slam/relative_measurements", 1))
    , particles_pub_(n.advertise<geometry_msgs::PoseArray>("/slam/particles", 1))
    , odom_pub_(n.advertise<nav_msgs::Odometry>("/odometry/slam", 1))
    , midway_pub_(n.advertise<nav_msgs::Path>("/slam/midway", 1))
    , status_pub_(n.advertise<hphlib::SlamStatus>("/slam/status", 1))
    , map_frame_id_(getRequiredRosParam<std::string>(n, "map_frame_id"))
    , particle_count(getRequiredRosParam<int, size_t>(n, "particle_count"))
    , delete_cones(getRequiredRosParam<bool>(n, "delete_cones_when_left_fov"))
    , associate_same_color_only(getRequiredRosParam<bool>(n, "associate_same_color_only"))
    , observation_increment(getRequiredRosParam<int>(n, "observation_increment"))
    , particle_resample_factor(getRequiredRosParam<double, float>(n, "particle_resample_factor"))
    , loop_closure_particle_factor(getRequiredRosParam<double, float>(n, "loop_closure_particle_factor"))
    , cone_radius(getRequiredRosParam<double, float>(n, "cone_radius"))
    , fov(n, "/slam/fov", "base_link",
          getRequiredRosParam<double, float>(n, "near_observation_distance"),
          getRequiredRosParam<double, float>(n, "far_observation_distance"),
          getRequiredRosParam<double, float>(n, "max_observation_angle"))
    , load_map_from_file(getRequiredRosParam<bool>(n, "load_map_from_file"))
    , load_skid_pad(getRequiredRosParam<bool>(n, "load_skid_pad"))
    , map_location(getRequiredRosParam<std::string>(n, "map_location"))
    , load_map_filename(getRequiredRosParam<std::string>(n, "load_map_filename"))
    , tele_("slam")
    , io_service_lock(io_service)
    , vehicle_mon_(n)
{
    MotionModel::setStandardDeviations(getRequiredRosParam<double, float>(n, "motion_std_dev_x"),
                                       getRequiredRosParam<double, float>(n, "motion_std_dev_y"),
                                       getRequiredRosParam<double, float>(n, "motion_std_dev_theta"));

    for (auto &topic: getRequiredRosParam<std::vector<std::string>>(n, "cone_topics")) {
        cone_subs_.push_back(n.subscribe(topic, 10, &FastSlam::conesCallback, this));
    }

    // register vehicle status callbacks
    vehicle_mon_.set_ready_callback([&] () {
        onVehicleReady(vehicle_mon_.mission_on_ready());
    });

    setupThreadPool(getRequiredRosParam<int, size_t>(n, "thread_count"));

    // fixup map location
    if (map_location.back() != '/')
        map_location += '/';

    init(hphlib::Status::AMI_TRACKDRIVE);
}

void FastSlam::setupThreadPool(size_t count) {
    // start worker threads
    for (size_t i = 0; i < count; ++i)
    {
        worker_threads.create_thread(boost::bind(&boost::asio::io_service::run,
            &io_service));
    }
}

// a few handy typedefs
typedef boost::packaged_task<int> update_task_t;
typedef boost::shared_ptr<update_task_t> pupdate_task_t;

int updateParticle(Particle *particle, Control &control, float delta,
                    std::vector<Observation> observations, SLAM_PHASE slam_phase) {
    particle->updatePose(control, delta);
    particle->updateLandmarks(observations, slam_phase);
    return 0;
}

void FastSlam::update(std::vector<Observation> observations, ros::Time observation_time) {
    tele_.reportProcessingTime([&] () {
        float delta = static_cast<float>((observation_time - last_update_time).toSec());

        if (first_update) {
            delta = 0;
            first_update = false;
        }

        Vector3f m = Vector3f::Zero();
        float acc_update_time = 0;
        // find all odometry measurements between last_update_time and observation_time + one more
        // this is used to calculate the movement in between the last updates
        for(size_t i = 0; i < odo_measurements.size(); i++) {
            OdoMeasurement &odo = odo_measurements[i];
            // remove older measurements than 0.5s
            if ((odo.timestamp - last_update_time).toSec() < -0.5f) {
                odo_measurements.erase(odo_measurements.begin() + i);
                i--;
            } else if (delta < 0 && odo.timestamp > observation_time){
                if (i == 0 || odo_measurements[i-1].timestamp < observation_time) {
                    acc_update_time = (last_update_time - observation_time).toSec();
                    m += odo.val * acc_update_time;
                    break;
                }
            } else if (odo.timestamp > last_update_time) {
                // this is an interesting odometry update since it occured after the last update time
                ros::Time start, end;
                bool _break = false;
                if (i == 0 || odo_measurements[i-1].timestamp < last_update_time) {
                    start = last_update_time;
                    if (odo.timestamp > observation_time)
                        end = observation_time;
                    else
                        end = odo.timestamp;
                } else {
                    start = odo_measurements[i-1].timestamp;
                    if (odo.timestamp > observation_time) {
                        end = observation_time;
                        _break = true;
                    } else {
                        end = odo.timestamp;
                    }
                }
                m += odo.val * (end - start).toSec();
                acc_update_time += (end - start).toSec();
                if (_break)
                    break;
            }
        }
        Control control;

        // if the update time is sufficiently large actually fill the control
        if (acc_update_time >= 1e-10)
            control = Control(m / acc_update_time);
        control.dtheta = clamp_angle_pi_pi(control.dtheta);

        bool late_update = delta < 0; // this observations are older than the previous ones;

        // ignore observations if we are localizing ourself and this observations are older than the last ones
        if (current_slam_phase == SLAM_PHASE_LOCALIZATION && late_update) {
            return;
        }

        last_update_time = observation_time;

        // distribute particle updates over thread pool
        std::vector<boost::shared_future<int>> pending_jobs;
        for (auto &p: particles) {
            pupdate_task_t task = boost::make_shared<update_task_t>(
                    boost::bind(&updateParticle, &p, control, delta, observations, current_slam_phase)
            );
            boost::shared_future<int> fut(task->get_future());
            pending_jobs.push_back(fut);
            io_service.post(boost::bind(&update_task_t::operator(), task));
        }

        // wait for all updates to be done
        boost::wait_for_all(pending_jobs.begin(), pending_jobs.end());


        // get all particle weights and number of loop closures
        int number_loop_closures = 0;
        Eigen::ArrayXf weights(particle_count);
        for(size_t i = 0; i < particle_count; i++) {
            weights(i) = particles[i].weight;

            if (particles[i].loop_closure_detected)
                number_loop_closures++;
        }

        // calc effective sample size
        float effective_sample_size = 1 / ((weights / weights.sum()).square()).sum();

        // only resample particles if the effictive sample size is smaller than the resample factor
        if (effective_sample_size <= particle_resample_factor * particle_count) {
            particles = resampleParticles();
        } else {
            // only update most likely particle
            std::vector<float> weights;
            weights.reserve(particles.size());

            std::transform(particles.begin(), particles.end(), std::back_inserter(weights),
                           [](Particle p) -> float { return p.weight; });

            auto max_it = std::max_element(weights.begin(), weights.end());
            most_likely_particle = particles[std::distance(weights.begin(), max_it)];
        }

        // check if loop closure is probable in map building phase
        if (current_slam_phase == SLAM_PHASE_MAP_BUILDING &&
            number_loop_closures > particle_count * loop_closure_particle_factor) {
            float x_dev, y_dev, theta_dev;
            particlesPoseStdDeviation(x_dev, y_dev, theta_dev);
            ROS_INFO_STREAM("Loop closure detected by " << static_cast<float>(number_loop_closures) / particle_count * 100 << "%");
            ROS_INFO_STREAM("Particle std dev " << theta_dev);
            if (sqrtf(x_dev * x_dev + y_dev * y_dev) <= 0.1f) {
                // all particles estimate our position better than 0.1m
                ROS_INFO_STREAM("LOOP CLOSURE CERTAIN\n");

                std::stringstream name_stream;

                name_stream << map_location << "map_" << ros::WallTime::now() << ".map";
                // save the best particle to file
                std::ofstream fout(name_stream.str(), std::ios::out | std::ios::binary);
                most_likely_particle.save(fout);
                current_slam_phase = SLAM_PHASE_LOCALIZATION;

                // TODO go over landmarks and adjust their colors based on the side of the car they were observed on

                // copy most probable map to all particles
                for (auto &p: particles) {
                    p.landmarks = most_likely_particle.landmarks;
                }
            }
        }

        if (current_slam_phase == SLAM_PHASE_LOCALIZATION) {
            publishOdometry(last_update_time);
            calcDriveline();
        }

        publishCones();
        publishFrontFacingCones();
        publishParticles();
        publishRelativeLandmarksAndMeasurements(observations);
        publishStatus();

        fov.publish();
    });
}

std::vector<Particle> FastSlam::resampleParticles() {
    std::vector<Particle> new_particles;
    std::vector<float> weights;

    // reserve space for new particles
    new_particles.reserve(particles.size());
    weights.reserve(particles.size());

    // fill weights
    std::transform(particles.begin(), particles.end(), std::back_inserter(weights),
                   [](Particle p) -> float { return p.weight; });

    std::uniform_real_distribution<float> dist(0, 1);
    size_t idx = (size_t)(dist(rng) * particles.size());
    float beta = 0;
    auto max_it = std::max_element(weights.begin(), weights.end());
    float max_weight = *max_it;

    // perform weighted random samping
    for (size_t i = 0; i < particles.size(); i++) {
        beta += dist(rng) * 2 * max_weight;
        while (beta > weights[idx]) {
            beta -= weights[idx];
            idx = (idx + 1) % particles.size();
        }
        Particle new_p = particles[idx];
        new_p.weight = 1;
        new_particles.push_back(new_p);
    }

    // determine most likely particle
    most_likely_particle = particles[std::distance(weights.begin(), max_it)];

    return new_particles;
}

void FastSlam::init(uint8_t mission) {
    (void) mission;
    if (load_skid_pad) {
        loadSkipPad();
        // do this for debug purposes
        publishCones();
        return;
    } else if (!load_map_from_file) {
        initParticles();
        current_slam_phase = SLAM_PHASE_MAP_BUILDING;
    } else {
        initParticles();
    }
}

void FastSlam::initParticles() {
    Particle p;
    // check if we're loading the map from file
    if (load_map_from_file) {
        std::ifstream fin(map_location + load_map_filename, std::ios::in | std::ios::binary);
        p.load(fin);

        // TODO remove
        p.purgeLandmarks();

        current_slam_phase = SLAM_PHASE_LOCALIZATION;
        most_likely_particle = p;
    }

    std::normal_distribution<float> g_pos(0, 0.2);

    particles.clear();
    particles.reserve(particle_count);

    // create particles with normal distributed positions around the origin
    for (size_t i = 0; i < particle_count; i++) {
        Particle particle(&fov, delete_cones, observation_increment, associate_same_color_only);

        if (load_map_from_file) {
            particle.pose.pos = Vector2f(g_pos(rng), g_pos(rng));
           particle.landmarks = p.landmarks;
        }
        particles.push_back(particle);
    }
}

void FastSlam::loadSkipPad() {
    boost::optional<hphlib::Map> map = hphlib::MapLoader::load("skidpad.json");

    Particle p;
    if (map) {
        // set up the particles with the landmarks loaded from file
        for (auto &cone: map->cones) {
            Landmark lm;
            // set landmark position and estimated covariance
            lm.mu.x() = cone.x;
            lm.mu.y() = cone.y;
            lm.sigma << 0.04f, 0, 0, 0.04f;
            // set color
            for (int i = 0; i < 50; i++)
                lm.updateColor(cone.rgba);
            p.landmarks.push_back(lm);
        }

        particles.clear();
        particles.reserve(particle_count);

        std::normal_distribution<float> g_pos(0, 1);

        for (size_t i = 0; i < particle_count; i++) {
            Particle particle(&fov, delete_cones, observation_increment, associate_same_color_only);

            particle.pose.pos = Vector2f(g_pos(rng), g_pos(rng));
            particle.pose.theta = std::normal_distribution<float>(0, static_cast<float>(5.0f * M_PI / 180.0f))(rng);
            particle.landmarks = p.landmarks;
            particles.push_back(particle);
        }

        most_likely_particle = p;

        current_slam_phase = SLAM_PHASE_LOCALIZATION;

        // prevent midway from being published with wrong data
        midway_published = true;
    }
}

void FastSlam::particlePoses(Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta) {
    x.resize(particle_count);
    y.resize(particle_count);
    theta.resize(particle_count);

    for (size_t i = 0; i < particle_count; i++) {
        x[i] = particles[i].pose.pos.x();
        y[i] = particles[i].pose.pos.y();
        theta[i] = particles[i].pose.theta;
    }
}

void FastSlam::tempParticlePoses(Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta) {
    x.resize(particle_count);
    y.resize(particle_count);
    theta.resize(particle_count);

    for (size_t i = 0; i < particle_count; i++) {
        x[i] = particles[i].temp_pose.pos.x();
        y[i] = particles[i].temp_pose.pos.y();
        theta[i] = particles[i].temp_pose.theta;
    }
}

void FastSlam::particlesPoseMeanAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std, Eigen::ArrayXf &x, Eigen::ArrayXf &y, Eigen::ArrayXf &theta) {
    mean.x() = x.sum() / particle_count;
    mean.y() = y.sum() / particle_count;
    mean.z() = theta.sum() / particle_count;

    float x_sqrd_sum = (x - mean.x()).square().sum();
    float y_sqrd_sum = (y - mean.y()).square().sum();
    float t_sqrd_sum = (theta - mean.z()).square().sum();

    if (x_sqrd_sum != 0)
        std.x() = sqrtf(x_sqrd_sum / particle_count);

    if (y_sqrd_sum != 0)
        std.y() = sqrtf(y_sqrd_sum / particle_count);

    if (t_sqrd_sum != 0)
        std.z() = sqrtf(t_sqrd_sum / particle_count);
}

void FastSlam::particlesPoseMeanAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std) {
    ArrayXf x, y, theta;
    particlePoses(x, y, theta);
    particlesPoseMeanAndStdDev(mean, std, x, y, theta);
}

void FastSlam::tempParticlesPoseMeanAndStdDev(Eigen::Vector3f &mean, Eigen::Vector3f &std) {
    ArrayXf x, y, theta;
    tempParticlePoses(x, y, theta);
    particlesPoseMeanAndStdDev(mean, std, x, y, theta);
}

void FastSlam::particlesPoseStdDeviation(float &x_std, float &y_std, float &theta_std) {
    Eigen::Vector3f mean, std;
    particlesPoseMeanAndStdDev(mean, std);
    x_std = std.x();
    y_std = std.y();
    theta_std = std.z();
}

void FastSlam::odometryCallback(nav_msgs::Odometry odo) {
    OdoMeasurement odo_measurement;
//    float delta = static_cast<float>((odo.header.stamp - last_odo_time).toSec());
    last_odo_time = odo.header.stamp;

    odo_measurement.val = Eigen::Vector3f(static_cast<float>(odo.twist.twist.linear.x),
                                          static_cast<float>(odo.twist.twist.linear.y),
                                          static_cast<float>(odo.twist.twist.angular.z));
    odo_measurement.timestamp = odo.header.stamp;

    odo_measurements.push_back(odo_measurement);

    if (current_slam_phase == SLAM_PHASE_LOCALIZATION) {
        if (first_odo_update) {
            first_odo_update = false;
        } else {
            publishOdometry(odo_measurement.timestamp);
            publishParticles();
        }
    }
}

void FastSlam::conesCallback(pcl::PointCloud<pcl::PointXYZRGBA> input_cones) {
    pcl_ros::transformPointCloud ("base_link", input_cones, input_cones, tf_listener);

    std::vector<Observation> observations;

    std::transform(input_cones.begin(), input_cones.end(), std::back_inserter(observations),
                   [&](pcl::PointXYZRGBA cone) -> Observation {
        float d = sqrtf(cone.x * cone.x + cone.y * cone.y) + cone_radius;
        return Observation(d, atan2f(cone.y, cone.x), cone.rgba);
    });

    std::sort(observations.begin(), observations.end(), 
              [](Observation &a, Observation &b) -> bool { return a.z.x() < b.z.x(); });

    update(observations, pcl_conversions::fromPCL(input_cones.header.stamp));
}

void FastSlam::publishCones() {
    pcl::PointCloud<pcl::PointXYZRGBA> cones;

    for (auto &lm: most_likely_particle.landmarks) {
        if (lm.num_observed <= observation_increment)
            continue; // dont publish landmarks that we observed only once. good landmarks will be seen more often

        pcl::PointXYZRGBA p;
        p.x = lm.mu.x();
        p.y = lm.mu.y();
        p.z = 0;
        p.rgba = lm.color;

        if (lm.travel_idx == -1)
            p.a = static_cast<uint8_t>(128);

        cones.push_back(p);
    }

    cones.header.frame_id = map_frame_id_;
    cones.header.stamp = ros::Time::now().toNSec()/1e3;
    cones_pub_.publish(cones);
}

void FastSlam::publishFrontFacingCones() {
    pcl::PointCloud<pcl::PointXYZRGBA> cones;

    auto cone_to_point = [](float theta, float d_norm, uint32_t color) {
        pcl::PointXYZRGBA p;
        p.x = cosf(theta) * d_norm;
        p.y = sinf(theta) * d_norm;
        p.z = 0;
        p.rgba = color;
        return p;
    };

    auto landmark_to_point = [&cone_to_point](Landmark *lm, Pose p) {
        Vector2f d = lm->mu - p.pos;
        float theta = clamp_angle_pi_pi(atan2f(d.y(), d.x()) - p.theta);
        return cone_to_point(theta, d.norm(), lm->color);
    };

    float nearest_landmark_left_distance = std::numeric_limits<float>::max();
    float nearest_landmark_right_distance = std::numeric_limits<float>::max();
    Landmark *nearest_landmark_left = nullptr;
    Landmark *nearest_landmark_right = nullptr;

    Pose p = most_likely_particle.pose;
    for (auto &lm: most_likely_particle.landmarks) {
        if (lm.num_observed <= observation_increment)
            continue; // dont publish landmarks that we observed only once. good landmarks will be seen more often

        Eigen::Vector2f d = (lm.mu - p.pos);
        float d_norm = d.norm();
        float theta = clamp_angle_pi_pi(atan2f(d.y(), d.x()) - p.theta);

        // check if landmark is in front of car and not more than 20m away, or the nearest landmark on either side behind
        if (theta > -M_PI_2 && theta < M_PI_2 && d_norm <= 20) {
            cones.push_back(cone_to_point(theta, d_norm, lm.color));
        } else if (theta > M_PI_2 && d_norm < nearest_landmark_left_distance) {
            nearest_landmark_left_distance = d_norm;
            nearest_landmark_left = &lm;
        }  else if (theta < -M_PI_2 && d_norm < nearest_landmark_right_distance) {
            nearest_landmark_right_distance = d_norm;
            nearest_landmark_right = &lm;
        }
    }

    // add the nearest landmarks from behind for better boundary performance
    if (nearest_landmark_left) {
        cones.push_back(landmark_to_point(nearest_landmark_left, p));
    }
    if (nearest_landmark_right) {
        cones.push_back(landmark_to_point(nearest_landmark_right, p));
    }

    cones.header.frame_id = "base_link";
    cones.header.stamp = ros::Time::now().toNSec()/1e3;
    ff_cones_pub_.publish(cones);
}

void FastSlam::publishRelativeLandmarksAndMeasurements(std::vector<Observation> &observations) {
    pcl::PointCloud<pcl::PointXYZRGBA> cones;
    pcl::PointCloud<pcl::PointXYZRGBA> measurements;

    auto cone_to_point = [](Landmark &lm, Pose &pose) {
        pcl::PointXYZRGBA p;
        float dx = lm.mu.x() - pose.pos.x();
        float dy = lm.mu.y() - pose.pos.y();
        p.x = cosf(-pose.theta) * dx - sinf(-pose.theta) * dy;
        p.y = sinf(-pose.theta) * dx + cosf(-pose.theta) * dy;
        p.z = 0;
        p.rgba = lm.color;
        return p;
    };

    auto observation_to_point = [](Observation &ob) {
        pcl::PointXYZRGBA p;
        p.x = cosf(ob.z.y()) * ob.z.x();
        p.y = sinf(ob.z.y()) * ob.z.x();
        p.z = 0;
        p.rgba = ob.color;
        return p;
    };

    for (auto &lm: most_likely_particle.landmarks) {
        if (lm.num_observed <= observation_increment)
            continue; // dont publish landmarks that we observed only once. good landmarks will be seen more often

        cones.push_back(cone_to_point(lm, most_likely_particle.pose));
    }

    cones.header.frame_id = "base_link";
    cones.header.stamp = ros::Time::now().toNSec()/1e3;
    rel_cones_pub_.publish(cones);

    for (auto &ob: observations) {
        measurements.push_back(observation_to_point(ob));
    }

    measurements.header.frame_id = "base_link";
    measurements.header.stamp = cones.header.stamp;
    rel_measurements_pub_.publish(measurements);
}

void FastSlam::publishParticles() {
    geometry_msgs::PoseArray poses;


    for (auto &pa: particles) {
        geometry_msgs::Pose pose;
        tf::Quaternion rot;

        rot.setEuler(0, 0, pa.pose.theta);
        pose.position.x = pa.pose.pos.x();
        pose.position.y = pa.pose.pos.y();
        tf::quaternionTFToMsg(rot, pose.orientation);

        poses.poses.push_back(pose);
    }

    poses.header.stamp = ros::Time::now();
    poses.header.frame_id = map_frame_id_;

    particles_pub_.publish(poses);

}

void FastSlam::publishOdometry(ros::Time time) {
    nav_msgs::Odometry odom;
    Eigen::Vector3f mean(0,0,0), std(0,0,0);

    odom.header.frame_id = "map";
    odom.header.stamp = time;

    tempParticlesPoseMeanAndStdDev(mean, std);

    odom.pose.pose.position.x = mean.x();
    odom.pose.pose.position.y = mean.y();
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(mean.z());

    if (std::isnan(std.x()) || std::isnan(std.y()) || std::isnan(std.x())) {
        std::cout << mean << std::endl << std << std::endl;
        return; // something went really wrong here;
    }

    odom.pose.covariance = {std.x() * std.x(), 0, 0, 0, 0, 0,
                           0, std.y() * std.y(), 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, std.z() * std.z()};

    odom_pub_.publish(odom);
}

void FastSlam::publishMidway(const std::vector<Vector2f> &midway) {
    if (midway_published)
        return;

    midway_published = true;

    std::cout << midway.size() << std::endl;
    if (!midway.empty()) {
        std::cout << "mid way published" << std::endl;
        nav_msgs::Path midway_path;
        midway_path.poses.reserve(midway.size());
        std::transform(midway.begin(), midway.end(), std::back_inserter(midway_path.poses),
                       [](auto &point) -> geometry_msgs::PoseStamped {
                           geometry_msgs::PoseStamped pose;
                           pose.pose.position.x = point.x();
                           pose.pose.position.y = point.y();
                           pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
                           pose.header.frame_id = "map";
                           return pose;
                       });

        midway_path.header.stamp = ros::Time::now();
        midway_path.header.frame_id = "map";

        midway_pub_.publish(midway_path);
    }
}

void FastSlam::calcDriveline() {
    // use the most likely particle
    Particle &p = most_likely_particle;

    // vectors for left and right cones
    std::vector<Vector2f> left_cones;
    std::vector<Vector2f> right_cones;

    // load the landmarks
    std::vector<Landmark> landmarks = p.landmarks;

    // sort the landmarks by the time it was visited
    std::sort(landmarks.begin(), landmarks.end(), [](auto &a, auto &b) -> bool { return a.travel_idx < b.travel_idx; });

    // sort landmarks to left and right
    for(auto &lm: landmarks) {
        if (lm.side == LANDMARK_SIDE_LEFT)
            left_cones.push_back(lm.mu);
        else if (lm.side == LANDMARK_SIDE_RIGHT)
            right_cones.push_back(lm.mu);
    }

    // calculate the midway of the cones
    std::vector<Vector2f> midway;

    bool calc_left = left_cones.size() < right_cones.size();
    size_t left_idx = 0, right_idx = 0;
    while (left_idx < left_cones.size() && right_idx < right_cones.size()) {
        Vector2f left = left_cones[left_idx];
        Vector2f right = right_cones[right_idx];

        // alternate between left and right side
        Vector2f result;
        if (calc_left) {
            result = left + (right - left) / 2.0f;
        } else {
            result = right + (left - right) / 2.0f;
        }

        // get the next cone on either side and check its distance to the previous point
        float left_dist = std::numeric_limits<float>::max();
        float right_dist = std::numeric_limits<float>::max();
        if (left_idx + 1 < left_cones.size()) {
            right_dist = (left_cones[left_idx + 1] - right).squaredNorm();
        }
        if (right_idx + 1 < right_cones.size()) {
            left_dist = (right_cones[right_idx + 1] - left).squaredNorm();
        }
        midway.push_back(result);

        // if the left cone is close we continue to look into the left side, otherwise go over to the right side
        if (left_dist < right_dist) {
            right_idx++;
            calc_left = true;
        } else {
            left_idx++;
            calc_left = false;
        }
    }

    if (!midway.empty()) {
        midway.push_back(midway.front());

        publishMidway(midway);
    }

}

void FastSlam::publishStatus() {
    hphlib::SlamStatus status;

    // this may not always be true. Skidpad for example
    status.loop_closure = current_slam_phase == SLAM_PHASE_LOCALIZATION;

    switch(current_slam_phase) {
        case SLAM_PHASE_MAP_BUILDING:
            status.state = status.STATE_MAPPING;
            break;
        case SLAM_PHASE_LOCALIZATION:
            status.state = status.STATE_LOCALIZATION;
            break;
    }

    status_pub_.publish(status);
}

void FastSlam::onVehicleReady(uint8_t mission) {
    init(mission);
}

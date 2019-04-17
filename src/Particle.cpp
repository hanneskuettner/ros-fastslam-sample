#include <iostream>
#include <iomanip>
#include "MotionModel.h"
#include "Particle.h"
#include "util.h"

using namespace Eigen;

constexpr float NEW_LANDMARK_THRESHOLD = 0.001f;
constexpr float START_HEADING_DEV = (10.0f / 180.0f * M_PI);

std::default_random_engine Particle::rng;

Particle::Particle(FovHelper *fovHelper, bool delete_cones, int obs_incr, bool same_color)
    : fovHelper(fovHelper)
    , observation_increment(obs_incr)
    , delete_cones_when_left_fov(delete_cones)
    , associate_same_color_only(same_color)
{
    processNoise << 0.1f, 0,
                    0, pow(3.0f * M_PI / 180, 2);
}

void Particle::updatePose(const Control& u, float delta_time) {
    // draw new pose
    temp_pose = pose;
    pose = MotionModel::updatePose(pose, u, delta_time);
}

void Particle::updateLandmarks(std::vector<Observation> observations, SLAM_PHASE slam_phase) {
    Pose s = pose;
    std::vector<bool> observed_lm(landmarks.size(), false);

    weight = 1;

    // shuffle vector to improve greedy association results (this might be crashing right now)
    // std::shuffle(observations.begin(), observations.end(), rng);
    for (auto &ob: observations) {
        float p = expf(-70);
        float ass_d = 0;
        Vector2f ass_z(0, 0);
        Matrix2f ass_G;
        Matrix2f ass_Z_inv;
        size_t ass_idx = std::numeric_limits<size_t>::max();

        ass_G.setZero();

        size_t idx = 0;

        if (!landmarks.empty()) {
            for (auto &lm: landmarks) {
                // check if this landmark has been observed already (greedy strategy to ensure better data association
                if (idx >= observed_lm.size() || observed_lm[idx] ||
                    (associate_same_color_only && !lm.couldBeColor(ob.color))) {
                    idx++;
                    continue;
                }

                // compute z_n_t and G_theta_n
                float dx = lm.mu.x() - s.pos.x();
                float dy = lm.mu.y() - s.pos.y();
                float d2 = dx * dx + dy * dy;
                float d = sqrtf(d2);

                // check if distance further than 20m and twice the distance to the observation
                // or if distance is further than 2m and half of the distance to the observation.
                // if it is, ignore it,  since our sensors wont see it anyways
                if ((d > 20 && d > ob.z.x() * 2) || (d > 2 && d < ob.z.x() / 2)) {
                    idx++;
                    continue;
                }

                float theta = clamp_angle_pi_pi(atan2f(dy, dx) - s.theta);

                // check if landmark is behind us. if it is, ignore it, since we cannot see behind us
                if (theta < -M_PI_2 || theta > M_PI_2) {
                    idx++;
                    continue;
                }

                // Calculate requirements for postiori propability calculation
                Vector2f z_n_t(d, theta);

                Matrix2f G;
                G << dx / d, dy / d,
                        -dy / d2, dx / d2;

                Matrix2f Z = G * lm.sigma * G.transpose() + processNoise;

                Vector2f dz = (ob.z - z_n_t);

                float p_n = static_cast<float>(exp(-0.5f * dz.transpose() * Z.inverse() * dz) /
                                               (2 * M_PI * sqrtf(Z.determinant())));

                // check if this landmark fits better
                if (p_n > p) {
                    // it does. lets remember it
                    ass_d = d;
                    ass_z = z_n_t;
                    ass_G = G;
                    ass_Z_inv = Z.inverse();
                    ass_idx = idx;
                    p = p_n;
                }
                idx++;
            }

            if (slam_phase == SLAM_PHASE_MAP_BUILDING && !loop_closure_detected) {
                // only update landmarks if we are build the map
                if (p < NEW_LANDMARK_THRESHOLD) {
                    // create new landmark
                    landmarks.push_back(createLandmark(ob));

                    p = NEW_LANDMARK_THRESHOLD; // update weight of particle
                } else if (ass_idx < landmarks.size()) {
                    // update existing landmark
                    observed_lm[ass_idx] = true;
                    Landmark &lm = landmarks[ass_idx];
                    Matrix2f K = lm.sigma * ass_G.transpose() * ass_Z_inv;
                    lm.mu += K * (ob.z - ass_z);
                    lm.sigma -= K * ass_G * lm.sigma;
                    lm.updateColor(ob.color);
                    lm.num_observed += observation_increment; // its okay for a landmark to be missed X times for one observation

                    // needed for loop closure (left view before, 20m proximity, heading not to far off)
                    if (lm.state == LANDMARK_LEFT_VIEW && ass_d <= 20 &&
                        std::abs(s.theta - lm.first_observed_heading) <= START_HEADING_DEV) {
                        lm.state = LANDMARK_RETURNED;
                    }
                }
            }
        } else {
            // create initial landmark
            landmarks.push_back(createLandmark(ob));
        }
        // update particle weight
        weight *= p;
    }

    if (slam_phase == SLAM_PHASE_MAP_BUILDING && !loop_closure_detected) {
        updateStatistics(observed_lm);
    }

    // TODO
    // - apply penalty for unobserved particles
}

void Particle::purgeLandmarks() {
    return;
    std::vector<size_t> tbd;
    // combine landmarks that have the same color and are closer then 20cm
    for (size_t i = 0; i < landmarks.size(); i++) {
        for (size_t j = i + 1; j < landmarks.size(); j++) {
            if (landmarks[i].color != hphlib::REF_COLOR_FINISH &&
                landmarks[j].color != hphlib::REF_COLOR_FINISH &&
                (landmarks[i].mu - landmarks[j].mu).norm() <= 0.2f) {
                // merge landmarks
                tbd.push_back(j);
            }
        }
    }

    // instead of properly mergin them we just remove one of the landmarks
    for (auto i = static_cast<int>(tbd.size()); i >= 0; i--) {
        landmarks.erase(landmarks.begin() + tbd[i]);
    }

    std::cout << landmarks.size() << std::endl;
}

Landmark Particle::createLandmark(Observation ob) {
    Landmark lm;
    lm.mu = Vector2f(pose.pos.x() + ob.z.x() * cosf(ob.z.y() + pose.theta),
                     pose.pos.y() + ob.z.x() * sinf(ob.z.y() + pose.theta));
    lm.sigma = Matrix2f::Identity() * 20;
    lm.updateColor(ob.color);
    lm.num_observed = observation_increment;
    lm.first_observed_heading = pose.theta;
    return lm;
}


void Particle::updateStatistics(std::vector<bool> &observed_lm) {
    size_t seen_landmarks = 0;
    size_t missed_landmarks = 0;
    size_t returned_landmarks = 0;

    float nearest_landmark_left_distance = std::numeric_limits<float>::max();
    float nearest_landmark_right_distance = std::numeric_limits<float>::max();
    Landmark *nearest_landmark_left = nullptr;
    Landmark *nearest_landmark_right = nullptr;

    // do some checks on the landmarks
    for (size_t i = 0; i < landmarks.size(); i++) {
        // check if landmark was observed
        if (i < observed_lm.size() && observed_lm[i]) {
            // count number of returned landmarks
            if (landmarks[i].state == LANDMARK_RETURNED)
                returned_landmarks++;
            seen_landmarks++;
            continue;
        }

        // check if landmark in fov but unobserved
        Vector2f d = landmarks[i].mu - pose.pos;
        float d_norm = d.norm();
        float theta = clamp_angle_pi_pi(atan2f(d.y(), d.x()) - pose.theta);
        if (fovHelper->angleDistInFov(theta, d_norm)) {
            landmarks[i].num_observed--;

            if (landmarks[i].num_observed < 0) {
                landmarks.erase(landmarks.begin() + i);
                i--;
                continue;
            }

            // if this landmark has returned before increment number of returned landmarks
            if (landmarks[i].state == LANDMARK_RETURNED)
                returned_landmarks++;

            // nontheless we missed it. increase number of missed landmarks
            missed_landmarks++;
        } else if (landmarks[i].state != LANDMARK_LEFT_VIEW && d_norm > 10 && (theta < -M_PI_2 || theta > M_PI_2)) {
            // outside of fov and behind the car. this cone left us.
            landmarks[i].state = LANDMARK_LEFT_VIEW;
        }

        // check if landmark is left or right of us and behind us and closer than closest landmark
        if (theta >= M_PI_2 && d_norm < nearest_landmark_left_distance) {
            nearest_landmark_left = &landmarks[i];
            nearest_landmark_left_distance = d_norm;
        } else if (theta <= -M_PI_2 && d_norm < nearest_landmark_right_distance) {
            nearest_landmark_right = &landmarks[i];
            nearest_landmark_right_distance = d_norm;
        }
        // if we want to delete all cones behind us check if this landmark has to be deleted
        if (delete_cones_when_left_fov && landmarks[i].state == LANDMARK_LEFT_VIEW) {
            landmarks.erase(landmarks.begin() + i);
            i--;
        }
    }

    // since we passed those landmarks and they are closest to us we can assume we travelled in-between them
    if (nearest_landmark_left) {
        if (nearest_landmark_left->travel_idx == -1) {
            nearest_landmark_left->travel_idx = travel_idx++;
            nearest_landmark_left->side = LANDMARK_SIDE_LEFT;
        }
    }
    if (nearest_landmark_right) {
        if (nearest_landmark_right->travel_idx == -1) {
            nearest_landmark_right->travel_idx = travel_idx++;
            nearest_landmark_right->side = LANDMARK_SIDE_RIGHT;
        }
    }

    // TODO: decide if we want to reset the loop closure indicator just to be more certain
    if (returned_landmarks > 0 &&
        returned_landmarks >= (seen_landmarks + missed_landmarks) * 0.8) {
        loop_closure_detected = true;
    }
}

/**
 * Save landmarks of this particle to stream
 * @param fout File stream
 */
void Particle::save(std::ofstream &fout) {
    size_t count = landmarks.size();
    fout.write((char*)&count, sizeof(size_t));
    fout.write((char*)&landmarks[0], count * sizeof(Landmark));
}

/**
 * Load landmarks for this particle from stream
 * @param fin File stream
 */
void Particle::load(std::ifstream &fin) {
    size_t count;
    fin.read(reinterpret_cast<char*>(&count), sizeof(size_t));
    landmarks.resize(count);
    fin.read(reinterpret_cast<char*>(&landmarks[0]), count * sizeof(Landmark));
    std::cout << "Loaded particle with " << count << " landmarks." << std::endl;
}





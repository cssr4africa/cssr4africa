/* robotLocalizationImplementation.cpp    Function definitions and implementation
*
* Author:   Ibrahim Olaide Jimoh, Carnegie Mellon University Africa
* Email:    ioj@andrew.cmu.edu
* Date:     June 25, 2025
* Version:  v1.0
*
* Copyright (C) 2025 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*
* This program comes with ABSOLUTELY NO WARRANTY.
*/

#include "robotLocalization/robotLocalizationInterface.h"


void RobotLocalizationNode::initializePoseAdjustments() {
    ros::Rate rate(10);
    while (!first_odom_received_ && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    adjustment_x_ = initial_robot_x - odom_x_;
    adjustment_y_ = initial_robot_y - odom_y_;
    adjustment_theta_ = angles::normalize_angle(initial_robot_theta - odom_theta_);
}

void RobotLocalizationNode::loadTopicNames() {
    try {
        std::ifstream file(topics_file_);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open topics file %s", topics_file_.c_str());
            return;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#' || line[0] == ';') {
                continue;
            }
            
            size_t pos = line.find_first_of(" \t");
            if (pos != std::string::npos) {
                std::string key = line.substr(0, pos);
                std::string value = line.substr(pos);
                
                key.erase(key.find_last_not_of(" \t") + 1);
                
                value.erase(0, value.find_first_not_of(" \t"));
                
                value.erase(value.find_last_not_of(" \t") + 1);
                
                if (!key.empty() && !value.empty()) {
                    topic_map_[key] = value;
                }
            }
        }

        if (verbose_) {
            ROS_INFO("Loaded %zu topics from %s", topic_map_.size(), topics_file_.c_str());
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error reading topics file %s: %s", topics_file_.c_str(), e.what());
    }
}

void RobotLocalizationNode::loadLandmarks() {
    try {
        // Read JSON file
        std::ifstream config_stream(landmark_file_);
        if (!config_stream.is_open()) {
            ROS_ERROR("Failed to open landmarks file %s", landmark_file_.c_str());
            return;
        }
        
        Json::Value root;
        Json::Reader reader;
        if (!reader.parse(config_stream, root)) {
            ROS_ERROR("Failed to parse landmarks file %s: %s", 
                     landmark_file_.c_str(), reader.getFormattedErrorMessages().c_str());
            return;
        }
        
        if (!root.isMember("landmarks") || !root["landmarks"].isArray()) {
            ROS_ERROR("No 'landmarks' array found in %s", landmark_file_.c_str());
            return;
        }
        
        const Json::Value& landmarks = root["landmarks"];
        for (const Json::Value& marker : landmarks) {
            // Check for required fields
            if (!marker.isMember("id") || !marker.isMember("x") || 
                !marker.isMember("y") || !marker.isMember("z")) {
                ROS_WARN("Skipping invalid landmark entry in %s", landmark_file_.c_str());
                continue;
            }
            
            Landmark3D lm;
            lm.id = marker["id"].asInt();
            lm.x = marker["x"].asDouble();
            lm.y = marker["y"].asDouble();
            lm.z = marker["z"].asDouble();

            landmarks3D_[lm.id] = lm;

            // Project to camera plane
            double dz = lm.z - camera_height_;
            double distance = std::sqrt(lm.x * lm.x + lm.y * lm.y + dz * dz);
            if (distance < 0.1) {
                ROS_WARN("Landmark %d too close to camera, skipping projection", lm.id);
                continue;
            }
            double scale = std::sqrt(lm.x * lm.x + lm.y * lm.y) / distance;
            double x_proj = lm.x * scale;
            double y_proj = lm.y * scale;

            projected_landmarks_[lm.id] = {x_proj, y_proj};

            if (verbose_) {
                ROS_INFO("Loaded landmark ID %d: (%.2f, %.2f, %.2f) -> Projected (%.2f, %.2f)",
                         lm.id, lm.x, lm.y, lm.z, x_proj, y_proj);
            }
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error loading landmarks from %s: %s", landmark_file_.c_str(), e.what());
    }
}

void RobotLocalizationNode::loadCameraInfoFromFile() {
    try {
        // Read JSON file
        std::ifstream config_stream(camera_info_file_);
        if (!config_stream.is_open()) {
            ROS_ERROR("Failed to open camera info file %s", camera_info_file_.c_str());
            return;
        }
        
        Json::Value root;
        Json::Reader reader;
        if (!reader.parse(config_stream, root)) {
            ROS_ERROR("Failed to parse camera info file %s: %s", 
                     camera_info_file_.c_str(), reader.getFormattedErrorMessages().c_str());
            return;
        }

        if (!root.isMember("cameraInfo")) {
            ROS_ERROR("No 'cameraInfo' object found in %s", camera_info_file_.c_str());
            return;
        }

        const Json::Value& camera_info = root["cameraInfo"];
        
        // Check for required fields
        if (!camera_info.isMember("fx") || !camera_info.isMember("fy") || 
            !camera_info.isMember("cx") || !camera_info.isMember("cy")) {
            ROS_ERROR("Missing required camera intrinsic parameters in %s", camera_info_file_.c_str());
            return;
        }
        
        fx_ = camera_info["fx"].asDouble();
        fy_ = camera_info["fy"].asDouble();
        cx_ = camera_info["cx"].asDouble();
        cy_ = camera_info["cy"].asDouble();
        
        if (fx_ <= 0 || fy_ <= 0 || cx_ <= 0 || cy_ <= 0) {
            ROS_ERROR("Invalid camera intrinsics in %s", camera_info_file_.c_str());
            return;
        }
        
        camera_info_received_ = true;
        ROS_INFO("Loaded camera intrinsics from file: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                 fx_, fy_, cx_, cy_);
                 
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to load camera info from %s: %s", camera_info_file_.c_str(), e.what());
    }
}

void RobotLocalizationNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    if (camera_info_received_) return;
    fx_ = msg->K[0];
    fy_ = msg->K[4];
    cx_ = msg->K[2];
    cy_ = msg->K[5];
    if (fx_ <= 0 || fy_ <= 0 || cx_ <= 0 || cy_ <= 0) {
        ROS_WARN("Invalid camera intrinsics received from topic");
        return;
    }
    camera_info_received_ = true;
    camera_info_timer_.stop();
    ROS_INFO("Received camera intrinsics from camera: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
}

void RobotLocalizationNode::cameraInfoTimeoutCallback(const ros::TimerEvent& event) {
    if (!camera_info_received_) {
        ROS_WARN("No camera info received within %.1f seconds, falling back to %s", camera_info_timeout_, camera_info_file_.c_str());
        loadCameraInfoFromFile();
    }
}

void RobotLocalizationNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    odom_theta_ = 2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    first_odom_received_ = true;

    double x = odom_x_ + adjustment_x_ - initial_robot_x;
    double y = odom_y_ + adjustment_y_ - initial_robot_y;

    double current_x = x * cos(adjustment_theta_) - y * sin(adjustment_theta_);
    double current_y = x * sin(adjustment_theta_) + y * cos(adjustment_theta_);

    current_x += initial_robot_x;
    current_y += initial_robot_y;

    double current_theta = odom_theta_ + adjustment_theta_;

    relative_pose.x = current_x;
    relative_pose.y = current_y;
    relative_pose.theta = current_theta;

    geometry_msgs::Pose2D pose_msg;
    pose_msg.x = relative_pose.x;
    pose_msg.y = relative_pose.y;
    // pose_msg.theta = angles::to_degrees(relative_pose.theta);
    pose_msg.theta = fmod(angles::to_degrees(relative_pose.theta) + 360.0, 360.0);


    pose_pub_.publish(pose_msg);

    if (verbose_) {
        ROS_INFO_THROTTLE(1, "Odometry: position = (%5.3f, %5.3f) orientation = %5.3f degrees", current_x, current_y, angles::to_degrees(current_theta));
    }
}

void RobotLocalizationNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Optional using IMU angular velocity
}

void RobotLocalizationNode::jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    bool found = false;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == head_yaw_joint_name_) {
            head_yaw_ = msg->position[i];
            found = true;
            if (verbose_) {
                ROS_INFO("Head yaw update: %.3f radians", head_yaw_);
            }
            break;
        }
    }
    if (!found && verbose_) {
        std::string names;
        for (const auto& name : msg->name) names += name + ", ";
        ROS_WARN("Head yaw joint '%s' not found in joint_states: %s", head_yaw_joint_name_.c_str(), names.c_str());
    }
}

void RobotLocalizationNode::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        latest_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
        if (verbose_) {
            ROS_INFO("Received image: width=%d, height=%d", latest_image_.cols, latest_image_.rows);
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void RobotLocalizationNode::depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        latest_depth_ = cv_bridge::toCvShare(msg, "32FC1")->image;
        if (verbose_) {
            ROS_INFO("Received depth image: width=%d, height=%d", latest_depth_.cols, latest_depth_.rows);
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

bool RobotLocalizationNode::setPoseCallback(cssr_system::setPose::Request& req, cssr_system::setPose::Response& res) {
    initial_robot_x = req.x;
    initial_robot_y = req.y;
    initial_robot_theta = angles::from_degrees(req.theta);

    ros::Rate rate(10);
    while (!first_odom_received_ && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    adjustment_x_ = initial_robot_x - odom_x_;
    adjustment_y_ = initial_robot_y - odom_y_;
    adjustment_theta_ = initial_robot_theta - odom_theta_;
    res.success = true;
    return true;
}

bool RobotLocalizationNode::resetPoseCallback(cssr_system::resetPose::Request& req, cssr_system::resetPose::Response& res) {
    if (computeAbsolutePose()) {
        res.success = true;
        ROS_INFO("Pose reset successfully");
    } else {
        res.success = false;
        ROS_WARN("Failed to reset pose");
    }
    return true;
}

void RobotLocalizationNode::resetTimerCallback(const ros::TimerEvent& event) {
    computeAbsolutePose();
}

bool RobotLocalizationNode::computeAbsolutePose() {
    if (projected_landmarks_.empty()) {
        ROS_WARN("No landmarks loaded");
        return false;
    }
    if (!camera_info_received_) {
        ROS_WARN("Camera intrinsics not received");
        return false;
    }
    if (use_depth_) {
        return computeAbsolutePoseWithDepth();
    } else {
        if (latest_image_.empty()) {
            ROS_WARN("No image available for absolute robot localization");
            return false;
        }

        // Detect ArUco markers
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        cv::aruco::detectMarkers(latest_image_, dictionary, marker_corners, marker_ids);

        // Draw and publish marker image
        cv::Mat output_image = latest_image_.clone();
        if (!marker_ids.empty()) {
            cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
        }
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_image).toImageMsg();
        image_pub_.publish(img_msg);
        if (verbose_) {
            ROS_INFO("Published marker image with %zu markers", marker_ids.size());
        }

        if (marker_ids.size() < 3) {
            ROS_WARN("Detected %zu markers, need at least 3", marker_ids.size());
            return false;
        }

        // Sort by position using indices
        if (!marker_ids.empty()) {
            // Create index vector [0, 1, 2, 3, ...]
            std::vector<size_t> indices(marker_ids.size());
            std::iota(indices.begin(), indices.end(), 0);
            
            // Sort indices by marker center x-coordinate
            std::sort(indices.begin(), indices.end(), 
                    [&marker_corners, &marker_ids](size_t a, size_t b) {
                        float center_a_x = (marker_corners[a][0].x + marker_corners[a][1].x + 
                                            marker_corners[a][2].x + marker_corners[a][3].x) / 4.0f;
                        float center_b_x = (marker_corners[b][0].x + marker_corners[b][1].x + 
                                            marker_corners[b][2].x + marker_corners[b][3].x) / 4.0f;
                        
                        // Primary sort: x-coordinate  
                        if (std::abs(center_a_x - center_b_x) > 1.0f) {
                            return center_a_x < center_b_x;
                        }
                        
                        // Secondary sort: marker ID (for consistent ordering)
                        return marker_ids[a] < marker_ids[b];
                    });
            
            // Create sorted copies using the sorted indices
            std::vector<int> sorted_ids;
            std::vector<std::vector<cv::Point2f>> sorted_corners;
            
            for (size_t idx : indices) {
                sorted_ids.push_back(marker_ids[idx]);
                sorted_corners.push_back(marker_corners[idx]);
            }
            
            // Replace original vectors
            marker_ids = std::move(sorted_ids);
            marker_corners = std::move(sorted_corners);
        }

        // Print detected markers
        if (!marker_ids.empty()) {
            if (verbose_) {
                ROS_INFO("Detected markers:");
            }
            for (size_t i = 0; i < marker_ids.size(); ++i) {
                std::stringstream ss;
                if (verbose_) {
                    ss << "Marker " << marker_ids[i] << " corners: ";
                }
                for (size_t j = 0; j < marker_corners[i].size(); ++j) {
                    if (verbose_) {
                        ss << "(" << marker_corners[i][j].x << "," << marker_corners[i][j].y << ")";
                    }
                    if (j < marker_corners[i].size() - 1) ss << " ";
                }
                if (verbose_) {
                    ROS_INFO("%s", ss.str().c_str());
                }
            }
        } else {
            ROS_INFO("No markers detected");
        }

        // Compute angles and triangulate
        std::vector<std::pair<double, double>> marker_centers;
        for (size_t i = 0; i < marker_ids.size(); ++i) {
            auto& corners = marker_corners[i];
            double cx = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
            double cy = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0;
            marker_centers.push_back({cx, cy});
        }

        // Assume first three detected markers are used
        int id1 = marker_ids[0], id2 = marker_ids[1], id3 = marker_ids[2];
        if (projected_landmarks_.find(id1) == projected_landmarks_.end() || 
            projected_landmarks_.find(id2) == projected_landmarks_.end() || 
            projected_landmarks_.find(id3) == projected_landmarks_.end()) {
            ROS_WARN("Unknown marker IDs detected: %d, %d, %d", id1, id2, id3);
            return false;
        }

        double x1 = projected_landmarks_[id1].first, y1 = projected_landmarks_[id1].second;
        double x2 = projected_landmarks_[id2].first, y2 = projected_landmarks_[id2].second;
        double x3 = projected_landmarks_[id3].first, y3 = projected_landmarks_[id3].second;

        if (verbose_) {
            ROS_INFO("Used Markers:");
            ROS_INFO("Marker 1: ID %d (%.3f, %.3f)", id1, x1, y1);
            ROS_INFO("Marker 2: ID %d (%.3f, %.3f)", id2, x2, y2);
            ROS_INFO("Marker 3: ID %d (%.3f, %.3f)", id3, x3, y3);
        }

        // Check for collinear markers and small landmark triangle area
        double landmark_triangle_area = std::abs((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)) / 2.0;
        if (landmark_triangle_area < 0.5) {
            ROS_WARN("Landmark triangle area too small (%.3f), rejecting configuration", landmark_triangle_area);
            return false;
        }
        double cross_product = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        if (std::abs(cross_product) < 0.01) {
            ROS_WARN("Markers are nearly collinear, triangulation may be inaccurate");
            return false;
        }

        // Compute angles alpha1 (between markers 1 and 2) and alpha2 (between markers 2 and 3)
        double alpha1 = computeAngle(marker_centers[0], marker_centers[1]);
        double alpha2 = computeAngle(marker_centers[1], marker_centers[2]);

        // Reject extreme angles
        if (alpha1 < 5.0 || alpha2 < 5.0 || alpha1 > 120.0 || alpha2 > 120.0) {
            ROS_WARN("Angles too extreme (alpha1=%.3f, alpha2=%.3f), rejecting configuration", alpha1, alpha2);
            return false;
        }

        if (verbose_) {
            ROS_INFO("Computed angles: alpha1=%.3f degrees, alpha2=%.3f degrees", alpha1, alpha2);
        }

        // Triangulation
        double xc1a, yc1a, xc1b, yc1b, xc2a, yc2a, xc2b, yc2b, r1, r2;
        circle_centre(x2, y2, x1, y1, alpha1, &xc1a, &yc1a, &xc1b, &yc1b, &r1);
        circle_centre(x3, y3, x2, y2, alpha2, &xc2a, &yc2a, &xc2b, &yc2b, &r2);

        if (verbose_) {
            // Log circle parameters for debugging
            ROS_INFO("Circle 1 centers: (%.3f, %.3f), (%.3f, %.3f) with radius %.3f", xc1a, yc1a, xc1b, yc1b, r1);
            ROS_INFO("Circle 2 centers: (%.3f, %.3f), (%.3f, %.3f) with radius %.3f", xc2a, yc2a, xc2b, yc2b, r2);
        }

        // Try all 4 combinations of circle centres
        double best_xr = 0, best_yr = 0;
        double best_score = -1;
        bool found_valid = false;

        struct CirclePair {
            double xc1, yc1, xc2, yc2;
            std::string name;
        };

        std::vector<CirclePair> combinations = {
            {xc1a, yc1a, xc2a, yc2a, "closest-closest"},
            {xc1a, yc1a, xc2b, yc2b, "closest-farthest"}, 
            {xc1b, yc1b, xc2a, yc2a, "farthest-closest"},
            {xc1b, yc1b, xc2b, yc2b, "farthest-farthest"}
        };

        if (verbose_) {
            ROS_INFO("=== Testing all circle combinations ===");
        }

        for (const auto& combo : combinations) {
            double x1_int, y1_int, x2_int, y2_int;
            int result = circle_circle_intersection(combo.xc1, combo.yc1, r1, 
                                                combo.xc2, combo.yc2, r2,
                                                &x1_int, &y1_int, &x2_int, &y2_int);
            
            if (result == 0) {
                ROS_INFO("%s: No circle intersection for localization", combo.name.c_str());
                continue;
            }
            
            // Check both intersection points
            std::vector<std::pair<double, double>> candidates = {{x1_int, y1_int}, {x2_int, y2_int}};
            
            for (int i = 0; i < candidates.size(); i++) {
                double xr_test = candidates[i].first;
                double yr_test = candidates[i].second;
                
                // Calculate distances to landmarks
                double dist_to_m1 = std::sqrt((xr_test - x1)*(xr_test - x1) + (yr_test - y1)*(yr_test - y1));
                double dist_to_m2 = std::sqrt((xr_test - x2)*(xr_test - x2) + (yr_test - y2)*(yr_test - y2));
                double dist_to_m3 = std::sqrt((xr_test - x3)*(xr_test - x3) + (yr_test - y3)*(yr_test - y3));
                double min_dist_to_landmarks = std::min({dist_to_m1, dist_to_m2, dist_to_m3});
                double avg_dist_to_landmarks = (dist_to_m1 + dist_to_m2 + dist_to_m3) / 3.0;
                
                // ONLY reject obvious numerical errors (extremely close to landmarks)
                if (min_dist_to_landmarks < 0.01) {
                    if (verbose_) {
                        ROS_INFO("%s point %d: (%.3f, %.3f) - REJECTED: Numerical error (%.6f)", 
                                combo.name.c_str(), i+1, xr_test, yr_test, min_dist_to_landmarks);
                    }
                    continue;
                }
                
                // Calculate triangle areas formed by robot and landmark pairs
                double area1 = std::abs((xr_test - x1) * (y2 - y1) - (x2 - x1) * (yr_test - y1)) / 2.0;
                double area2 = std::abs((xr_test - x2) * (y3 - y2) - (x3 - x2) * (yr_test - y2)) / 2.0;
                double area3 = std::abs((xr_test - x1) * (y3 - y1) - (x3 - x1) * (yr_test - y1)) / 2.0;
                double avg_area = (area1 + area2 + area3) / 3.0;
                
                // Calculate viewing angles from robot to landmark pairs
                double angle12 = std::abs(std::atan2(y1 - yr_test, x1 - xr_test) - std::atan2(y2 - yr_test, x2 - xr_test));
                double angle23 = std::abs(std::atan2(y2 - yr_test, x2 - xr_test) - std::atan2(y3 - yr_test, x3 - xr_test));
                double angle13 = std::abs(std::atan2(y1 - yr_test, x1 - xr_test) - std::atan2(y3 - yr_test, x3 - xr_test));
                
                // Normalize angles to [0, π]
                if (angle12 > M_PI) angle12 = 2*M_PI - angle12;
                if (angle23 > M_PI) angle23 = 2*M_PI - angle23;
                if (angle13 > M_PI) angle13 = 2*M_PI - angle13;
                
                double min_angle = std::min({angle12, angle23, angle13});
                double max_angle = std::max({angle12, angle23, angle13});
                double min_angle_deg = min_angle * 180.0 / M_PI;
                double max_angle_deg = max_angle * 180.0 / M_PI;
                
                // 1. Area quality score (larger triangulation areas = better precision)
                double area_score = avg_area;
                
                // 2. Distance reasonableness (prefer moderate distances, avoid extremes)
                double distance_score = 1.0;
                if (avg_dist_to_landmarks < 1.0 || avg_dist_to_landmarks > 15.0) {
                    distance_score = 0.1; // Penalize unrealistic distances
                } else if (avg_dist_to_landmarks < 2.0 || avg_dist_to_landmarks > 10.0) {
                    distance_score = 0.5; // Penalize marginal distances
                } else {
                    distance_score = 1.0 / (1.0 + std::abs(avg_dist_to_landmarks - 5.0)); // Prefer ~5m
                }
                
                // 3. Angle quality (avoid degenerate triangulation)
                double angle_score = 1.0;
                if (min_angle_deg < 10.0) {
                    angle_score = 0.05; // Severe penalty for very acute angles
                } else if (min_angle_deg < 20.0) {
                    angle_score = 0.3;  // Moderate penalty for acute angles
                } else if (min_angle_deg > 25.0 && max_angle_deg < 100.0) {
                    angle_score = 2.0;  // Bonus for well-distributed angles
                }
                
                // 4. Landmark separation score (avoid degenerate landmark configurations)
                double landmark_separation = std::min({
                    std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1)),
                    std::sqrt((x3-x2)*(x3-x2) + (y3-y2)*(y3-y2)),
                    std::sqrt((x3-x1)*(x3-x1) + (y3-y1)*(y3-y1))
                });
                double separation_score = (landmark_separation > 1.0) ? 1.0 : 0.1;
                
                // 5. Proximity penalty (slightly penalize being too close to any landmark)
                double proximity_penalty = 1.0;
                if (min_dist_to_landmarks < 0.5) {
                    proximity_penalty = 0.2;
                } else if (min_dist_to_landmarks < 1.0) {
                    proximity_penalty = 0.7;
                }
                
                // Combine all geometric factors
                double score = area_score * distance_score * angle_score * separation_score * proximity_penalty;
                if (verbose_) {
                    ROS_INFO("%s point %d: (%.3f, %.3f) - Score: %.3f", 
                            combo.name.c_str(), i+1, xr_test, yr_test, score);
                    ROS_INFO("  Components: area=%.2f, dist=%.2f(%.1fm), angle=%.2f(%.1f°), sep=%.2f, prox=%.2f", 
                            area_score, distance_score, avg_dist_to_landmarks, angle_score, min_angle_deg, 
                            separation_score, proximity_penalty);
                }
                
                if (score > best_score) {
                    best_score = score;
                    best_xr = xr_test;
                    best_yr = yr_test;
                    found_valid = true;
                    if (verbose_) {
                        ROS_INFO("  -> NEW BEST SOLUTION!");
                    }
                }
            }
        }

        if (verbose_) {
            ROS_INFO("=== Final selection: (%.3f, %.3f) with score %.3f ===", best_xr, best_yr, best_score);
        }

        if (!found_valid) {
            ROS_WARN("No valid triangulation solution found!");
            return false;
        }

        double xr = best_xr;
        double yr = best_yr;

        if (verbose_) {
            // Debug information
            ROS_INFO("=== Debug Information ===");
            ROS_INFO("Distance between landmarks: d12=%.3f, d23=%.3f, d13=%.3f", 
                    std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1)),
                    std::sqrt((x3-x2)*(x3-x2) + (y3-y2)*(y3-y2)),
                    std::sqrt((x3-x1)*(x3-x1) + (y3-y1)*(y3-y1)));
            ROS_INFO("Robot distance from landmarks: d1=%.3f, d2=%.3f, d3=%.3f",
                    std::sqrt((xr-x1)*(xr-x1) + (yr-y1)*(yr-y1)),
                    std::sqrt((xr-x2)*(xr-x2) + (yr-y2)*(yr-y2)),
                    std::sqrt((xr-x3)*(xr-x3) + (yr-y3)*(yr-y3)));
            ROS_INFO("Computed robot position: (%.3f, %.3f)", xr, yr);
        }

        // Simple sanity check - only reject truly absurd results
        double max_distance = std::max({
            std::sqrt((xr-x1)*(xr-x1) + (yr-y1)*(yr-y1)),
            std::sqrt((xr-x2)*(xr-x2) + (yr-y2)*(yr-y2)),
            std::sqrt((xr-x3)*(xr-x3) + (yr-y3)*(yr-y3))
        });

        if (max_distance > 20.0) { // Only reject if robot is impossibly far
            ROS_WARN("Computed robot position seems unrealistic (max distance to landmark: %.3f)", max_distance);
            return false;
        }

        // Compute yaw using first marker
        double theta = computeYaw(marker_centers[0], x1, y1, xr, yr);

        // Update pose
        baseline_pose_.x = xr;
        baseline_pose_.y = yr;
        baseline_pose_.theta = theta; // Radians
        current_pose_ = baseline_pose_;
        last_absolute_pose_time_ = ros::Time::now();

        // Alternate update pose
        initial_robot_x = xr;
        initial_robot_y = yr;
        initial_robot_theta = theta;
        adjustment_x_ = initial_robot_x - odom_x_;
        adjustment_y_ = initial_robot_y - odom_y_;
        adjustment_theta_ = initial_robot_theta - odom_theta_;

        ROS_INFO("ROBOT POSE: x = %.3f, y = %.3f, theta = %.3f degrees", xr, yr, theta * 180.0 / M_PI);
        if (verbose_) {
            cv::imshow("ArUco Markers", output_image);
            cv::waitKey(1);
        }

        return true;
    }
}

bool RobotLocalizationNode::computeAbsolutePoseWithDepth() {
    if (latest_image_.empty() || latest_depth_.empty()) {
        ROS_WARN("No image or depth data available");
        return false;
    }

    // Detect ArUco markers
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    cv::aruco::detectMarkers(latest_image_, dictionary, marker_corners, marker_ids);

    // Draw and publish marker image
    cv::Mat output_image = latest_image_.clone();
    if (!marker_ids.empty()) {
        cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
    }
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_image).toImageMsg();
    image_pub_.publish(img_msg);
    if (verbose_) {
        ROS_INFO("Published marker image with %zu markers", marker_ids.size());
    }

    if (marker_ids.size() < 3) {
        ROS_WARN("Detected %zu markers, need at least 3", marker_ids.size());
        return false;
    }


    // // Sort by position using indices
    // if (!marker_ids.empty()) {
    //     // Create index vector [0, 1, 2, 3, ...]
    //     std::vector<size_t> indices(marker_ids.size());
    //     std::iota(indices.begin(), indices.end(), 0);
        
    //     // Sort indices by marker center x-coordinate
    //     std::sort(indices.begin(), indices.end(), 
    //             [&marker_corners, &marker_ids](size_t a, size_t b) {
    //                 float center_a_x = (marker_corners[a][0].x + marker_corners[a][1].x + 
    //                                     marker_corners[a][2].x + marker_corners[a][3].x) / 4.0f;
    //                 float center_b_x = (marker_corners[b][0].x + marker_corners[b][1].x + 
    //                                     marker_corners[b][2].x + marker_corners[b][3].x) / 4.0f;
                    
    //                 // Primary sort: x-coordinate  
    //                 if (std::abs(center_a_x - center_b_x) > 1.0f) {
    //                     return center_a_x < center_b_x;
    //                 }
                    
    //                 // Secondary sort: marker ID (for consistent ordering)
    //                 return marker_ids[a] < marker_ids[b];
    //             });
        
    //     // Create sorted copies using the sorted indices
    //     std::vector<int> sorted_ids;
    //     std::vector<std::vector<cv::Point2f>> sorted_corners;
        
    //     for (size_t idx : indices) {
    //         sorted_ids.push_back(marker_ids[idx]);
    //         sorted_corners.push_back(marker_corners[idx]);
    //     }
        
    //     // Replace original vectors
    //     marker_ids = std::move(sorted_ids);
    //     marker_corners = std::move(sorted_corners);
    // }

    // Sort by marker ids using indices
    if (!marker_ids.empty()) {
        // Create index vector and sort indices instead
        std::vector<size_t> indices(marker_ids.size());
        std::iota(indices.begin(), indices.end(), 0);

        // Sort indices by marker ID
        std::sort(indices.begin(), indices.end(), 
                [&marker_ids](size_t a, size_t b) { return marker_ids[a] < marker_ids[b]; });

        // Create sorted copies
        std::vector<int> sorted_ids;
        std::vector<std::vector<cv::Point2f>> sorted_corners;
        for (size_t idx : indices) {
            sorted_ids.push_back(marker_ids[idx]);
            sorted_corners.push_back(marker_corners[idx]);
        }

        // Replace original vectors
        marker_ids = std::move(sorted_ids);
        marker_corners = std::move(sorted_corners);
    }

    // Print detected markers
    if (!marker_ids.empty()) {
        ROS_INFO("Detected markers:");
        for (size_t i = 0; i < marker_ids.size(); ++i) {
            std::stringstream ss;
            ss << "Marker " << marker_ids[i] << " corners: ";
            for (size_t j = 0; j < marker_corners[i].size(); ++j) {
                ss << "(" << marker_corners[i][j].x << "," << marker_corners[i][j].y << ")";
                if (j < marker_corners[i].size() - 1) ss << " ";
            }
            ROS_INFO("%s", ss.str().c_str());
        }
    } else {
        ROS_INFO("No markers detected");
    }


    // Get distances from depth image
    std::vector<std::tuple<int, double, double, double>> markers; // id, x, y, distance
    for (size_t i = 0; i < marker_ids.size(); ++i) {
        auto& corners = marker_corners[i];
        double cx = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
        double cy = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0;
        float distance = latest_depth_.at<float>(int(cy), int(cx)) / 1000.0; // Convert mm to m
        if (projected_landmarks_.find(marker_ids[i]) != projected_landmarks_.end() && !std::isnan(distance)) {
            markers.push_back({marker_ids[i], projected_landmarks_[marker_ids[i]].first, projected_landmarks_[marker_ids[i]].second, distance});
        }
        if (verbose_) {
            ROS_INFO("Marker ID %d: Distance = %.3f m", marker_ids[i], distance);
        }
    }

    std::sort(markers.begin(), markers.end(), 
        [](const std::tuple<int, double, double, double>& a, 
        const std::tuple<int, double, double, double>& b) {
            return std::get<3>(a) < std::get<3>(b); // Sort by distance (closest first)
        });

    if (verbose_) {
        ROS_INFO("Detected %zu markers:", markers.size());
        for (const auto& marker : markers) {
            ROS_INFO("  Marker ID %d: Position (%.3f, %.3f), Distance = %.3f m", 
                    std::get<0>(marker), std::get<1>(marker), std::get<2>(marker), std::get<3>(marker));
        }
    }

    if (markers.size() < 3) {
        ROS_WARN("Insufficient valid depth measurements");
        return false;
    }

    // Solve for (xr, yr) using three markers
    double x1 = std::get<1>(markers[0]), y1 = std::get<2>(markers[0]), d1 = std::get<3>(markers[0]);
    double x2 = std::get<1>(markers[1]), y2 = std::get<2>(markers[1]), d2 = std::get<3>(markers[1]);
    double x3 = std::get<1>(markers[2]), y3 = std::get<2>(markers[2]), d3 = std::get<3>(markers[2]);

    // Check for collinear markers
    double cross_product = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    if (std::abs(cross_product) < 0.01) {
        ROS_WARN("Markers are nearly collinear, triangulation may be inaccurate");
        return false;
    }

    // Intersection of two circles)
    double x12 = x2 - x1, y12 = y2 - y1;
    double d12 = std::sqrt(x12 * x12 + y12 * y12);
    double a = (d1 * d1 - d2 * d2 + d12 * d12) / (2.0 * d12);
    double h = std::sqrt(d1 * d1 - a * a);
    double xm = x1 + a * x12 / d12;
    double ym = y1 + a * y12 / d12;

    double xr1 = xm + h * (y2 - y1) / d12;
    double yr1 = ym - h * (x2 - x1) / d12;
    double xr2 = xm - h * (y2 - y1) / d12;
    double yr2 = ym + h * (x2 - x1) / d12;

    if (verbose_) {
        ROS_INFO("Circle intersection solutions:");
        ROS_INFO("Solution 1: %.3f, %.3f", xr1, yr1);
        ROS_INFO("Solution 2: %.3f, %.3f", xr2, yr2);
    }

    // Check which solution satisfies the third circle
    double dist1 = std::sqrt((xr1 - x3) * (xr1 - x3) + (yr1 - y3) * (yr1 - y3));
    double dist2 = std::sqrt((xr2 - x3) * (xr2 - x3) + (yr2 - y3) * (yr2 - y3));
    double xr, yr;
    if (std::abs(dist1 - d3) < std::abs(dist2 - d3)) {
        xr = xr1;
        yr = yr1;
    } else {
        xr = xr2;
        yr = yr2;
    }

    // Compute yaw
    double theta = computeYaw({marker_corners[0][0].x, marker_corners[0][0].y}, x1, y1, xr, yr);

    // Update pose
    baseline_pose_.x = xr;
    baseline_pose_.y = yr;
    baseline_pose_.theta = theta; // Radians
    current_pose_ = baseline_pose_;
    last_absolute_pose_time_ = ros::Time::now();

    // Alternate update pose
    initial_robot_x = xr;
    initial_robot_y = yr;
    initial_robot_theta = theta;
    adjustment_x_ = initial_robot_x - odom_x_;
    adjustment_y_ = initial_robot_y - odom_y_;
    adjustment_theta_ = initial_robot_theta - odom_theta_;

    ROS_INFO("Robot Pose (Depth): x=%.3f, y=%.3f, theta=%.3f degrees", xr, yr, theta * 180.0 / M_PI);
    if (verbose_){
        cv::imshow("ArUco Markers", output_image);
        cv::waitKey(1);
    }


    // publishPose();
    return true;
}

double RobotLocalizationNode::computeAngle(const std::pair<double, double>& center1, const std::pair<double, double>& center2) {
    // Output of marker center
    if (verbose_){
        ROS_INFO("Point 1: (%.2f, %.2f), Point 2: (%.2f, %.2f)", center1.first, center1.second, center2.first, center2.second);
    }
    
    // Convert pixel coordinates to normalized camera coordinates
    double x1_norm = (center1.first - cx_) / fx_;
    double y1_norm = (center1.second - cy_) / fy_;
    double x2_norm = (center2.first - cx_) / fx_;
    double y2_norm = (center2.second - cy_) / fy_;
    
    // Create 3D direction vectors
    cv::Vec3d dir1(x1_norm, y1_norm, 1.0);
    cv::Vec3d dir2(x2_norm, y2_norm, 1.0);
    
    // Normalize the vectors
    dir1 = dir1 / cv::norm(dir1);
    dir2 = dir2 / cv::norm(dir2);
    
    // Compute angle using dot product
    double dot_product = dir1.dot(dir2);
    // Clamp manually to avoid numerical errors in acos
    if (dot_product > 1.0) dot_product = 1.0;
    if (dot_product < -1.0) dot_product = -1.0;
    
    double angle_rad = std::acos(dot_product);
    
    return angle_rad * 180.0 / M_PI; // Convert to degrees
}

double RobotLocalizationNode::computeYaw(const std::pair<double, double>& marker_center, double marker_x, double marker_y, double robot_x, double robot_y) {
    // Compute direction to marker in world frame
    double dx = marker_x - robot_x;
    double dy = marker_y - robot_y;
    double world_angle = std::atan2(dy, dx);
    
    // Convert pixel offset to angle in camera frame (horizontal only)
    double pixel_offset_x = marker_center.first - cx_;
    double image_angle = std::atan(pixel_offset_x / fx_);

    double raw_yaw = world_angle + image_angle - (use_head_yaw_ ? head_yaw_ : 0.0);
    
    // Normalize raw_yaw first
    while (raw_yaw > M_PI) raw_yaw -= 2 * M_PI;
    while (raw_yaw < -M_PI) raw_yaw += 2 * M_PI;
    
    // Calibrate yaw
    double calibrated_yaw = raw_yaw + (346.0 * M_PI / 180.0);
    
    // Final normalization
    while (calibrated_yaw > M_PI) calibrated_yaw -= 2 * M_PI;
    while (calibrated_yaw < -M_PI) calibrated_yaw += 2 * M_PI;

    double yaw_positive = calibrated_yaw * 180.0 / M_PI;
    if (yaw_positive < 0) {
        yaw_positive += 360.0;
    }

    if (verbose_) {
        ROS_INFO("Theta=%.3f degrees (positive: %.3f degrees)", calibrated_yaw * 180.0 / M_PI, yaw_positive);
    }

    return calibrated_yaw;
}

int RobotLocalizationNode::circle_centre(double x1, double y1, double x2, double y2, double alpha, double *xc1, double *yc1, double *xc2, double *yc2, double *r) {
    bool debug = false;
    double d, h, theta, beta, delta_x, delta_y, alphar, temp_x, temp_y;
    
    alphar =  3.14159 * (alpha / 180.0); // convert to radians
    
    d = sqrt((x1-x2) * (x1-x2) + (y1-y2) * (y1-y2));
    
    if (alpha == 0 || alpha == 180) {
        h = 0;
        *r = 0;
    }
    else {
        h  = (d / 2) / tan(alphar);
        *r = (d / 2) / sin(alphar);
    }
    
    theta = atan2(y2-y1, x2-x1);
    beta  = theta - (3.14159 / 2);
    delta_x = h * cos(beta);
    delta_y = h * sin(beta);
    
    *xc1 = (x1 + x2)/2 - delta_x;  
    *yc1 = (y1 + y2)/2 - delta_y;  
    
    /* note: there is a second circle that satisfies the required condition */
    /* it is a reflection of the first circle in the given chord            */ 
    /* its centre is obtained by adding the delta_x and delta_y             */
    
    *xc2 = (x1 + x2)/2 + delta_x; 
    *yc2 = (y1 + y2)/2 + delta_y; 
        
    /* sort them in order of increasing distance from the origin */
    
    if ((*xc1 * *xc1 + *yc1 * *yc1) > (*xc2 * *xc2 + *yc2 * *yc2)) {
        temp_x = *xc1;
        *xc1 = *xc2;
        *xc2 = temp_x;
    
        temp_y = *yc1;
        *yc1 = *yc2;
        *yc2 = temp_y;
    }
    return 1;
}

int RobotLocalizationNode::circle_circle_intersection(double x0, double y0, double r0, double x1, double y1, double r1, double *xi, double *yi, double *xi_prime, double *yi_prime) {
    double a, dx, dy, d, h, rx, ry;
    double x2, y2;

    /* dx and dy are the vertical and horizontal distances between
    * the circle centers.
    */
    dx = x1 - x0;
    dy = y1 - y0;

    /* Determine the straight-line distance between the centers. */
    //d = sqrt((dy*dy) + (dx*dx));
    d = hypot(dx,dy); // Suggested by Keith Briggs

    /* Check for solvability. */
    if (d > (r0 + r1))
    {
    /* no solution. circles do not intersect. */
    return 0;
    }
    if (d < fabs(r0 - r1))
    {
    /* no solution. one circle is contained in the other */
    return 0;
    }

    /* 'point 2' is the point where the line through the circle
    * intersection points crosses the line between the circle
    * centers.  
    */

    /* Determine the distance from point 0 to point 2. */
    a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d);

    /* Determine the coordinates of point 2. */
    x2 = x0 + (dx * a/d);
    y2 = y0 + (dy * a/d);

    /* Determine the distance from point 2 to either of the
    * intersection points.
    */
    h = sqrt((r0*r0) - (a*a));

    /* Now determine the offsets of the intersection points from
    * point 2.
    */
    rx = -dy * (h/d);
    ry = dx * (h/d);

    /* Determine the absolute intersection points. */
    *xi = x2 + rx;
    *xi_prime = x2 - rx;
    *yi = y2 + ry;
    *yi_prime = y2 - ry;

    return 1;
}

void RobotLocalizationNode::publishPose() {
    pose_pub_.publish(current_pose_);
}

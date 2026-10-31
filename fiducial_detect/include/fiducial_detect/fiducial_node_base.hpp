#pragma once

#include <memory>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <fiducial_msgs/msg/fiducial_transform_array.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/calib3d.hpp>

#include "fiducial_core/observation.hpp"

namespace fiducial_detect {

template <typename Derived, typename DetectorType>
class FiducialNodeBase : public rclcpp::Node {
public:
    explicit FiducialNodeBase(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node(node_name, options)
    {
        declareCommonParameters();
        setupPublishers();
        setupSubscribers();
        setupParameterCallback();

        RCLCPP_INFO(this->get_logger(), "Fiducial detector component '%s' initialized with real-time SensorDataQoS.", node_name.c_str());
    }

    virtual ~FiducialNodeBase() = default;

protected:
    DetectorType detector_;

    void declareCommonParameters() {
        this->declare_parameter<bool>("publish_tf", true);
        this->declare_parameter<std::string>("tf_prefix", "fiducial_");
        this->declare_parameter<double>("fiducial_len", 0.14);
        this->declare_parameter<std::string>("fiducial_len_override", "");
        this->declare_parameter<std::string>("ignore_fiducials", "");
        this->declare_parameter<bool>("publish_legacy_fiducial_msgs", false);
        this->declare_parameter<int>("dock_marker_id", 105);

        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        tf_prefix_ = this->get_parameter("tf_prefix").as_string();
        default_len_ = this->get_parameter("fiducial_len").as_double();
        publish_legacy_ = this->get_parameter("publish_legacy_fiducial_msgs").as_bool();
        dock_marker_id_ = this->get_parameter("dock_marker_id").as_int();

        parseIgnoreString(this->get_parameter("ignore_fiducials").as_string());
        parseOverrideString(this->get_parameter("fiducial_len_override").as_string());
    }

    void setupPublishers() {
        // Standard Nav2 Perception Stream (REP-145)
        pub_detection3d_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
            "detections/fiducials", rclcpp::SensorDataQoS());

        // Standard LaMa / Legacy Dual-Streaming Topic
        pub_detection2d_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
            "fiducial_transforms", 10);

        // RViz2 Multi-Pose Visualization
        pub_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "fiducial_poses", 10);

        // State Estimation EKF Fusion
        pub_pose_cov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "fiducial_pose", 10);

        // Nav2 Docking Framework Target
        pub_dock_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "dock_pose", 10);

        // Backward Compatibility for Legacy Bag Playback
        pub_legacy_fta_ = this->create_publisher<fiducial_msgs::msg::FiducialTransformArray>(
            "fiducial_transforms_legacy", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

    void setupSubscribers() {
        // Ingest camera stream with SensorDataQoS (KeepLast 1, BestEffort)
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", rclcpp::SensorDataQoS(),
            std::bind(&FiducialNodeBase::imageCallback, this, std::placeholders::_1));

        sub_cam_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", rclcpp::SensorDataQoS(),
            std::bind(&FiducialNodeBase::cameraInfoCallback, this, std::placeholders::_1));
    }

    void setupParameterCallback() {
        param_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto& param : parameters) {
                    if (param.get_name() == "publish_tf") {
                        publish_tf_ = param.as_bool();
                    } else if (param.get_name() == "tf_prefix") {
                        tf_prefix_ = param.as_string();
                    } else if (param.get_name() == "publish_legacy_fiducial_msgs") {
                        publish_legacy_ = param.as_bool();
                    } else if (param.get_name() == "dock_marker_id") {
                        dock_marker_id_ = param.as_int();
                    } else if (param.get_name() == "ignore_fiducials") {
                        parseIgnoreString(param.as_string());
                    } else if (param.get_name() == "fiducial_len_override") {
                        parseOverrideString(param.as_string());
                    }
                }
                return result;
            });
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (!have_camera_info_) {
            camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
            if (!msg->d.empty()) {
                dist_coeffs_ = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F, const_cast<double*>(msg->d.data())).clone();
            } else {
                dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
            }
            have_camera_info_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera intrinsics acquired: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                        camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1),
                        camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (!have_camera_info_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for camera_info before processing frames...");
            return;
        }

        // Direct zero-copy loan pointer share
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        std::vector<fiducial_core::FiducialObservation> observations;
        detector_.detect(cv_ptr->image, camera_matrix_, dist_coeffs_, observations);

        if (observations.empty()) {
            return;
        }

        // Standard Nav2 3D Detection Array
        vision_msgs::msg::Detection3DArray det3d_array;
        det3d_array.header = msg->header;

        // Lazy Evaluation checks to guarantee 0.0% CPU overhead when legacy topics are unsubscribed
        const bool need_2d = (pub_detection2d_->get_subscription_count() > 0);
        const bool need_poses = (pub_pose_array_->get_subscription_count() > 0);
        const bool need_pose_cov = (pub_pose_cov_->get_subscription_count() > 0);
        const bool need_dock = (pub_dock_pose_->get_subscription_count() > 0);
        const bool need_legacy = publish_legacy_ && (pub_legacy_fta_->get_subscription_count() > 0);

        vision_msgs::msg::Detection2DArray det2d_array;
        if (need_2d) {
            det2d_array.header = msg->header;
        }

        geometry_msgs::msg::PoseArray pose_array;
        if (need_poses) {
            pose_array.header = msg->header;
        }

        fiducial_msgs::msg::FiducialTransformArray legacy_fta;
        if (need_legacy) {
            legacy_fta.header = msg->header;
        }

        int closest_obs_idx = -1;
        double min_dist = 1e9;

        for (size_t i = 0; i < observations.size(); ++i) {
            const auto& obs = observations[i];
            if (ignore_ids_.find(obs.id) != ignore_ids_.end()) {
                continue;
            }

            // Convert Rodrigues rvec to Quaternion
            cv::Mat R;
            cv::Rodrigues(obs.rvec, R);
            tf2::Matrix3x3 tf_rot(
                R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
            tf2::Quaternion q;
            tf_rot.getRotation(q);

            geometry_msgs::msg::Pose pose;
            pose.position.x = obs.tvec[0];
            pose.position.y = obs.tvec[1];
            pose.position.z = obs.tvec[2];
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();

            // 1. Official Nav2 Detection3D message (REP-145)
            vision_msgs::msg::Detection3D det3d;
            det3d.header = msg->header;
            vision_msgs::msg::ObjectHypothesisWithPose hyp3d;
            hyp3d.hypothesis.class_id = std::to_string(obs.id);
            hyp3d.hypothesis.score = obs.score;
            hyp3d.pose.pose = pose;
            hyp3d.pose.covariance = obs.covariance;
            det3d.results.push_back(hyp3d);
            det3d.bbox.center.position = pose.position;
            det3d.bbox.center.orientation = pose.orientation;
            det3d.bbox.size.x = obs.marker_length;
            det3d.bbox.size.y = obs.marker_length;
            det3d.bbox.size.z = 0.01;
            det3d_array.detections.push_back(det3d);

            // 2. Broadcast TF (camera_optical_frame -> fiducial_<id>)
            if (publish_tf_) {
                geometry_msgs::msg::TransformStamped ts;
                ts.header = msg->header;
                ts.child_frame_id = tf_prefix_ + std::to_string(obs.id);
                ts.transform.translation.x = pose.position.x;
                ts.transform.translation.y = pose.position.y;
                ts.transform.translation.z = pose.position.z;
                ts.transform.rotation = pose.orientation;
                tf_broadcaster_->sendTransform(ts);
            }

            // 3. Lazy Detection2DArray stream for LaMa SLAM / Legacy Ubiquity Nodes
            if (need_2d) {
                vision_msgs::msg::Detection2D det2d;
                det2d.header = msg->header;
                vision_msgs::msg::ObjectHypothesisWithPose hyp2d;
                hyp2d.hypothesis.class_id = std::to_string(obs.id);
                hyp2d.hypothesis.score = obs.score;
                hyp2d.pose.pose = pose;
                hyp2d.pose.covariance = obs.covariance; // Analytical IPPE covariance
                det2d.results.push_back(hyp2d);
                det2d_array.detections.push_back(det2d);
            }

            // 4. Lazy RViz2 PoseArray
            if (need_poses) {
                pose_array.poses.push_back(pose);
            }

            // 5. Check closest marker for EKF
            double d = cv::norm(obs.tvec);
            if (d < min_dist) {
                min_dist = d;
                closest_obs_idx = static_cast<int>(i);
            }

            // 6. Autonomous Docking Target for nav2_docking
            if (need_dock && obs.id == dock_marker_id_) {
                geometry_msgs::msg::PoseStamped dock_pose;
                dock_pose.header = msg->header;
                dock_pose.pose = pose;
                pub_dock_pose_->publish(dock_pose);
            }

            // 7. Legacy FiducialTransformArray
            if (need_legacy) {
                fiducial_msgs::msg::FiducialTransform ft;
                ft.fiducial_id = obs.id;
                ft.transform.translation.x = pose.position.x;
                ft.transform.translation.y = pose.position.y;
                ft.transform.translation.z = pose.position.z;
                ft.transform.rotation = pose.orientation;
                ft.image_error = obs.reprojection_error;
                ft.object_error = obs.object_error;
                ft.fiducial_area = 0.0;
                legacy_fta.transforms.push_back(ft);
            }
        }

        // Publish active streams
        pub_detection3d_->publish(det3d_array);

        if (need_2d && !det2d_array.detections.empty()) {
            pub_detection2d_->publish(det2d_array);
        }
        if (need_poses && !pose_array.poses.empty()) {
            pub_pose_array_->publish(pose_array);
        }
        if (need_legacy && !legacy_fta.transforms.empty()) {
            pub_legacy_fta_->publish(legacy_fta);
        }
        if (need_pose_cov && closest_obs_idx >= 0) {
            const auto& obs = observations[closest_obs_idx];
            geometry_msgs::msg::PoseWithCovarianceStamped pwc;
            pwc.header = msg->header;
            pwc.pose.pose.position.x = obs.tvec[0];
            pwc.pose.pose.position.y = obs.tvec[1];
            pwc.pose.pose.position.z = obs.tvec[2];
            cv::Mat R;
            cv::Rodrigues(obs.rvec, R);
            tf2::Matrix3x3 tf_rot(
                R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
            tf2::Quaternion q;
            tf_rot.getRotation(q);
            pwc.pose.pose.orientation.x = q.x();
            pwc.pose.pose.orientation.y = q.y();
            pwc.pose.pose.orientation.z = q.z();
            pwc.pose.pose.orientation.w = q.w();
            pwc.pose.covariance = obs.covariance;
            pub_pose_cov_->publish(pwc);
        }
    }

    void parseIgnoreString(const std::string& str) {
        ignore_ids_.clear();
        if (str.empty()) return;
        std::stringstream ss(str);
        std::string token;
        while (std::getline(ss, token, ',')) {
            token.erase(0, token.find_first_not_of(" \t\n\r"));
            token.erase(token.find_last_not_of(" \t\n\r") + 1);
            if (token.empty()) continue;
            auto dash_pos = token.find('-');
            if (dash_pos != std::string::npos) {
                try {
                    int start = std::stoi(token.substr(0, dash_pos));
                    int end = std::stoi(token.substr(dash_pos + 1));
                    for (int i = start; i <= end; ++i) {
                        ignore_ids_.insert(i);
                    }
                } catch (...) {}
            } else {
                try {
                    ignore_ids_.insert(std::stoi(token));
                } catch (...) {}
            }
        }
    }

    void parseOverrideString(const std::string& str) {
        detector_.clearMarkerLengthOverrides();
        if (str.empty()) return;
        std::stringstream ss(str);
        std::string element;
        while (std::getline(ss, element, ',')) {
            element.erase(0, element.find_first_not_of(" \t\n\r"));
            element.erase(element.find_last_not_of(" \t\n\r") + 1);
            if (element.empty()) continue;

            auto colon_pos = element.find(':');
            if (colon_pos == std::string::npos) continue;

            std::string part0 = element.substr(0, colon_pos);
            std::string part1 = element.substr(colon_pos + 1);
            part0.erase(0, part0.find_first_not_of(" \t\n\r"));
            part0.erase(part0.find_last_not_of(" \t\n\r") + 1);
            part1.erase(0, part1.find_first_not_of(" \t\n\r"));
            part1.erase(part1.find_last_not_of(" \t\n\r") + 1);

            double len = 0.0;
            std::string range_str;
            if (part0.find('-') != std::string::npos) {
                range_str = part0;
                try { len = std::stod(part1); } catch (...) { continue; }
            } else if (part1.find('-') != std::string::npos) {
                range_str = part1;
                try { len = std::stod(part0); } catch (...) { continue; }
            } else {
                try {
                    len = std::stod(part0);
                    range_str = part1;
                } catch (...) {
                    try {
                        len = std::stod(part1);
                        range_str = part0;
                    } catch (...) { continue; }
                }
            }

            auto dash_pos = range_str.find('-');
            if (dash_pos != std::string::npos) {
                try {
                    int start = std::stoi(range_str.substr(0, dash_pos));
                    int end = std::stoi(range_str.substr(dash_pos + 1));
                    for (int j = start; j <= end; ++j) {
                        detector_.setMarkerLengthOverride(j, len);
                    }
                } catch (...) {}
            } else {
                try {
                    int fid = std::stoi(range_str);
                    detector_.setMarkerLengthOverride(fid, len);
                } catch (...) {}
            }
        }
    }

private:
    bool publish_tf_{true};
    std::string tf_prefix_{"fiducial_"};
    double default_len_{0.14};
    bool publish_legacy_{false};
    int dock_marker_id_{105};

    std::set<int> ignore_ids_;
    bool have_camera_info_{false};
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cam_info_;

    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_detection3d_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_detection2d_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose_array_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_dock_pose_;
    rclcpp::Publisher<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr pub_legacy_fta_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

} // namespace fiducial_detect

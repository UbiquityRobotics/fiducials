#ifndef TRANSFORM_VARIANCE_H
#define TRANSFORM_VARIANCE_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TransformWithVariance {
public:
    tf2::Transform transform;
    double variance;

    TransformWithVariance() = default;

    // Constructors that make a transform out of the different implementations
    TransformWithVariance(const tf2::Transform& t, double var) : transform(t), variance(var){};
    TransformWithVariance(const geometry_msgs::Transform& t, double var) : variance(var) {
        fromMsg(t, transform);
    };
    TransformWithVariance(const tf2::Vector3& tvec, const tf2::Quaternion& q, double var)
        : transform(q, tvec), variance(var){};

    // Used to combine this transform with another one increasing total variance
    TransformWithVariance& operator*=(const TransformWithVariance& rhs) {
        // Update this transform, increasing variance
        transform *= rhs.transform;

        // Do simple addition of the variances
        // In a multi-variate case RMS may be more correct
        variance += rhs.variance;
        return *this;
    }
    friend TransformWithVariance operator*(TransformWithVariance lhs,
                                           const TransformWithVariance& rhs) {
        lhs *= rhs;
        return lhs;
    }
    friend tf2::Stamped<TransformWithVariance> operator*(
        tf2::Stamped<TransformWithVariance> lhs, const tf2::Stamped<TransformWithVariance>& rhs) {
        lhs *= rhs;
        return lhs;
    }

    // Used to combine this transform with another one keeping variance the same
    TransformWithVariance& operator*=(const tf2::Transform& rhs) {
        transform *= rhs;
        // No need to change the variance, we are assuming that rhs has variance of 0
        return *this;
    }
    friend TransformWithVariance operator*(TransformWithVariance lhs, const tf2::Transform& rhs) {
        lhs *= rhs;
        return lhs;
    }
    friend TransformWithVariance operator*(tf2::Transform lhs, const TransformWithVariance& rhs) {
        lhs *= rhs.transform;
        return TransformWithVariance(lhs, rhs.variance);
    }

    // Update this transform with a new one, with variances as weights
    // combine variances using David method
    void update(const TransformWithVariance& newT);
};

// Weighted average of 2 transforms, with new variance
TransformWithVariance averageTransforms(const TransformWithVariance& t1,
                                        const TransformWithVariance& t2);

inline geometry_msgs::PoseWithCovarianceStamped toPose(
    const tf2::Stamped<TransformWithVariance>& in) {
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = in.stamp_;
    msg.header.frame_id = in.frame_id_;

    toMsg(in.transform, msg.pose.pose);

    std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0);

    for (int i = 0; i <= 5; i++) {
        msg.pose.covariance[i * 6 + i] = in.variance;  // Fill the diagonal
    }

    return msg;
}

inline geometry_msgs::TransformStamped toMsg(const tf2::Stamped<TransformWithVariance>& in) {
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = in.stamp_;
    msg.header.frame_id = in.frame_id_;
    msg.transform = toMsg(in.transform);

    return msg;
}

#endif

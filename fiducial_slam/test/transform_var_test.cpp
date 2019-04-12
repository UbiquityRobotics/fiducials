#include <gtest/gtest.h>

#include <fiducial_slam/transform_with_variance.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

static tf2::Quaternion quaternionfromrpy(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

TEST (TransformWithVariance, simple_fusion) {
    auto t1 = tf2::Transform(tf2::Quaternion(0,0,0,1), tf2::Vector3(0,0,0));
    auto t2 = tf2::Transform(tf2::Quaternion(0,0,0,1), tf2::Vector3(0.1,0,0));

    auto tv1 = TransformWithVariance(t1, 0.3);
    auto tv2 = TransformWithVariance(t2, 0.3);

    auto out_tv = averageTransforms(tv1, tv2);

    // Make sure that the new position is between the originals 
    ASSERT_GT(out_tv.transform.getOrigin().x(), 0); 
    ASSERT_LT(out_tv.transform.getOrigin().x(), 0.1);

    // The new variance should be less than the origial, but non negative
    ASSERT_LT(out_tv.variance, 0.3);
    ASSERT_GT(out_tv.variance, 0);
}

TEST (TransformWithVariance, simple_rotation_fusion) {
    auto t1 = tf2::Transform(quaternionfromrpy(0,0,0), tf2::Vector3(0,0,0));
    auto t2 = tf2::Transform(quaternionfromrpy(0.1,0,0), tf2::Vector3(0,0,0));

    auto tv1 = TransformWithVariance(t1, 0.3);
    auto tv2 = TransformWithVariance(t2, 0.3);

    auto out_tv = averageTransforms(tv1, tv2);

    // Make sure that the new position is between the originals 
    ASSERT_GT(out_tv.transform.getRotation().getAngle(), 0); 
    ASSERT_LT(out_tv.transform.getRotation().getAngle(), 0.1);

    // The new variance should be less than the origial, but non negative
    ASSERT_LT(out_tv.variance, 0.3);
    ASSERT_GT(out_tv.variance, 0);
}

TEST (TransformWithVariance, same_fusion_iterative) {
    auto t1 = tf2::Transform(tf2::Quaternion(0,0,0,1), tf2::Vector3(0,0,0));
    auto t2 = tf2::Transform(tf2::Quaternion(0,0,0,1), tf2::Vector3(0,0,0));

    auto tv1 = TransformWithVariance(t1, 0.3);
    auto tv2 = TransformWithVariance(t2, 0.3);

    auto out_tv = averageTransforms(tv1, tv2);

    // Make sure that the new position is between the originals 
    ASSERT_DOUBLE_EQ(out_tv.transform.getOrigin().x(), 0); 

    // The new variance should be less than the origial, but non negative
    ASSERT_LT(out_tv.variance, 0.3);
    ASSERT_GT(out_tv.variance, 0);

    // Ensure no catastrophic effects when fusing over and over
    for (int i = 0; i < 10000; ++i) {
       out_tv.update(tv2); 
       // Make sure that the new position is between the originals 
       ASSERT_DOUBLE_EQ(out_tv.transform.getOrigin().x(), 0); 

       // The new variance should be less than the origial, but non negative
       ASSERT_LT(out_tv.variance, 0.3);
       ASSERT_GT(out_tv.variance, 1e-9);
    }
}

TEST (TransformWithVariance, outlier_with_large_variance) {
    auto t1 = tf2::Transform(quaternionfromrpy(0,0,0), tf2::Vector3(0,0,0));
    auto t2 = tf2::Transform(quaternionfromrpy(0,0,0), tf2::Vector3(0.1,0,0));
    auto t3 = tf2::Transform(quaternionfromrpy(0,0,0), tf2::Vector3(0.1,0,0));
    auto t4 = tf2::Transform(quaternionfromrpy(0,1,0), tf2::Vector3(1.0,0,0));

    auto tv1 = TransformWithVariance(t1, 0.2);
    auto tv2 = TransformWithVariance(t2, 0.2);
    auto tv3 = TransformWithVariance(t3, 0.2);
    auto tv4 = TransformWithVariance(t4, 2.0);

    auto out_tv = averageTransforms(tv1, tv2);
    out_tv = averageTransforms(out_tv, tv3);
    out_tv = averageTransforms(out_tv, tv4);

    // Make sure that the new position is between the originals 
    ASSERT_GT(out_tv.transform.getOrigin().x(), 0); 
    ASSERT_LT(out_tv.transform.getOrigin().x(), 1.0);
    ASSERT_GT(out_tv.transform.getRotation().getAngle(), 0); 
    ASSERT_LT(out_tv.transform.getRotation().getAngle(), 1.0);

    // The new variance shouldn't be large
    ASSERT_LT(out_tv.variance, 1.0);
    ASSERT_GT(out_tv.variance, 0);

    // The new mean shouldn't be affected much by the outlier
    ASSERT_NEAR(out_tv.transform.getOrigin().x(), 0.1, 0.05);
    ASSERT_NEAR(out_tv.transform.getRotation().getAngle(), 0, 0.1);
}

TEST (TransformWithVariance, different_with_similar_variance) {
    auto t1 = tf2::Transform(quaternionfromrpy(0,0,0), tf2::Vector3(0,0,0));
    auto t2 = tf2::Transform(quaternionfromrpy(1,0,0), tf2::Vector3(1.0,0,0));

    auto tv1 = TransformWithVariance(t1, 0.1);
    auto tv2 = TransformWithVariance(t2, 0.2);

    auto out_tv = averageTransforms(tv1, tv2);

    // Make sure that the new position is between the originals 
    ASSERT_GT(out_tv.transform.getOrigin().x(), 0); 
    ASSERT_LT(out_tv.transform.getOrigin().x(), 1.0);
    ASSERT_GT(out_tv.transform.getRotation().getAngle(), 0); 
    ASSERT_LT(out_tv.transform.getRotation().getAngle(), 1.0);

    // The new variance should be larger because of disagreeing data
    ASSERT_GT(out_tv.variance, 0.2);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


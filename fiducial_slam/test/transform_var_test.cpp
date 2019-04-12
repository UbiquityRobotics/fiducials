#include <gtest/gtest.h>

#include <fiducial_slam/transform_with_variance.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

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
       ASSERT_GT(out_tv.variance, 0);
    }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


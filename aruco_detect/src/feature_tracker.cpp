/*
 * Copyright (c) 2017, Ubiquity Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#include "aruco_detect/feature_tracker.h"

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
 
using namespace cv;
using namespace std;

FeatureTracker::FeatureTracker()
{
  frameNum = 0;
}

void FeatureTracker::findObjects(const cv::Mat& image, 
                                 std::map<int, cv::Rect>& objects)
{
  // TODO: make these params
  int maxCount = 10;
  double quality = 0.1;
  double minDist = 5.;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  frameNum++;
  ros::Time startTime;
  startTime = ros::Time::now();

  if (objects.size() == 0) {
    return;
  }
  cvtColor(image, currentImage, cv::COLOR_RGB2GRAY);

  ROS_INFO("cvtColor in %f seconds\n", (ros::Time::now() - startTime).toSec());

  map<int, cv::Rect>::iterator it;
  for (it = objects.begin(); it != objects.end(); it++) {
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8U);
    Mat roi(mask, it->second);
    roi = Scalar(255, 255, 255);
    
    std::vector<cv::Point2f> features;

    startTime = ros::Time::now();
     
    cv::goodFeaturesToTrack(currentImage, features, maxCount, 
                            quality, minDist, mask, blockSize, 
                            useHarrisDetector, k);

    ROS_INFO("goodFeatures in %f seconds\n", (ros::Time::now() - startTime).toSec());

    ROS_INFO("Found %d features for object %d\n", 
             (int)features.size(), it->first);

    prevFeatures[it->first] = features;
    initialFrames[it->first] = frameNum;

    for (int i = 0; i<features.size(); i++) {
      cv::circle(image, features[i], 10, cv::Scalar(255.), -1);
    }
  }
  prevImage = currentImage.clone();
}

void FeatureTracker::trackObjects(const cv::Mat& image, map<int, cv::Mat>& shifts)
{
  // TODO: params
  int maxAge = 20;
  int minFeatures = 10;
  cv::Size winSize(15, 15);
  int maxLevel = 1;
  cv::TermCriteria criteria = 
    TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 10, 0.1);
  int flags = 0;
  double minEig = 1e4;
  

  frameNum++;

  ros::Time startTime;
  startTime = ros::Time::now();

  int count = 0; 
  map<int, std::vector<cv::Point2f> >::iterator it;
  for (it = prevFeatures.begin(); it != prevFeatures.end(); it++) {
    count += it->second.size();
  }
  if (count == 0) {
    return;
  }

  cvtColor(image, currentImage, cv::COLOR_RGB2GRAY);

  ROS_INFO("cvtColor in %f seconds\n", (ros::Time::now() - startTime).toSec());

  std::vector<uchar> status;
  std::vector<float> err;

  for (it = prevFeatures.begin(); it != prevFeatures.end(); it++) {
    int age = frameNum - initialFrames[it->first];
    std::vector<cv::Point2f> features = it->second;

    if (age > maxAge || it->second.size() < 3) {
        continue;
    }

    startTime = ros::Time::now();

    features = it->second;
    cv::calcOpticalFlowPyrLK(prevImage, currentImage, it->second, features,
                             status, err, winSize, maxLevel, criteria, flags,
                             minEig);
 
    ROS_INFO("LKFlow in %f seconds\n", (ros::Time::now() - startTime).toSec());

    std::vector<cv::Point2f> goodFeatures;
    std::vector<cv::Point2f> prevGoodFeatures;
    for (int i = 0; i<features.size(); i++) {
      if (status[i]) {
        goodFeatures.push_back(features[i]);
        prevGoodFeatures.push_back(it->second[i]);
        cv::circle(image, features[i], 10, cv::Scalar(0,0,255), -1);
      }
      else {
        cv::circle(image, features[i], 10, cv::Scalar(255,0,0), -1);
      }
    }

    ROS_INFO("Tracked %d features for object %d\n", 
             (int)goodFeatures.size(), it->first);

    if (goodFeatures.size() > minFeatures) {
      startTime = ros::Time::now();

      cv::Mat T = cv::estimateRigidTransform(prevGoodFeatures, goodFeatures, false);
   
      ROS_INFO("RigidTransform in %f seconds\n", (ros::Time::now() - startTime).toSec());

      if (T.empty()) {
        ROS_INFO("Giving up on object %d\n", it->first);
        it->second.clear();
        continue;
      }

      shifts[it->first] = T;

      it->second = goodFeatures;
    }
    else {
      it->second.clear();
    }
  }

  prevImage = currentImage.clone();
}

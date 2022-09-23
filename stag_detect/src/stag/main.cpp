#include <opencv2/opencv.hpp>
#include "stag/Stag.h"

int main() {
  cv::Mat image = cv::imread("../images/1.jpg", 0);

  Stag stag(15, 7, true);

  stag.detectMarkers(image);
  stag.logResults("log/");

  return 1;
}
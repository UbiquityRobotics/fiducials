
/* 

Generate an Aruco Board object from a fiducial map

*/

#ifndef ARUCO_MAP_BOARD_H
#define ARUCO_MAP_BOARD_H

#include <math.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/aruco.hpp>

#include <list>


class MapBoard : public cv::aruco::Board {

public:
	const static int BUFSIZE = 1024;

	MapBoard() {}

	MapBoard(cv::Ptr<cv::aruco::Dictionary> dictionary) 
	{
		this->dictionary = dictionary;
	}

	static double d2r(double deg)
	{
		return deg * M_PI / 180.0;
	}

	static void print(tf2::Vector3 v)
	{
		printf("%lf, %lf, %lf\n", v.x(), v.y(), v.z());
	}

	static void print(cv::Point3f v)
	{
		printf("%lf, %lf, %lf\n", v.x, v.y, v.z);
	}

	static cv::Point3f point(tf2::Vector3 v)
	{
		return(cv::Point3f(v.x(), v.y(), v.z()));
	}

	int readMap(const char *file, double len)
	{
		FILE *fp = fopen(file, "r");
		if (fp == NULL) {
			fprintf(stderr, "Error opening %s\n", file);
			return -1;
		}

		tf2::Vector3 vx(len/2.0, 0.0, 0.0);
		tf2::Vector3 vy(0.0, len/2.0, 0.0);

		while (!feof(fp)) {
 			int id;
			double x, y, z, roll, pitch, yaw;
			char buf[BUFSIZE];
	
			if (fgets(buf, BUFSIZE - 1, fp) == NULL)
       	                break;

			if (sscanf(buf, "%d %lf %lf %lf %lf %lf %lf",
				&id, &x, &y, &z, &roll, &pitch, &yaw) == 7) {
	
				tf2::Vector3 p(x, y, z);
	
				tf2::Quaternion q;
				q.setEuler(d2r(roll), d2r(pitch), d2r(yaw));

				tf2::Vector3 axis = q.getAxis();
				double angle = q.getAngle();

				tf2::Vector3 dx = vx.rotate(axis, angle);
				tf2::Vector3 dy = vy.rotate(axis, angle);

				std::vector <cv::Point3f> opts;
				opts.push_back(point(p - dx + dy));
				opts.push_back(point(p + dx + dy));
				opts.push_back(point(p + dx - dy));
				opts.push_back(point(p - dx - dy));
				objPoints.push_back(opts);
				ids.push_back(id);
			}
		}
        fclose(fp);
		return ids.size();
	}

	void print() 
	{
		for (int i=0; i<ids.size(); i++) {
			printf("%d\n", ids[i]);
			std::vector <cv::Point3f> opts = objPoints[i];
			for (int j=0; j<opts.size(); j++) {
				MapBoard::print(opts[j]);
			}
		}
	}
};

#endif

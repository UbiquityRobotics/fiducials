
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>

#include <aruco_detect/map_board.h>

int main(int argc, char **argv)
{
	if (argc != 2) {
		fprintf(stderr, "Usage %s map\n", argv[0]);
		return 1;
	}

	MapBoard board;
	board.readMap(argv[1], 0.2);	
	board.print();
}

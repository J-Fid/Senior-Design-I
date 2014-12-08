//drawImageSimulator.cpp

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <errno.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <fcntl.h> 

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../include/drawImageSimulator.h"
#include "../include/Line.h"
#include "../include/point.h"
#include "../include/drawing.h"

/**
Simulator class to simulate the full functionality of our project. Processes the points received
from the line object, and outputs them to a drawing.

*/

using namespace std;

/**
	**@param drawing object (which includes all lines found through the ImageProcessing class)
	**And the robot height and Width (for scaling)
	**Using the lines collected from an image, Use opencv functions to draw each set of coordinates
	**to a new image to represent the image we accuired
**/
void drawImageSimulator::drawPic(Drawing* drawing, int robotHeight, int robotWidth)
{

	std::vector<Line> lines = drawing->getLines();
	cv::Mat img;
	for(int i = 0; i < (int)lines.size(); i++){

		

		cv::Vec4i vect;

		//pull vertices from the Line
		Point * v = new Point(0,0);
		int points[2], points2[2];

		//Scale each line so that it fits in the range of the blank image
		scale((Line*) &lines[i], drawing->getDrawingWidth(), drawing->getDrawingHeight(), robotHeight, robotWidth);

		v = lines[i].getPoint1();
		v->getCoords(points);

		v = lines[i].getPoint2();
		v->getCoords(points2);

		//store the points in the vector to be used for opencv image
		vect[0] = points[0];
		vect[1] = points[1];
		vect[2] = points2[0];
		vect[3] = points2[1];
		cout << "Points sent: = " << points[0] << "," << points[1] << "," << points2[0] << "," << points2[1] << 	"\n"; 

		img = cv::imread("black.png", CV_LOAD_IMAGE_COLOR);
		cv::line(img,cv::Point(vect[0],vect[1]),cv::Point(vect[2],vect[3]),cv::Scalar(0,0,255),2,CV_AA);
	
		//write the line to a blank image
		bool wri = cv::imwrite("black.png", img);
		if(!wri){
			printf("Error drawing image!\n");
			exit(EXIT_FAILURE);
		}

	}

	//show the image
	cv::imshow("Robot Simulated Image", img);

	cv::waitKey(100);

}
/**
	**Takes a line to scale, the image and robot dimensions
	**Each point in an image will not correspond to the plane in
	**Which the robot will draw. Scale the points assoiciated to 
	**each vertex from a line pulled from an image to that of 
	**the robot's plane.
	** ScaledX = (ImageX/ImageWidth) * RobotWidth 
	** ScaledY = (ImageY/ImageHeight) * RobotHeight
	**@return scaled line
**/
Line* drawImageSimulator::scale(Line* i, int imgH, int imgL, int robH, int robW)
{	
	int pointsI1[2], pointsI2[2];
	float tempRX, tempRY, tempRX2, tempRY2;
	Point * v = new Point(0,0);
	
	//get vertex coordinates
	v = i->getPoint1();
	v->getCoords(pointsI1);

	v = i->getPoint2();
	v->getCoords(pointsI2);

	//apply scaling factors
	tempRX = ((float) pointsI1[0] / imgH) * robH;
	tempRY = ((float) pointsI1[1] / imgL) * robW;
	tempRX2 = ((float) pointsI2[0] / imgH) * robH;
	tempRY2 = ((float) pointsI2[1] / imgL) * robW;

	//return new line with scaled vertices
	i->getPoint1()->setCoords((int) tempRX, (int) tempRY);
	i->getPoint2()->setCoords((int) tempRX2, (int) tempRY2);
	return i;
}

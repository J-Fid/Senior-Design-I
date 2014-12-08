//main.cpp

#include <stdio.h>
#include <errno.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "../include/drawImageSimulator.h"
#include "../include/drawImageRobot.h"
#include "../include/imageProcessor.h"
#include "../include/drawing.h"
#include "../include/commandInterface.h"
#include "../include/relaxCommand.h"
#include "../include/quitCommand.h"
#include "../include/RobotComm.h"

using namespace cv;

int main(int argc, char** argv){
	
	//make sure path to the image was entered
	if(argc < 3){
		printf("Missing Arguments:\n"); 
		printf("Did you forget to add the path of the image? (images/name-of-image.*) \n\t***Please do not use jpg or jpeg images***\n");
		printf("Or did you forget the Simulation mode? (1 for Simulation or 2 for robot)\n");
		printf("Example for robot execution on file 'img.png': ./project images/img.png 2\n");
		exit(1);
	}
	
	//load the image
	Mat img = imread(argv[1], CV_LOAD_IMAGE_COLOR);
	int mode = atoi(argv[2]); 
	int robotHeight = 75, robotWidth = 110;

	//if the image is not found, exit program
	if(img.empty()){
	    printf("image is empty.\n\n");
	    exit(1);
	}

	//Store the black image so it can be reset later
	Mat img2 = imread("black.png", CV_LOAD_IMAGE_COLOR);
	if(img2.empty()){
	    printf("Please supply a blank image.\n\n");
	    exit(1);
	}

	//open a window to display the image and enter any key to close the window
	imshow("Source Image", img);

	static ImageProcessor imageProcessor;

	//process the image
	Drawing* drawing = imageProcessor.processImage(img);
	//std::cout << "numlines =  " << drawing->getLines().size();

	//write image dimensions to CLI
	Size imgSize = img.size();
	printf("Image Dimensions: %i x %i\n", imgSize.width, imgSize.height);

	//use either the drawing simulator or the drawing robot based on the mode parameter
	drawImageSimulator sim;
	DrawImageRobot drawingRobot;
	switch(mode){
		case 1 : //use drawImageSimulator
			sim.drawPic(drawing, robotHeight, robotWidth);
			break;
		case 2 : //use drawImageRobot
			drawingRobot.drawPic(drawing,robotHeight, robotWidth);
			break;
		default : //default: use drawImageSimulator
			sim.drawPic(drawing, imgSize.width, imgSize.height);
	}
	
	//clear the black image for next run
	bool wri= cv::imwrite("black.png", img2);
	if(wri < 0){
		printf("Error clearing black image.\n");
	}
	
	std::cout << "press any key to quit\n";

	//crashes program vvvvv
	///////////////////////////
	int c;
	std::cin >> c; 
	//////////////////

	//shut down robot
	if(mode == 2){
		commandInterface *quit = new quitCommand();
		commandInterface *relax = new relaxCommand();
		RobotComm robot;
		robot.RobotCommunication(relax);
		//give robot time to relax
		waitKey(100);	
		robot.RobotCommunication(quit);			
	}

	return 0;
}

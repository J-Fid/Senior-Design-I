//Point.cpp

#include <stddef.h>
#include "../include/point.h"

//constructor
Point::Point(int x, int y){
	this->x = x;
	this->y = y;
}

//destructor
Point::~Point(){
}

/**
	**Getter for a Point's x and y points
	**writes the points to a presupplied array
**/
void Point::getCoords(int point[2]){
	point[0] = x;
	point[1] = y;
}
/**
	**sets the x and y coordinates of the Point
**/
void Point::setCoords(int xcord, int ycord){
	x = xcord;
	y = ycord;
}


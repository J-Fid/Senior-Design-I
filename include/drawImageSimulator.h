#ifndef DRAWIMAGESIMULATOR_H
#define DRAWIMAGESIMULATOR_H

#include "drawImageInterface.h"
#include "Line.h"

class drawImageSimulator: public drawImageInterface
{
public:
	virtual void drawPic(Line * l); //CHANGE PARAMETER

};

#endif // DRAWIMAGESIMULATOR_H

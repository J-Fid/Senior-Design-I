//drawImageInterface.h

#ifndef DRAWIMAGEINTERFACE_H
#define DRAWIMAGEINTERFACE_H

#include <stdio.h>
#include "Line.h"
#include "drawing.h"

class drawImageInterface
{
	public:
		drawImageInterface();
		virtual ~drawImageInterface();
		virtual void drawPic(Drawing* drawing, int robotHeight, int robotWidth) = 0;
		virtual Line* scale(Line* i, int imgH, int imgL, int robH, int robW) = 0; 
	
};

#endif //DRAWIMAGEINTERFACE_H

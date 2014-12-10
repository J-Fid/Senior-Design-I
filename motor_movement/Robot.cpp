/*
  Robot.cpp - Library for Robot object to control arm outside of main
  Created November 17, 2014
  Released into the public domain.



#include <BioloidController.h>
#include <ax12.h>
#include <Robot.h>
#include <Motors2.h>
#include <stdlib.h>
#include <math.h>

Robot::Robot()
//vertex arrays for important coordinates on the plane of drawing
	: topRight {534, 404},
	  bottomLeft {795, 179},
	  bottomRight {762, 98},
	  deadCenter {684, 233},
<<<<<<< HEAD
	  relaxed {529, 467},
	  upOrDown {300, 475},
=======
	  relaxed {538, 520},
	  upOrDown {300, 420},
>>>>>>> d1968ebab072f732904c188217144b52e9d0c43f
          prevousCoord {0,0},
          previousMotorAngle{529, 467}
{
	 //starting coordinates for the motors to position to top left of drawing area
	 backMotor = relaxed[0];
	 frontMotor = relaxed[1];


}

//lift the pen
void Robot::penUp()
{
  	SetPosition(3, upOrDown[0]);
}

//drop the pen
void Robot::penDown()
{
  	SetPosition(3, upOrDown[1]); 
}
 
//move the arm back to the top left corner
void Robot::relaxArm(BioloidController bioloid)
{
	penUp();
	bioloid.poseSize = 2; 
	bioloid.readPose();//find where the servos are currently
	bioloid.setNextPose(1, relaxed[0]);  
	bioloid.setNextPose(2, relaxed[1]); 
	
	bioloid.interpolateSetup(500); // setup for interpolation from current->next

	while(bioloid.interpolating > 0)
	{ 
	 // do this while we have not reached our new pose
	 bioloid.interpolateStep();     // move servos, if necessary. 
	 delay(3);
	}
}

//move the arm to the top right corner of the drawing plane
void Robot::topRightCorner()
{
	  backMotor = topRight[0];
	  frontMotor = topRight[1];
	  SetPosition(1, backMotor);
	  SetPosition(2, frontMotor);  
}


//move to the bottom right corner of the drawing plane
void Robot::bottomRightCorner()
{
	  backMotor = bottomRight[0];
	  frontMotor = bottomRight[1];
	  SetPosition(1, backMotor);
	  SetPosition(2, frontMotor);
}

//move to the bottom left corner of the drawing plane
void Robot::bottomLeftCorner()
{
	  backMotor = bottomLeft[0];
	  frontMotor = bottomLeft[1];
	  SetPosition(1, backMotor);
	  SetPosition(2, frontMotor);
}

//move the the very center of the drawing plane
void Robot::toDeadCenter()
{
	 backMotor = deadCenter[0];
	 frontMotor = deadCenter[1];
	 SetPosition(1, backMotor);
	 SetPosition(2, frontMotor); 
}

void Robot::drawLine(int points[4], BioloidController bioloid)
{	
        InverseKinematics(points); 
	bioloid.poseSize = 2; // load two poses in, one for each vertex
	bioloid.readPose();//find where the servos are currently
	penUp();
	bioloid.setNextPose(1, points[0]);  //set the coordinates for the vertex to which the arm is moving first
	bioloid.setNextPose(2, points[1]); 

	bioloid.interpolateSetup(5000); // setup for interpolation from current->next

	while(bioloid.interpolating > 0)
	{ 
		 //keep moving until next pose is reached
		 bioloid.interpolateStep();    
		 delay(3);
	}

	penDown();

	//bioloid.readPose();//find where the servos are currently
	bioloid.setNextPose(1, points[2]);  
	bioloid.setNextPose(2, points[3]); 

	bioloid.interpolateSetup(1000); // setup for interpolation from current->next over 1/2 a second
	while(bioloid.interpolating > 0)
	{  
		 //keep moving until next pose is reached
		 bioloid.interpolateStep();   
		 delay(3);
	}relaxArm(bioloid);
} 	

void Robot::InverseKinematics(int points[4]){//points = {x1,y1,x2,y2};

	float B = 0.0, B2 = 0.0;            //distance that is needed to move
	float q1_1= 0.0, q1_2 = 0.0;           //angle between X-axis and line to be drawn
	float q2_1= 0.0,q2_2 =0.0;           //angle of front motor link l1
	float Q1_1= 0.0, Q1_2 =0.0;           //Q1: angle between x-axis and "l1"
	float Q2_1= 0.0, Q2_2 = 0.0 ;          //Q2: angle between "l1" and "l2"
	long l1 = 60;          //l1: length first bracket
	long l2 = 70;         //l2: length of tip bracket
	long l1_sqr = l1 * l1;
	long l2_sqr = l2 * l2;

	float X = (points[2] -points[0]);
	float Y = (points[3] -points[1]);
	float slope = (Y/X);


	//Where the robot is going
	long Xpos1= 0.0, Xpos2 =0.0;       //x coordinate where the arm should move to
	long Ypos1= 0.0, Ypos2 =0.0;       //y corrdinate where the arm should move to      

	Xpos1 = points[0];     //relative distance to travel on x previous is 0
	Ypos1 = points[1];     //relative distance to travel on y previous is 0
	Xpos2 = points[2]-points[0];     //relative distance to travel on x
	Ypos2 = points[3]-points[1];     //relative distance to travel on y

	B = sqrt((Xpos1*Xpos1) + (Ypos1*Ypos1));           //the Pythagorean theorem
	B2 = sqrt((Xpos2*Xpos2) + (Ypos2*Ypos2));          //the Pythagorean theorem
	long B_sqr = B * B;
	long B2_sqr = B2 * B2;

	q1_1 = atan2(Ypos1,Xpos1);
	q1_2 = atan2(Ypos2,Xpos2);
	q2_1 = acos((l1_sqr - l2_sqr + B_sqr)/(2*l1*B)); //the law of cosines   
	q2_2 = acos((l1_sqr - l2_sqr + B2_sqr)/(2*l1*B2)); //the law of cosines         
	Q1_1 = degrees(q2_1) - degrees(q1_1)-45 ;     
	Q1_2 = degrees(q2_2) - degrees(q1_2)-45 ;                                   
	Q2_1 = degrees(acos((l1_sqr + l2_sqr - B_sqr)/(2*l1*l2)))+45;//the law of cosines    
	Q2_2 = degrees(acos((l1_sqr + l2_sqr - B2_sqr)/(2*l1*l2)))+45;//the law of cosines    


	//for AX-12 servos 0.29 degrees is equal to an increase of 1
	float robotAngleConversion = 0.71;
	if(slope < 0 && Y < 0){//+bm -fm
	
		points[0] =  previousMotorAngle[0] + ((int)(Q1_1 / (robotAngleConversion))); //x1
		points[1] = previousMotorAngle[1] - ((int)(Q2_1 / (robotAngleConversion)));//y1
		points[2] = points[0] + ((int)(Q1_2 / (robotAngleConversion)));//x2
		points[3] = points[1] - ((int)(Q2_2 / (robotAngleConversion)));//y2
		
	}else if(slope < 0 && X < 0){//-bm +fm
\
		points[0] =  previousMotorAngle[0]  - ((int)(Q1_1 / (robotAngleConversion))); //x1
		points[2] = points[0] - ((int)(Q1_2 / (robotAngleConversion)));//x2
		points[1] = previousMotorAngle[1] + ((int)(Q2_1 / (robotAngleConversion)));//y1
		points[3] = points[1] + ((int)(Q2_2 / (robotAngleConversion)));//y2
		
	}else if(slope > 0 && (X < 0 && Y<0)){//+bm +fm

		points[0] =  previousMotorAngle[0]  + ((int)(Q1_1 / (robotAngleConversion))); //x1
		points[1] = previousMotorAngle[1] + ((int)(Q2_1 / (robotAngleConversion)));//y1
		points[2] = points[0] + ((int)(Q1_2 / (robotAngleConversion)));//x2
		points[3] = points[1] + ((int)(Q2_2 / (robotAngleConversion)));//y2

	}else if(slope > 0 && (X > 0 && Y > 0)){//-bm -fm
		
		points[0] =  previousMotorAngle[0]  - ((int)(Q1_1 / (robotAngleConversion))); //x1
		points[1] = previousMotorAngle[1] - ((int)(Q2_1 / (robotAngleConversion)));//y1
		points[2] = points[0] - ((int)(Q1_2 / (robotAngleConversion)));//x2
		points[3] = points[1] - ((int)(Q2_2 / (robotAngleConversion)));//y2
		
	}

	else {}*/

/*
Serial.println();
Serial.println(points[0]);
Serial.println(points[1]);
Serial.println(points[2]);
Serial.println(points[3]);
Serial.println();


 Serial.println(Q1_1);
 Serial.println(Q2_1);
 Serial.println();
 Serial.println(B);
 Serial.println();
 Serial.println(q1_1);
 Serial.println(q2_1);
 Serial.println(degrees(q1_1));
 Serial.println(degrees(q2_1));
 Serial.println();
 Serial.println(Xpos1);
 Serial.println(Ypos1);
 Serial.println();
 Serial.println();
 Serial.println(Q1_2);
 Serial.println(Q2_2);
 Serial.println();
 Serial.println(B2);
 Serial.println();
 Serial.println(q1_2);
 Serial.println(q2_2);
 Serial.println(degrees(q1_2));
 Serial.println(degrees(q2_2));
 Serial.println();
 Serial.println(Xpos2);
 Serial.println(Ypos2);
 Serial.println();
 Serial.println("------------------------------------------------------");
 Serial.println();


 delay(50);  
}


*/

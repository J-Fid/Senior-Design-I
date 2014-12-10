#include <BioloidController.h>
#include <ax12.h>
#include <Motors2.h>
#include <stdlib.h>
#include <Robot.h>

// we always need to create an instance of the bioloid control, usually with baud = 1Mbps.
BioloidController bioloid = BioloidController(1000000);

char input;
char buff[5], ack ='0';
int points[5] ; 
int previouspoint[2] = {0,0};
int testPoints[4] = { 5,5,90,90};
int p, b, num, j;


Robot robot; //MODEL OBJECT

//super turtle arrays
int t1[4] = {538,520,534,404};
int t2[4]= {538,520,684,233};
int t3[4] = {534,404,684,233};
int s1[4] = {545,300,545,350};
int s2[4] = {545,350,675,350};
int s3[4] = {675,350,645,300};
int s4[4] = {645,300,695,300};
int s5[4] = {695,300,695,350};



void setup(){
    
  //robot is already set to topL0ft coordinates
    robot.penUp();
    Serial.begin(9600); //start serial communications at 960bps
    Relax(1);
    Relax(2);
    Relax(3);
}

void loop(){

  delay(50);
  Serial.print(ack);
  Serial.flush();
  
   if(Serial.available()>0){
        int inByte = Serial.read();
        switch (inByte)
        {
          case 'q':    
            robot.relaxArm(bioloid);
            exit(0);
            break;    
          
          case 'u':
            robot.penUp();
            break;
          
          case 'd':
            robot.penDown();
            break; 
            
         case 't': //test case
           
           robot.drawLine(testPoints, bioloid); //TEST LINE WITHOUT PROCESSING
           break;
         
         case 's': //superTurtle case
           robot.drawLine(t1, bioloid);
           robot.drawLine(t2, bioloid);
           robot.drawLine(t3, bioloid);
           robot.drawLine(s1, bioloid);
           robot.drawLine(s2, bioloid);
           robot.drawLine(s3, bioloid);
           robot.drawLine(s4, bioloid);
           robot.drawLine(s5, bioloid);
           break;
         case 'm':
            robot.backMotor+= 10; //moves down and to rightn
            break; 
            
         case 'n':
            robot.backMotor-= 10; //moves to left in arch
            break; 
          
           case 'b':
            robot.frontMotor+= 10; //moves down and to rightn
            break; 
          
           case 'v':
            robot.frontMotor-= 10; //moves down and to rightn
            break; 
            
           case 'a': //coordinates have been sent
            //wait for all information
              delay(50); 
              //reset buffer each time new points come in
              j=0;
              for(j;j<5;j++){
                  buff[j] = '0';
                  points[j] = -1; 
              }
              readCoordinates();
    
            break;
          
          case 'l':
           robot.backMotor +=5;
           SetPosition(1,robot.backMotor);
           robot.frontMotor-=20;
           SetPosition(2,robot.frontMotor);
           break;  
          
           case 'r':
            robot.relaxArm(bioloid);
            break;    
      }
}
}

void readCoordinates(){
  p=0;
  b=-1;
  ack = '0';
  while(Serial.available()>0)
    {
        input=Serial.read();
        
        if(input == ','){//comma has been reached, convert numbers from buffer to in
         points[p] = calc();
         b=-1;
         p++;
        }
        else{//value is not a comma, add to buffer
          b++;
          buff[b]=input;
       }
    }     

    int checksum = points[0] + points[1] + points[2] + points[3];
    if(checksum == points[4]){
      //draw the line here
      int temp[4] = {points[0], points[1], points[2], points[3]};
      robot.drawLine(temp, bioloid);
      Serial.print("y"); //correct checksum and line is drawn
      ack = 'y';
    }else{
      Serial.print("n"); //wrong checksum, resend 
      ack = 'n';
    }
    Serial.flush();
   
}

//methond to convert char to an int
int calc()
{
    int num=0,x=0;
 
    for(x;x<=b;x++){
          num=num+(buff[x]-'0')*pow(10,b-x);
    }
    if(num >= 99){
     num=num+1;  
    }
    return num;
}



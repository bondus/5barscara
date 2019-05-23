


#include <stdio.h>
#include "FiveBarScaraKinematics.h"


void dump(float machinePos[3], int32_t motorPos[3], float stepsPerMm[3])
{
  printf("%8.2f %8.2f  ", machinePos[0], machinePos[1]);
  printf("%5d %5d  ", motorPos[0], motorPos[1]);

  float l = (float)motorPos[0] / stepsPerMm[0];
  float r = (float)motorPos[1] / stepsPerMm[1];
  printf("%6.2f %6.2f \n", l, r);
}

int main(void)
{
  FiveBarScaraKinematics fbs;

  float machinePos[3];
  float stepsPerMm[3] = { 106.666*8, 106.666*8, 800.00};
  int32_t motorPos[3];

  float machinePos2[3];


  machinePos[0] = 0;
  machinePos[1] = 0;
  machinePos[2] = 0;


  float a = fbs.getAngle(0,0,  0,1, -0.001,0);
  printf("a: %f\n", a);

  /*

  float aL = 90;
  float aR = 90;  
  float coords[2] = {aL, aR};
  float resultcoords[6];
  fbs.getForward(resultcoords, aL, aR);
  machinePos[0] = resultcoords[4];
  machinePos[1] = resultcoords[5];
  fbs.getInverse(machinePos);
  */

#define PI 3.1415
  for(int a = 0; a<360; a++) {
    float x = 100 + cos(a*PI*2/360.0)*100;
    float y = 0+   sin(a*PI*2/360.0)*100;
    float ab = fbs.getAngle(0, 0, 100, 0, x, y);
    printf("%f, %f ", x, y);
    printf("%d     %f\n", a, ab);
  }

  
/*
  
  printf("      "); 
  for(float x = -250; x<=250; x+=10) {
    printf("%4d, ",(int32_t)x);
  }
  printf("\n"); 
  for(int y = 350; y>=-50; y-=5) {
    printf("%4d: ",(int32_t)y);
    for(int x = -250; x<=250; x+=10) {
      if(x == 0 && y == 0) {
        printf("XXXX, "); 
      }
      else
      {
      
        machinePos[0] = x;
        machinePos[1] = y;
        //printf("--- %f %f\n", x, y);
      
        if(!fbs.CartesianToMotorSteps(machinePos, stepsPerMm, 3, 3, motorPos, false))
        {
          printf("%4s, ","");
        }
        else
        {
          bool bad = false;
          //dump(machinePos, motorPos, stepsPerMm);

          int32_t motorPosL[3];
          motorPosL[0] = motorPos[0] + 5;
          motorPosL[1] = motorPos[1] + 0;
          motorPosL[2] = motorPos[2] + 0;
          float machinePosL[3];
          fbs.MotorStepsToCartesian(motorPosL, stepsPerMm, 3, 3, machinePosL);

          //dump(machinePos2, motorPosx, stepsPerMm);

          int32_t motorPosR[3];
          motorPosR[0] = motorPos[0] + 0;
          motorPosR[1] = motorPos[1] + 5;
          motorPosR[2] = motorPos[2] + 0;
          float machinePosR[3];
          fbs.MotorStepsToCartesian(motorPosR, stepsPerMm, 3, 3, machinePosR);


          bad |= !fbs.CartesianToMotorSteps(machinePosR, stepsPerMm, 3, 3, motorPosR, false);
          bad |= !fbs.CartesianToMotorSteps(machinePosL, stepsPerMm, 3, 3, motorPosL, false) ;

          if(!bad) {
            float dL = sqrt(fsquare(machinePosL[0] - machinePos[0]) + fsquare(machinePosL[1] - machinePos[1]));            
            float dR = sqrt(fsquare(machinePosR[0] - machinePos[0]) + fsquare(machinePosR[1] - machinePos[1]));

            int32_t sL = 5.0/dL;
            int32_t sR = 5.0/dR;
            int32_t m = min(sL, sR);
            printf("%4d, ", m);
          }
          else
            printf("%4s, ","");          
        }
        //fbs.MotorStepsToCartesian(motorPos, stepsPerMm, 3, 3, machinePos);
        //dump(machinePos, motorPos, stepsPerMm);
      }
    }
    printf("\n");
  }
*/
  return 0; 
}


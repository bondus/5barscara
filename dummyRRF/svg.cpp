
#include <stdio.h>
#include "FiveBarScaraKinematics.h"

float anglediff(float a, float b) {
  float d = fabs(a -b);

  if(d <= 180) return d;
  
  d = fabs(360 - a -b);
  return d;
}

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

  float machinePos[2];
  float stepsPerMm[2] = { 106.666*8, 106.666*8};
  int32_t motorPos[2];

  float machinePosGrid[2*360][2*360][2];

    float xsize = 600; 
    float xoff  = 300;
    float ysize = 800; 
    float yoff  = 400;

  
    printf(
      "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n"
      "<svg width=\"%f\" height=\"%f\"  xmlns=\"http://www.w3.org/2000/svg\">\n", 
      xsize, ysize);

    printf(
      "    <defs>\n"
      "  <pattern id=\"smallGrid\" width=\"10\" height=\"10\" patternUnits=\"userSpaceOnUse\">\n"
      "    <path d=\"M 10 0 L 0 0 0 10\" fill=\"none\" stroke=\"lightblue\" stroke-width=\"0.5\"/>\n"
      "  </pattern>\n"
      "  <pattern id=\"grid\" width=\"100\" height=\"100\" patternUnits=\"userSpaceOnUse\">\n"
      "    <rect width=\"100\" height=\"100\" fill=\"url(#smallGrid)\"/>\n"
      "    <path d=\"M 100 0 L 0 0 0 100\" fill=\"none\" stroke=\"lightblue\" stroke-width=\"1\"/>\n"
      "  </pattern>\n"
      "</defs>");

  
    printf("<rect width=\"%f\" height=\"%f\" fill=\"url(#grid)\" />", xsize, ysize);

  for(int workmode = 1; workmode <=4; workmode +=1) {
    //if (workmode == 2) continue;
    fbs.workmode = workmode;
    for(int32_t aL = 0; aL <2*360; aL+=1)
      for(int32_t aR = 0; aR <2*360; aR+=1)
      {
        machinePosGrid[aL][aR][0] = NAN;
        machinePosGrid[aL][aR][1] = NAN;
      }



    int32_t minL = 0; //fbs.actuatorAngleLMin;
    int32_t maxL = 360; //fbs.actuatorAngleLMax;

    int32_t minR = 0; // fbs.actuatorAngleRMin;
    int32_t maxR = 360; // fbs.actuatorAngleRMax;

  
    float step = 3;

    int l;
    int r;

    l = 0;
    for(float aL = minL; aL <=maxL; l++, aL+=step)
    {
      r = 0;
      for(float aR = minR; aR <=maxR; r++, aR+=step)
      {
        float resultcoords[6];
        fbs.getForward(resultcoords, aL, aR);
        machinePos[0] = resultcoords[4];
        machinePos[1] = resultcoords[5];

        fbs.getInverse(machinePos);
        fbs.cachedThetaL = fmod(fbs.cachedThetaL, 360);
        fbs.cachedThetaR = fmod(fbs.cachedThetaR, 360);
      
        float al = aL; if(al < 0) al+=360; if(al >= 360) al -=360;
        float ar = aR; if(ar < 0) ar+=360; if(ar >= 360) ar -=360;
      
        float dL = anglediff(fbs.cachedThetaL, al);
        float dR = anglediff(fbs.cachedThetaR, ar);
      
        if(dL > 0.1 || dR > 0.1) {
          fprintf(stderr, "Something wrong wm: %d\n", workmode);
          fprintf(stderr, "     %f %f => %f %f\n", al, ar, machinePos[0], machinePos[1]);
          float resultcoords2[6];
          fbs.getForward(resultcoords2, fbs.cachedThetaL, fbs.cachedThetaR);
          fprintf(stderr, "     %f %f => %f %f\n", fbs.cachedThetaL, fbs.cachedThetaR,
                  resultcoords2[4], resultcoords2[5]);
          printf("<circle cx=\"%f\" cy=\"%f\" r=\"4\" style=\"stroke:rgb(255,0,0)\" />\n",
                 xoff+machinePos[0], yoff-machinePos[1]);

          machinePos[0] = NAN;
          machinePos[1] = NAN;
        }
      
        if(!fbs.IsReachable(machinePos[0], machinePos[1], false))
        {
          //machinePos[0] = NAN;
          //machinePos[1] = NAN;
        }
        machinePosGrid[l][r][0] = machinePos[0];
        machinePosGrid[l][r][1] = machinePos[1];
        //dump(machinePos, motorPos, stepsPerMm);      
      }
    }



    char color[128];
    sprintf(color, "%d,%d,%d",
            (workmode&1)?128:0,
            (workmode&2)?128:0, (workmode & 4)?128:0);
            
    l=1;
    for(float aL = minL; aL <=maxL+step; l++, aL+=step)
    {
      r = 1;
      for(float  aR = minR; aR <=maxR+step; r++, aR+=step)
      {
        float x = machinePosGrid[l][r][0];
        float y = machinePosGrid[l][r][1];
        if(!std::isnan(x) && !std::isnan(y)) {
          float x0 = machinePosGrid[l-1][r][0];
          float y0 = machinePosGrid[l-1][r][1];
          if(!std::isnan(x0) && !std::isnan(y0)) {
            printf("<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" style=\"stroke:rgb(%s);stroke-width:0.2\" />\n",
                   xoff + x,  yoff - y,
                   xoff + x0, yoff - y0, color);
          }
        
          float x1 = machinePosGrid[l][r-1][0];
          float y1 = machinePosGrid[l][r-1][1];
          if(!std::isnan(x1) && !std::isnan(y1)) {
            printf("<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" style=\"stroke:rgb(%s);stroke-width:0.2\" />\n",
                   xoff + x,  yoff - y,
                   xoff + x1, yoff - y1, color);
          }

        }
      }
    }
    printf("<circle cx=\"%f\" cy=\"%f\" r=\"4\" style=\"stroke:rgb(%s);stroke-width:1\" />\n",
           xoff +fbs.xOrigL, yoff - fbs.yOrigL,
           "0,0,0");
    
    printf("<circle cx=\"%f\" cy=\"%f\" r=\"4\" style=\"stroke:rgb(%s);stroke-width:1\" />\n",
           xoff +fbs.xOrigR, yoff - fbs.yOrigR,
           "0,0,0");



    
  }
  printf("</svg>\n");
}

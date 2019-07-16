#include "DFRobot_BNO055.h"
#include "Wire.h"


//accelerometer, gyroscope, geomagnetic
typedef DFRobot_BNO055_IIC   BNO;
BNO    bno(&Wire, 0x28);

//analog data
BNO::sAxisAnalog_t   sAccAnalog, sMagAnalog, sGyrAnalog, sLiaAnalog, sGrvAnalog; //accelerometer, magnetometer, gyroscope, linear acceleration, gravity vector

int timer;
bool onboard_led = false; //status of onboard led
 
const float xUnit[] = {1,0,0}; //unit vector for x direction
char* compass; //compass direction the sensor is facing
const float compres = 22.5; //resolution of compass in degress

//compass labels
const String comp1[] = {"E","NEE","NE","NEE","N","NNE","NW","NWW","W"};
const String comp2[] = {"E","SEE","SE","SSE","S","SWW","SW","SWW","W"};


#define LED 2 //onboard LED

// show last sensor operate status
void printLastOperateStatus(BNO::eStatus_t eStatus)
{
  switch(eStatus) {
  case BNO::eStatusOK:   Serial.println("everything ok"); break;
  case BNO::eStatusErr:  Serial.println("unknow error"); break;
  case BNO::eStatusErrDeviceNotDetect:   Serial.println("device not detected"); break;
  case BNO::eStatusErrDeviceReadyTimeOut:    Serial.println("device ready time out"); break;
  case BNO::eStatusErrDeviceStatus:    Serial.println("device internal status error"); break;
  default: Serial.println("unknow status"); break;
  }
}

void setup() {
  Serial.begin(115200);
  bno.reset(); //reset 

  timer = millis(); //start timer

  Serial.begin(115200);
  bno.reset();
  while(bno.begin() != BNO::eStatusOK) {
    Serial.println("bno begin failed");
    printLastOperateStatus(bno.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bno begin success");
}

void loop() {

  //get analog data
  sMagAnalog = bno.getAxis(BNO::eAxisMag); //read geomagnetic
  sGyrAnalog = bno.getAxis(BNO::eAxisGyr); //read gyroscope
  sAccAnalog = bno.getAxis(BNO::eAxisAcc); // read acceleration
  sMagAnalog = bno.getAxis(BNO::eAxisMag); // read geomagnetic
  sGyrAnalog = bno.getAxis(BNO::eAxisGyr); // read gyroscope
  sLiaAnalog = bno.getAxis(BNO::eAxisLia); // read linear acceleration
  sGrvAnalog = bno.getAxis(BNO::eAxisGrv); // read gravity vector

  //get projected vectors
  float gravVect[] = {float(sGrvAnalog.x),float(sGrvAnalog.y),float(sGrvAnalog.z)}; //store gravity vector as array
  float magVect[] = {float(sMagAnalog.x),float(sMagAnalog.y),float(sMagAnalog.z)}; //store gravity vector as array
  float* magProj = projectVector(magVect,gravVect); //project magnetic vector onto plane
  float* xProj = projectVector(xUnit, gravVect); //project x axis onto plane
  
  float angle = angleVector(xProj,magProj); //angle between mag vector and x axis

  //find bearing

  //upper quadrants
  if (magVect[1] > 0){
    //loop through options
    for (int c = 0; c < 9; c++){
      //if angle is within the range
      if (ang > (compres*c - compres/2) && ang < (compres*(c+1) - compres/2)){
        compass = comp1[c]; //set compass bearing
        break; //end loop
      }
    }
  }
  //lower quadrants
  else if (magVect[1] < 0){
    //loop through options
    for (int c = 0; c < 9; c++){
      //if angle is within the range
      if (ang > (compres*c - compres/2) && ang < (compres*(c+1) - compres/2)){
        compass = comp2[c]; //set compass bearing
        break; //end loop
      }
    }
  }
  //on y axis
  else if (magVect[0] == 0){
    if (magVect[1] > 0){
      compass = "N";
    }
    else{
      compass = "S";
    }
  }
  //on x axis
  else{
    if (magVect[0] > 0){
      compass = "E";
    }
    else{
      compass = "W";
    }
  }

  
  
  
  //if 250ms have passed
  if(millis() - timer > 250){
    timer = millis(); //reset timer
    

    if (onboard_led){
        digitalWrite(LED, LOW); //turn off LED
      }
      else{
        digitalWrite(LED,HIGH); //turn on LED        
      }
      onboard_led = !onboard_led; //invert led status
  }
}


/*
 * VECTOR FUNCTIONS
 */

//returns a vector that has been projected on a plane
float* projectVector(float vector[], float plane[]){
  float proj[3]; //projected vector

  //loop through vector components
  for (int a = 0; a <= 2; a++){
    proj[a] = vector[a] - (dotProduct(vector,plane)/pow(magnitude(plane),2))*plane[a]; //calculate magnitude of projected vector component a 
  }
  
  return proj;
}

//returns dot project of two vectors
float dotProduct(float vector1[], float vector2[]){
  return vector1[0]*vector2[0] + vector1[1]*vector2[1] + vector1[2]*vector2[2];
}

//returns magnitude of vector
float magnitude(float vector[]){
  return sqrt(pow(vector[0],2) + pow(vector[1],2) + pow(vector[2],2));
}

//returns angle between two vectors in degrees
float angleVector(float vector1[], float vector2[]){
  return  RAD_TO_DEG * acos(dotProduct(vector1,vector2)/(magnitude(vector1)*magnitude(vector2)));
}

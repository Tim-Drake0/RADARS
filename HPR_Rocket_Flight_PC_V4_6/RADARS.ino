 //----------------------------mass
//LIST OF FUNCTIONS & ROUTINES
//----------------------------
//checkAngle(): Checks that input angle [deg] is within servo deflection limits
//getAirbrakeDeflection(): returns current angle of deflection value
//setAirbrakeDeflection(): deflects and edits the current angle of deflection
//getIdealTrajectory(): simulates ideal trajectory from initial velocity and altitude (at burnout when in flight)
//setAirbrakesTest(): Simulates airbrake deflection from the ideal trajectory
//interpCD(): interpolates the CD value from Table_CD based on the airbrake angle of deflection
//setAirbrakes(): in flight funciton to control airbrakes
//-----------CHANGE LOG------------
//15 MAR 24: initial file breakout created
//---------------------------------
float MAXANGLE = 105;
float MINANGLE = 56;
// Rocket Constants
float diameter = 0.1016; // [m] body tube diameter (4in)
float mass = 6.64; // [kg] at burnout
int num_AB = 4; // Number of airbrakes
float width_AB = 0.0508; // [m]
float length_AB = 0.0762; // [m]
float delta=0.0; // [deg] Airbrake deflection angle
float Cd=.38;
float Swet_tube; // Referebce area of body tube
float Table_CD[80]; // CD Values From CFD Simulations

// Flight path
float angle_rad = 3.14159/180.0;
float flight_angle = 86.95;

float Drag;
float delta_S;
float deltaAngle;
float delta_Dreq;
float rho; // [kg/m^3] 
const float R = 287.05; //R value for dry air

int mult=1;
int t=1;

float vy_next=223.12; float vy; // [m/sec]
float y_next=603.452184; float y; // [m]
float ay;

float checkAngle(float angle){
  if(angle > MAXANGLE){return MAXANGLE;}
  else if(angle < MINANGLE){return MINANGLE;}
  else {return angle; }
}

float getAirbrakeDeflection(){return delta;}

void setAirbrakeDeflection(float newDelta){
  if(isnan(newDelta)){newDelta=0;}
  airbrake1.write(checkAngle((0.6125*newDelta)+56-servo1trim));
  airbrake2.write(checkAngle((0.6125*newDelta)+56-servo2trim));
  airbrake3.write(checkAngle((0.6125*newDelta)+56-servo3trim));
  airbrake4.write(checkAngle((0.6125*newDelta)+56-servo4trim));
  Serial.print(" Airbrake Deflected: "); Serial.println(delta); 
  delta=newDelta;
}

void getIdealTrajectory(){ 
  Table_CD[0]=0.38; Table_CD[5]=0.395; Table_CD[10]=0.421; Table_CD[15]=0.465; 
  Table_CD[20]=0.495; Table_CD[25]=0.528; Table_CD[30]=0.56; Table_CD[35]=0.59;
  Table_CD[40]=0.644; Table_CD[45]=0.649; Table_CD[50]=0.689; Table_CD[55]=0.69;
  Table_CD[60]=0.689; Table_CD[65]=0.698; Table_CD[70]=0.69; Table_CD[75]=0.71; Table_CD[80]=0.729;
  float vy_next=fusionVel*sin(yawY*0.1*flight_angle*angle_rad)*3.281; float vy;  // [m/s] initial velocity in y at burnout 
  float y_next=fusionAlt*3.281; float y; // [m]
  //Calculate air density
  float rho_low =(baro.pressure * 100) / (R * (baro.temperature + 273.15)); float rho_high = 1.05549; // [kg/m^3] 
  float alt_low = fusionAlt; float alt_high = 1524.0; // [m] 
  float ay;
  float g = -9.81; // [m/s^2]
  if(settings.testMode){
    vy_next=223.12; y_next=603.452184;
    rho_low = 1.225; // [kg/m^3] 
    alt_low = 0.0; // [m]
  }
  for (int t=0; t<800; t=t+1){
    vy=vy_next;
    y=y_next;
    float rho = rho_low+((y-alt_low)*(rho_high-rho_low)/(alt_high-alt_low)); // [kg/m^3] 
    float Swet_tube = 3.14159*sq(0.5*diameter); // [m^2]
    Drag = 0.5*rho*vy*vy*CD*Swet_tube; // [N]
    
    ay = ((0.0-Drag-(mass*-g))*sinf(flight_angle*angle_rad))/mass; // [m/s^2]
    vy_next = vy + ay*0.05;  // [m/s]
    y_next = y + vy_next*0.05;  // [m]
    if(y_next-y<0.0 || y_next > 1828.8){break;} // Only simulate until apogee
    if(y_next > yBest){
      yBest=y_next;
    }
    dragIdeal[t] = Drag; // [N]
    if(settings.testMode){
      Serial.print("y_next: "); Serial.print(y_next); 
      Serial.print(" | rho: "); Serial.print(rho); 
      Serial.print(" | vy_next: "); Serial.print(vy_next); 
      Serial.print(" | ay: "); Serial.print(ay); 
      Serial.print(" | Drag: "); Serial.println(Drag); 
    }
  }
}//end getIdealTrajectory

void setAirbrakesTest(){
  if(t==1){vy_next=223.12; y_next=603.452184; }
  float g = -9.81; // [m/s^2]
  vy=vy_next; y=y_next;
  float RocketArea = 3.14159*sq(0.5*diameter) + 4*width_AB*length_AB*sinf(delta*angle_rad);// [m^2] area of rocket including airbrakes
  float rho_low = (baro.pressure * 100) / (R * (baro.temperature + 273.15)); float rho_high = 1.05549; // [kg/m^3] 
  float alt_low = fusionAlt; float alt_high = 1524.000; // ft
  float rho = rho_low+((y-alt_low)*(rho_high-rho_low)/(alt_high-alt_low)); // [kg/m^3] 
  int d0=0; int d1=10;
  CD = interpCD();
  Drag = 0.5*rho*vy*vy*CD*RocketArea; // [N]
  
  ay = ((0.0-Drag-(mass*-g))*sinf(flight_angle*angle_rad))/mass; // [m/s^2]
  vy_next = vy + ay*0.05; // [m/2]
  y_next = y + vy_next*0.05; // [m]
  if(y_next < y){setAirbrakeDeflection(0.0); return;} // Only deflect until apogee
  delta_Dreq = dragIdeal[t] - Drag; t=t+1; // [N]
  
  Cd = Cd+(2*delta_Dreq)/(rho*vy*vy*RocketArea); 
  delta_S = (2*delta_Dreq)/(rho*vy*vy*Cd); // [m^2]
  if(delta_Dreq < 0.0){
    mult=-1;
  }else{
    mult=1;
  }
  deltaAngle = mult*asinf((abs(delta_S)/(num_AB*width_AB*length_AB)))*(1.0/angle_rad); // [deg]
  delta=delta+deltaAngle; // [deg]
  if(delta > 80.0){delta = 80.0;} // limit to 80deg max deflection
  if(delta < 0.0){delta = 0.0;} // limit to 0deg min deflection
  
  setAirbrakeDeflection(delta);
  
  //Debug variables
  Serial.print("y_next: "); Serial.print(y_next); 
  Serial.print(" | vy_next: "); Serial.print(vy_next);
  Serial.print(" | ay: "); Serial.print(ay);
  Serial.print(" | Drag: "); Serial.print(Drag); 
  Serial.print(" | Ideal Drag: "); Serial.print(dragIdeal[t-1]); 
  Serial.print(" | delta_Dreq: "); Serial.print(delta_Dreq); 
  Serial.print(" | delta_S: "); Serial.print(delta_S,4);
  Serial.print(" | deltaAngle: "); Serial.print(deltaAngle); 
  Serial.print(" | delta: "); Serial.println(delta);
  if(dragIdeal[t]<=0.0){return;} // Only simulate positive ideal drag values
}//end setAirbrakesTest

float interpCD(){
  int d0=0; int d1=10;
  if(delta!=0.0){
    for(int i=0; i<8; i++){
      if(delta >= d0 && delta <= d1){break;}
      d0 = d0 + 10;
      d1 = d1 + 10;
    }
    CD = Table_CD[d0] + (delta-d0)*((Table_CD[d1]-Table_CD[d0])/(d1-d0));
  } else {
    CD = Table_CD[0];
  }
  return CD;
}//end interpCD

void setAirbrakes(){
  float currentVel = fusionVel; // [m/s]
  if(fabs(currentVel)<1.0F){currentVel = 1.0F;}
  //Calculate air density
  float airDensity = (baro.pressure * 100) / (R * (baro.temperature + 273.15)); // [kg/m^3]
  // Calculate area of rocket with airbrakes
  float RocketArea = 3.14159*sq(0.5*diameter) + 4*width_AB*length_AB*sinf(delta*angle_rad);// [m^2] 
  CD = interpCD();

  Drag = 0.5*airDensity*currentVel*currentVel*CD*RocketArea; // [N]

  delta_Dreq = dragIdeal[t] - Drag; t=t+1; // Required drag to match ideal drag 
  Cd = Cd+(2*delta_Dreq)/(airDensity*currentVel*currentVel*RocketArea); 
  delta_S = (2*delta_Dreq)/(airDensity*currentVel*currentVel*Cd); // [m^2]
  if(delta_S < 0.0){
    mult=-1;
  }else{
    mult=1;
  }
  deltaAngle = mult*asinf((abs(delta_S)/(num_AB*width_AB*length_AB)))*(1.0/angle_rad); // [deg]
  delta=delta+deltaAngle; // [deg]
  if(delta > 80.0){delta = 80.0;} // limit to 80deg max deflection
  if(delta < 0.0){delta = 0.0;} // limit to 0deg min deflection
  // Deploy airbrakes with corrected servo deflection ratio
  setAirbrakeDeflection(delta);
}//end setAirbrakes




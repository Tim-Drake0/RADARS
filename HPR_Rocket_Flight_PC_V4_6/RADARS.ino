 //----------------------------
//LIST OF FUNCTIONS & ROUTINES
//----------------------------
//getIdealTrajectory(): simulates ideal trajectory from initial velocity and altitude (at burnout when in flight)
//setAirbrakesTest(): Simulates airbrake deflection from the ideal trajectory
//interpCD(): interpolates the CD value from Table_CD based on the airbrake angle of deflection
//setAirbrakes(): in flight funciton to control airbrakes
//-----------CHANGE LOG------------
//15 MAR 24: initial file breakout created
//---------------------------------

// Rocket Constants
float diameter = 4.0/12.0; // [ft] body tube diameter (4in)
float weight = 13; // [lb] at burnout
int num_AB = 4; // Number of airbrakes
float width_AB = 2.0/12.0; // [ft]
float length_AB = 3.0/12.0; // [ft]
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
float rho; // [slug/ft]
const float R = 287.05; //R value for dry air

int mult=1;
int t=0;

float vy_next=732.020997; float vy; // [ft/sec]
float y_next=1979.83; float y; // [ft]
float ay;

float getCurrentDeflection(){return delta;}

void setCurrentDeflection(float newDelta){delta=newDelta;}

void getIdealTrajectory(){ 
  Table_CD[0]=0.38; Table_CD[5]=0.395; Table_CD[10]=0.421; Table_CD[15]=0.465; 
  Table_CD[20]=0.495; Table_CD[25]=0.528; Table_CD[30]=0.56; Table_CD[35]=0.59;
  Table_CD[40]=0.644; Table_CD[45]=0.649; Table_CD[50]=0.689; Table_CD[55]=0.69;
  Table_CD[60]=0.689; Table_CD[65]=0.698; Table_CD[70]=0.69; Table_CD[75]=0.71; Table_CD[80]=0.729;
  float vy_next=fusionVel*sin(yawY*0.1*flight_angle*angle_rad); float vy;  // initial velocity in y at burnout
  float y_next=fusionAlt; float y;
  //Calculate air density
  float rho_low =(baro.pressure * 100) / (R * (baro.temperature + 273.15)); float rho_high = 0.002048; // [slug/ft] ===Check Units===
  float alt_low = fusionAlt; float alt_high = 5000.0; // [ft] ===Check Units===
  float ay;
  float g = -32.2; // [ft/s^2]
  if(settings.testMode){
    vy_next=671.5; y_next=1979.83;
    rho_low = 0.002377; // [slug/ft]
    alt_low = 0.0; // [ft]
  }
  for (int t=0; t<800; t=t+1){
    vy=vy_next;
    y=y_next;
    float rho = rho_low+((y-alt_low)*(rho_high-rho_low)/(alt_high-alt_low)); // [slug/ft] ===Check Units===
    float Swet_tube = 3.14159*sq(0.5*diameter);
    Drag = 0.5*rho*vy*vy*CD*Swet_tube; // lbf
    
    ay = -(((0.0-Drag)*sinf(flight_angle*angle_rad))-weight)/(weight/g); // [ft/s^2]
    vy_next = vy + ay*0.05;  // [ft/s]
    y_next = y + vy_next*0.05;  // [ft]
    if(y_next-y<0.0 || y_next > 6000.0){break;} // Only simulate until apogee
    if(y_next > yBest){
      yBest=y_next;
    }
    dragIdeal[t] = Drag;
    if(settings.testMode){
      Serial.print("y_next: "); Serial.print(y_next); 
      Serial.print(" | vy_next: "); Serial.print(vy_next); 
      Serial.print(" | ay: "); Serial.print(ay); 
      Serial.print(" | Drag: "); Serial.println(Drag); 
    }
  }
}//end getIdealTrajectory

void setAirbrakesTest(){
  //Serial.println("Simulating Airbrakes Until Apogee!");
  float g = -32.2; // [ft/s^2]
  vy=vy_next; y=y_next;
  float RocketArea =3.14159*sq(0.5*diameter) + 4*width_AB*length_AB*sinf(delta*angle_rad);// [ft^2] area of rocket including airbrakes
  float rho_low = 0.002377; float rho_high = 0.002048; // slug/ft 
  float alt_low = 0.0; float alt_high = 5000.000; // ft
  float rho = rho_low+((y-alt_low)*(rho_high-rho_low)/(alt_high-alt_low)); // [slug/ft]
  int d0=0; int d1=10;
  CD = interpCD();
  Drag = 0.5*rho*vy*vy*CD*RocketArea; // [lbf]
  
  ay = -(((0-Drag)*sin(flight_angle*angle_rad))-weight)/(weight/g);
  vy_next = vy + ay*0.05;
  y_next = y + vy_next*0.05;
  delta_Dreq = dragIdeal[t] - Drag; t=t+1;
  
  Cd = Cd+(2*delta_Dreq)/(rho*vy*vy*RocketArea); 
  delta_S = (2*delta_Dreq)/(rho*vy*vy*Cd); 
  if(delta_S < 0.0){
    mult=-1;
  }else{
    mult=1;
  }
  deltaAngle = mult*asinf((abs(delta_S)/(num_AB*width_AB*length_AB)))*(1.0/angle_rad);
  delta=delta+deltaAngle;
  if(delta >= 80.0){delta = 80.0;} // limit to 80deg max deflection
  
  airbrake1.write((0.6125*delta)+56-servo1trim);
  airbrake2.write((0.6125*delta)+56-servo2trim);
  airbrake3.write((0.6125*delta)+56-servo3trim);
  airbrake4.write((0.6125*delta)+56-servo4trim);
  
  //Debug variables
  Serial.print("y_next: "); Serial.print(y_next); 
  Serial.print(" | Drag: "); Serial.print(Drag); 
  Serial.print(" | Ideal Drag: "); Serial.print(dragIdeal[t-1]); 
  Serial.print(" | delta_Dreq: "); Serial.print(delta_Dreq); 
  Serial.print(" | delta_S: "); Serial.print(delta_S,4);
  Serial.print(" | deltaAngle: "); Serial.print(deltaAngle); 
  Serial.print(" | delta: "); Serial.println(delta);
  if(y_next-y<0){return;} // Only simulate until apogee
  if(dragIdeal[t]<=0.0){return;} // Only simulate positive ideal drag values
  
  //}
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
  float accelZ=accel.z;
  float g = -32.2; // [ft/s^2]
  float currentVel = fusionVel;
  if(fabs(currentVel)<1.0F){currentVel = 1.0F;}
  //Calculate air density
  float airDensity = (baro.pressure * 100) / (R * (baro.temperature + 273.15));
  // Calculate area of rocket with airbrakes
  float RocketArea =3.14159*sq(0.5*diameter) + 4*width_AB*length_AB*sinf(delta*angle_rad);//area of rocket including airbrakes in ft^2
  CD = interpCD();
  Drag = 0.5*airDensity*currentVel*currentVel*CD*RocketArea; // [lbf]
  //float dragForce = -(((accelZ*accel.gainZ)/(weight*sinf((offVert*.1)*angle_rad))) + weight); // not sure which equation to use
  delta_Dreq = dragIdeal[t] - Drag; t=t+1; // Required drag to match ideal drag
  Cd = Cd+(2*delta_Dreq)/(airDensity*currentVel*currentVel*RocketArea); 
  delta_S = (2*delta_Dreq)/(airDensity*currentVel*currentVel*Cd); // ft^2
  if(delta_S < 0.0){
    mult=-1;
  }else{
    mult=1;
  }
  deltaAngle = mult*asinf((abs(delta_S)/(num_AB*width_AB*length_AB)))*(1.0/angle_rad); // [deg]
  delta=delta+deltaAngle; // [deg]
  if(delta >= 80.0){delta = 80.0;} // limit to 80deg max deflection
  // Deploy airbrakes with corrected servo deflection ratio
  airbrake1.write((0.6125*delta)+56-servo1trim);
  airbrake2.write((0.6125*delta)+56-servo2trim);
  airbrake3.write((0.6125*delta)+56-servo3trim);
  airbrake4.write((0.6125*delta)+56-servo4trim);
}//end setAirbrakes




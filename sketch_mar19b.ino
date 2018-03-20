#include<Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <waypointClass.h>
#include <MechaQMC5883.h>
#include <math.h> 


const int trigPin = 9; //for ultrasonic sensor
const int echoPin = 10;       
#define MAX_DISTANCE_CM 250                        // Maximum distance we want to ping for (in CENTIMETERS). Maximum sensor distance is rated at 400-500cm.  
#define MAX_DISTANCE_IN (MAX_DISTANCE_CM / 2.5)

int straight=5; //front facing vibration motor
int back=6; //back vibration motor
int right=7; // right vibration motor
int left=8; //left vibration motor

MechaQMC5883 compass;

// Compass navigation
int targetHeading;              // where we want to go to reach current waypoint
int currentHeading;             // where we are actually facing now
int headingError;               // signed (+/-) difference between targetHeading and currentHeading
#define HEADING_TOLERANCE 10    // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

int val1, val2, val3, val4;    

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

float currentLat,
      currentLong,
      targetLat,
      targetLong;
int distanceToTarget,            // current distance to target (current waypoint)
    originalDistanceToTarget;    // distance to original waypoing when we started navigating to it

// Waypoints
#define WAYPOINT_DIST_TOLERANE  10   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define NUMBER_WAYPOINTS 1          // enter the numebr of way points here (will run from 0 to (n-1))
int waypointNumber = -1;            // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()
/*if(Serial.available())
 while(Serial.read()!='\n')
 {
  data1=Serial.parseInt();
  data2=Serial.parseInt();
  
}*/
waypointClass waypointList[NUMBER_WAYPOINTS] = {waypointClass(26.881977, 75.813613) }; //destination location

#define SAFE_DISTANCE 50 //allowed distance from obstacle

float duration, distance;

void setup() {
  pinMode(straight, OUTPUT);
  pinMode(back, OUTPUT);
  pinMode(right, OUTPUT);
  pinMode(left, OUTPUT);
  digitalWrite(straight,LOW);
   digitalWrite(back,LOW);
  digitalWrite(right,LOW);
  digitalWrite(left,LOW);
 
 Serial.begin(115200);        // we need this speed for the GPS
 ss.begin(GPSBaud);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
 nextWaypoint();
}

void loop() {

  while (ss.available() > 0)
    if (gps.encode(ss.read()))
  {
    processGPS(); 
  }
  Serial.println(targetLat); 
  Serial.println(targetLong);
  Serial.print("Distance= ");
  Serial.println(distanceToTarget);
  Serial.print("target heading= ");
  Serial.println(targetHeading);
    //navigate 
    currentHeading = readCompass(); // get our current heading
    Serial.print("current heading=");
    Serial.println(currentHeading);
    calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles      
    Serial.print("heading error=");
    Serial.println(headingError);

    
    // distance in front of us, move, and avoid obstacles as necessary
    checkSonar();
   Serial.print("sonarDistance=");
  Serial.println(distance);
   moveAndAvoid();  
}

}
void LEFT_()
{
digitalWrite(left,HIGH);
digitalWrite(right, LOW);
digitalWrite(straight,LOW);
digitalWrite(back, LOW);
}
void RIGHT_()
{
 digitalWrite(left,LOW);
digitalWrite(right, HIGH);
digitalWrite(straight,LOW);
digitalWrite(back, LOW);
}
void STRAIGHT_()
{
  digitalWrite(left,LOW);
digitalWrite(right, LOW);
digitalWrite(straight,HIGH);
digitalWrite(back, LOW);
}
void BACK_()
{
  digitalWrite(left,LOW);
digitalWrite(right, LOW);
digitalWrite(straight,LOW);
digitalWrite(back, HIGH);
}
void processGPS(void)
{
  currentLat = (gps.location.lat());
  currentLong = (gps.location.lng());
             
  if ((gps.location.lng()) == 'S')            // make them signed
    currentLat = -currentLat;
  if ((gps.location.lng()) == 'W')  
    currentLong = -currentLong; 
   Serial.print("current lat=");
   Serial.println(currentLat,6);
   Serial.print("current long=");
   Serial.println(currentLong,6);
          
  // update the course and distance to waypoint based on our new position
  distanceToWaypoint();
  courseToWaypoint();         
  
} 

void checkSonar(void) //read distance from obstacle
{   
   digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  if(distance==0)
  distance=250;
  return distance;

} // checkSonar

int readCompass(void) //read heading degrees
{
int x,y,z;
  compass.read(&x,&y,&z);
  float heading = atan2(y, x);
  
  #define DEC_ANGLE -0.0059
  heading += DEC_ANGLE;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  return ((int)headingDegrees); 
}

void calcDesiredTurn(void) 
{
    // calculate where we need to turn to head to destination
    headingError = targetHeading - currentHeading;
    
    // adjust for compass wrap
    if (headingError < -180)      
      headingError += 360;
    if (headingError > 180)
      headingError -= 360;
  
    // calculate which way to turn to intercept the targetHeading
    if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
      STRAIGHT_();  
    else if (headingError < 0)
      LEFT_();
    else if (headingError > 0)
      RIGHT_();
    else
      STRAIGHT_();
 
} 

void moveAndAvoid(void) //function for avoiding obstacle and following new path
{
    val1=digitalRead(straight);
    val2=digitalRead(back);
    val3=digitalRead(right);
    val4=digitalRead(left);
    if(val1==1&&val2==0&&val3==0&&val4==0)
    {
    if (distance <= SAFE_DISTANCE)       // close to obstacle
        {
         RIGHT_();
         delay(50000);
         /*STRAIGHT_();
         delay(50000);
         LEFT_();
         delay(50000);
         STRAIGHT_();
         delay(50000);
         LEFT_();
         delay(5000);
         STRAIGHT_();
         delay(5000);
         RIGHT_();
         delay(5000);
         STRAIGHT_();*/  
        }
      
        }}
      
void nextWaypoint(void)//obtaining the next stop 
{
  waypointNumber++;
  targetLat = waypointList[waypointNumber].getLong();
  targetLong = waypointList[waypointNumber].getLat();
  
  if ((targetLat == 0 && targetLong == 0) || waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached? 
    {
  digitalWrite(left,LOW);;
digitalWrite(right, LOW);
digitalWrite(straight,LOW);;
digitalWrite(back, LOW);
      //lcd.println(F("* LAST WAYPOINT *"));
      loopForever();
    }
    
   processGPS();
   distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
   courseToWaypoint();
   
}  // nextWaypoint()


void loopForever(void)
{
  while (1)
    ;
}

int distanceToWaypoint()//source to destination distance 
{
  
  float delta = radians(currentLong - targetLong);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  float lat1 = radians(currentLat);
  float lat2 = radians(targetLat);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  delta = sq(delta); 
  delta += sq(clat2 * sdlong); 
  delta = sqrt(delta); 
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
  delta = atan2(delta, denom); 
  distanceToTarget =  delta * 6372795; 
   
  // check to see if we have reached the current waypoint
  if (distanceToTarget <= WAYPOINT_DIST_TOLERANE)
    nextWaypoint();
    
  return distanceToTarget;
}  // distanceToWaypoint()


int courseToWaypoint()//error heading  
{
  float dlon = radians(targetLong-currentLong);
  float cLat = radians(currentLat);
  float tLat = radians(targetLat);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  targetHeading = degrees(a2);
  return targetHeading;
}  // courseToWaypoint()




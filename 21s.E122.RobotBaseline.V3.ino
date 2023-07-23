  // Public library preinstalled in Arduino IDE
#include <Ultrasonic.h> 
#include <Servo.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ESP8266WiFi.h>
#include <Wire.h>

// Private library, need to have them in the main program folder
#include "PubSubClient.h"
#include "info.h"
#include "WiFiManager.h"
#include "SSD1306.h"
void forward();   //Subroutine to have the robot move forward
void reverse();   //Subroutine to have the robot move backwards
void right();     //Subroutine to have the robot turn right
void left();      //Subroutine to have the robot turn left
void halt();      //Subroutine to have the robot stop
void pathfind();
//MQTT Communication associated variables
String inString = ""; 
int testvariable;
char payload_global[100];
boolean flag_payload;
int k = 0; // done target constant
int target = 1;

//MQTT Setting variables
const char* mqtt_server = "155.246.62.110";
const int mqtt_port = 1883;
const char* MQusername = "jojo";
const char* MQpassword = "hereboy";
double x, y = 0;
//WiFi Define
WiFiClient espClient;
info board_info;
PubSubClient client(espClient);

////////////////////////////////////////Robot Logic Variables//////////////////////////////////////////////////
//////////////////////////Students change this section for their modification//////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////// Wifi Settings variables // Remote students change to your network parameters ///////////////////////
const char* ssid = "Stevens-Media";
const char* password = "Stevens1870";

////////// If on-campus, remember to add your WeMos MAC address to Stevens network ////////////////////////////

//////********CHANGE FOR EACH ARENA***********////////
//const char* MQtopic = "louis_lidar1"; // Topic for Arena_1 (remote)
//const char* MQtopic = "louis_lidar2"; // Topic for Arena_2 (BC 219)
const char* MQtopic = "louis_lidar3"; // Topic for Arena_3 (BC 220)

// Define the DC motor contorl signal pins
#define motorRpin D0  //GPIO pin setting for motorR
#define motorLpin D2  //GPIO pin setting for motorL

Servo motor1;        //Create servo motorR object to control a servo
Servo motor2;        //Create servo motorL object to control a servo

// Define the Ultrasonic sensor pins
Ultrasonic ultrasonic_front(D9, D6); // Ultasonic sensor, front (trig, echo)
Ultrasonic ultrasonic_right(D10, D7); // Ultasonic sensor, right (trig, echo)
Ultrasonic ultrasonic_left(D8, D5); // Ultasonic sensor, left (trig, echo)

// Define the OLED display pins D14 SDA, D15 SCL
SSD1306  display(0x3C, D14, D15); //Address set here 0x3C
                                  //Pins defined as D2 (SDA/Serial Data) and D5 (SCK/Serial Clock).

////////////////////////// Define the variables needed for your algorithm ////////////////////////////////////

int distance_left = 0;
int distance_right = 0;
int distance_front = 0;
double previous_x = -1; // The previous coordinate
double previous_y = -1; // The previous coordinate
double targetVectorX, targetVectorY = 0;

////////////////////////////////////Robot Logic Variables End//////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Setup the wifi, Don't touch
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  randomSeed(micros());
}

// MQTT msg callback
void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    payload_global[i] = (char)payload[i];
  }
  payload_global[length] = '\0';
  flag_payload = true;
}

// MQTT reconnection setup
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    
    // Attempt to connect
    if (client.connect(clientId.c_str(),MQusername,MQpassword)) {
       client.subscribe(MQtopic);
    }
  }
}

/////////////////////////////// SETUP LOOP. Don't Touch ///////////////////////////////////////////

void setup() {
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback); 
  motor1.attach(motorRpin);   //motorR is attached using the motorRpin
  motor2.attach(motorLpin);   //motorL is attached using the motorLpin
  // Display Setup
  display.init();
  display.flipScreenVertically();
  display.drawString(0, 0, "Stevens Smart Robot 1");
  display.display();
}

/////////////////////////////////// MAIN LOOP /////////////////////////////////////////////

void loop() { 
  //subscribe the data from mqtt server
  if (!client.connected()) {
      reconnect();
  }
  const char *msg = "target";
  char temp1[50];
  sprintf(temp1,"%d",k);
  const char *c = temp1;
  
  client.publish( msg , c);
  client.loop();
  String payload(payload_global);
 
  int testCollector[10];
  int count = 0;
  int prevIndex, delimIndex;
  
  prevIndex = payload.indexOf('[');
  while( (delimIndex = payload.indexOf(',', prevIndex +1) ) != -1){
    testCollector[count++] = payload.substring(prevIndex+1, delimIndex).toInt();
    prevIndex = delimIndex;
  }
  delimIndex = payload.indexOf(']');
  testCollector[count++] = payload.substring(prevIndex+1, delimIndex).toInt();

  forward();
  previous_x = x;
  previous_y = y;
  x = testCollector[0];
  y = testCollector[1];
 //calculates vectors for current trajectory vector of the robot, and the magnitude of the vector
  double previousVectorX = x-previous_x;
  double previousVectorY = y-previous_y;
  double magnitudePrevious = sqrt((previousVectorX*previousVectorX)+(previousVectorY*previousVectorY));

  //Setting up the target destination, xt[]={A,B,C,D ~} 
  double xt[] = {1000,500,100,1700};
  double yt[] = {200,100,100,300};
  //Checks if robot is on its designated target, changes target to the next one if it is
   if(x <= (xt[target-1]+80)&& x >= (xt[target-1]-80)&& y <= (yt[target-1]+80)&& y >= yt[target-1]-80){
    halt();
    delay(2000);
    target = target + 1;
    forward();
  }
  //sees which target the robot needs to go to, calculates target vector based on this
  if(target == 1){
    targetVectorX = xt[0] - x;
    targetVectorY = yt[0] - y;
  } else if(target == 2){
    targetVectorX = xt[1] - x;
    targetVectorY = yt[1] - y;
  } else if(target == 3){
    targetVectorX = xt[2] - x;
    targetVectorY = yt[2] - y;
  } else if(target == 4){
    targetVectorX = xt[3] - x;
    targetVectorY = yt[3] - y;
  }
 //caluclates magnitude of target vector, dot product, angle, and vector product
  double magnitudeTarget = sqrt((targetVectorX*targetVectorX)+(targetVectorY*targetVectorY));
  double dotProduct = (previousVectorX*targetVectorX)+(previousVectorY*targetVectorY);
  double angle1 = dotProduct/(magnitudeTarget*magnitudePrevious);
  double angleRad = acos(angle1);
  double angle = (angleRad/(3.142857))*180;
  double VP = (previousVectorX*targetVectorY)-(previousVectorY*targetVectorX);
  //makes sure lidar coordinates are updated before moving, if it is then the robot turns according to calculations above
  if(previous_x != x && previous_x != -1){
    halt();
    if(VP < 0){
      right();
      delay(590*(angle/90));
    } else {
      left();
      delay(590*(angle/90));
    }
    forward();
  }
  //checks again if robot is on a target
  if(x <= (xt[target-1]+80)&& x >= (xt[target-1]-80)&& y <= (yt[target-1]+80)&& y >= yt[target-1]-80){
    halt();
    delay(3000);
    target = target + 1;
  }

////////////////////////////////////////Robo10t Logic Begin//////////////////////////////////////////////////
////////////////////////Students change this section for their modification////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////// find the closest obstacle ///////////////////////////////////////

  // Read the distances from the Utrasonic sensors. Output unit is mm
  distance_left = ultrasonic_left.read(CM)* 10;
  distance_front = ultrasonic_front.read(CM)* 10;
  distance_right = ultrasonic_right.read(CM)* 10;
//checks if something is on the left and right, turns accordingly
   if ( distance_left <= 20 )
   {
      right();
      delay(60);
      forward();
   }   
   if( distance_right <= 20 )
   {
      left();
      delay(60);
      forward();
   }
   //checks if something is in front, if it is then the robot reverses, takes ultrasonic sensor measurements, turns according to what side has more space
   if ( distance_front <= 30 )
   {
    reverse();
    delay(1000);
    halt();
    delay(200); 
    distance_left = ultrasonic_left.read(CM)* 10;
    distance_front = ultrasonic_front.read(CM)* 10;
    distance_right = ultrasonic_right.read(CM)* 10;
    
    if (distance_right < distance_left)
   {
      halt();
      left();
      delay (400);
      forward();
      delay(1000);
   }else {
      halt();
      right();
      delay (400);
      forward();
      delay(1400);
   }
   } 
    // Display the x,y location in the OLED
    display.clear(); // Clear the buffer
    String str_1 = "x: " + String(x); // We need to cast the x value from 'int' to 'String'
    String str_2 = "y: " + String(y); // Same thing for the y value
    display.drawString(0, 0, "Stevens Smart Robot 1");
    display.drawString(0, 15, String(previousVectorX));
    display.drawString(0, 30, String(VP));
    display.drawString(0, 45, String(angle));
    display.display();


////////////////////////////////////////Robot Logic End////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
}
void forward()  // Has the motors go forward- even for 2020
  {
   motor1.write(82); // left  motor D14 - furthest from 90 goes faster. 60 is faster than 80
   motor2.write(82); // right motor D15 
   }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void reverse()  // Has the motors go in reverse - even for 2020
  {
    motor1.write(100);  //left  motor D14 - furthest from 90 goes faster. 120 is faster than 100
    motor2.write(100);  //right motor D15 - 100 is slow but good for testing; 100 is much slower than 80 is fast
  }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void halt() // Has the motors stop
  {
    motor1.write(90); //Turns motor1 off
    motor2.write(90); //Turns motor2 off
  }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void left() // Has the motors turn gently left ---- f=urthest from 90 and a harder left
{
  motor1.write(100);
  motor2.write(80);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void right() // Has the motors turn left
{
  motor1.write(80);
  motor2.write(100);

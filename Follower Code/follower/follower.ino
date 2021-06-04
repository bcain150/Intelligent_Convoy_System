//Writen by Brendan Cain 1/15/2020
//UMBC ISC team 1 CMPE capstone
//This code is run on the client side, it connects to an access point and 
#include <TinyGPS.h>
#include <Checkpoint.h>

//MOVE FLAGS for info that describes how the follower is moving
#define MOVE_STOP 0      //The vehicle is stopped.
#define DISCON 1         //The bluetooth control is disconnected from the lead vehicle
#define MOVE_FORW 2
#define MOVE_BACK 3
#define HARD_LEFT 4
#define SOFT_LEFT 5
#define HARD_RIGHT 6
#define SOFT_RIGHT 7
//lights
#define left_blink 11
#define mid_blink 12
#define right_blink 13

//********MOTOR CONTROL STUFF*********//
// Motor A (LEFT) connections
#define enA 2
#define in1 4
#define in2 5
// Motor B (RIGHT) connections
#define enB 3
#define in3 6
#define in4 7
const int MAX_SPEED =  255;

int Ltrim = 0, Rtrim = 0;
int speed_sub;
double speed_divider;
double turn_degrees = 0; //the amount of degrees that the follower must turn to get near its target course negative is CCW (Left) positive is CW (right)


//********GPS STUFF*********//
TinyGPS gps; // create gps object
long my_lat=0, my_lon=0, my_cours=0; //Global Packet Data
unsigned long my_fix_age;  //ms since last data was encoded (used to see if gps data is stale)
unsigned int my_MOVE_FLAG = MOVE_STOP;  //comes from leader gives info on how it's moving
bool new_follower_data;  //says whether tinyGPS has encoded new data

//********PACKET STUFF*********//
#define DATA_AVAIL "+IPD,"  //this tag is appended to any data being sent (along with string byte size and a colon)
#define PACKET_TERMINATOR "#"
#define READ_TERMINATOR '$'
#define PACKET_DELIM ','
#define DATA 1        //valid data was received and globals updated
#define ESP_INFO 0    //esp info came in, connection may be bad
#define INVAL_DATA -1 //data was invalid, or data got cut off (leader ESP may be trying to send data too fast)
#define NO_SAVE -2  //data was valid but it wasnt saved to the buffer (could contain stop or disconnect flag)
//const int MIN_SIZE = 25; //minimum data packet size (16 lat/long characters, 3 info characters, 5 commas, 1 #)
const int NUM_DATA = 6; //number of data points in a packet -> lat, long, course, flag, fix
int receivePacket();  //returns 1 of the 3 above literals
Checkpoint old_packet;

//********PATH STUFF*********//
#define MAX_PATH_LENGTH 10
#define MIN_DISTANCE 4.0//min distance to say that the follower has reached a checkpoint (in meters)
double updateDistance(double lat2, double lon2);
double updateCourse(double lat2, double lon2);
Checkpoint path [MAX_PATH_LENGTH] = {Checkpoint(), Checkpoint(), Checkpoint(), Checkpoint(), Checkpoint(), Checkpoint(), Checkpoint(), Checkpoint(), Checkpoint(), Checkpoint()};
int path_length = 0;
int cur_point = 0;
int save_point = 0;
double lead_lat, lead_lon, lead_cours;  //the positon and course of the leader at the current checkpoint
double dist_to = 0.0, crs_to = 0.0;     //the vector that the follower has to travel in order to get to the current checkpoint

//********WIFI STUFF*********//
#define RES_OK "OK"
#define RES_ERROR "ERROR"
#define RES_READY "ready"

String ssid = "host"; //these should be hardcoded on station side as well
String pass = "12345ics!";
String cwmode = "3"; //Station mode
String cipmux = "0"; //single tcp connection mode for client
String wifi_channel = "1"; //wifi band channel for soft ap
String port = "80"; //port to host the tcp server on
String enc_mode = "3"; //encryption mode is WPA2_PSK
String host_ip = "192.168.4.1"; //the ip address for the host/softAP ESP

//********SETUP FUNCTION*********//
long counter = 0;
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // pins 18 (TX1), 19 (RX1) from GPS
  Serial2.begin(115200);  //pins 16 (TX2), 17(RX2) goes to ESP8266
  
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //lights for directional notification
  pinMode(left_blink, OUTPUT);
  pinMode(mid_blink, OUTPUT);
  pinMode(right_blink, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(left_blink, LOW);
  digitalWrite(mid_blink, LOW);
  digitalWrite(right_blink, LOW);
  
  bool reset = reset_ESP();
  while(!reset) Serial.println("Reset Failed!"); //TODO: MAKE LED BLINK HERE?
  Serial.println("Reset Successful.");

  bool wifi_connected = connect_ap();
  if(wifi_connected) Serial.println("Connected to AP."); //connect to access point
  else Serial.println("WIFI Failed!");

  bool connect_tcp = connect_tcp_client();
  if(connect_tcp)Serial.println("TCP Connection Successful."); //connects to existing tcp connection
  else Serial.println("TCP Connection Failed");

  while(!connect_tcp){
    //TODO: blink some led?
    if(!wifi_connected) Serial.println("failed due to wifi");
    else Serial.println("failed due to tcp");
  }

  int count = 0;
  while(count < 200000) {
    if(Serial1.available() > 0) {
      if(gps.encode(Serial1.read())){
        analogWrite(enA, (MAX_SPEED-Ltrim));
        analogWrite(enB, (MAX_SPEED-Rtrim));
        moveForward();
        my_MOVE_FLAG = MOVE_FORW;
      }
    }
    count++;
  }
}

//********LOOP FUNCTION*********//

void loop() {
  int new_packet = 0;
  //tiny GPS object needs to be constantly updated to work properly
  if(Serial1.available() > 0) {
    if(gps.encode(Serial1.read())){
     //TODO: SET A LIGHT HIGH
      gps.get_position(&my_lat, &my_lon, &my_fix_age);
      my_cours = gps.course();
      new_follower_data = true;
    }
    else new_follower_data = false; //TODO SET A LIGHT LOW
  }else new_follower_data = false;

  //if the ESP outputs, read it because it's probably a packet.
  //if it is a packet the path buffer is updated.
  if(Serial2.available() > 0) new_packet = receivePacket();

  if(new_follower_data && path_length > 0) {
    if(my_fix_age != TinyGPS::GPS_INVALID_AGE) {  //updates relative position
      path[cur_point].trackLeader(lead_lat, lead_lon, lead_cours);
      dist_to = updateDistance(lead_lat, lead_lon);
      crs_to = updateCourse(lead_lat, lead_lon);
      turn_degrees = course_change(crs_to);
    }
    
    if(dist_to <= MIN_DISTANCE) {  //see if the follower has reached its current checkpoint
      path_length --;
      if(path_length == 0) my_MOVE_FLAG = MOVE_STOP;
      else cur_point = (cur_point + 1)%MAX_PATH_LENGTH;
    }


    //SHOULD THE FOLLOWER STOP MOVING OR GO
    if(my_MOVE_FLAG == MOVE_STOP && path_length > 0) { //course cant be properly acheived while stationary. this occurs on start up and when leader moves and stops again
        analogWrite(enA, (MAX_SPEED-Ltrim));
        analogWrite(enB, (MAX_SPEED-Rtrim));
        moveForward();
        my_MOVE_FLAG = MOVE_FORW;
        //digitalWrite(mid_blink,HIGH);
        //turn_degrees = getGoodCourse();//try to get a good course to move to
    }
    else if(my_MOVE_FLAG == MOVE_STOP) {
      moveStop(); 
    }
    else{ // do movement
      if(turn_degrees < 0) { //Left Turn CCW
        if(turn_degrees > -10) { //go forward instead
          analogWrite(enA, (MAX_SPEED-Ltrim));
          analogWrite(enB, (MAX_SPEED-Rtrim));
          moveForward();
          my_MOVE_FLAG = MOVE_FORW;
        }
        else {
          speed_divider = turn_degrees/-180.0;
          speed_sub = (MAX_SPEED-Ltrim)*speed_divider; //modify speed of the turn based off of how many degrees it needs to turn to correct its direction
          //only do slight turns, this code is too sensitive for hard turns
          analogWrite(enA, (MAX_SPEED-Ltrim)-speed_sub);
          analogWrite(enB, (MAX_SPEED-Rtrim));
          moveForward();
          my_MOVE_FLAG = SOFT_LEFT;
        }
      }//end left turn
      else { //Turn right CW
        if(turn_degrees < 10){ //go forward instead
          analogWrite(enA, (MAX_SPEED-Ltrim));
          analogWrite(enB, (MAX_SPEED-Rtrim));
          moveForward();
          my_MOVE_FLAG = MOVE_FORW;
        }
        else {
          speed_divider = turn_degrees/180.0;
          speed_sub = (MAX_SPEED-Rtrim)*speed_divider; //modify speed of the turn based off of how many degrees it needs to turn to correct its direction
          //only do slight turns, this code is too sensitive for hard turns
          analogWrite(enA, (MAX_SPEED-Ltrim));
          analogWrite(enB, (MAX_SPEED-Rtrim)-speed_sub);
          moveForward();
          my_MOVE_FLAG = SOFT_RIGHT;
        }
      } 
    }//end movement

    switch(my_MOVE_FLAG) {//set directional lights
      case MOVE_FORW:
        digitalWrite(right_blink, LOW);
        digitalWrite(left_blink, LOW);
        digitalWrite(mid_blink, HIGH);
      break;
      case SOFT_LEFT:
        digitalWrite(right_blink, LOW);
        digitalWrite(left_blink, HIGH);
        digitalWrite(mid_blink, LOW);
      break;
      case SOFT_RIGHT:
        digitalWrite(right_blink, HIGH);
        digitalWrite(left_blink, LOW);
        digitalWrite(mid_blink, LOW);
      break;
      default:
        digitalWrite(right_blink, LOW);
        digitalWrite(left_blink, LOW);
        digitalWrite(mid_blink, LOW);
      break;
    }

    Serial.println("FLAG: " + String(my_MOVE_FLAG) + "\tdist_to: " + String(dist_to, 2) 
    + " turn_angle:" + String(turn_degrees, 2) + ", crs: " + String((double(my_cours)/100.0), 3) + ", Leader: " + String(lead_lat, 6) + "," + String(lead_lon, 6));
  }
}



/*
 * calculates the change of course angle for the follower inorder to reach a checkpoint
 */
double course_change(double target) {
  double current = double(my_cours)/100.0;
  double turn = target - current;

  if(turn < 0) {
    if(turn > -180.0) return turn;
    else return (turn + 360.0);
  }
  else {
    if(turn < 180) return turn;
    else return (turn - 360.0);
  }
}
/*  Update Distance
 * -takes in the desired point
 * -converts updated follower position to double
 * -returns the distance between follower and leader.
 */
double updateDistance(double lat2, double lon2) {
  double lat, lon;
  lat = double(my_lat)/1000000.0;
  lon = double(my_lon)/1000000.0;
  return gps.distance_between(lat, lon, lat2, lon2);
}
/*  Update Course
 * -takes in desired point
 * -converts updated follower positon to a double
 * -returns the course in degrees that the desired location
 * is at with respect to the followers current location.
 */
double updateCourse(double lat2, double lon2) {
  double lat, lon;
  lat = double(my_lat)/1000000.0;
  lon = double(my_lon)/1000000.0;
  return gps.course_to(lat, lon, lat2, lon2);
}

/*  attempts to read leader data packet from ESP serial buffer
 *  -data timeout time is based on baud rate and the amount of time it takes to read 15 bytes from a buffer
 *  this allows for ample time to clear ESP notifications that would result in losing TCP connection anyways (no data flow)
 *  -checks to make sure the packet is the right format.
 *  -If it's not in the right format the packet is thrown out and an error code is returned.
 *  -If it is in the right format, it is parsed tries to be saved in the path buffer as an instance of checkpoint
 *  -it is only saved in the path buffer if the point indicates movement, or it is the first packet to notify bluetooth disconnect or stop.
 */
int receivePacket() {
  long lead_Packet[NUM_DATA]; //array where the data is stored
  int newLat = 0, newLong = 1, newCourse = 2, newFlag = 3, newTime = 4, newFix = 5; //indexes of most recent data strings in lead_Packet array
  long data_timeout = 3;
  int len = 0, cur_i = 0, next_i; //length of read in packet string, next index in the string, current index in the string
  String packet;
  bool full_packet, save = true;
  
  Serial2.setTimeout(data_timeout);
  if(Serial2.find(DATA_AVAIL)){
    len = Serial2.parseInt(SKIP_NONE)-1; //gets length of packet
    Serial2.read();  //get rid of :
    packet = Serial2.readStringUntil(READ_TERMINATOR);
    full_packet = packet.endsWith(PACKET_TERMINATOR);
    //above  gets packet, and then ensures that its a full packet
    
    if(full_packet && len == packet.length()) {
      for(int d=0; d<NUM_DATA; d++) { //parses and saves data strings in new lead packet array
        next_i = packet.indexOf(PACKET_DELIM, cur_i);
        lead_Packet[d] = (packet.substring(cur_i, next_i)).toInt();
        cur_i = next_i + 1;
      }
      //Serial.println(String(lead_Packet[newCourse]));
    }
    else return INVAL_DATA; //if it's less than the minimum size then something errored with sending the data (leader could be sending too fast for its esp)
  }
  else return ESP_INFO; //no data was found

  //if valid data create a new checkpoint object and add to the list.
  if(path_length > 0) { //dont want to accidentally access old_packet if it hasnt been initialized
    if((lead_Packet[newFlag] == MOVE_STOP && old_packet.getFlag() == MOVE_STOP) or (lead_Packet[newFlag] == DISCON && old_packet.getFlag() == DISCON))
      save = false;
    else save = true;
  }
  //this statement implements the path array's rotating-buffer-like save meathod
  if(save){//if its ok to save (path length is 0 or the last and current packet are different points.)
    Checkpoint new_point(lead_Packet[newLat], lead_Packet[newLong], lead_Packet[newCourse], lead_Packet[newFlag], lead_Packet[newTime], lead_Packet[newFix]);
    path[save_point] = new_point;
    save_point = (save_point + 1)%MAX_PATH_LENGTH; //every time you save a new point to the path, update the next saving location
  
    if(path_length>=MAX_PATH_LENGTH) cur_point = (cur_point + 1)%MAX_PATH_LENGTH; //if the list is full then the new packet just overwrote the current one so increase the current point
    else path_length++; //if the length isnt at max then update the length. 
    
    old_packet = new_point; //if you saved, update the new packet.
  }
  else return NO_SAVE;
  return DATA; //data has been read, globals updated
}

/*Connects to host's access point:
 * sets the ESP's Mode to station mode
 * connects to the SoftAP with the given global parameters
 * returns true if successful returns false if not
 * Uses Cur version of wifi AT commands inorder to not save connection data to flash (causes issues)
 */
bool connect_ap() {
  //sets to station mode
  String MODE = "AT+CWMODE_CUR=" + cwmode + "\r\n";
  bool connect_ok = send_command(MODE, 5000, RES_OK);
  if(!connect_ok) return false; // couldnt set to station mode
  
  //set up the access point
  String JAP = "AT+CWJAP_CUR=\"" + ssid + "\",\"" + pass + "\"" + "\r\n";
  connect_ok = send_command(JAP, 5000, RES_OK);
  if(!connect_ok) return false; //couldnt join the access point (maybe it isnt up yet?)
  return true;
}

/*connects to a tcp server
 * sets CIPMUX to 0
 * connects server to port 80 from given local ip address (host ip)
 * returns a string with "OK" on success, error message if failed
 */
bool connect_tcp_client() {
  //set single connection mode
  String MUX = "AT+CIPMUX=" + cipmux + "\r\n";
  bool connect_ok = send_command(MUX, 5000, RES_OK);
  if(!connect_ok) return false; //could not set to TCP single connection mode
  
  /*connects to server on port 80
   * This fails most often. could be because leader hasnt set it up yet
   * or sometimes when the leader tried to send too many times to a disconnected tcp port,
   * the esp would block the port, AT+RESTORE may be necessary on both ESPs
   */
  String START = "AT+CIPSTART=\"TCP\",\"" + host_ip + "\"," + port + "\r\n";
  connect_ok = send_command(START, 10000, RES_OK);
  if(!connect_ok) return false; //could not start a TCP connection. Maybe its not up

  return true;
}

/*Resets ESP module
 * 
 */
bool reset_ESP() {
  String RST = "AT+RST\r\n";
  return send_command(RST, 10000, RES_READY);
}

/*sends a command via serial terminal to ESP
 * command is the command to send in a String
 * num_resonse is the number of times ok, error, or ready would be sent
 * returns the string that was returned as a result
  */
bool send_command(String command, long timeout, char* response) {

  Serial2.write(command.c_str()); //writes the command to the ESP port
  Serial2.flush(); //waits until all info is done sending to serial
  Serial2.setTimeout(timeout);
  return Serial2.find(response);
}

// Move Both Wheels Forward
void moveForward() {
  leftWheelForward();
  rightWheelForward();
}

//// Move Both Wheels In Reverse
//void moveReverse(int spd) {
//  analogWrite(enA, spd+Ltrim);
//  analogWrite(enB, spd+Rtrim);
//  leftWheelReverse();
//  rightWheelReverse();
//}

// Stop Both Wheels
void moveStop() {
  leftWheelStop();
  rightWheelStop();
}

// Turn Vehicle Left
void moveLeft() {
  leftWheelReverse();
  rightWheelForward();
}

//void softLeft(int spd) {
//  analogWrite(enA, (spd+Ltrim)/2);
//  analogWrite(enB, spd+Rtrim);
//  leftWheelForward();
//  rightWheelForward();  
//}

// Turn Vehicle Right
void moveRight() {
  leftWheelForward();
  rightWheelReverse();
}


//void moveRight() {
//  leftWheelForward();
//  rightWheelForward();  
//}

// Set Left Wheel Forward
void leftWheelForward() {
  leftWheel(HIGH, LOW);
}

void leftWheelReverse() {
  leftWheel(LOW, HIGH);
}

// Turn Both Left Wheel Inputs Off
void leftWheelStop() {
  leftWheel(LOW, LOW);
}

// Set Right Wheel Forward
void rightWheelForward() {
  rightWheel(HIGH, LOW);
}

// Set Right Wheel In Reverse
void rightWheelReverse() {
  rightWheel(LOW, HIGH);
}

// Turn Both Right Inputs Wheel Off
void rightWheelStop() {
  rightWheel(LOW, LOW);
}

// Left Wheel Motion Control
void leftWheel(uint8_t m1, uint8_t m2) {
  digitalWrite(in1, m1);
  digitalWrite(in2, m2);
}

// Right Wheel Motion Control
void rightWheel(uint8_t m3, uint8_t m4) {
  digitalWrite(in3, m3);
  digitalWrite(in4, m4);
}

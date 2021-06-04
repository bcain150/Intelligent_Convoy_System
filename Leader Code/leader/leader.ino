#include <TinyGPS.h>
#include <ArduinoBlue.h>

// Motor A (LEFT) connections
#define enA 2
#define in1 4
#define in2 5
// Motor B (RIGHT) connections
#define enB 3
#define in3 6
#define in4 7
//state pin for HM-11
#define state 8
//MOVE FLAGS for info to send to follower vehicle regarding the lead vehicles movement etc
#define MOVE_STOP 0      //The lead vehicle is stopped.
#define DISCON 1         //The bluetooth control is disconnected from the lead vehicle
#define MOVE_FORW 2
#define MOVE_BACK 3
#define HARD_LEFT 4
#define SOFT_LEFT 5
#define HARD_RIGHT 6
#define SOFT_RIGHT 7

//********BLUETOOTH STUFF*********//
ArduinoBlue phone(Serial3);
//min number of loop cycles that the State pin(8) on HM-11 must be high for inorder to register a valid bluetooth connection
const unsigned long MIN_HIGH_TIME = 150000;
int prevThrottle = 49;
int prevSteering = 49;
int throttle, steering;
float spd;
unsigned long high_cnt = 0;
bool blu_connect = false;   //turns to true if the blutooth is connected

//********MOTOR STUFF*********//
float scaleSpeed(int spd);
void moveForward();
void moveReverse();
void moveStop();
void turnLeft();
void turnRight();
void leftWheelForward();
void leftWheelReverse();
void rightWheelForward();
void rightWheelReverse();
void leftWheel(uint8_t m1, uint8_t m2);
void rightWheel(uint8_t m3, uint8_t m4);
int Ltrim = 100;
int Rtrim = 0;

//********GPS STUFF*********//
TinyGPS gps; // create gps object
long lat=0, lon=0, cours=0; //Global Packet Data
unsigned long fix_age, time_, date;  //ms since last data was encoded (used to see if gps data is stale)
int send_to_follower(long lat, long lon, long cours, int movement, unsigned long time_, unsigned long fix);
int MOVE_FLAG;  //sent to follower, tells how the leader is moving
bool new_data;  //says whether tinyGPS has encoded new dats

//********WIFI STUFF*********//
//responses to look for in serial
#define RES_OK "OK"
#define RES_ERROR "ERROR" 
#define RES_READY "ready"
#define RES_SENT "SEND OK"
//0 in the below define is important. if the follower disconnects and rejoins before the lead esp registers that it lost tcp connection,
//then the follower could rejoin with TCP id 1 and the lead vehicle won't be able to send info. (send AT command will fail)
//this is easily solved by a system reset of the follower a few times, if that doesnt work do a simultaneous system reset of both vehicles.
#define RES_CONNECT "0,CONNECT" 
//LED notifier pinouts
#define WIFI_LED 12 //turns on when softAP is set up in leader
#define TCP_LED LED_BUILTIN  //blinks when TCP port is set up, holds on when follower is connected
//constant ids, modes, ports, and passwords used by the ESP wifi and TCP connection
const String ssid = "host"; //these should be hardcoded on station side as well
const String pass = "12345ics!";
const String cwmode = "2"; //Access Point mode
const String cipmux = "1"; //Multiple TCP connection mode for host
const String wifi_channel = "1"; //wifi band channel for soft ap
const String port = "80"; //port to host the tcp server on
const String enc_mode = "3"; //encryption mode is WPA2_PSK
//functions used for ESP setup and communication/
bool reset_ESP();
bool setup_ap();
bool setup_tcp_host();
bool send_command(String command, long timeout, String res);


//********SETUP FUNCTION*********//
/*
 * enables serial communcation and registers digital pins as inputs/outputs
 * names the HM-11 bluetooth device as something recognizable
 * reboots the esp (this clears previous wifi settings because of use of CUR commands instead of normal ones)
 * sets up soft access point -> displays success via led
 * sets up TCP connection -> displays success via led blink
 * Waits until follower connects to TCP connection, if not setup loops infinitely -> registers success with solid led
 */
void setup() {
  Serial.begin(9600); // pins 0 (Rx), 1 (TX) goes to PC
  Serial1.begin(9600); // pins 18 (TX1), 19 (RX1) from GPS
  Serial2.begin(115200);  //pins 16 (TX2), 17(RX2) goes to ESP8266
  Serial3.begin(9600); // pins 14 (TX3), 15 (RX3) connects to HM-11
  
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  //set the state pin for HM-11 to an input
  pinMode(state, INPUT);
  //sets connection indicator LED's
  pinMode(WIFI_LED, OUTPUT);
  pinMode(TCP_LED, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(WIFI_LED,LOW);
  digitalWrite(TCP_LED,LOW);
  
  Serial3.write("AT+NAMEICS1_Leader\r\n"); //sets display name for HM-11
  Serial3.flush();
  if(!reset_ESP()) {  //resets the ESP
    while(1) {
      //if this blinks that means ESP reset failed and thats very bad. Suggest powering off arduino, reflashing it, or if worse comes to worse, calling AT+RESTORE on ESP
      digitalWrite(WIFI_LED, HIGH);
      delay(500);
      digitalWrite(WIFI_LED, LOW);
      delay(500);
    }
  }
  //set up wifi robustly (inorder to not break ESP and time program correctly
  bool set_ap = setup_ap();
  if(set_ap) {
    digitalWrite(WIFI_LED, HIGH); //sets up access point
    Serial.println("Access Point Set");
  }
  bool tcp_set = setup_tcp_host(); //sets up tcp connection
  bool tcp_connect = false;

  Serial2.setTimeout(500);
  while(!tcp_connect) { //will loop infinitely unless the follower finds the tcp link
    if(tcp_set) {// blinks and waits for connect if there was a successful tcp server hosted
      digitalWrite(TCP_LED, HIGH);
      tcp_connect = Serial2.find(RES_CONNECT);
      digitalWrite(TCP_LED, LOW);
      delay(500);
    }
  }
  digitalWrite(TCP_LED, HIGH);
  Serial.println("TCP connection successful");
}

//********LOOP FUNCTION*********//
/*
 * constantly checks the state pin to register valid bluetooth connection
 * -basically if its high for a certain amount of time theres a connection (when theres a disconnection it switches between low and high)
 * feeds tinyGPS object constantly as long as there is valid data from Serial 1 (the gps has to recieve a signal for this to be true)
 * if there is a bluetooth connection, motor control is enabled
 * if there is valid GPS data coming in, then it is sent to the follower
 * Since arduino can only process things sequentially and the motor control takes a lot of clock cycles,
 * -the GPS sends data once every ~0.8 seconds or so. This can be increased by increasing baud rates of all serial communcation.
 * -However the ESP is at the max baud rate.
 */
void loop() {
  //tests whether or not the blutooth is connected
  if(digitalRead(state) == HIGH) {
    if(!blu_connect && MIN_HIGH_TIME < high_cnt) {  //if the connection registers and previously wasnt
        Serial.println("Bluetooth connection successful.");
        //Serial.flush();
        blu_connect = true;       
    }
    //could check for high_cnt overflow here but it's type long
    //so it's REALLY unlikely to do so (would have to run for 7hrs straight)
    high_cnt++;
  }
  else {//detects when the bluetooth disconnects
    high_cnt = 0;
    if(blu_connect) { //if it was previously connected tell the serial monitor that the connection was lost.
      Serial.println("Bluetooth Connection was Lost.");
      //Serial.flush();
      blu_connect = false;
      moveStop();
    }
    MOVE_FLAG = DISCON; 
  }

  //tiny GPS object needs to be constantly updated to work properly
  if(Serial1.available() > 0) new_data = gps.encode(Serial1.read());

  //if the bluetooth is connected do motor control and GPS data transmission over wifi
  if(blu_connect){
    throttle = phone.getThrottle();
    steering = phone.getSteering();
    // Only change movement if either throttle or steering changes
    if (prevThrottle != throttle || prevSteering != steering) {
      prevThrottle = throttle;
      prevSteering = steering;
      spd = scaleSpeed(throttle);
      if (steering > 35 && steering < 65) { // STRAIGHT
        analogWrite(enA, spd-Ltrim);
        analogWrite(enB, spd-Rtrim);
        if (throttle < 49) moveReverse();      // REVERSE
        else if (throttle > 49) moveForward();  // FORWARD
        else if (throttle == 49) moveStop();    // STILL
      }//end STRAIGHT
      else if (steering <= 35 && steering > 25) { // SLIGHTLY LEFT
        analogWrite(enA, (spd-Ltrim)/2); // Left Wheel
        analogWrite(enB, spd-Rtrim); // Right Wheel
        turnLeft();
        MOVE_FLAG = SOFT_LEFT;
      }//end SLIGHTLY LEFT
      else if (steering >= 65 && steering < 75) { // SLIGHTLY RIGHT
        analogWrite(enA, spd-Ltrim); // Left Wheel
        analogWrite(enB, (spd-Rtrim)/2); // Right Wheel
        turnRight();
        MOVE_FLAG = SOFT_RIGHT;
      }//end SLIGHTLY RIGHT
      else {                                  //HARD LEFT OR RIGHT
        analogWrite(enA, spd-Ltrim);
        analogWrite(enB, spd-Rtrim);
        if (steering < 25) {
          turnLeft();        //LEFT
          MOVE_FLAG = HARD_LEFT;
        }
        else if (steering > 75) {
          turnRight();  //RIGHT
          MOVE_FLAG = HARD_RIGHT;
        }
      }//end HARD TURNING
    }//end steering
    
  }//end Bluetooth if
  //sends gps to follower via ESP if new data has been encoded
  if(new_data) {
    gps.get_position(&lat, &lon, &fix_age);
    cours = gps.course();
    gps.get_datetime(&date, &time_, &fix_age);
    //package and send data to follower
    int send_complete = send_to_follower(lat, lon, cours, MOVE_FLAG, time_, fix_age);
    //switches LEDS based on return case
    if(send_complete == 1) {             //successful return
      digitalWrite(WIFI_LED, HIGH);
      digitalWrite(TCP_LED, HIGH);
    }
    else if(send_complete == 0) {   //Wifi most likely dropped
      digitalWrite(WIFI_LED, LOW);
      digitalWrite(TCP_LED, LOW);
    }else if(send_complete == -1){                         //data send failed ESP is busy, the GPS data is invalid, or the data is too big
      digitalWrite(WIFI_LED, HIGH);
      digitalWrite(TCP_LED, LOW);
    }
    //if the send_complete was -2 then the GPS data was bad so it shouldnt be sent.
  }
  /*if(Serial2.available()) { //UNCOMMENT FOR ESP COMMUNICATION
    byte b = Serial2.read();
    Serial.write(b);
  }

  if(Serial.available()) {
    byte a = Serial.read();
    Serial2.write(a);
  }*/
}

//********FUNCTION DEFINITIONS*********//

/*Sends info to follower
 *  sends position in lat/long
 *  sends course in degrees (0 is N, 90 is E)
 *  sends fix value
 *  sends directional data
 *  adds ',' delimiter, '#' packet terminator, and '$' read terminator (helps make follower reading faster and more robust)
 */
int send_to_follower(long lat, long lon, long cours, int movement, unsigned long time_, unsigned long fix) {
  int data_bytes;
  long timeout = 5000;
  /*String retval = "";
  char c;*/
  
  //compiles data into a string to be sent, makes sure to add packet delimeters and read/packet terminators
  String packet = String(lat) + "," + String(lon) + "," + String(cours) + "," + String(movement) + "," + String(time_) + ',';
  //Serial.println(String(time_));
  //decides whether to send fix (could be invalid meaning that the GPS object's update was invalid)
  if(fix == TinyGPS::GPS_INVALID_AGE) return -1; //dont send the command if the fix is invalid
  else packet += String(fix) + ",#$"; //append the fix to the send packet

  //send the CIPSEND command and immediately after send the data packet
  //calculate data length in bytes, initialize command string
  data_bytes = strlen(packet.c_str());
  String SEND = "AT+CIPSEND=0," + String(data_bytes) + "\r\n";
  if(send_command(SEND, timeout, ">")) {
    if(send_command(packet, timeout, RES_SENT)) return 1;  //send command succeeded
    else return -1; // TCP send could be busy or data took too long to send
  } 
  else return 0; //failed to start transmission:either wifi failed(Most likely) or tcp link was lost
  /*
   * NOTE: if send fails too many times, ESP could mess up and block TCP connection completely, this should be fixed by a lead vehicle reset
   * if that doesnt work, calling AT+RESTORE on both ESP's may be required however the CUR version of wifi commands should prevent this by not saving to flash
   */
}


/*Sets up the access point:
 * sets the ESP's Mode to access point mode
 * starts the softAP with the given global parameters
 * returns OK on success or an Error message with the ESP output
 * uses AT _CUR commands so that things arent saved in ESP flash
 * ONLY CALLED DURING SETUP
 */
bool setup_ap() {
  long timeout = 5000;
  String response = "";
  String MODE = "AT+CWMODE_CUR=" + cwmode + "\r\n";
  bool send_ok = send_command(MODE, timeout, RES_OK);
  if(!send_ok) return false;
  //response += "ESP Recieved the " + String(RES_OK) + " response from: \n" + MODE;
  //set up the access point
  String SAP = "AT+CWSAP_CUR=\"" + ssid + "\",\"" + pass + "\"," + wifi_channel + "," + enc_mode + "\r\n";
  send_ok = send_command(SAP, timeout, RES_OK);
  if(!send_ok) return false;
  //response += "ESP Recieved the " + String(RES_OK) + " response from: \n" + SAP;
  return true;
}

/*sets up a tcp server
 * sets CIPMUX to 1
 * turns on server to port 80
 * returns a string with success or error message
 * ONLY CALLED DURING SETUP
 */
bool setup_tcp_host() {
  //set multiple connection mode
  long timeout = 5000;
  String response = "";
  String MUX = "AT+CIPMUX=" + cipmux + "\r\n";
  bool send_ok = send_command(MUX, timeout, RES_OK);
  if(!send_ok) return false;
  //response += "ESP Recieved the " + String(RES_OK) + " response from: \n" + MUX;
  
  //turns on tcp server on port 80
  String SERV = "AT+CIPSERVER=1," + port + "\r\n";
  send_ok = send_command(SERV, timeout, RES_OK);
  if(!send_ok) return false;
  //response += "ESP Recieved the " + String(RES_OK) + " response from: \n" + SERV;
  return true;
}

/*
 * Resets ESP Module (reboots)
 * ONLY CALLED DURING SETUP
 */
bool reset_ESP() {
  int timeout = 10000; //10 second timeout time (will probably be much shorter than this if there isnt a problem)
  String RST = "AT+RST\r\n";
  bool send_ok = send_command(RST, timeout, RES_READY);
  if(!send_ok) {
    return false;//"Something went wrong when trying to send the reset command to the ESP\n" + String(RES_READY) + " was not recieved or timed out after " + String (timeout) + "ms";
  }
  return true;//RST + "command successful\n";
}


/*sends a command via serial terminal to ESP
 * -command is the command to send in a String
 * -timeout is the MAX amount of time to wait for a response
 * -response is the response look for before the function returns
 * 
 * sends the command and then sets the serial2 time out
 * returns the call to find with the requested responses string
 * (find reads from serial until a string is found or the timeout is reached)
 * 
*/
bool send_command(String command, long timeout, char* response) {

  Serial2.write(command.c_str()); //writes the command to the ESP port
  Serial2.flush(); //waits until all info is done sending to serial
  Serial2.setTimeout(timeout);
  return Serial2.find(response);
}

/*  These next functions basically just make it easier to control
 *   the vehicle's differential steering with the bluetooth joystick.
 *   movement functions are as follows
 *   -forward (right forward and left forward)
 *   -reverse (right back and left back)
 *   -stop (all stop)
 *   -left (right forward and left back)
 *   -right (left forward and right back)
 */

// Move Both Wheels Forward
void moveForward() {
  leftWheelForward();
  rightWheelForward();
  MOVE_FLAG = MOVE_FORW;
}

// Move Both Wheels In Reverse
void moveReverse() {
  leftWheelReverse();
  rightWheelReverse();
  MOVE_FLAG = MOVE_BACK;
}

// Stop Both Wheels
void moveStop() {
  leftWheelStop();
  rightWheelStop();
  MOVE_FLAG = MOVE_STOP;
}

// Turn Vehicle Left
void turnLeft() {
  leftWheelReverse();
  rightWheelForward();
}

// Turn Vehicle Right
void turnRight() {
  leftWheelForward();
  rightWheelReverse();
}

// Set Left Wheel Forward
void leftWheelForward() {
  leftWheel(HIGH, LOW);
}

// Set Left Wheel In Reverse
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

/*Scales speed for setting the throttle 
 *  Outputs value from 0 to 255
 */
float scaleSpeed(int spd) {
  return abs((spd - 49) * 5.1); // abs((spd - 49) * 255 / (99 - 49))
}

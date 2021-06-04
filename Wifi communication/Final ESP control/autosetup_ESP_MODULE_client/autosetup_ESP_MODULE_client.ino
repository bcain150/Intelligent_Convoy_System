//Writen by Brendan Cain 1/15/2020
//UMBC ISC team 1 CMPE capstone
//This code is run on the client side, it connects to an access point and 

String res_OK = "OK";
String res_ERROR = "ERROR";
String res_ready = "ready";
String ssid = "host"; //these should be hardcoded on station side as well
String pass = "12345ics!";
String cwmode = "2"; //Access Point mode
String cipmux = "1"; //Multiple TCP connection mode for host
String wifi_channel = "1"; //wifi band channel for soft ap
String port = "80"; //port to host the tcp server on
String enc_mode = "3"; //encryption mode is WPA2_PSK

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  
  delay(1000);
  Serial.println(reset_ESP());  //resets the ESP
  Serial.println(connect_ap()); //connect to access point
  Serial.println(connect_tcp_client()); //connects to existing tcp connection
}

void loop() {
   // put your main code here, to run repeatedly:

  if(Serial1.available()) {
    byte b = Serial1.read();
    Serial.write(b);
  }

  if(Serial.available()) {
    byte a = Serial.read();
    Serial1.write(a);
  }
  
}

/*Connects to host's access point:
 * sets the ESP's Mode to station mode
 * connects to the SoftAP with the given global parameters
 * returns OK on success or an Error message with the ESP output
 */
String connect_ap() {
  String MODE = "AT+CWMODE=" + cwmode + "\r\n";
  String retval = send_command(MODE, 1);
  if(retval.endsWith(res_ERROR)) {
    return "ERROR: could not set the ESP MODE to Station mode\n" + retval;
  }
  

  //set up the access point
  String JAP = "AT+CWJAP=\"" + ssid + "\",\"" + pass + "\"" + "\r\n";
  retval = send_command(JAP, 1);
  if(retval.endsWith(res_ERROR)) {
    return "ERROR: could not join Access Point\n" + retval;
  }

  return res_OK;
}

/*connects to a tcp server
 * sets CIPMUX to 0
 * connects server to port 80 from given local ip address (host ip)
 * returns a string with "OK" on success, error message if failed
 */
String connect_tcp_client() {
  //set single connection mode
  String MUX = "AT+CIPMUX=" + cipmux + "\r\n";
  String retval = send_command(MUX, 1);
  if(retval.endsWith(res_ERROR)) {
    return "ERROR: could not set to multiple connection mode\n" + retval;
  }
  
  //connects to server on port 80
  String START = "AT+CIPSTART=\"TCP\",\"" + host_ip + "\"," + port + "\r\n";
  retval = send_command(START, 1);
  if(retval.endsWith(res_ERROR)) {
    return "ERROR: could not set to multiple connection mode\n" + retval;
  }

  return retval;
}
/*Resets ESP module
 * 
 */
String reset_ESP() {
  String RST = "AT+RST\r\n";
  return send_command(RST, 2);
}

/*sends a command via serial terminal to ESP
 * command is the command to send in a String
 * num_resonse is the number of times ok, error, or ready would be sent
 * returns the string that was returned as a result
  */
String send_command(String command, int num_responses) {
  char c;
  String retval;
  String response = "";
  bool responded = false;
  
  Serial2.write(command.c_str()); //writes the command to the ESP port
  for(int i=0; i<num_responses; i++){ //loops for number of ok, error, or ready responses
    retval = "";
    while(!responded){ //loops until OK, ERROR, or ready is recieved
      while(Serial2.available()) {
        c = Serial2.read();
        if(c != '\0') {
          retval += c;
        }
      }
      if(retval.indexOf(res_OK) >= 0) responded = true;
      else if(retval.indexOf(res_ERROR) >= 0) responded = true;
      else if(retval.indexOf(res_ready) >= 0) responded = true;
    }
    response += retval;
    responded = false;
  }
  return response;
}

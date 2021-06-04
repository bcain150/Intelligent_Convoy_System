//Writen by Brendan Cain 1/15/2020
//UMBC ISC team 1 CMPE capstone
//This code is run on the Host side, sets up an access point and hosts a TCP connection

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
  Serial2.begin(115200);

  delay(1000);
  Serial.println(reset_ESP());  //resets the ESP
  Serial.println("RST DONE");
  Serial.println(setup_ap()); //sets up access point
  Serial.println("Access Point Set");
  Serial.println(setup_tcp_host()); //sets up tcp connection
  Serial.println("TCP Server Set");
}

void loop() {
   // put your main code here, to run repeatedly:

  if(Serial2.available()) {
    byte b = Serial2.read();
    Serial.write(b);
  }

  if(Serial.available()) {
    byte a = Serial.read();
    Serial2.write(a);
  }
  
}

/*Sets up the access point:
 * sets the ESP's Mode to access point mode
 * starts the softAP with the given global parameters
 * returns OK on success or an Error message with the ESP output
 */
String setup_ap() {
  String response = "";
  String MODE = "AT+CWMODE_CUR=" + cwmode + "\r\n";
  String retval = send_command(MODE, 1);
  if(retval.endsWith(res_ERROR)) {
    return "ERROR: could not set the ESP to AP mode\n" + retval;
  }
  response += retval;
  
  //set up the access point
  String SAP = "AT+CWSAP_CUR=\"" + ssid + "\",\"" + pass + "\"," + wifi_channel + "," + enc_mode + "\r\n";
  retval = send_command(SAP, 1);
  if(retval.endsWith(res_ERROR)) {
    return "ERROR: could not setup Access Point\n" + retval;
  }
  response += retval;
  return response;
}

/*sets up a tcp server
 * sets CIPMUX to 1
 * turns on server to port 80
 * returns a string with "OK" on success, error message if failed
 */
String setup_tcp_host() {
  //set multiple connection mode
  String response;
  String MUX = "AT+CIPMUX=" + cipmux + "\r\n";
  String retval = send_command(MUX, 1);
  if(retval.endsWith(res_ERROR)) {
    return "ERROR: could not set multiple connection mode\n" + retval;
  }
  response += retval;
  
  //turns on tcp server on port 80
  String SERV = "AT+CIPSERVER=1," + port + "\r\n";
  retval = send_command(SERV, 1);
  if(retval.endsWith(res_ERROR)) {
    return "ERROR: could not set up server\n" + retval;
  }
  response += retval;
  return response;
}

/*
 * Resets ESP Module
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

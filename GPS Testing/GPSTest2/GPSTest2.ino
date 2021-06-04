#include <TinyGPS.h>

TinyGPS gps; // create gps object

static const uint32_t BaudRate = 9600; // set Serial & Serial1 Baud Rate
long lat = 0,lon = 0;
double spd = 0;
unsigned long dt = 0,tm = 0,ag = 0;

void printinfo1(int dt, int tm, double lat1, double lon1, double lat2, double lon2, double spd, double dis, double crs); // with date
void printinfo2(int dt, int tm, double lat1, double lon1, double lat2, double lon2, double spd, double dis, double crs); // without date
void printdate(int dt);
void printtime(int tm);

void setup(){
  Serial.begin(9600); // pins 0 (Rx), 1 (TX)
  Serial1.begin(9600); // pins 18 (TX1), 19 (RX1)
  delay(1000);
  if(Serial1.available() > 0){ // if RX1 is able to receive data (buffer not full)
    Serial.println("The GPS Received Signal:");
  }
}

void loop(){
  while(Serial1.available() > 0){ // while RX1 is able to receive data (buffer not full)
    if(gps.encode(Serial1.read())){ // read RX1 in
      gps.get_datetime(&dt, &tm, &ag); // get date, time and longitude
      gps.get_position(&lat, &lon); // get latitude and longitude
      spd = gps.f_speed_mps(); // get speed

      double lat2 = double(lat)/1000000.0; // test
      double lon2 = double(lon)/1000000.0;

      double lat3 = lat2 - 0.000007;
      double lon3 = lon2 - 0.000007;


      double dis = gps.distance_between(lat2, lon2, lat3, lon3); // get distance to next position
      double crs = gps.course()/100.0; // get angle to next position. North = 0, East = 90
      printinfo2(dt, tm, lat2, lon2, lat3, lon3, spd, dis, crs); // print to Serial
    }
  }
}

void printinfo1(int dt, int tm, double lat1, double lon1, double lat2, double lon2, double spd, double dis, double crs){
  Serial.print("Date\t\tTime\t\tSpeed (mps)\tAt\t\t\tTo\t\t\tDistance (m)\tCourse (Degrees)\n");
  printdate(dt);
  printtime(tm);
  
  if((spd < -2) || (spd > 10)){ // speed limit check
    Serial.print("*");
    Serial.print(spd,3);
    Serial.print("*");
  }
  else{
    Serial.print(spd,3);
  }
  Serial.print("\t\t");
  
  Serial.print(lat1,6); // current position
  Serial.print(", ");
  Serial.print(lon1,6);
  Serial.print("\t");

  Serial.print(lat2,6); // next position
  Serial.print(", ");
  Serial.print(lon2,6);
  Serial.print("\t");
  
  Serial.print(dis,3); // distance & angle
  Serial.print("\t");
  Serial.print(crs,3);
  Serial.print("\n\n");
}

void printinfo2(int dt, int tm, double lat1, double lon1, double lat2, double lon2, double spd, double dis, double crs){
  Serial.print("Time\t\tSpeed (mps)\tAt\t\t\tTo\t\t\tDistance (m)\tCourse (Degrees)\n");
  printtime(tm);
  
  if((spd < -2) || (spd > 10)){ // speed limit check
    Serial.print("*");
    Serial.print(spd,3);
    Serial.print("*");
  }
  else{
    Serial.print(spd,3);
  }
  Serial.print("\t\t");
  
  Serial.print(lat1,6); // current position
  Serial.print(", ");
  Serial.print(lon1,6);
  Serial.print("\t");

  Serial.print(lat2,6); // next position
  Serial.print(", ");
  Serial.print(lon2,6);
  Serial.print("\t");
  
  Serial.print(dis,3); // distance & angle
  Serial.print("\t");
  Serial.print(crs,3);
  Serial.print("\n\n");
}

void printdate(int dt){
  String day = String(dt).substring(0, 2);
  String month = String(dt).substring(2, 4);
  String year = String(dt).substring(4, 6);
  Serial.print(month);
  Serial.print("/");
  Serial.print(day);
  Serial.print("/");
  Serial.print(year);
  Serial.print("\t");
}

void printtime(int tm){
  String hour = String(tm).substring(0, 2);
  String minute = String(tm).substring(2, 4);
  String second = String(tm).substring(4, 6);
  Serial.print(hour);
  Serial.print(":");
  Serial.print(minute);
  Serial.print(":");
  Serial.print(second);
  Serial.print("\t");
}

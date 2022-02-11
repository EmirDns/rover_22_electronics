#include <Arduino.h>
#include "HardwareSerial.h"
#include "VescUart.h"
#include "ros.h"
#include "std_msgs/Float64.h"

#define ARRAY_LEN 1

char inc_char;
String rpmString;
int unmapped_rpm_command_array[ARRAY_LEN];
int mapped_rpm_command_array[ARRAY_LEN];

std_msgs::Float64 rpm_data;
ros::NodeHandle nh;
ros::Publisher rpmPub("rpm_rover_topic", &rpm_data);

HardwareSerial mySerial(PB11, PB10);
HardwareSerial pcSerial(PC5, PC4);

VescUart motor1;

void readNdrive(void);
void assignRpmArray(String rpmStr);
void mapData(void);
void drive(void);

void setup() {
  mySerial.begin(115200);
  pcSerial.begin(9600);

  while(!mySerial){;}
  motor1.setSerialPort(&mySerial);
}

void loop() {

  readNdrive();
}

void readNdrive(void){
  static bool receive_flag=false;
  inc_char=pcSerial.read();
  delay(20);
  if(pcSerial.available()>0){
    if(inc_char=='S'){
      rpmString="";
      receive_flag=true;
    }
    if(receive_flag && inc_char!='S' && inc_char!='F'){
      rpmString+=inc_char;
    }
    if(inc_char=='F'){
      
      assignRpmArray(rpmString);
      
      mapData();
      
      pcSerial.println(mapped_rpm_command_array[0]);
      motor1.setRPM(mapped_rpm_command_array[0]);

      receive_flag=false;
      rpmString="";
    }
  }
}

void assignRpmArray(String rpmStr){
  String str_buffer;
  char direction_char;
  int direction;
  for(int i=0;i<ARRAY_LEN;i++){
    direction_char=rpmStr[4*i];
    if(direction_char=='0'){
      direction=-1;
    }
    if(direction_char=='1'){
      direction=1;
    }
    for(int j=i*4;j<(i*4)+3;j++){
      str_buffer+=rpmStr[j+1];    
    }
    unmapped_rpm_command_array[i]=direction*str_buffer.toInt();
    str_buffer="";
  }
}

void mapData(void){
  for(int i=0;i<ARRAY_LEN;i++){
    if(unmapped_rpm_command_array[i]<=0){
      mapped_rpm_command_array[i]=((unmapped_rpm_command_array[i]+255)*1000/255)-1000;
    }
    if(unmapped_rpm_command_array[i]>0){
      mapped_rpm_command_array[i]=unmapped_rpm_command_array[i]*1000/255;
    }
  }
}

void drive(void){
  
}

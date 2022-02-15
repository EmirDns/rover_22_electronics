#include <Arduino.h>
#include "HardwareSerial.h"
#include "VescUart.h"
#include "ros.h"
#include "std_msgs/Float64.h"

#define ARRAY_LEN 4

char inc_char;
String rpmString;
int unmapped_rpm_command_array[ARRAY_LEN];
int mapped_rpm_command_array[ARRAY_LEN];

std_msgs::Float64 rpm_data;
ros::NodeHandle nh;
ros::Publisher rpmPub("rpm_rover_topic", &rpm_data);

HardwareSerial RBSerial(PA3, PA2);
HardwareSerial RFSerial(PB11, PB10);
HardwareSerial LBSerial(PC11, PA0);
HardwareSerial LFSerial(PD2, PC12);
HardwareSerial motherBoardSerial(PA10, PA9);

VescUart RBmotor;
VescUart RFmotor;
VescUart LBmotor;
VescUart LFmotor;

void readNdrive(void);
void assignRpmArray(String rpmStr);
void mapData(void);
void drive(void);

char getDir(int x);
int unmapRpm(void);
void createFeedbackMsg(int a, int b, int c, int d);

void setup() {
  RBSerial.begin(115200);
  RFSerial.begin(115200);
  LBSerial.begin(115200);
  LFSerial.begin(115200);

  motherBoardSerial.begin(9600);

  while(!RBSerial){;}
  RBmotor.setSerialPort(&RBSerial);
  
  while(!RFSerial){;}
  RFmotor.setSerialPort(&RFSerial);
  
   while(!LBSerial){;}
  LBmotor.setSerialPort(&LBSerial);

  while(!LFSerial){;}
  LFmotor.setSerialPort(&LFSerial);

  RBmotor.setRPM(0);
  RFmotor.setRPM(0);
  LBmotor.setRPM(0);
  LFmotor.setRPM(0);
  }

void loop() {
  //drive();
  readNdrive();
  
  
  RBmotor.setRPM(mapped_rpm_command_array[0]);
  RFmotor.setRPM(mapped_rpm_command_array[1]);
  LBmotor.setRPM(mapped_rpm_command_array[2]);
  LFmotor.setRPM(mapped_rpm_command_array[3]);
  

  delay(2);
}

void readNdrive(void){
  static bool receive_flag=false;
  inc_char=motherBoardSerial.read();
  delay(1);
  if(motherBoardSerial.available()>0){
    if(inc_char=='S'){
      rpmString="";
      receive_flag=true;
    }
    if(receive_flag && inc_char!='S' && inc_char!='F'){
      rpmString+=inc_char;
    }
    if(inc_char=='F'){
      //RFSerial.println(rpmString);
      assignRpmArray(rpmString);
      
      mapData();
      //drive();
      

      
      if(RBmotor.getVescValues() && RFmotor.getVescValues() && LBmotor.getVescValues() && LFmotor.getVescValues()){
        //RFSerial.println(RBmotor.data.rpm);
        createFeedbackMsg(RBmotor.data.rpm, RFmotor.data.rpm, LBmotor.data.rpm, LFmotor.data.rpm);     
      }
      

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
      mapped_rpm_command_array[i]=((unmapped_rpm_command_array[i]+255)*10000/255)-10000;
    }
    if(unmapped_rpm_command_array[i]>0){
      mapped_rpm_command_array[i]=unmapped_rpm_command_array[i]*10000/255;
    }
  }
}

void drive(void){
  RBmotor.setRPM(mapped_rpm_command_array[0]);
  RFmotor.setRPM(mapped_rpm_command_array[1]);
  LBmotor.setRPM(mapped_rpm_command_array[2]);
  LFmotor.setRPM(mapped_rpm_command_array[3]);
}

char getDir(int x){
  char direction;
  if(x>0){
    direction='1';
  }
  if(x<=0){
    direction='0';
  }
  return direction;
}

int unmapRpm(int x){
  int unmapped_data;
  if(x>0){
    unmapped_data=x*255/10000;
  }
  if(x<=0){
    unmapped_data=((x+10000)*255/10000)-255;
  }
  return unmapped_data;
}

void createFeedbackMsg(int a, int b, int c, int d){
  String sentString="A";
  
  sentString+=getDir(a);
  String processedStringRB = String(abs(unmapRpm(a)));
  while(processedStringRB.length()<3){
    processedStringRB = "0" + processedStringRB;
  }
  sentString+=processedStringRB;
  
  sentString+=getDir(b);
  String processedStringRF = String(abs(unmapRpm(b)));
  while(processedStringRF.length()<3){
    processedStringRF = "0" + processedStringRF;
  }
  sentString+=processedStringRF;

  sentString+=getDir(c);
  String processedStringLB = String(abs(unmapRpm(c)));
  while(processedStringLB.length()<3){
    processedStringLB = "0" + processedStringLB;
  }
  sentString+=processedStringLB;
  
  sentString+=getDir(d);
  String processedStringLF = String(abs(unmapRpm(a)));
  while(processedStringLF.length()<3){
    processedStringLF = "0" + processedStringLF;
  }
  sentString+=processedStringLF;
  
  sentString+="B";
  motherBoardSerial.println(sentString);
}

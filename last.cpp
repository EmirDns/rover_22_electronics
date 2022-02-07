#include <Arduino.h>
#include "HardwareSerial.h"
#include "VescUart.h"
#include "ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64MultiArray.h"

#define ARRAY_LEN 4

std_msgs::Int64 rpm_data;
int inc_char;
static String rpmString;

std_msgs::Float64MultiArray rpm_marray;

int unmapped_rpm_command_array[ARRAY_LEN];
int mapped_rpm_command_array[ARRAY_LEN];

ros::NodeHandle nh;
ros::Publisher rpm_pub("rpm_topic",&rpm_data);

HardwareSerial mySerial3(PB11, PB10);
HardwareSerial mySerial2(PC5, PC4);

VescUart motor1;
VescUart motor2;
VescUart motor3;
VescUart motor4; 

void printRpm(void);
void publishRpm(void);
void readNdrive(void);
void mapData(void);
void assignRpmArray(String rpmStr);
void drive(void);

void setup() {
  mySerial3.begin(115200);
  mySerial2.begin(9600);

  rpm_marray.data=(float *)malloc(sizeof(float)*ARRAY_LEN);
  rpm_marray.data_length=ARRAY_LEN;

  while(!mySerial3) {;}

  motor1.setSerialPort(&mySerial3);
  nh.initNode();
  nh.advertise(rpm_pub);
}

void loop() {
  readNdrive();
  
  delay(50);
}

void printRpm(void){
  if(motor1.getVescValues()){
    mySerial2.print("RPM 1:");
    mySerial2.println(motor1.data.rpm);
    mySerial2.print("RPM 2:");
    mySerial2.println(motor2.data.rpm);
    mySerial2.print("RPM 3:");
    mySerial2.println(motor3.data.rpm);
    mySerial2.print("RPM 4:");
    mySerial2.println(motor4.data.rpm);
  }
  else{
    mySerial2.println("Failed to get the RPM data.");
  }
}

void publishRpm(void){
  nh.spinOnce();
  rpm_marray.data[0]=motor1.data.rpm;
  rpm_marray.data[1]=motor2.data.rpm;
  rpm_marray.data[2]=motor3.data.rpm;
  rpm_marray.data[3]=motor4.data.rpm;

  if(motor1.getVescValues() && motor2.getVescValues() && motor3.getVescValues() && motor4.getVescValues()){
    rpm_data.data=motor1.data.rpm;
    rpm_pub.publish(&rpm_data);
  }
  else{
    nh.logerror("Failed to publish rpm data.");
  }
}

void readNdrive(void){
  static bool receive_flag=false;
  static int receive_counter=0;
  inc_char=mySerial2.read();
  delay(1);
  if(inc_char=='S'){
    rpmString="";
    receive_flag=true;
    receive_counter++;
  }
  if(receive_flag && inc_char!='S' && inc_char!='B'){
    rpmString+=(char)inc_char;
    receive_counter++;
  }
  if(inc_char=='F'){
    assignRpmArray(rpmString);
    mapData();
    drive();
    receive_flag=false;
    receive_counter=0;
    rpmString="";
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

void drive(void){
  motor1.setRPM(mapped_rpm_command_array[0]);
  motor2.setRPM(mapped_rpm_command_array[1]);
  motor3.setRPM(mapped_rpm_command_array[2]);
  motor4.setRPM(mapped_rpm_command_array[3]);
}

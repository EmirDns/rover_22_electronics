#include "drive_interface.h"

int DriveInterface::mapData(float x){
    x=255*x;
    if(x>255){x=255;}
    if(x<-255){x=-255;}

    
    
    return x;
}

String DriveInterface::getDirection(float coming_float){
    String direction;
    if(coming_float<=0){
        direction='0';
    }
    if(coming_float>0){
        direction='1';
    }
    return direction;
}

String DriveInterface::generateString(float x){
    String processedString = String(mapData(fabs(x)));
    while(processedString.length()<3){
        processedString = "0" + processedString;
    }
    return DriveInterface::getDirection(x) + processedString;
}

String DriveInterface::generateMCUMessage(void){
    String sentString = "S";
    for(int i=0;i<_array_length;i++){
        sentString+=generateString(commands_to_send[i]);
    }
    sentString+='F';
    return sentString;
}

void DriveInterface::getThrustings(String encoderwDir){
    String str_buffer;
    char direction_char;
    int direction;
    for(int i=0;i<_array_length;i++){
        direction_char=encoderwDir[4*i];
        if(direction_char=='0'){
            direction=-1;
        }
        if(direction_char=='1'){
            direction=1;
        }
        for(int j=i*4;j<(i*4)+3;j++){
            str_buffer+=encoderwDir[j+1];    
        }
        feedback_arr.data[i]=direction*str_buffer.toFloat()/255;
        str_buffer="";
    }
}

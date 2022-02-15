#ifndef BOARD_INTERFACE_H
#define BOARD_INTERFACE_H

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

class BoardInterface
{
    protected:
        int _array_length;
        int _str_msg_length;
        int _str_piece_len;
        float _radian_map_coef;
        // Parametric data that can be assigned while constructing a new class

        float *commands_to_send;
        float *feedback_to_send;
        // Command and feedback array that is used to interface with arm or drive sys...

        std_msgs::Float64MultiArray feedback_arr;
        std_msgs::Float64MultiArray command_arr;
        // Float arrays to communicate with ROS

        String str_command;
        String str_feedback;
        // String arrays to communicate with MCUs 

    public:

        BoardInterface(int arr_len, int str_msg_len, int str_piece_len, float mapping_coef);
        // Constructor. Assings necessary data and allocates memory

        ~BoardInterface();
        // Destructor. Frees the memory

        void assignFeedbackArr(std_msgs::Float64MultiArray &goal_arr);
        // Assigns the feedback_arr to a global variable

        void assignCommandArr(const std_msgs::Float64MultiArray &given_arr);
        // Assigns the data of the give command array to the private variable

        std_msgs::Float64MultiArray returnFeedbackMultiArr();
        // Returns feedback_arr

        void multiArrToArr();
        // Assigns the data in the command to the commands to send array

        void arrToMultiArr();
        // Assigns the data in the feedback array to the feedback multi array (std_msg)

        void assignStrFeedback(const String &enc_str);
        // Assigns the data to the private variable str_feedback

        String returnCommandStr();
        // Returns command as string variable

        //TEST
        float detectDirection(char direction_byte);
        // Detects the direction (1: Positive, 0: Negative) -> Helper method for children classes' parseEncoderFeedback

};

#endif

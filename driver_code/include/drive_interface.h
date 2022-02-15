#ifndef DRIVE_INTERFACE_H
#define DRIVE_INTERFACE_H

#include "board_interface.h"

class DriveInterface : public BoardInterface
{
    public:
        DriveInterface(int arr_len, int str_msg_len, int str_piece_len, float mapping_coef) : BoardInterface(arr_len, str_msg_len, str_piece_len, mapping_coef){};
        
        ~DriveInterface()
        {
            free(commands_to_send);
            free(feedback_to_send);
            free(feedback_arr.data);
            free(command_arr.data);
        }

        int mapData(float x);

        String getDirection(float coming_float);

        String generateString(float x);

        String generateMCUMessage(void);

        void getThrustings(String encoderwDir);
};

#endif

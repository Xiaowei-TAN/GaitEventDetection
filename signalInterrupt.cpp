//
// Created by sia on 20-8-27.
//

#include "signalInterrupt.h"
#include <iostream>

//capture ctrl+c signal to elegantly close these threads
extern volatile bool FLG_THD_IMU;
extern volatile bool FLG_THD_IMU_L;
extern volatile bool FLG_THD_IMU_R;
extern volatile bool FLG_THD_FSR;


void handle(int sig) {
    if (sig == SIGINT) {

        //end threads
        FLG_THD_IMU     = false;
        FLG_THD_IMU_L   = false;
        FLG_THD_IMU_R   = false;
        FLG_THD_FSR     = false;

        std::cout<< "[NOTE]: Interrupt Signal Captured!"<<std::endl;
    }
}

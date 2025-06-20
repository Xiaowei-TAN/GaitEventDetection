
#include <stdlib.h>
#include <thread>
#include <iostream>
#include <fstream> //log file
#include <vector>
#include "signalInterrupt.h"
#include "macroDefinition.h"
#include "xsensImu.h"
#include "logCreate.h"
#include "fsrReading.h"


using namespace std;
ofstream logFile;

//define which side of legs will be monitored
vector<int> activeDevice = {sLeft}; //{sLeft, sRight}

//----------------------------------------------------------------------
// Main Function
//----------------------------------------------------------------------
pthread_mutex_t mutMainTrd;
pthread_cond_t  cndMainTrd;
int main(int argc, char *argv[]) {

    //signal interrupt
    signal(SIGINT, handle);

    //create a log file
    string fileName = logCreate(&logFile, "./logFile/");

    //------------------------initialize-------------------------------------
    //init signal lock
    pthread_mutex_init(&mutMainTrd, 0);
    pthread_cond_init(&cndMainTrd, NULL);

    //create thread variables
    pthread_t imu, fsr;
    pthread_attr_t attr_imu, attr_fsr;
    int8_t ret_imu{-1}, ret_fsr{-1};

    try {
        //------------------------imu thread----------------------------------
        if (pthread_attr_init(&attr_imu) != EOK)
            throw std::runtime_error("[ERROR] IMU-THREAD INIT ERROR");
        if ( (ret_imu = pthread_create(&imu, &attr_imu, imuThread, (void *) NULL)) != EOK)
            throw std::runtime_error("[ERROR] IMU-THREAD CREATE ERROR");

        //wait to receive the thread reply
        pthread_cond_wait(&cndMainTrd, &mutMainTrd);
        cout << "[IMU]: SIGNAL RECEIVED" << endl;

        //test if the thread has kept running
        this_thread::sleep_for(std::chrono::milliseconds(100));

        //the thread has exited due to some unknown reason
        if (pthread_kill(imu, 0) == ESRCH)
            throw std::runtime_error("[ERROR] IMU-THREAD exited unexpectedly");


        //------------------------FSR thread----------------------------------
        if (pthread_attr_init(&attr_fsr) != EOK)
            throw std::runtime_error("[ERROR] FSR-THREAD INIT ERROR");
        if ( (ret_fsr = pthread_create(&fsr, &attr_fsr, fsrThread, (void *) NULL)) != EOK)
            throw std::runtime_error("[ERROR] FSR-THREAD CREATE ERROR");

        //wait to receive the thread reply
        pthread_cond_wait(&cndMainTrd, &mutMainTrd);
        cout << "[FSR]: SIGNAL RECEIVED" << endl;

        //test if the thread has kept running
        this_thread::sleep_for(std::chrono::milliseconds(100));

        //the thread has exited due to some unknown reason
        if (pthread_kill(fsr, 0) == ESRCH)
            throw std::runtime_error("[ERROR] FSR-THREAD exited unexpectedly");

    }

    catch (std::exception const &ex) {
        std::cout << ex.what() << std::endl;

        //signal to all alive threads to exit
        handle(SIGINT);

        //wait other threads completing exit
        this_thread::sleep_for(std::chrono::milliseconds(100));

        //close and delete log file since no data have been recorded
        logFile.close();
        remove(fileName.c_str());

        //exit the whole process [bad, variables have not been freed]
        exit(EXIT_SUCCESS);
    }


    //---------------------------deinit-------------------------------------
    //wait threads to exit
    if (ret_imu == EOK) {
        pthread_join(imu, NULL);
        pthread_attr_destroy(&attr_imu);
    }
    if (ret_fsr == EOK) {
        pthread_join(fsr, NULL);
        pthread_attr_destroy(&attr_fsr);
    }

    //destroy conditional signal
    pthread_cond_destroy(&cndMainTrd);
    pthread_mutex_destroy(&mutMainTrd);

    //close log file
    logFile.close();

    //exit the whole process
    exit(EXIT_SUCCESS);
}

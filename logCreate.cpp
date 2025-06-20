//
// Created by sia on 21-11-4.
//

#include <fstream>
#include "logCreate.h"

std::string logCreate(std::ofstream* logFile, std::string address = "./logFile/") {

    //get the time
    time_t timer;
    time(&timer);

    //convert to the local time
    struct tm *localTime;
    localTime = localtime(&timer);

    //convert the time format
    char fileTime[80];
    strftime(fileTime, 80, "%Y%m%d_%H%M%S", localTime);
    std::string fileName(fileTime);
    fileName = address + fileName + ".csv";

    //create the log file
    logFile->open(fileName, std::ios::out | std::ios::app);

    //output the file name (including file path)
    return fileName;
}
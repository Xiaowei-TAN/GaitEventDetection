cmake_minimum_required(VERSION 3.14)
project(gaitEventDetect)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILE_TYPE DEBUG)


#Xsens
INCLUDE_DIRECTORIES(./Xsens/include)
LINK_DIRECTORIES(./Xsens/lib64)


add_executable(${CMAKE_PROJECT_NAME} main.cpp signalInterrupt.cpp signalInterrupt.h macroDefinition.h timeInterval.cpp timeInterval.h logCreate.cpp logCreate.h xsensImu.cpp xsensImu.h fsrReading.cpp fsrReading.h)

#Extra Libs
set(EXTRALIBS
        m #math
        pthread #thread
        rt) #real time extension library
target_link_libraries(${CMAKE_PROJECT_NAME} ${EXTRALIBS})

#Xsens
target_link_libraries(${CMAKE_PROJECT_NAME} xsensdeviceapi xstypes)
//
// Created by sia on 24-12-17.
//

#include "fsrReading.h"
#include <iostream> //cout
#include <iomanip> //setprecision
#include <numeric> //accumulate
#include <asm/ioctls.h> //ioctl
#include <sys/ioctl.h> //ioctl
#include <chrono> //sleep_for
#include <thread> //sleep_for
#include <unistd.h> //serial port set
#include <fcntl.h> //serial port set
#include <termios.h> //serial port set
#include "macroDefinition.h"


#define READBYTES  39 //[Unit] bytes

using namespace std;

//mutex for main thread
extern pthread_mutex_t mutMainTrd;
extern pthread_cond_t  cndMainTrd;

//use for other files
_fsrDataStruct fsrValue;

//flags for thread operation
volatile bool FLG_THD_FSR = false;


//----------------------------------------------------------------------
/*! \txw parse channel data [1-12] */
/*! \brief input and return [Unit: gram] values must be positive */
//----------------------------------------------------------------------
uint32_t parseChannel(u_char data[], int channel) {

    if (channel > 12 || channel < 1) {
        return 0;
    }

    uint32_t val =  static_cast<uint32_t>(data[channel*3-1]) << 16 | //high 8bit
                    static_cast<uint32_t>(data[channel*3+0]) << 8 | //middle 8bit
                    static_cast<uint32_t>(data[channel*3+1]); //low 8bit

    return val;
}


//----------------------------------------------------------------------
/*! \txw customize the serial port settings */
/*! \brief settings must be customized for different serials */
//----------------------------------------------------------------------
int setSerialPort(int fd) {
    struct termios options;

    if (tcgetattr(fd, &options) == -1) {
        return -1;
    }

    options.c_cflag|=(CLOCAL|CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;

    cfsetispeed(&options,B115200);
    cfsetospeed(&options,B115200);

    options.c_cflag &= ~CSTOPB;

    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN]  = READBYTES;

    tcflush(fd, TCIFLUSH);
    tcflush(fd, TCOFLUSH);

    if (tcsetattr(fd, TCSANOW, &options) == -1) {
        return -1;
    }

    return true;
}


//----------------------------------------------------------------------
/*! \txw main thread function for FSRs */
//----------------------------------------------------------------------
void *fsrThread(void *pVoid) {

    //lock the main thread
    pthread_mutex_lock(&mutMainTrd);
    cout << "[FSR]: Thread is running" << endl;

    //---------------------------------------------------------------------------
    //init
    string sn = "/dev/ttyFSR";
    int fd;

    try {
        //open port (block function)
        fd = open(sn.c_str(), O_RDONLY | O_NOCTTY); //block mode
        if (fd == -1) {
            printf("error code: %d\n", errno);
            perror("error: ");
            throw std::runtime_error("[ERROR] open port failed");
        }

        //set port
        if (setSerialPort(fd) == -1) {
            throw std::runtime_error("[ERROR] set port failed");
        }

        //unlock the main thread
        pthread_cond_signal(&cndMainTrd);
        pthread_mutex_unlock(&mutMainTrd);

        FLG_THD_FSR = true;
    }
    catch (std::exception const &ex) {
        std::cout << ex.what() << std::endl;
    }


    //---------------------------------------------------------------------------
    //consume ~50ms (30-60ms) per cycle
    //main code
    while (FLG_THD_FSR) {

        u_char rcv[READBYTES]; // must be positive
        memset(rcv, 0, sizeof(rcv));

        //block until READBYTES bytes of data has been read
        while (read(fd, rcv, READBYTES) <= 0);

        //frame header
        if ((rcv[0] != 0x40 && rcv[1] != 0x5c)) {
            tcflush(fd, TCIFLUSH); //clear the input buffer
            continue;
        }

        //check
        u_char check = (accumulate(rcv, rcv + (sizeof(rcv) - 1) / sizeof(rcv[0]), 0)) & 0xff; //extract the lower 8bit
        if (check != rcv[READBYTES - 1]) {
            tcflush(fd, TCIFLUSH); //clear the input buffer
            continue;
        }

        //parse data [Unit: Newton]
        fsrValue.forefootOuter = double(parseChannel(rcv, 1)) / 1000 * 9.8; //channel 1
        fsrValue.forefootInner = double(parseChannel(rcv, 2)) / 1000 * 9.8; //channel 2
        fsrValue.heel = double(parseChannel(rcv, 3)) / 1000 * 9.8; //channel 3

        //print
//        std::cout << setiosflags(std::ios::fixed) << std::setprecision(0)
//                  << fsrValue.forefootOuter << "\t"
//                  << fsrValue.forefootInner << "\t"
//                  << fsrValue.heel << "\t"
//                  << endl;
    }

    //close port
    close(fd);

    //unlock the main thread (in case throw happens)
    pthread_cond_signal(&cndMainTrd);
    pthread_mutex_unlock(&mutMainTrd);

    std::cout << "[FSR]: Thread has exited!" << std::endl;
    pthread_exit(NULL); //terminate the thread (not the process)
}
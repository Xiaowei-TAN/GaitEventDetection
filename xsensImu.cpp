//
// Created by sia on 22-08-03.
//

#include <xsensdeviceapi.h> // Xsens API
#include <xsens/xsmutex.h> // Xsens API
#include <set>
#include <list>

#include <iomanip>
#include <iostream>
#include <fstream>
#include <math.h>
#include <sys/timeb.h> //isStaticFnc()

#include "timeInterval.h"
#include "xsensImu.h"
#include "macroDefinition.h"
#include "fsrReading.h"

/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream &operator<<(std::ostream &out, XsPortInfo const &p) {
    out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
        << std::setw(7) << p.baudrate() << " Bd"
        << ", " << "ID: " << p.deviceId().toString().toStdString();
    return out;
}

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream &operator<<(std::ostream &out, XsDevice const &d) {
    out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
    return out;
}

/*! \brief Given a list of update rates and a desired update rate, returns the closest update rate to the desired one */
int findClosestUpdateRate(const XsIntArray &supportedUpdateRates, const int desiredUpdateRate) {
    if (supportedUpdateRates.empty()) {
        return 0;
    }

    if (supportedUpdateRates.size() == 1) {
        return supportedUpdateRates[0];
    }

    int uRateDist = -1;
    int closestUpdateRate = -1;
    for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin();
         itUpRate != supportedUpdateRates.end(); ++itUpRate) {
        const int currDist = std::abs(*itUpRate - desiredUpdateRate);

        if ((uRateDist == -1) || (currDist < uRateDist)) {
            uRateDist = currDist;
            closestUpdateRate = *itUpRate;
        }
    }
    return closestUpdateRate;
}

//----------------------------------------------------------------------
// Callback handler for wireless master
//----------------------------------------------------------------------
class WirelessMasterCallback : public XsCallback {
public:
    typedef std::set<XsDevice *> XsDeviceSet;

    XsDeviceSet getWirelessMTWs() const {
        XsMutexLocker lock(m_mutex);
        return m_connectedMTWs;
    }

protected:
    virtual void onConnectivityChanged(XsDevice *dev, XsConnectivityState newState) {
        XsMutexLocker lock(m_mutex);
        switch (newState) {
            case XCS_Disconnected:        /*!< Device has disconnected, only limited informational functionality is available. */
                std::cout << "\nEVENT: MTW Disconnected -> " << *dev << std::endl;
                m_connectedMTWs.erase(dev);
                break;
            case XCS_Rejected:            /*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
                std::cout << "\nEVENT: MTW Rejected -> " << *dev << std::endl;
                m_connectedMTWs.erase(dev);
                break;
            case XCS_PluggedIn:            /*!< Device is connected through a cable. */
                std::cout << "\nEVENT: MTW PluggedIn -> " << *dev << std::endl;
                m_connectedMTWs.erase(dev);
                break;
            case XCS_Wireless:            /*!< Device is connected wirelessly. */
                std::cout << "\nEVENT: MTW Connected -> " << *dev << std::endl;
                m_connectedMTWs.insert(dev);
                break;
            case XCS_File:                /*!< Device is reading from a file. */
                std::cout << "\nEVENT: MTW File -> " << *dev << std::endl;
                m_connectedMTWs.erase(dev);
                break;
            case XCS_Unknown:            /*!< Device is in an unknown state. */
                std::cout << "\nEVENT: MTW Unknown -> " << *dev << std::endl;
                m_connectedMTWs.erase(dev);
                break;
            default:
                std::cout << "\nEVENT: MTW Error -> " << *dev << std::endl;
                m_connectedMTWs.erase(dev);
                break;
        }
    }

private:
    mutable XsMutex m_mutex;
    XsDeviceSet m_connectedMTWs;
};

//----------------------------------------------------------------------
// Callback handler for MTw
// Handles onDataAvailable callbacks for MTW devices
//----------------------------------------------------------------------
class MtwCallback : public XsCallback {
public:
    MtwCallback(int mtwIndex, XsDevice *device)
            : m_mtwIndex(mtwIndex), m_device(device) {}

    bool dataAvailable() const {
        XsMutexLocker lock(m_mutex);
        return !m_packetBuffer.empty();
    }

    XsDataPacket const *getOldestPacket() const {
        XsMutexLocker lock(m_mutex);
        XsDataPacket const *packet = &m_packetBuffer.front();
        return packet;
    }

    void deleteOldestPacket() {
        XsMutexLocker lock(m_mutex);
        m_packetBuffer.pop_front();
    }

    //[Txw]
    void clearAllPacket() {
        XsMutexLocker lock(m_mutex);
        m_packetBuffer.clear();
    }

    //[Txw]
    int availablePacketCount() const {
        XsMutexLocker lock(m_mutex);
        return m_packetBuffer.size();
    }

    int getMtwIndex() const {
        return m_mtwIndex;
    }

    XsDevice const &device() const {
        assert(m_device != 0);
        return *m_device;
    }

protected:
    virtual void onLiveDataAvailable(XsDevice *, const XsDataPacket *packet) {
        XsMutexLocker lock(m_mutex);
        // NOTE: Processing of packets should not be done in this thread.

        m_packetBuffer.push_back(*packet);
        if (m_packetBuffer.size() > 30)
        {
            deleteOldestPacket();
        }

    }

private:
    mutable XsMutex m_mutex;
    std::list<XsDataPacket> m_packetBuffer;
    int m_mtwIndex;
    XsDevice *m_device;
};




//----------------------------------------------------------------------
/*! \txw macro and global variable definition and declaration */
//----------------------------------------------------------------------
//[MACRO] peak point detection
#define IMU_MIN     0
#define IMU_MAX     1

//[MACRO] static state detector
#define VarACC      2.441394827172289e-04   //acc      m/s2
#define VarVEL      0.008732730867865       //angular velocity  deg/s
#define GRAVITY     9.8035

//[GLOBAL] flags for thread operation
volatile bool FLG_THD_IMU   = true;
volatile bool FLG_THD_IMU_L = true;
volatile bool FLG_THD_IMU_R = true;

//[GLOBAL] may be used in exoskeleton control system
std::vector<int>    eventOn(DOFS); //gait events
std::vector<double> gThld_FC(DOFS);
std::vector<double> gThld_FO(DOFS);
std::vector<double> gThld_FO_AGL(DOFS);

//[GLOBAL] fsrReading
extern _fsrDataStruct fsrValue;

//[GLOBAL] logfile
extern std::ofstream logFile;

//[THREAD] mutex for main thread
extern pthread_mutex_t mutMainTrd;
extern pthread_cond_t  cndMainTrd;

//[THREAD] decide which side of legs will be monitored
extern std::vector<int> activeDevice; //{sLeft, sRight}


//----------------------------------------------------------------------

/*! \txw data struct for static state detector */
//----------------------------------------------------------------------
typedef struct _ImuData {
    //calibrated acceleration including the gravity component [m/s2]
    double a[3];

    //calibrated ruler angle [deg]
    double r[3];

    //calibrated angular velocity [deg/s]
    double w[3];

}ImuData;

//----------------------------------------------------------------------
/*! \txw transform quaternion to ZXY Euler Angle (deg) */
//----------------------------------------------------------------------
XsEuler threeAxisRot(double r[5]) {
    XsEuler agl;
    agl[0] = atan2(r[0], r[1]);
    if(r[2] < -1)
        r[2] = -1;
    else if(r[2]> 1)
        r[2] = 1;
    agl[1] = asin(r[2]);
    agl[2] = atan2(r[3], r[4]);

    return agl;
}
XsEuler quat2angleZXY(XsQuaternion q) {
    double r[5];
    double q1 = q.w();
    double q2 = q.x();
    double q3 = q.y();
    double q4 = q.z();

    r[0] = -2 * (q2 * q3 - q1 * q4);
    r[2] =  2 * (q3 * q4 + q1 * q2);
    r[3] = -2 * (q2 * q4 - q1 * q3);
    r[1] = pow(q1, 2) - pow(q2, 2) + pow(q3, 2) - pow(q4, 2);
    r[4] = pow(q1, 2) - pow(q2, 2) - pow(q3, 2) + pow(q4, 2);

    XsEuler agl = threeAxisRot(r); //radian
    agl[0] *= 57.3; //convert to degree
    agl[1] *= 57.3;
    agl[2] *= 57.3;

    return agl;
}

//----------------------------------------------------------------------
/*! \txw bounded output */
//----------------------------------------------------------------------
double boxLimit(double val, double ceil, double floor) {

    double ret = val;

    if (val > ceil)
        ret = ceil;
    else if(val < floor)
        ret = floor;

    return ret;
}

//----------------------------------------------------------------------
/*! \txw check if a peak is found, Flg = IMU_MIN/ IMU_MAX */
//----------------------------------------------------------------------
bool FindPeak(bool Flg, std::vector<double> data) {
    bool result = false;

    if (data.size() < 4)
        return result;

    if (Flg == IMU_MAX) {
        if (*data.begin()       < *(data.end() - 2) &&
            *(data.begin() + 1) > *(data.end() - 1))
            result = true;
    } else if (Flg == IMU_MIN) {
        if (*data.begin()       > *(data.end() - 2) &&
            *(data.begin() + 1) < *(data.end() - 1))
            result = true;
    }

    return result;
}

//----------------------------------------------------------------------
/*! \txw calculate GLRT ratio value, DO NOT USE FREE-ACCELERATION, ACC[m/s2], gyro[deg/s] */
//----------------------------------------------------------------------
double GLRTratio(std::vector<ImuData> imu) {

    double Acc = 0;
    double Vel = 0;
    double AverAcc[3] = {0, 0, 0};

    //get the average value of 3-dimension acc
    for (auto iter : imu) {
        AverAcc[0] += iter.a[0];
        AverAcc[1] += iter.a[1];
        AverAcc[2] += iter.a[2];
    }

    AverAcc[0] = AverAcc[0] / imu.size();
    AverAcc[1] = AverAcc[1] / imu.size();
    AverAcc[2] = AverAcc[2] / imu.size();

    //get the norm(acc)
    double norm = sqrt(AverAcc[0] * AverAcc[0] + AverAcc[1] * AverAcc[1] + AverAcc[2] * AverAcc[2]);

    AverAcc[0] = AverAcc[0] / norm;
    AverAcc[1] = AverAcc[1] / norm;
    AverAcc[2] = AverAcc[2] / norm;

    //get GLRT ratio
    for (auto iter : imu) {
        Acc += pow( (iter.a[0] - GRAVITY * AverAcc[0]), 2) +
               pow( (iter.a[1] - GRAVITY * AverAcc[1]), 2) +
               pow( (iter.a[2] - GRAVITY * AverAcc[2]), 2);

        Vel += pow(iter.w[0], 2) +
               pow(iter.w[1], 2) +
               pow(iter.w[2], 2);
    }

    double GLRTratio = (Acc / VarACC + Vel / VarVEL) / imu.size();

    return GLRTratio;
}

//----------------------------------------------------------------------
/*! \txw static state detection */
//----------------------------------------------------------------------
bool ssDetector(double glrt, bool leftORright, double limit = 5e3, float timeval = 1.8) {
    struct timeb lastMovingTime;
    static struct timeb lastMovingTime_l, lastMovingTime_r;

    //left or right side
    if(leftORright == sLeft)
        lastMovingTime = lastMovingTime_l;
    else
        lastMovingTime = lastMovingTime_r;

    //if the glrt ratio value less than the moving threshold
    if(glrt < limit) {
        struct timeb var;
        ftime(&var);

        // calculate the interval between the last moving timing (the first static timing) and the current static timing
        double ntime = var.time - lastMovingTime.time + (var.millitm - lastMovingTime.millitm) * 1.0e-3;

        // the duration keeping the static state exceeds the threshold
        if(ntime > timeval)
            return true; //yes, it's static
    }
    else {
        // set the last moving timing to the current
        ftime(&lastMovingTime);

        if(leftORright == sLeft)
            lastMovingTime_l = lastMovingTime;
        else
            lastMovingTime_r = lastMovingTime;
    }

    return false; //no, it's moving
}

//----------------------------------------------------------------------
/*! \txw calculate static state density */
//----------------------------------------------------------------------
double ssDensity(std::vector<double> glrtPacket, double threshold) {
    int num = 0;

    //how many components are greater than the threshold
    for(auto glrt : glrtPacket) {
        if (glrt < threshold) {
            num++;
        }
    }

    double density = double(num) /glrtPacket.size();

    return density;
}

//----------------------------------------------------------------------
/*! \txw print gait event */
//----------------------------------------------------------------------
void printGaitEvent(int gaitEvent) {

    switch (gaitEvent) {
        case FC:
            std::cout << "[IMU]: Gait Event: Foot Contact" << std::endl;
            break;
        case SF:
            std::cout << "[IMU]: Gait Event: Static Foot" << std::endl;
            break;
        case DF:
            std::cout << "[IMU]: Gait Event: Dynamic Foot" << std::endl;
            break;
        case FO:
            std::cout << "[IMU]: Gait Event: Foot Off" << std::endl;
            break;
        default:
            std::cout << "[IMU]: Gait Event: Error" << std::endl;
            break;
    }
}

//----------------------------------------------------------------------
/*! \txw show full name of gait events */
//----------------------------------------------------------------------
std::string showGaitEventFullName(int gaitEvent) {
    std::string fullName;

    switch (gaitEvent) {
        case FC:
            fullName = "Foot Contact";
            break;
        case SF:
            fullName = "Static Foot";
            break;
        case DF:
            fullName = "Dynamic Foot";
            break;
        case FO:
            fullName = "Foot Off";
            break;
        default:
            fullName = "Error";
            break;
    }

    return fullName;
}


//----------------------------------------------------------------------
/*! \txw show full name of gait events */
//----------------------------------------------------------------------
std::string showGaitEventAbbrName(int gaitEvent) {
    std::string abbrName;

    switch (gaitEvent) {
        case FC:
            abbrName = "FC";
            break;
        case SF:
            abbrName = "SF";
            break;
        case DF:
            abbrName = "DF";
            break;
        case FO:
            abbrName = "FO";
            break;
        default:
            abbrName = "ER";
            break;
    }

    return abbrName;
}


//----------------------------------------------------------------------
/*! \txw left-side IMU thread function */
//----------------------------------------------------------------------
void *imuRead_l(void *arg) {

    std::vector<MtwCallback *> * p_arg = (std::vector<MtwCallback *> *) arg;

    //create IMU handler
    std::vector<MtwCallback *> mtwCallbacks;
    mtwCallbacks.push_back( (*p_arg)[0] );

    //get the base/initial angle of IMU
    while (!mtwCallbacks[0]->dataAvailable() ||
           !mtwCallbacks[0]->getOldestPacket()->containsOrientation()) {
        XsTime::msleep(1);
    }
    XsQuaternion qFT = mtwCallbacks[0]->getOldestPacket()->orientationQuaternion();
    double aglBaseFT = quat2angleZXY(qFT).z(); //PITCH: last element of ZXY euler



    //---------------------------------------------------------------------------
    //room to store data
    std::vector<ImuData> imuDataGlrt;   //static state-use
    std::vector<double> agl, aglVel;    //gait event-use
    std::vector<double> historyMinFootPitchVel{-100};   //used for calculation of threshold for FO detection
    std::vector<double> historyMaxFootPitch{aglBaseFT}; //used for calculation of threshold for FC detection
    std::vector<double> historyMinFootPitch{aglBaseFT}; //used for calculation of threshold for FC detection

    //default gait event is static foot
    eventOn[sLeft] = SF;

    int axis        = 1; //intended axis 3-D IMU data
    int windowPeak  = 10; //peak
    int windowGlrt  = 5; //glrt
    int cycle       = 0;

    unsigned int printCounter = 1;
    double minFootPitchVel    = 0; // store minimal foot pitch angular velocity for the current cycle
    double minFootPitch       = 200; // store minimal foot pitch angle
    double maxFootPitch       = -200; // store maximal foot pitch angle



    //---------------------------------------------------------------------------
    //foot static state detection
    int samplingFrequency   = 120; //Hz

    //Factor #1: confirm the threshold for glrt value
    int glrtThreshold_Base  = 1e5;
    double glrtThresholdCorr  = 0;
    double glrtThresholdFinal = 0;

    //Factor #2: confirm the size of glrt window for stationary detection
    int glrtWindowSize_base     = int(0.01 * samplingFrequency); // 0.1s
    int glrtWindowSize_limit    = int(0.25 * samplingFrequency); // 0.3s
    int glrtWindowSizeCorr    = 0;
    int glrtWindowSizeFinal   = 0;

    //Model #1: relation between the glrt value range and the threshold
    double modelPara_glrtRange   = 0.11; //determine the correction value "glrtThresholdCorr"

    //Model #2: relation between the glrt value range and the window size
    double modelPara_glrtWindowSize  = 30; //determine the correction value "glrtWindowSizeCorr"

    //threshold to confirm if the static state density satisfies the requirement
    double glrtDensityThreshold = 0.7;

    //single calculation of glrt value
    double singleGlrtValue = 0;

    //foot(IMU) state
    bool movingState    = false;

    //to help calculate the glrt value range
    std::vector<double> glrtPacketforRange;

    //to help calculate the static state density
    std::vector<double> glrtPacketforDensity;

    //---------------------------------------------------------------------------
    bool neglectFC  = false;
    bool neglectFO  = false;
    bool ssDetected = false;
    bool clearedSearch  = false;

    struct timeb initialTime, presentTime;
    ftime(&initialTime);

    int gaitEventFSR   = FC; //it could only be FC or FO. FC means that FC has been detected, not in the FC event.
    double fsrThreshold = 15; //Newton (raw force data are also collected)

    //---------------------------------------------------------------------------
    //main loop
//    std::cout << "start left side imu loop code" << std::endl;
    while (FLG_THD_IMU_L) {

        ImuData imuDataFT;

        //wait until the imu has data
        while( !mtwCallbacks[0]->dataAvailable() )
            XsTime::msleep(1);

        ftime(&presentTime);

        //get a complete packet of foot imu data
        {
            XsDataPacket const *pkt = mtwCallbacks[0]->getOldestPacket();

            //foot awinda is placed with the LED on the left-front-outside side
            //euler [deg]
            XsQuaternion q = pkt->orientationQuaternion();
            XsEuler tAgl = quat2angleZXY(q);
            memcpy(imuDataFT.r, &tAgl, sizeof(XsEuler));
            imuDataFT.r[1] = imuDataFT.r[2]; //default imuDataFT.r[1] used for gait recognition

            //angular velocity [deg/s]
            XsVector tAglVel = pkt->calibratedGyroscopeData().operator*(57.3);
            memcpy(imuDataFT.w, tAglVel.data(), tAglVel.size()*sizeof(XsReal));

            //acceleration [m/s2]
            XsVector tAcc = pkt->calibratedAcceleration(); //do not use free-acc in ZUPT
            memcpy(imuDataFT.a, tAcc.data(), tAcc.size()*sizeof(XsReal));

            mtwCallbacks[0]->deleteOldestPacket();
        }


        //update foot data window
        {
            //update angle and angular velocity data window [peak-use]
            if (agl.size() < windowPeak) {
                agl.push_back(imuDataFT.r[axis]);       //ankle DF(+)/PF(-)
                aglVel.push_back(imuDataFT.w[axis]);    //ankle DF(+)/PF(-)
            } else {
                agl.erase(agl.begin(), agl.end() - windowPeak + 1);
                agl.push_back(imuDataFT.r[axis]);

                aglVel.erase(aglVel.begin(), aglVel.end() - windowPeak + 1);
                aglVel.push_back(imuDataFT.w[axis]);
            }

            //update imu packet data window [glrt-use]
            if (imuDataGlrt.size() < windowGlrt) {
                imuDataGlrt.push_back(imuDataFT);
            } else {
                imuDataGlrt.erase(imuDataGlrt.begin(), imuDataGlrt.end() - windowGlrt + 1);
                imuDataGlrt.push_back(imuDataFT);
            }

            //calculate glrt value
            singleGlrtValue = GLRTratio(imuDataGlrt);
        }


        //foot static state detection
        {
            //record glrt values since the last static state (for Factor #1 and #2)
            if (movingState) {
                glrtPacketforRange.push_back(singleGlrtValue);
            }

            //calculate corrections for Factor #1 and #2
            if (!glrtPacketforRange.empty()) {

                double max = *(std::max_element(glrtPacketforRange.begin(), glrtPacketforRange.end()));
                double min = *(std::min_element(glrtPacketforRange.begin(), glrtPacketforRange.end()));

                //calculate correction for Factor #1 (threshold)
                glrtThresholdCorr = (max - min) * modelPara_glrtRange;

                //calculate correction for Factor #2 (window size)
                if (max - min != 0) {
                    double N_range = 1e7; // baseline difference between the max and the min
                    glrtWindowSizeCorr =  int(N_range / (max-min) * modelPara_glrtWindowSize);
                }
            }

            //get the final threshold (Factor #1)
            glrtThresholdFinal  = glrtThreshold_Base + glrtThresholdCorr;

            //get the final window size (Factor #2)
            glrtWindowSizeFinal = fmin(glrtWindowSize_base + glrtWindowSizeCorr, glrtWindowSize_limit);

            //record glrt values (for density)
            if (glrtPacketforDensity.size() < glrtWindowSize_limit) {
                glrtPacketforDensity.push_back(singleGlrtValue);
            } else {
                glrtPacketforDensity.erase(glrtPacketforDensity.begin(),
                                           glrtPacketforDensity.end() - glrtWindowSize_limit + 1);

                glrtPacketforDensity.push_back(singleGlrtValue);
            }

            //detect the foot state
            if (glrtPacketforDensity.size() >= glrtWindowSizeFinal) {

                //get the glrt window with the final window size
                std::vector<double> subVec(glrtPacketforDensity.end() - glrtWindowSizeFinal,
                        glrtPacketforDensity.end());

                //confirm the foot(IMU) state
                if (ssDensity(subVec, glrtThresholdFinal) >= glrtDensityThreshold) {
                    movingState = false;

                    std::vector<double>().swap(glrtPacketforRange);
                } else {
                    movingState = true;
                }
            }

            ssDetected = ssDetector(singleGlrtValue, sLeft, glrtThresholdFinal, 1.8);
        }


        //detect FC/FO using FSR
        // raw force data are collected. The FSR-based detection results can be evaluated later.
        {
            double sum = fsrValue.forefootOuter + fsrValue.forefootInner + fsrValue.heel; //[Unit] Newton
            if (sum > fsrThreshold) {
                gaitEventFSR = FC;
            } else {
                gaitEventFSR = FO;
            }
        }


		//only search max/min during stance phase
		if (eventOn[sLeft] != FO) {
			//find the minimal foot pitch angular velocity for the current cycle
			if (imuDataFT.w[axis] < minFootPitchVel) {
				minFootPitchVel = imuDataFT.w[axis];
			}

			//find the minimal foot pitch angle for the current cycle
			if (imuDataFT.r[axis] < minFootPitch) {
				minFootPitch = imuDataFT.r[axis];
			}

			//find the maximal foot pitch angle for the current cycle
			if (imuDataFT.r[axis] > maxFootPitch) {
				maxFootPitch = imuDataFT.r[axis];
			}
		}

        neglectFC = 0;
        neglectFO = 0;

        //---------------------------------------------------------------------------
        //finite state machine
        switch (eventOn[sLeft]) {
            case FO: // Foot Off

                //to detect FC event
                if (FindPeak(IMU_MAX, agl)) {

                    if (!neglectFC) {
                        //average the `windowPeak` cycles of values
                        double threshold_upper =
                                std::accumulate(historyMaxFootPitch.begin(), historyMaxFootPitch.end(), 0.0)
                                / historyMaxFootPitch.size();

                        double threshold_lower =
                                std::accumulate(historyMinFootPitch.begin(), historyMinFootPitch.end(), 0.0)
                                / historyMinFootPitch.size();

                        double threshold = threshold_lower + 0.5 * (threshold_upper - threshold_lower);

                        gThld_FC[sLeft] = threshold;
                    } else {
                        gThld_FC[sLeft] = minFootPitch + 0.5 * (maxFootPitch - minFootPitch);
                    }


                    if (std::accumulate(agl.begin(), agl.end(), 0.0) / agl.size() > gThld_FC[sLeft]) {
                        eventOn[sLeft] = FC;
                        neglectFC = false;

                        cycle++;

                        //iterate minimal foot pitch angular velocity
                        minFootPitchVel = boxLimit(minFootPitchVel, -100, -500);
                        if (historyMinFootPitchVel.size() < windowPeak) {
                            historyMinFootPitchVel.push_back(minFootPitchVel);
                        } else {
                            historyMinFootPitchVel.erase( historyMinFootPitchVel.begin() );
                            historyMinFootPitchVel.push_back(minFootPitchVel);
                        }
                        minFootPitchVel = 0; //default value

                        //iterate maximal foot pitch angle
                        if (historyMaxFootPitch.size() < windowPeak) {
                            historyMaxFootPitch.push_back(maxFootPitch);
                        } else {
                            historyMaxFootPitch.erase( historyMaxFootPitch.begin() );
                            historyMaxFootPitch.push_back(maxFootPitch);
                        }
                        maxFootPitch = -200; //default value

                        //iterate minimal foot pitch angle
                        if (historyMinFootPitch.size() < windowPeak) {
                            historyMinFootPitch.push_back(minFootPitch);
                        } else {
                            historyMinFootPitch.erase( historyMinFootPitch.begin() );
                            historyMinFootPitch.push_back(minFootPitch);
                        }
                        minFootPitch =  200; //default value
                    }
                }
                else if (ssDetected) {
                    eventOn[sLeft]  = SF;
                    neglectFC       = true;
                }
                break;

            case FC: // Foot Contact
                // to detect SF event
                if (!movingState) {
                    eventOn[sLeft] = SF;
                }
                break;

            case SF: // static foot
                //re-search the peak values during stance phase of the current cycle
                if (neglectFO && !clearedSearch) {
                    minFootPitchVel = 0;
                    maxFootPitch    = -200;
                    minFootPitch    =  200;
                    clearedSearch   = true;
                }

                //to detect DF event
                if (movingState) {
                    eventOn[sLeft] = DF;
                }
                break;

            case DF: // dynamic foot
                //to detect FO event
                if (FindPeak(IMU_MIN, aglVel)) {

                    if (!neglectFO) {
                        //average the `windowPeak` cycles of minimal foot pitch angular velocity
                        double threshold =
                                std::accumulate(historyMinFootPitchVel.begin(), historyMinFootPitchVel.end(), 0.0)
                                / historyMinFootPitchVel.size() / 3.0;

                        gThld_FO[sLeft] = threshold;
                    } else {
                        gThld_FO[sLeft] = minFootPitchVel / 3.0;
                    }


                    if (std::accumulate(aglVel.begin(), aglVel.end(), 0.0) / aglVel.size() < gThld_FO[sLeft]) {

                        if (!neglectFO) {
                            double threshold_upper =
                                    std::accumulate(historyMaxFootPitch.begin(), historyMaxFootPitch.end(), 0.0)
                                    / historyMaxFootPitch.size();

                            double threshold_lower =
                                    std::accumulate(historyMinFootPitch.begin(), historyMinFootPitch.end(), 0.0)
                                    / historyMinFootPitch.size();

                            double threshold_agl = threshold_lower + 0.5 * (threshold_upper - threshold_lower);

                            gThld_FO_AGL[sLeft] = threshold_agl;
                        } else {
                            gThld_FO_AGL[sLeft] = minFootPitch + 0.5 * (maxFootPitch - minFootPitch);
                        }
                        
                        //in case of entering Toe-Off early (due to too large glrt threshold)
                        if (agl.back() < gThld_FO_AGL[sLeft]) {
                            eventOn[sLeft] = FO;
                            neglectFO = false;
                        }
                    }
                }
                else if (ssDetected) {
                    eventOn[sLeft]  = SF;
                    clearedSearch   = false;
                    neglectFO       = true;
                }
                break;
        }

        double nTime = presentTime.time - initialTime.time +
                (presentTime.millitm - initialTime.millitm) * 1.0e-3;


        logFile << std::setiosflags(std::ios::fixed) << std::setprecision(3)
                << nTime << ","
                << cycle << ","
//                << showGaitEventFullName(eventOn[sLeft]) << ","
//                << showGaitEventFullName(gaitEventFSR) << ","
                << eventOn[sLeft] << ","
                << gaitEventFSR << ","
                << fsrValue.forefootOuter << ","
                << fsrValue.forefootInner << ","
                << fsrValue.heel << ","
                << agl.back() << ","
                << aglVel.back() << ","
                << singleGlrtValue << ","
//                << glrtThresholdCorr << ","
                << glrtThresholdFinal << ","
//                << glrtWindowSizeCorr << ","
                << glrtWindowSizeFinal << ","
                << movingState << ","
                << neglectFC << ","
                << neglectFO << ","
                << minFootPitch << ","
                << maxFootPitch << ","
                << minFootPitchVel << ","
                << gThld_FC[sLeft] << ","
                << gThld_FO[sLeft] << ","
                << gThld_FO_AGL[sLeft] << ","
//                << historyMinFootPitchVel.back() << ","
//                << historyMaxFootPitch.back() << ","
//                << historyMinFootPitch.back() << ","
                << std::endl;


		// Don't print too often for performance. Console output should be very slow.
        if (printCounter % 25 == 0) { //25
            std::cout << setiosflags(std::ios::fixed) << std::setprecision(3)
                      << nTime << "\t"
                      << imuDataFT.r[axis] - aglBaseFT << "\t"
                      << showGaitEventFullName(eventOn[sLeft]) << "\t"
                      << fsrValue.forefootOuter << "\t"
                      << fsrValue.forefootInner << "\t"
                      << fsrValue.heel << "\t"
//                      << mtwCallbacks[0]->availablePacketCount() << "\t"
                      << std::endl;
        }
        ++printCounter;

    }

    pthread_exit(NULL);
}

//----------------------------------------------------------------------
/*! \txw right-side IMU thread function */
//----------------------------------------------------------------------
void *imuRead_r(void *arg) {

    std::vector<MtwCallback *> * p_arg = (std::vector<MtwCallback *> *) arg; //&mtwCallbacks

    //left side first if existed. each side contains two IMUs
    int dvcNum = activeDevice.size();
    std::vector<MtwCallback *> mtwCallbacks;

    //create IMU handler
    mtwCallbacks.push_back( (*p_arg)[(dvcNum-1)*2] );   //foot


    struct timeb initialTime, presentTime;
    ftime(&initialTime);

    unsigned int printCounter = 1;

    while (FLG_THD_IMU_L) {

        //wait until the imu has data
        while (!mtwCallbacks[0]->dataAvailable()) {
            XsTime::msleep(1);
        }

        mtwCallbacks[0]->deleteOldestPacket();

        ftime(&presentTime);

        double nTime = presentTime.time - initialTime.time +
                       (presentTime.millitm - initialTime.millitm) * 1.0e-3;

        // Don't print too often for performance. Console output should be very slow.
        if (printCounter % 1 == 0) { //25
            std::cout << setiosflags(std::ios::fixed) << std::setprecision(3)
                      << nTime << "\t"
                      << mtwCallbacks[0]->availablePacketCount() << "\t"
                      << std::endl;
        }
        ++printCounter;

    }
    pthread_exit(NULL);
}

//----------------------------------------------------------------------
/*! \txw main thread function for IMUs */
//----------------------------------------------------------------------
void *imuThread (void *pVoid) {

    const int desiredUpdateRate   = 120;    // update rate for MTWs
    const int desiredRadioChannel = 25;    // radio channel for wireless master

    std::vector<std::vector<XsDeviceId>> ImuId{{11816019}, //left foot
                                               {11813997}};

    WirelessMasterCallback wirelessMasterCallback; // Callback for wireless master
    std::vector<MtwCallback *> mtwCallbacks; // Callbacks for mtw devices


    //--------------- 1 scan & open a port and request a master device instance (CONNECTING) --------------
    // 1.1 Construct XsControl (the main control object)
    XsControl *control = XsControl::construct();
    XsDevicePtr wirelessMasterDevice = NULL;

    try {
        // 1.2 Scan ports (scanPorts is a static class, so no instance of it is used to call its member function)
        XsPortInfoArray detectedDevices = XsScanner::scanPorts(XBR_Invalid, 100, false, false); //station
//        XsPortInfoArray detectedDevices = XsScanner::scanPorts(XBR_Invalid, 100, true, false); //dongle

        // 1.3 Find a wireless master (get the master information)
        XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();
        while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster()) {
            ++wirelessMasterPort;
        }
        if (wirelessMasterPort == detectedDevices.end()) {
            throw std::runtime_error("No wireless masters found");
        }

        // 1.4 Open port
        if (!control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate())) {
            std::ostringstream error;
            error << "Failed to open port " << *wirelessMasterPort;
            throw std::runtime_error(error.str());
        }

        // 1.5 Get XsDevice instance for wireless master (so as to manage child devices (MTw imu))
        wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
        if (wirelessMasterDevice == 0) {
            std::ostringstream error;
            error << "Failed to construct XsDevice instance: " << *wirelessMasterPort;
            throw std::runtime_error(error.str());
        }


        //--------------- 2 configure the master device (CONNECTED) --------------
        // 2.1 Set config mode (so as to configure the master device's update rate and radio channel)
        if (!wirelessMasterDevice->gotoConfig()) {
            std::ostringstream error;
            error << "Failed to goto config mode: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }

        // 2.2 Attach callback handler for the master device
        wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);

        // 2.3 Set update rate
        const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();
        const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);
        if (!wirelessMasterDevice->setUpdateRate(newUpdateRate)) {
            std::ostringstream error;
            error << "Failed to set update rate: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }

        // 2.4 Disable radio channel if previously enabled
        if (wirelessMasterDevice->isRadioEnabled()) {
            if (!wirelessMasterDevice->disableRadio()) {
                std::ostringstream error;
                error << "Failed to disable radio channel: " << *wirelessMasterDevice;
                throw std::runtime_error(error.str());
            }
        }

        // 2.5 Set radio channel (the LED on the USB dongle starts to flicker slowly)
        if (!wirelessMasterDevice->enableRadio(desiredRadioChannel)) {
            std::ostringstream error;
            error << "Failed to set radio channel: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }


        //--------------- 3 wait child devices (imus) to join in the network (ENABLED) --------------
        // 3.1 Waiting for MTW to wirelessly connect
        size_t DesImuCount = 0; // Count of imus desired to be connected
        for(auto item : activeDevice)
            DesImuCount += ImuId[item].size();

        bool waitForConnections = true;
        do {
            if (!FLG_THD_IMU) throw std::runtime_error("Interrupt Signal Detected");

            // are the intended imus connected to the master?
            waitForConnections = false;
            for (auto item : activeDevice) {
                for (auto id : ImuId[item]) {
                    XsDevice *device = wirelessMasterDevice->findDevice(id);
                    if (!device) {
                        waitForConnections = true;
                        std::cout << "[IMU]: wait to connect the IMU (id: " << id << ")."
                                  << std::endl;
                    } else {
                        // accept the intended devices if previously reject
                        if (device->connectivityState() != XCS_Wireless) {
                            waitForConnections = true;
                            wirelessMasterDevice->setDeviceAccepted(device->deviceId());
                        }
                    }
                }
            }

            XsTime::msleep(100);
        } while (waitForConnections);

        // 3.2 reject undesired devices from the master
        while (wirelessMasterCallback.getWirelessMTWs().size() != DesImuCount) {
            if (!FLG_THD_IMU) throw std::runtime_error("Interrupt Signal Detected");

            std::vector<XsDeviceId> undesiredDeivceId;

            // confirm connected but undesired devices
            std::set<XsDevice *>::const_iterator it;
            for (it = wirelessMasterCallback.getWirelessMTWs().begin();
                 it != wirelessMasterCallback.getWirelessMTWs().end(); it++) {
                int isUndesired = 0;
                for (auto item : activeDevice) {
                    for (auto id : ImuId[item]) {
                        if ((*it)->deviceId() != id) {
                            isUndesired++;
                        }
                    }
                }

                if (isUndesired == DesImuCount) {
                    undesiredDeivceId.push_back((*it)->deviceId());
                }
            }

            // reject connected but undesired devices
            for (auto id : undesiredDeivceId) {
                wirelessMasterDevice->setDeviceRejected(id);
            }

            XsTime::msleep(100);
        }


        //--------------- 4 measure data from connected child devices (MEASURING) --------------
        // 4.1 Start measurement (the LED on the USB dongle starts to flicker quickly)
        if (!wirelessMasterDevice->gotoMeasurement()) {
            std::ostringstream error;
            error << "Failed to goto measurement mode: " << *wirelessMasterDevice;
            throw std::runtime_error(error.str());
        }

        // recheck if the update rate has been successfully set
        if (wirelessMasterDevice->updateRate() != newUpdateRate) {
            throw std::runtime_error("Failed to set the update rate");
        }

        // 4.2 Get XsDevice instances for all MTws
        XsDevicePtrArray mtwDevices;
        for (auto item : activeDevice) {
            for (auto id : ImuId[item]) {
                XsDevicePtr mtwDevice = control->device(id);
                if (mtwDevice != 0) {
                    mtwDevices.push_back(mtwDevice);
                } else {
                    throw std::runtime_error("Failed to create an MTW XsDevice instance");
                }
            }
        }

        // 4.3 Attaching callback handlers to MTWs
        mtwCallbacks.resize(mtwDevices.size());
        for (int i = 0; i < (int) mtwDevices.size(); ++i) {
            mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
            mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);
        }


        //--------------- 5 start your own code --------------
        //construct different thread for each imu data reading
        pthread_t imu[DOFS];
        pthread_attr_t attr_imu[DOFS];

        for(auto i : activeDevice) {
            if (pthread_attr_init(&attr_imu[i]) != EOK) {
                throw std::runtime_error("[ERROR] IMU-THREAD INIT ERROR");
            }

            switch(i)
            {
                case sLeft:
                    if (pthread_create(&imu[i], &attr_imu[i], imuRead_l, (void *) &mtwCallbacks) != EOK)
                        throw std::runtime_error("[ERROR] IMU-LEFT-THREAD CREATE ERROR");
                    break;
                case sRight:
                    if (pthread_create(&imu[i], &attr_imu[i], imuRead_r, (void *) &mtwCallbacks) != EOK)
                        throw std::runtime_error("[ERROR] IMU-RIGHT-THREAD CREATE ERROR");
                    break;
                default:
                    throw std::runtime_error("[ERROR] IMU ERROR IN ACTIVE DEVICE VALUE");
            }
        }

        //signal to unlock the main thread
        pthread_cond_signal(&cndMainTrd);
        pthread_mutex_unlock(&mutMainTrd);

        //wait the imu thread finished
        for(auto i : activeDevice)
        {
            pthread_join(imu[i], NULL);
            pthread_attr_destroy(&attr_imu[i]);
        }


    }
    catch (std::exception const &ex) {
        std::cout << ex.what() << std::endl;
        std::cout << "****ABORT****" << std::endl;
    }
    catch (...) {
        std::cout << "An unknown fatal error has occurred. Aborting." << std::endl;
        std::cout << "****ABORT****" << std::endl;
    }


    //--------------- 6 close (measuring->enabled->connected->close) --------------
    // 6.1 disabling radio
    if (NULL != wirelessMasterDevice) {
        switch(wirelessMasterDevice->deviceState()) {
            case XDS_Config:
                // Disabling radio
                if (!wirelessMasterDevice->disableRadio()) {
                    std::cout << "[IMU]: Failed to disable radio: " << *wirelessMasterDevice << std::endl;
                }
                break;
            case XDS_Measurement:
                // Setting config mode
                if (!wirelessMasterDevice->gotoConfig()) {
                    std::cout << "[IMU]: Failed to goto config mode: " << *wirelessMasterDevice << std::endl;
                }
                // Disabling radio
                if (!wirelessMasterDevice->disableRadio()) {
                    std::cout << "[IMU]: Failed to disable radio: " << *wirelessMasterDevice << std::endl;
                }
                break;
            default:
                std::cout << "[IMU]: wirelessMasterDevice state: " << wirelessMasterDevice->deviceState() << std::endl;
                break;
        }
    }

    // 6.2 Closing XsControl
    control->close();

    // 6.3 Deleting mtw callbacks
    for (auto item : mtwCallbacks)
        delete (item);

    // 6.4 Releasing the XsControl object
    control->destruct();

    // signal to unlock the main thread (in case throw happens)
    pthread_cond_signal(&cndMainTrd);
    pthread_mutex_unlock(&mutMainTrd);

    std::cout << "[IMU]: Thread has exited!" << std::endl;
    pthread_exit(NULL);
}

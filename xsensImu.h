//
// Created by sia on 21-1-28.
//

#ifndef XSENSAWINDATEST_XSENSIMU_H
#define XSENSAWINDATEST_XSENSIMU_H

#define FC          1   //foot contact
#define SF          2   //static foot
#define DF          3   //dynamic foot
#define FO          4   //foot off

void *imuThread (void *pVoid);

void printGaitEvent(int gaitPhase);

std::string showGaitEventFullName(int gaitPhase);

#endif //XSENSAWINDATEST_XSENSIMU_H

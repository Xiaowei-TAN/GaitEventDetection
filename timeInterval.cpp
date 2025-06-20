//
// Created by sia on 20-10-3.
//

#include "timeInterval.h"
#include <sys/timeb.h>
#include <iostream>

std::stack<time_t> tictoc_stack;

void tic() {
    struct timeb s;
    ftime(&s);
    if (tictoc_stack.size())
        tictoc_stack.pop();
    tictoc_stack.push(s.time * 1e3+ s.millitm);
}

void toc() {
    struct timeb s;
    ftime(&s);
    std::cout << "Time elapsed (s): "
              << (float)(s.time * 1e3 + s.millitm - tictoc_stack.top()) / 1.0e3
              << std::endl;
}

float toc_rtnFloat() {
    struct timeb s;
    ftime(&s);
    float tval = (float)(s.time * 1e3 + s.millitm - tictoc_stack.top()) / 1.0e3;

    return tval;
}

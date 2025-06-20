//
// Created by sia on 20-10-3.
//

#ifndef SOFTEXOS_TIMEINTERVAL_H
#define SOFTEXOS_TIMEINTERVAL_H

#include <stack>
#include <sys/timeb.h>

extern std::stack<time_t> tictoc_stack;

void tic();
void toc();
float toc_rtnFloat();

#endif //SOFTEXOS_TIMEINTERVAL_H

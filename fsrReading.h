//
// Created by sia on 24-12-17.
//

#ifndef GAITEVENTDETECT_FSRREADING_H
#define GAITEVENTDETECT_FSRREADING_H

#include <stdint.h>
#include <cstring>
#include <vector>

void *fsrThread (void *pVoid);

typedef struct _FsrDataStruct {
    _FsrDataStruct() {
        memset(this, 0, sizeof(struct _FsrDataStruct));
    }

    //[Unit]: gram
    double forefootOuter;
    double forefootInner;
    double heel;

} _fsrDataStruct;


#endif //GAITEVENTDETECT_FSRREADING_H

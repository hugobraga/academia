/* 
 * File:   delay.h
 * Author: hugo
 *
 * Created on 15 de Fevereiro de 2009, 11:38
 */

#ifndef _DELAY_H
#define	_DELAY_H

#include "communicationentity.h"

class Delay {
public:
    long long delay(CommunicationEntity &element) {
        static unsigned long long delay_value = 4000000LL;
        delay_value += 100LL;
        return delay_value;
    }
};

#endif	/* _DELAY_H */


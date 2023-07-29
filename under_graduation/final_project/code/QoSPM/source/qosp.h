/* 
 * File:   qosp.h
 * Author: hugo
 *
 * Created on 27 de Janeiro de 2009, 00:46
 */

#ifndef _QOSP_H
#define	_QOSP_H

#include "qospentity.h"
#include <netinet/in.h> //struct sockaddr_in

class QoSP : public QoSPEntity {
public:
    QoSP(struct sockaddr_in addr);
};

#endif	/* _QOSP_H */


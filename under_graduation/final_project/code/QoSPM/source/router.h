/* 
 * File:   router.h
 * Author: hugo
 *
 * Created on 27 de Janeiro de 2009, 00:57
 */

#ifndef _ROUTER_H
#define	_ROUTER_H

#include "qospentity.h"
#include "qospa.h"
#include <netinet/in.h> //struct sockaddr_in

class QoSPA;

class Router : public QoSPEntity {
    QoSPA *monitor;
public:
    Router(struct sockaddr_in addr, QoSPA *monitor);
    QoSPA* getMonitor();
};

#endif	/* _ROUTER_H */


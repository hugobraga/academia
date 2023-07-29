/* 
 * File:   qospa.h
 * Author: hugo
 *
 * Created on 27 de Janeiro de 2009, 00:50
 */

#ifndef _QOSPA_H
#define	_QOSPA_H

class Router;

#include "qospentity.h"
#include <netinet/in.h> //struct sockaddr_in
#include "router.h"

class QoSPA : public QoSPEntity {
    Router *router;
public:
    QoSPA(struct sockaddr_in addr);
    void setRouter(Router *router);
    Router* getRouter();
};

#endif	/* _QOSPA_H */


/* 
 * File:   verifier.h
 * Author: hugo
 *
 * Created on 2 de Março de 2009, 23:43
 */

#ifndef _VERIFIER_H
#define	_VERIFIER_H

#include "monitoringapplication.h"
#include "traffic_listener.h"
#include "flow.h"
#include <netinet/in.h> //struct sockaddr_in

class Verifier {
    MonitoringApplication   *qosConsultant;
    TrafficListener         *listener;
    Flow                    *flowBehv;
private:
    int sendRemoteVerMsg(struct sockaddr_in pxAddr, struct sockaddr_in pyAddr);
public:
    Verifier(MonitoringApplication *consultant, TrafficListener *listener, int cvDesc);
    int qos(struct sockaddr_in pxAddr, struct sockaddr_in pyAddr);
};

#endif	/* _VERIFIER_H */


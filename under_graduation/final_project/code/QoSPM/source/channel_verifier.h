/* 
 * File:   channel_verifier.h
 * Author: hugo
 *
 * Created on 25 de Fevereiro de 2009, 11:18
 */

#ifndef _CHANNEL_VERIFIER_H
#define	_CHANNEL_VERIFIER_H

class MonitoringApplicatoin;

#include <vector>
#include "monitoringapplication.h"
#include "traffic_listener.h"
#include "communicationentity.h"
#include "mt_support.h"


class ChannelVerifier {
    MonitoringApplication   *ma;
    TrafficListener         *listener;
    vector<bool>            *verifiersDesc;
    qosp_lock_t             verifiersLock;
private:
    int getVerifierDesc();
    void releaseVerDesc(int desc);
public:
    ChannelVerifier(MonitoringApplication *ma, TrafficListener *listener);
    int qos(struct sockaddr_in pxAddr, struct sockaddr_in pyAddr);
    int registerChannel(CommunicationEntity *px, CommunicationEntity *py);
    void unRegister(int channelDesc);
    virtual ~ChannelVerifier();
};

#endif	/* _CHANNEL_VERIFIER_H */


/* 
 * File:   mediator.h
 * Author: hugo
 *
 * Created on 25 de Fevereiro de 2009, 10:22
 */

#ifndef _MEDIATOR_H
#define	_MEDIATOR_H

#include <list>
#include "local_host.h"
#include "monitoringapplication.h"
#include "channel_verifier.h"
#include "channel.h"

class ChannelVerifier; //resolver problema de referencia cruzada

class Mediator {
    LocalHost               *localHost;
    MonitoringApplication   *ma;
    ChannelVerifier         *cv;
public:
    Mediator(MonitoringApplication *ma, ChannelVerifier *cv, LocalHost *localHost);
    void notifyChannelsDegrad(list<Channel *> &channels);
    int notifyTimelyChannel(Channel *channel);
};

#endif	/* _MEDIATOR_H */


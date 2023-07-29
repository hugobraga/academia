/* 
 * File:   channel.h
 * Author: hugo
 *
 * Created on 27 de Janeiro de 2009, 01:08
 */

#ifndef _CHANNEL_H
#define	_CHANNEL_H

#include "communicationentity.h"
#include "router.h"
#include <list>

class Channel {
    int                 listenerDesc;
    CommunicationEntity *px, *py;
    list<Router *>      routers;
    int                 qos;
public:
    Channel(CommunicationEntity* px, CommunicationEntity* py, int qos);
    bool insertRouter(Router* &router);
    CommunicationEntity* getPx();
    CommunicationEntity* getPy();
    int getRouters(list<Router *> &routers);
    int getQoS();
    void setQoS(int qos);
    bool equal(Channel &channel);
    int getListenerDesc();
    void setListenerDesc(int desc);
};

#endif	/* _CHANNEL_H */


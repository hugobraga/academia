/* 
 * File:   channelmanager.h
 * Author: hugo
 *
 * Created on 27 de Janeiro de 2009, 15:17
 */

#ifndef _CHANNELMANAGER_H
#define	_CHANNELMANAGER_H

#include "communicationentity.h"
#include "channel.h"
#include "router.h"
#include "qospa.h"
#include <vector>
#include "domain.h"
#include "mt_support.h"

using namespace std;

class ChannelManager {
    vector<CommunicationEntity *>   channelProcess;
    vector<Channel *>               channels;
    vector<Router *>                routers;
    vector<QoSPA *>                  nodeMonitors;
    qosp_lock_t                     lockDesc;
public:
    ChannelManager();
    void notifyQoSDowngrade(Channel &channel);
    int notifyQoSUpgrade(Channel &channel);
    int getChannel(CommunicationEntity &px, CommunicationEntity &py, Channel* &channel);
    int getTChannelsPerRouter(Router &router, list<Channel *> &timely_channels);
    int getRouters(vector<Router *>::iterator &routers, vector<Router *>::iterator &end);
    int getChannels(vector<Channel *>::iterator &channels, vector<Channel *>::iterator &end);
    Router* getRouterByAddr(struct sockaddr_in  sock_addr);
    int notifyNewRouter(struct sockaddr_in  router_addr, struct sockaddr_in  qospA_addr);
    int notifyNewChannel(struct sockaddr_in  px_addr, struct sockaddr_in  py_addr);
    Router* getRouterByDesc(int desc);
    Channel* getChannelByDesc(int desc);
    virtual ~ChannelManager();
};

#endif	/* _CHANNELMANAGER_H */


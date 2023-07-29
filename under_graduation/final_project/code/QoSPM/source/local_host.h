/* 
 * File:   local_host.h
 * Author: hugo
 *
 * Created on 22 de Fevereiro de 2009, 17:30
 */

#ifndef _LOCAL_HOST_H
#define	_LOCAL_HOST_H

#include "communicationentity.h"
#include "channel.h"

class LocalHost {
    CommunicationEntity *localHost;
private:
    bool isProcessInLocalHost(CommunicationEntity &otherProcess);
public:
    LocalHost(struct sockaddr_in addr);
    int whichLocalProcess(Channel &channel, CommunicationEntity* &proc);
    int whichRemoteProcess(Channel &channel, CommunicationEntity* &proc);
    ~LocalHost();
};

#endif	/* _LOCAL_HOST_H */


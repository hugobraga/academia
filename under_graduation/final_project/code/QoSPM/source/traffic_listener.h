/* 
 * File:   traffic_listener.h
 * Author: hugo
 *
 * Created on 25 de Fevereiro de 2009, 13:51
 */

#ifndef _TRAFFIC_LISTENER_H
#define	_TRAFFIC_LISTENER_H

#include <vector>
#include "communicationentity.h"
#include "mt_support.h"
#include <native/types.h> //RTIME

using namespace std;

typedef struct qosp_listener_str {
    CommunicationEntity *px, *py;
    RTIME               lmTime; //última mensagem trocada pelos processos
}qosp_listener_str;

class TrafficListener {
    vector<qosp_listener_str>   *channels;
    qosp_lock_t                 descLock;
public:
    TrafficListener();
    int registerChannel(CommunicationEntity *px, CommunicationEntity *py);
    int unRegister(int channelDesc);
    RTIME lastMessageTime(int channelDesc);
    void notifyMsgAppear(int channelDesc);
    virtual ~TrafficListener();
};

#endif	/* _TRAFFIC_LISTENER_H */


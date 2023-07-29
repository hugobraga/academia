/* 
 * File:   network.h
 * Author: hugo
 *
 * Created on 4 de Mar�o de 2009, 23:11
 */

#ifndef _NETWORK_H
#define	_NETWORK_H

#include "mt_support.h"
#include <netinet/in.h> //struct sockaddr_in

typedef struct {
    int sock_desc;
    int sock_domain;
}qosp_sock_t;

class Network {
public:
    int initializeSocket(qosp_sock_t &sock, int context, int type, int sockPort);
    int sendMsg(qosp_sock_t &sock, struct sockaddr_in destAddr, char *opMsg, int msgSize);
    int ping(qosp_sock_t *pingSock, struct sockaddr_in destAddr, u_int16_t seqId, u_int16_t msgId);
    int receiveMsg(qosp_sock_t &ssock, char *msgAddr, char* &msgData, int maxMsgLength, struct sockaddr_in *hostAddr, int sockType);
    int receivePingReply(qosp_sock_t *sock, char *buf, struct icmp* &icp, int maxMsgLength);
    void copySockAddr(struct sockaddr_in addr, struct sockaddr_in &newAddr);
    int equalAddr(struct sockaddr_in addr, struct sockaddr_in addr2);
};


#endif	/* _NETWORK_H */


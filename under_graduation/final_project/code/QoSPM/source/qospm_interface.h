/* 
 * File:   qospm_interface.h
 * Author: hugo
 *
 * Created on 14 de Março de 2009, 18:23
 */

#ifndef _QOSPM_INTERFACE_H
#define	_QOSPM_INTERFACE_H

#include <sys/socket.h>
#include <netinet/in.h>

#include "domain.h" //CQoS

typedef struct qosp_channel_t {
    int channel_manager_desc;
    int listener_desc;
    CQoS qos;
}qosp_channel_t;

typedef int qosp_router_t;

int qos(qosp_channel_t &channelDesc);
int verifyChannel(struct sockaddr_in pxAddr, struct sockaddr_in pyAddr);
int notifyTimelyChannel(qosp_channel_t &channelDesc);
void notifyUntimelyChannel(qosp_channel_t &channelDesc);
void notifyNewRouter(struct sockaddr_in  routerAddr, struct sockaddr_in  qospaAddr, qosp_router_t *routerDesc);
void notifyNewChannel(struct sockaddr_in  pxAddr, struct sockaddr_in  pyAddr, qosp_channel_t *channel);
void insertRouterInChannel(qosp_router_t router_desc, qosp_channel_t *channel);

/*Funcoes para envio e recepcao de mensagens pelos canais gerenciados pelo QoSP*/
int qosp_sendmsg (qosp_channel_t &desc, int sock, struct msghdr msg, int flags);
int qosp_recvmsg (qosp_channel_t &desc, int sock, struct msghdr msg, int flags);

#endif	/* _QOSPM_INTERFACE_H */


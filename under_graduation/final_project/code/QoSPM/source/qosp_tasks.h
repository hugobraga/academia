/* 
 * File:   qosp_tasks.h
 * Author: hugo
 *
 * Created on 22 de Fevereiro de 2009, 16:51
 */

#ifndef _QOSP_TASKS_H
#define	_QOSP_TASKS_H

#include "qosp.h" //QoSP
#include "mt_support.h" //qosp_thread_t
#include <netinet/in.h> //struct sockaddr_in
#include "conf.h"
#include "qospm_protocol.h"

#define PERIODIC_FD_PRIO        99
#define FD_LISTENER_PRIO        50
#define FD_STUB_PRIO            48
#define VERIFIER_SKEL_PRIO      47

#define MAX_REQUESTS_AT_A_TIME  15

typedef struct explicit_ping_str {
    QoSP    *qosp;
    int     flowDesc;
}explicit_ping_str;

//utilizada para guardar informações referentes a qos que está sendo provida a um canal
typedef struct received_msg_str {
    struct sockaddr_in  addr;       //contém o endereço de quem enviou a mensagem (NodeMonitor)
#ifdef HOLISTIC_QoSP
    /*
    #if MAX_QOSP_REC_MSG_LENGTH > MAX_QOSPA_REC_MSG_LENGTH
    char rec_msg[MAX_QOSP_REC_MSG_LENGTH];   //representa a msg de mon. enviada por NodeMonitor
    #else
    char rec_msg[MAX_QOSPA_REC_MSG_LENGTH];   //representa a msg de mon. enviada por NodeMonitor
    #endif
     */
    char recMsg[MAX_QOSP_REC_MSG_LENGTH];   //representa a msg de mon. enviada por NodeMonitor
#else
    #ifdef QoSP_MODULE
    char recMsg[MAX_QOSP_REC_MSG_LENGTH];   //representa a msg de mon. enviada por NodeMonitor
    #else
    char recMsg[MAX_QOSPA_REC_MSG_LENGTH];   //representa a msg de mon. enviada por NodeMonitor
    #endif
#endif
    char *msgData;
    qosp_thread_t threadDesc;
}received_msg_str;

void checkQoSP(void *arg);
void qosp_tasks_init();
void qosp_tasks_finish();

#endif	/* _QOSP_TASKS_H */


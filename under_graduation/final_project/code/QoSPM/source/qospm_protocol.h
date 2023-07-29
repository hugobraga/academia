/* 
 * File:   qospm_protocol.h
 * Author: hugo
 *
 * Created on 22 de Fevereiro de 2009, 16:46
 */

#ifndef _QOSPM_PROTOCOL_H
#define	_QOSPM_PROTOCOL_H

#include "domain.h"//IP_HEADER_LENGTH

#define PING_REPLY_MSG_LENGTH   IP_HEADER_LENGTH+8 //tamanho da mensagem da respostas do ping (cabecalho IP mais conteudo ICMP)

#define MON_MSG_LENGTH          7   //tamanho da mensagem enviada pelo monitoramento
#define SUBS_MSG_LENGTH         1   //tamanho da mensagem de subscrição
#define VER_MSG_LENGTH          2+4+(2*sizeof(struct sockaddr_in)) //tamanho da mensagem de verificação
#define VER_REPLY_MSG_LENGTH    4+4   //tamanho da mensagem da resposta da verificação remota
#define MON_REPLY_MSG_LENGTH    7   //tamanho da mensagem da resposta do monitoramento
#define QOS_DEGRAD_MSG_LENGTH   2   //tamanho da mensagem informando que a qos contratada nao pode mais ser provida
#define MAX_QOSP_REC_MSG_LENGTH VER_MSG_LENGTH  //contém o tamanho da maior mensagem recebida pelo QoSP
#define MAX_QOSPA_REC_MSG_LENGTH    MON_MSG_LENGTH

#define MON_MSG_ID              1   //id das mensagens enviadas pelo monitoramento
#define FD_MSG_ID               2   //id das mensagens enviadas pelo detector de falhas
#define SUBS_MSG_ID             3   //id das mensagens enviadas solicitando subscrição
#define UNSUBS_MSG_ID           4   //id das mensagens enviadas solicitando cancelamento da subscrição
#define VER_MSG_ID              5   //id das mensagens enviadas solicitando verificação remota de um canal
#define VER_REPLY_MSG_ID        6   //id das mensagens que contêm a resposta da verificação remota
#define MON_REPLY_MSG_ID        7   //id das mensagens que contêm a resposta do monitoramento
#define QOS_DEGRAD_MSG_ID       8   //id da mensagem informando que a qos contratada nao pode mais ser provida

#endif	/* _QOSPM_PROTOCOL_H */


/* 
 * File:   domain.h
 * Author: hugo
 *
 * Created on 24 de Janeiro de 2009, 15:34
 */

#ifndef _DOMAIN_H
#define	_DOMAIN_H

#include <sys/types.h> //u_int16_t
#include <netinet/in.h> //struct sockaddr_in
#include <netinet/ip_icmp.h> //struct icmp
#include "mt_support.h" //qosp_lock_t

#define IP_HEADER_LENGTH            20

#define MAX_T_CHANNELS_PER_ROUTER   15  //numero max de canais timely que passam por cada roteador
#define MAX_ROUTERS_PER_CHANNEL     15  //numero max de roteadores ao longo de um canal
#define MAX_CHANNELS                20  //número máx de canais gerenciados por um QoSP
#define MAX_ROUTERS                 15  //número máx de roteadores de conhecimento do QoSP
#define MAX_QOSP                    30  //número máx de módulos do QoSP existentes na rede

#define TIMELY                      1   //identifica que a qos de um canal é timely
#define UNTIMELY                    0   //identifica que a qos de um canal é untimely
typedef int CQoS;                   //tipo utilizado para identificar a qos de um canal (qualquer variável deste tipo possui valor TIMELY ou UNTIMELY)

#define MAX_FLOWS                   15

#endif	/* _DOMAIN_H */


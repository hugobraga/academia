/* 
 * File:   delays.h
 * Author: hugo
 *
 * Created on 24 de Janeiro de 2009, 15:34
 */

#ifndef _DELAYS_H
#define	_DELAYS_H

#define DELAY
//#define PING_QOSP_DELAY 1300000LL //1 milissegundo
#define PING_QOSP_DELAY 9000000LL //1 milissegundo
#define PING_OTHER_DELAY 300000LL //1 milissegundo
//#define PING_ROUTER_DELAY 1000000LL
#define PING_ROUTER_DELAY 9000000LL

struct sockaddr_in     router_addr;
struct sockaddr_in     qospA_addr;

#endif	/* _DELAYS_H */


/* 
 * File:   monitor.h
 * Author: hugo
 *
 * Created on 8 de Mar�o de 2009, 20:37
 */

#ifndef _MONITOR_H
#define	_MONITOR_H

#include <string>
#include <vector>
#include <list>
#include <netinet/in.h> //struct sockaddr_in
#include "domain.h"
#include "conf.h"
#include "snmp_cisco_toolkit.h"
#include "router.h"

//biblioteca para utilizar o SNMP
#include <net-snmp/net-snmp-config.h>
#include <net-snmp/net-snmp-includes.h>

#define MAX_ROUTER_IF 2
#define cbQosCMDropPkt_STR ".1.3.6.1.4.1.9.9.166.1.15.1.1.13"

class Monitor {
    list<struct sockaddr_in>    interestedQoSP;
    Router                      *router;
    SNMPCisco                   *snmp;
    vector<struct sockaddr_in>    routerInterfaces;
    vector<string>                cbQosCMDropPkt;
    vector<int>                   prevDropPkt;
    string                      communityStr;
    string                      timelyClass;
    qosp_lock_t                 lockDesc;
public:
    Monitor(string routerIP, list<string> &interfacesIP);
    bool isQoSMaintained();
    int getSubscribedQoSPs(list<struct sockaddr_in> &addrs);
    int releaseQoSPs();
    int subscribe(struct sockaddr_in qospAddr);
    int unSubscribe(struct sockaddr_in qospAddr);
    int unSubscribeAll();
    virtual ~Monitor();
};


#endif	/* _MONITOR_H */


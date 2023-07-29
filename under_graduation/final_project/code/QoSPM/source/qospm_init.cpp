#include "qospm_init.h"
#include <algorithm>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "conf.h"
#include "qospm_interface.h"
#include "channelmanager.h"
#include "mediator.h"
#include "channel_verifier.h"
#include "delay.h"
#include "local_host.h"
#include "monitoringapplication.h"
#include "traffic_listener.h"
#include "failuredetector.h"
#include "network.h"
#include "mt_support.h"
#include "monitor.h"
#include "qosp_tasks.h"
#include "qospA_tasks.h"

//globais
qosp_sock_t     sock_handler; //descritor do socket utilizado pelo QoSP para enviar e receber mensagens
qosp_sock_t     sock_ping;

ChannelManager          *glob_manager_obj;
Mediator                *glob_mediator_obj;
ChannelVerifier         *glob_cv_obj;
Delay                   *glob_delay_obj;
LocalHost               *glob_local_host_obj;
MonitoringApplication   *glob_ma_obj;
TrafficListener         *glob_listener_obj;
FailureDetector         *glob_fd_obj;
Network                 *glob_network_obj;
MtSupporter             *glob_mt_supporter_obj;
//qospA
Monitor                 *glob_node_monitor_obj;

#ifdef QoSP_MODULE
/*
 * Inicializa o QoSP.
 * qosp_ip: contem o ip do qosp.
 */
void qosp_init(string &qospIp) {
    struct sockaddr_in qospAddr;
    inet_aton(qospIp.c_str(), &(qospAddr.sin_addr));
    qospAddr.sin_port = htons(QoSP_PORT);
    qospAddr.sin_family = AF_INET;

    glob_local_host_obj = new LocalHost(qospAddr);
    glob_delay_obj = new Delay();
    glob_fd_obj = new FailureDetector(glob_delay_obj);
    glob_ma_obj = new MonitoringApplication(glob_fd_obj, glob_local_host_obj);
    glob_listener_obj = new TrafficListener();
    glob_cv_obj = new ChannelVerifier(glob_ma_obj, glob_listener_obj);
    glob_manager_obj = new ChannelManager();
    glob_mediator_obj = new Mediator(glob_ma_obj, glob_cv_obj, glob_local_host_obj);

    glob_network_obj->initializeSocket(sock_ping, QOSP_LINUX_DOMAIN, SOCK_RAW, FD_PORT);
    qosp_tasks_init();
}

/*
 * Finaliza o QoSP.
 */
void qosp_finish() {
    qosp_tasks_finish();
}
#endif

#ifdef QoSPA_MODULE
/*
 * Inicializa o QoSPA.
 * router_ip_s: contem o ip das interfaces do roteador a serem monitoradas pelo agente.
 */
void qospA_init(string &routerIp, list<string> &interfacesIp) {
    glob_node_monitor_obj = new Monitor(routerIp, interfacesIp);
    qospA_tasks_init();
}

/*
 * Finaliza o QoSPA.
 */
void qospA_finish() {
    qospA_tasks_finish();
}
#endif

void local_copy(list<string> &input, list<string> &output, int first_pos) {
    list<string>::iterator inputItr = input.begin(), endItr = input.end();
    string str;

    while(first_pos--) inputItr++;
    for (; inputItr != endItr; inputItr++) {
        str.assign((*inputItr).c_str());
        output.push_back(str);
    }
}

/*
 * Inicializa o QoSPM.
 * Caso esteja inicializando apenas o QoSP, ips[0] contera o ip do QoSP.
 * Caso esteja inicializando apenas o QoSPA, ips contera o ip das interfaces do roteador
 * a serem monitoradas, terminado com o nulo.
 * Caso esteja inicializando tanto QoSP como QoSPA, ips[0] sera utilizado por QoSP,
 * enquanto que a partir de ips[1] sera utilizado por QoSPA.
 */
void qospM_init(list<string> &ips) {
    list<string> interfaces;
    list<string>::iterator itr;//, end, interfacesItr = interfaces.begin();
    string qospIp, routerIp;
    glob_mt_supporter_obj = new MtSupporter();
    glob_network_obj = new Network();
    glob_network_obj->initializeSocket(sock_handler, QOSP_LINUX_DOMAIN, SOCK_DGRAM, QoSP_PORT);
    itr = ips.begin();
#ifdef HOLISTIC_QoSP
    qospIp = *itr;
    itr++;
    routerIp = *itr;
    local_copy(ips, interfaces, 2);
    qosp_init(qospIp);
    qospA_init(ro
    uterIp, interfaces);
#else
#ifdef QoSP_MODULE
    qospIp = *itr;
    qosp_init(qospIp);
#else
    routerIp = *itr;
    local_copy(ips, interfaces, 1);
    qospA_init(routerIp, interfaces);
#endif
#endif
}

/*
 * Finaliza QoSPM.
 */
void qospM_finish() {
    #ifdef QoSP_MODULE
    qosp_finish();
    #endif
    #ifdef QoSPA_MODULE
    qospA_finish();
    #endif
}

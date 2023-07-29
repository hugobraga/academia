/**/

#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <native/task.h>
#include <list>
#include "domain.h"
#include "qospm_interface.h"
#include "qospm_init.h"
#include <string>
#include <iostream>

using namespace std;

#define QoSP_IP "192.168.1.2"
#define OTHER_PROCESS   "192.168.1.5"
#define PROCESS_PORT    5010

//esta funcao deveria ser chamada pelo modulo de admissao para informar o modulo de monitoramento que um canal tornou-se TIMELY.
int notify_timely_channel(struct sockaddr_in p1_addr, struct sockaddr_in p2_addr, qosp_channel_t *channel) {
    #define QoSPA_IP "192.168.1.3"
    #define ROUTER_IP "192.168.1.4"

    struct sockaddr_in     router_addr;
    char            *router_ip_s = ROUTER_IP;
    struct sockaddr_in     qospA_addr;
    char            *qospA_ip_s = QoSPA_IP;
    qosp_router_t router_desc;

    inet_aton(router_ip_s, &(router_addr.sin_addr));
    router_addr.sin_family = AF_INET;
    inet_aton(qospA_ip_s, &(qospA_addr.sin_addr));
    qospA_addr.sin_family = AF_INET;
    qospA_addr.sin_port = htons(QoSP_PORT);
    notifyNewRouter(router_addr, qospA_addr, &router_desc);
    notifyNewChannel(p1_addr, p2_addr, channel);
    insertRouterInChannel(router_desc, channel);
    channel->listener_desc = notifyTimelyChannel(*channel); //problema
}

void get_channel_descriptor(qosp_channel_t *channel) {
    struct sockaddr_in  p1_addr, p2_addr;
    char *p1_ip_s = QoSP_IP;
    char *p2_ip_s = OTHER_PROCESS;

    p1_addr.sin_port = htons(PROCESS_PORT);
    p2_addr.sin_port = htons(PROCESS_PORT);

    inet_aton(p1_ip_s, &(p1_addr.sin_addr));
    p1_addr.sin_family = AF_INET;
    inet_aton(p2_ip_s, &(p2_addr.sin_addr));
    p2_addr.sin_family = AF_INET;

    notify_timely_channel(p1_addr, p2_addr, channel);
}

void catch_signal(int signal) {
    qospM_finish();
}

int main(int argc, char** argv) {
    #define TASK_MODE  0
    char *ip_qosp = (char *)QoSP_IP;
    string str;
    list<string> ips;
    RT_TASK MAIN;
    qosp_channel_t *channel;

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);
    signal(SIGHUP, catch_signal);

    str.assign(ip_qosp);
    ips.push_back(str);
    qospM_init(ips);

    cout << "Este modulo do QoSP ja foi inicializado...";
    cin.get();

    cout << "Coloque os agentes para funcionar e tecle enter...";
    cin.get();

    cout << "Inicialize o outro modulo do QoSP...";
    cin.get();

    mlockall(MCL_CURRENT|MCL_FUTURE);
    rt_task_shadow(&MAIN, NULL, 98, TASK_MODE);

    get_channel_descriptor(channel);

    cout << "Este QoSP ja esta ciente da presenca de um canal TIMELY." << endl;
    cout << "Quando quiser encerrar o programa, tecle CTRL+C." << endl;

    pause();
    return (EXIT_SUCCESS);
}

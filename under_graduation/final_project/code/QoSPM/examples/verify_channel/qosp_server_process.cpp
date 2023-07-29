/**/

#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <native/task.h>
#include <rtdm/rtdm.h>
#include <list>
#include "domain.h"
#include "qospm_interface.h"
#include "qospm_init.h"
#include <string>
#include <iostream>

#include <arpa/inet.h>
#include <sys/types.h> //u_int16_t e u_short
#include <netinet/ip.h> //struct ip
#include <sys/socket.h> //struct msghdr
#include <netdb.h> //struct protoent

using namespace std;

#define QoSP_IP "192.168.1.2"
#define OTHER_PROCESS   "192.168.1.5"
#define PROCESS_PORT    5010

int initialize_socket(int &sock_desc, int sock_port) {
    int ret;
    uint16_t port = sock_port;
    struct protoent *proto;
    unsigned int add_rtskbs = 75;
    struct sockaddr_in local_addr; //utilizado apenas para vincular um socket a uma porta

    if ((proto = getprotobyname("udp")) == NULL)
        return 0;

    sock_desc = rt_dev_socket(AF_INET,SOCK_DGRAM,0);
    if (sock_desc < 0)
        return 0;

    /* extend the socket pool */
    ret = rt_dev_ioctl(sock_desc, RTNET_RTIOC_EXTPOOL, &add_rtskbs);
    if (ret != (int)add_rtskbs) {
        rt_dev_close(sock_desc);
        return 0;
    }

    /* bind the rt-socket to a port */
    bzero((char *) &local_addr, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(port);
    local_addr.sin_addr.s_addr = htons(INADDR_ANY);
    ret = rt_dev_bind(sock.sock_desc, (struct sockaddr *)&local_addr, sizeof(struct sockaddr_in));
    if (ret < 0) {
        rt_dev_close(sock_desc);
        return 0;
    }
    return 1;
}

int run_app_process() {
    Network network_obj;
    int ret;
    struct sockaddr_in  local_addr;
    char *qosp_ip_s = QoSP_IP;
    qosp_channel_t  channel;
    char *other_ip_s = OTHER_PROCESS;
    struct sockaddr_in other_addr;
    int sock;

    struct msghdr recv_msg;//mensagem que sera enviada
    struct iovec recv_iov;//buffer de dados
    char recv_buffer[10];

    inet_aton(qosp_ip_s, &(local_addr.sin_addr));
    local_addr.sin_family = AF_INET;
    
    local_addr.sin_port = htons(PROCESS_PORT);
    inet_aton(other_ip_s, &(other_addr.sin_addr));
    other_addr.sin_port = htons(PROCESS_PORT);
    other_addr.sin_family = AF_INET;
    notify_timely_channel(local_addr, other_addr, &channel);
    initialize_socket(sock, PROCESS_PORT);

    recv_iov.iov_base = recv_buffer; //end da area de dados
    recv_iov.iov_len = 10; //tamanho da area de dados

    bzero((char *) &recv_msg, sizeof(recv_msg));
    recv_msg.msg_name = &other_addr; //endereço do host que enviou a resposta
    recv_msg.msg_namelen = sizeof(other_addr); //tamanho da estrutura do endereço
    recv_msg.msg_iov     = &recv_iov; //endereço do buffer dos dados
    recv_msg.msg_iovlen  = 1; //numero de buffer de dados
    recv_msg.msg_control = 0;
    recv_msg.msg_controllen = 0;
    recv_msg.msg_flags = 0;

    while(1)
        ret = qosp_recvmsg (channel, sock, recv_msg, 0);
}

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

void catch_signal() {
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

    get_channel_descriptor(&channel);

    cout << "Este QoSP ja esta ciente da presenca de um canal TIMELY." << endl;
    cout << "Quando quiser encerrar o programa, tecle CTRL+C." << endl;

    pause();
    return (EXIT_SUCCESS);
}

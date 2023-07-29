#include <stdio.h>
#include <stdlib.h>
#include "domain.h"
#include "network.h"
//#include <regex.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
//adicionado
//#include <rtdk.h>
//#include<native/pipe.h>
#include <native/task.h>
#include <native/timer.h>
#include <list>
//#include "delays.h"
#include "conf.h"
#include "qospm_interface.h"
#include "qospm_init.h"
#include <string>

//adicionado
using namespace std;


//#define QOSP
//#define QOSPA
//#define ROUTER

//#ifdef QOSP_MODULE
#define QoSP_IP "192.168.149.12"
#define QoSPA_IP "192.168.149.20"
#define ROUTER_IP "192.168.149.20"

//define se este host possuira processos aplicativos enviando e/ou recebendo mensagens
#define APPLIC_CHANNEL 1
#define HAVE_APPLIC_PROCESS
#ifdef HAVE_APPLIC_PROCESS
#if APPLIC_CHANNEL == 1
    #define USE_CHANNEL1
#else
    #define USE_CHANNEL2
#endif
//especifica se este host tera um processo cliente (envia mensagens) ou servidor (recebe mensagens)
#define CLIENT_PROCESS
//#define SERVER_PROCESS
#ifdef CLIENT_PROCESS
#define SEND_MSG_PERIOD 200000LL
#ifdef SERVER_PROCESS
#define CLIENT_SERVER_PROCESS //a logica deve ser definida la embaixo
#endif
#endif
#endif

//define se eh um testador das funcoes do QoSP
#ifndef HAVE_APPLIC_PROCESS
#define QoSP_TESTER
#endif
#ifdef QoSP_TESTER

//#define TEST_QoS_CV
#define TEST_QoS
//#define TEST_NOTIFY_TIMELY

    #ifdef TEST_QoS_CV
        #if APPLIC_CHANNEL == 1
            #define USE_CHANNEL1
            //#define QOSP_TESTER_CHANNEL 2
        #else
            #define USE_CHANNEL2
            //#define QOSP_TESTER_CHANNEL 1
        #endif
    #else
        #if APPLIC_CHANNEL == 1
            #define USE_CHANNEL2
        #else
            #define USE_CHANNEL1
        #endif
    #endif

#endif
//#endif

//

#ifdef USE_CHANNEL1
#define LOCAL_PROCESS_IP1   "192.168.149.12"
#define OTHER_PROCESS_IP1   "192.168.149.20"
#define PROCESS_PORT1       5010
#endif

#ifdef USE_CHANNEL2
#define LOCAL_PROCESS_IP2   "192.168.149.24"
#define OTHER_PROCESS_IP2   QoSPA_IP
#define PROCESS_PORT2       5011
#endif

RT_TASK TESTE1, TESTE2;

int notify_timely_channel(struct sockaddr_in p1_addr, struct sockaddr_in p2_addr, qosp_channel_t *channel) {
    extern struct sockaddr_in     router_addr;
    //struct sockaddr_in     router_addr;
    char            *router_ip_s = ROUTER_IP;
    extern struct sockaddr_in     qospA_addr;
    //struct sockaddr_in     qospA_addr;
    char            *qospA_ip_s = QoSPA_IP;
    //Router          *router;
    //int channel_desc;
    qosp_router_t router_desc;

    inet_aton(router_ip_s, &(router_addr.sin_addr));
    router_addr.sin_family = AF_INET;
    inet_aton(qospA_ip_s, &(qospA_addr.sin_addr));
    qospA_addr.sin_family = AF_INET;
    qospA_addr.sin_port = htons(QoSP_PORT);

    notifyNewRouter(router_addr, qospA_addr, &router_desc);
    notifyNewChannel(p1_addr, p2_addr, channel);
    insertRouterInChannel(router_desc, channel);
    //(*channel)->insertRouter(*channel, router);
    //printf("o canal notificado eh:\n");
    //printf("px: %s\n", inet_ntoa(p1_addr.sin_addr));
    //printf("py: %s\n", inet_ntoa(p2_addr.sin_addr));
    //printf("notificou canal\n");
    channel->listener_desc = notifyTimelyChannel(*channel);
}

#ifdef QoSP_TESTER
void test_qosp_api() {
    struct sockaddr_in  p1_addr, p2_addr;
    //Channel             channel;
    #ifdef TEST_QoS_CV
    //CommunicationEntity p1, p2;
    qosp_channel_t channel;
    #else
    qosp_channel_t channel;
    #endif

    #ifdef USE_CHANNEL1
    char *p1_ip_s = LOCAL_PROCESS_IP1;
    char *p2_ip_s = OTHER_PROCESS_IP1;
    //p1_ip_s = "192.168.149.15";
    //p2_ip_s = "192.168.149.20";
    #endif

    #ifdef USE_CHANNEL2
    char *p1_ip_s = LOCAL_PROCESS_IP2;
    char *p2_ip_s = OTHER_PROCESS_IP2;
    //p1_ip_s = "192.168.149.20";
    //p2_ip_s = "192.168.149.24";
    #endif
    //printf("dentro de test api\n");
    #ifdef USE_CHANNEL1
    //inet_aton(px_ip_s, &(px_addr.sin_addr));
    p1_addr.sin_port = htons(PROCESS_PORT1);
    //px_addr.sin_family = AF_INET;
    //inet_aton(py_ip_s, &(py_addr.sin_addr));
    p2_addr.sin_port = htons(PROCESS_PORT1);
    //py_addr.sin_family = AF_INET;
    #endif

    #ifdef USE_CHANNEL2
    //inet_aton(py_ip_s, &(py_addr.sin_addr));
    p1_addr.sin_port = htons(PROCESS_PORT2);
    //py_addr.sin_family = AF_INET;
    //inet_aton(pz_ip_s, &(pz_addr.sin_addr));
    p2_addr.sin_port = htons(PROCESS_PORT2);
    //pz_addr.sin_family = AF_INET;
    #endif

    inet_aton(p1_ip_s, &(p1_addr.sin_addr));
    p1_addr.sin_family = AF_INET;
    //printf("p1 para ser testado: %s\n", inet_ntoa(p1_addr.sin_addr));
    inet_aton(p2_ip_s, &(p2_addr.sin_addr));
    p2_addr.sin_family = AF_INET;
    //printf("p2 para ser testado: %s\n", inet_ntoa(p2_addr.sin_addr));

    #ifdef TEST_QoS_CV
    //CommunicationEntity p1, p2;
    //Channel channel;

    /*
    NEW(&channel, Channel);
    NEW(&p1, CommunicationEntity);
    NEW(&p2, CommunicationEntity);
    p1.construct_sock(&p1, p1_addr);
    p2.construct_sock(&p2, p2_addr);
    channel.construct(&channel, &p1, &p2, UNTIMELY);
     */
    //notify_timely_channel(p1_addr, p2_addr, &channel);
    //printf("end. processo: %s\n", inet_ntoa(p1_addr.sin_addr));
    //printf("end. processo: %s\n", inet_ntoa(p2_addr.sin_addr));
    if (verifyChannel(p1_addr, p2_addr))
        printf("verificou remotamente\n");
    else
        printf("lascou\n");
    #endif
    #ifdef TEST_NOTIFY_TIMELY
    //Channel *channel;
    notify_timely_channel(p1_addr, p2_addr, &channel);
    /*
    extern struct sockaddr_in     router_addr;
    char            *router_ip_s = ROUTER_IP;
    extern struct sockaddr_in     qospA_addr;
    char            *qospA_ip_s = QOSPA_IP;
    Router          *router;

    inet_aton(router_ip_s, &(router_addr.sin_addr));
    router_addr.sin_family = AF_INET;
    inet_aton(qospA_ip_s, &(qospA_addr.sin_addr));
    qospA_addr.sin_family = AF_INET;
    qospA_addr.sin_port = htons(QOSP_PORT);

    global_manager.notifyNewRouter(&global_manager, router_addr, qospA_addr, &router);
    global_manager.notifyNewChannel(&global_manager, px_addr, pz_addr, &channel);
    channel->insertRouter(channel, router);
    desc2 = global_api.notifyTimelyChannel(&global_api, channel);
     */
    #endif
    #ifdef TEST_QoS
    //Channel *channel;
    notify_timely_channel(p1_addr, p2_addr, &channel);
    printf("vai verificar a QoS\n");
    if (qos(channel))
        printf("verificou a qos\n");
    else
        printf("lascou\n");
    #endif
}
#endif//QOSP_TESTER
//#endif//QOSP

#ifdef HAVE_APPLIC_PROCESS
int run_app_process() {
    Network network_obj;
    int ret;
    struct sockaddr_in  local_addr;
    char *qosp_ip_s = QoSP_IP;
    qosp_channel_t  channel;
    #ifdef USE_CHANNEL1
    char *ip_s = OTHER_PROCESS_IP1;
    struct sockaddr_in addr;
    qosp_sock_t sock1;
    int desc1;// = notify_timely_channel();
    #endif

    #ifdef USE_CHANNEL2
    char *ip_s2 = OTHER_PROCESS_IP2;
    struct sockaddr_in addr2;
    qosp_sock_t sock2;
    int desc2;// = notify_timely_channel();
    #endif

    #ifdef CLIENT_PROCESS
    struct msghdr msg;//mensagem que sera enviada
    struct iovec iov;//buffer de dados
    char buffer[10] = "nada";
    #endif

    #ifdef SERVER_PROCESS
    struct msghdr recv_msg;//mensagem que sera enviada
    struct iovec recv_iov;//buffer de dados
    char recv_buffer[10];
    #endif

    //NEW(&network_obj, Network);
    inet_aton(qosp_ip_s, &(local_addr.sin_addr));
    local_addr.sin_family = AF_INET;

    #ifdef USE_CHANNEL1
    local_addr.sin_port = htons(PROCESS_PORT1);
    inet_aton(ip_s, &(addr.sin_addr));
    addr.sin_port = htons(PROCESS_PORT1);
    addr.sin_family = AF_INET;
    //other_addr = addr;
    notify_timely_channel(local_addr, addr, &channel);

    //network_obj.initialize_socket(&sock1, QOSP_XENOMAI_DOMAIN, SOCK_DGRAM, PROCESS_PORT1);
    printf("antes de inicializar o socket dos processos\n");
    network_obj.initializeSocket(sock1, QOSP_LINUX_DOMAIN, SOCK_DGRAM, PROCESS_PORT1);
    printf("inicializou o socket dos processos\n");
    #endif

    #ifdef USE_CHANNEL2
    local_addr.sin_port = htons(PROCESS_PORT2);
    inet_aton(ip_s2, &(addr2.sin_addr));
    addr2.sin_port = htons(PROCESS_PORT2);
    addr2.sin_family = AF_INET;
    //other_addr = addr2;
    //exit(0);
    //printf("notificar canal\n");
    notify_timely_channel(local_addr, addr2, &channel);
    //printf("antes de init socket\n");
    //network_obj.initialize_socket(&sock2, QOSP_XENOMAI_DOMAIN, SOCK_DGRAM, PROCESS_PORT2);
    network_obj.initializeSocket(sock2, QOSP_LINUX_DOMAIN, SOCK_DGRAM, PROCESS_PORT2);
    #endif

    #ifdef CLIENT_PROCESS
    iov.iov_base = buffer; //end da area de dados
    iov.iov_len = 4; //tamanho da area de dados

    bzero((char *) &msg, sizeof(msg));
    #ifdef USE_CHANNEL1
    msg.msg_name    = &addr; //socket
    msg.msg_namelen = sizeof(addr); //tamanho da estrutura do socket
    #else//USE_CHANNEL2
    msg.msg_name    = &addr2; //socket
    msg.msg_namelen = sizeof(addr2); //tamanho da estrutura do socket
    #endif
    msg.msg_iov     = &iov; //endereço do buffer dos dados
    msg.msg_iovlen  = 1; //numero de buffer de dados
    msg.msg_control = 0;
    msg.msg_controllen = 0;
    msg.msg_flags = 0;
    #endif

    #ifdef SERVER_PROCESS
    recv_iov.iov_base = recv_buffer; //end da area de dados
    recv_iov.iov_len = 10; //tamanho da area de dados

    bzero((char *) &recv_msg, sizeof(recv_msg));
    #ifdef USE_CHANNEL1
    recv_msg.msg_name = &addr; //endereço do host que enviou a resposta
    recv_msg.msg_namelen = sizeof(addr); //tamanho da estrutura do endereço
    #else//USE_CHANNEL2
    recv_msg.msg_name = &addr2; //endereço do host que enviou a resposta
    recv_msg.msg_namelen = sizeof(addr2); //tamanho da estrutura do endereço
    #endif
    recv_msg.msg_iov     = &recv_iov; //endereço do buffer dos dados
    recv_msg.msg_iovlen  = 1; //numero de buffer de dados
    recv_msg.msg_control = 0;
    recv_msg.msg_controllen = 0;
    recv_msg.msg_flags = 0;
    #endif

    while(1) {
        #ifdef CLIENT_SERVER_PROCESS //voce decide
        rt_task_sleep(rt_timer_ns2ticks(SEND_MSG_PERIOD));
        ret = qosp_sendmsg (channel, sock2.sock_desc, msg, 0);
        #endif
        #ifdef CLIENT_PROCESS
        //ret = qosp_recvmsg (desc, sock, recv_msg, 0);
        //usleep(delay);
        rt_task_sleep(rt_timer_ns2ticks(SEND_MSG_PERIOD));

        #ifdef USE_CHANNEL1
        ret = qosp_sendmsg (channel, sock1.sock_desc, msg, 0);
        #else
        ret = qosp_sendmsg (channel, sock2.sock_desc, msg, 0);
        //19/09ret = qosp_sendmsg (desc2, sock2, msg2, 0);
        #endif
        #endif

        #ifdef SERVER_PROCESS
        #ifdef USE_CHANNEL1
        ret = qosp_recvmsg (channel, sock1.sock_desc, recv_msg, 0);
        #else
        ret = qosp_recvmsg (channel, sock2.sock_desc, recv_msg, 0);
        #endif
        #endif
        if (ret <= 0) {
            printf("erro: %d\n", ret);
            exit(EXIT_FAILURE);
        }
    }
}
#endif//HAVE_APLIC_PROCESS

void *teste1(void *arg) {
    #define TASK_MODE  0
    rt_task_shadow(&TESTE1, NULL, 98, TASK_MODE);
    //test_qosp_api();
}

void *teste2(void *arg) {
    #define TASK_MODE  0
    rt_task_shadow(&TESTE2, NULL, 98, TASK_MODE);
    //test_qosp_api();
}

int main(int argc, char** argv) {
    int a;
    #define TASK_MODE  0
    char *ip_qosp[1] = {(char *)QoSP_IP};
    //char *ips_qospA[2] = {(char *)QoSPA_IP, NULL};
    char *ips_qospA[2] = {(char *)"10.10.20.2", (char *)"10.10.25.1"};
    #ifdef HOLISTIC_QoSP
    //char *ips[4] = {ip_qosp[0], ips_qospA[0], ips_qospA[1]};
    #endif
    string str;
    list<string> ips;
    RT_TASK TESTE, MAIN;
    pthread_t thread1, thread2;

    #ifdef HOLISTIC_QoSP
    #endif
    /*
    int len = 5;
    char nada;
    SERIALIZE(&nada, &len, 1);
    printf("valor: %d\n", get_decimal_value(nada));
    return(0);
     */
    //signal(SIGTERM, catch_signal);
    //signal(SIGINT, catch_signal);
    //signal(SIGHUP, catch_signal);
    mlockall(MCL_CURRENT|MCL_FUTURE);

    //pipe
    //len = sizeof("Hello");
    //rt_pipe_create(&pipe_desc, "pipe", PIPE_MINOR, 0);

	/* Perform auto-init of rt_print buffers if the task doesn't do so */
	//rt_print_auto_init(1);

	/* Initialise the rt_print buffer for this task explicitly */
	//rt_print_init(4096, "Task 1");
        //rt_printf("opa\n");

        //rt_printf("fora do kill\n");
        //rt_task_shadow(&MAIN, "Task 1", 98, TASK_MODE);
        //rt_printf("dentro\n");
    //rt_task_spawn(&TESTE, NULL, TASK_STKSZ, 49, TASK_MODE, &loop_imp, NULL);
    //rt_task_create(&TESTE, NULL, TASK_STKSZ, 49, TASK_MODE);
    //rt_task_start(&TESTE, loop_imp,NULL);
    rt_task_shadow(&MAIN, NULL, 98, TASK_MODE);

    #ifdef HOLISTIC_QoSP
    str.assign(ip_qosp[0]);
    ips.push_back(str);
    for (a = 0; a < 2; a++) {
        str.assign(ips_qospA[a]);
        ips.push_back(str);
    }
        //printf("antes do qospM_init\n");
        qospM_init(ips);
        //printf("depois do qospM_init\n");
        /*
        NEW(&glob_api_obj, API);
        glob_api_obj.construct(&glob_api_obj);
         */
    #else
        #ifdef QoSP_MODULE
        str.assign(ip_qosp[0]);
        ips.push_back(str);
            qospM_init(ips);
            /*
            NEW(&glob_api_obj, API);
            glob_api_obj.construct(&glob_api_obj);
             */
        #else
        str.assign(ips_qospA[0]);
        ips.push_back(str);
            qospM_init(ips);
        #endif
    #endif

    #ifdef QoSP_MODULE
    //printf("nada\n");
    //NEW(&global_router, Router);
    //global_router.construct(&global_router, router_addr);
    //printf("ai\n");
    //exit(0);
    //qosp_run_tasks();
    //printf("depois de run\n");
    #ifdef QoSP_TESTER
    pthread_create(&(thread1), NULL, teste1, NULL);
    pthread_create(&(thread2), NULL, teste2, NULL);
    //test_qosp_api();
    #else //HAVE_APLIC_PROCESS
    //printf("ai\n");
    //exit(0);
    run_app_process();
    #endif
    #endif
    //rt_printf("fora do kill2\n");
    pause();
    qospM_finish();
    /*
    #ifdef QoSPA_MODULE
    qospA_exit();
    #endif
    #ifdef QoSP_MODULE
    qosp_exit();
    #endif
     */
    return (EXIT_SUCCESS);
}


#include <stdlib.h>
#include "domain.h"
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <list>
#include "conf.h"
#include "qospm_interface.h"
#include "qospm_init.h"
#include <string>
#include <iostream>

using namespace std;

#define QoSP_IP       "192.168.1.2"
#define PROCESS_IP1   "192.168.1.2"
#define PROCESS_IP2   "192.168.1.2"
#define PROCESS_PORT    5010

RT_TASK TESTE1, TESTE2;

void test_qosp_api() {
    struct sockaddr_in  p1_addr, p2_addr;

    char *p1_ip_s = PROCESS_IP1;
    char *p2_ip_s = PROCESS_IP2;

    p1_addr.sin_port = htons(PROCESS_PORT);
    p2_addr.sin_port = htons(PROCESS_PORT);

    inet_aton(p1_ip_s, &(p1_addr.sin_addr));
    p1_addr.sin_family = AF_INET;
    inet_aton(p2_ip_s, &(p2_addr.sin_addr));
    p2_addr.sin_family = AF_INET;
    if (verifyChannel(p1_addr, p2_addr))
        cout << "O canal eh TIMELY\n";
    else
        cout << "O canal eh UNTIMELY\n";
}

void *teste1(void *arg) {
    #define TASK_MODE  0
    rt_task_shadow(&TESTE1, NULL, 98, TASK_MODE);
    test_qosp_api();
}

void *teste2(void *arg) {
    #define TASK_MODE  0
    rt_task_shadow(&TESTE2, NULL, 98, TASK_MODE);
    test_qosp_api();
}

int main(int argc, char** argv) {
    #define TASK_MODE  0
    char *ip_qosp = (char *)QoSP_IP;
    string str;
    list<string> ips;
    //RT_TASK MAIN;

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
    //rt_task_shadow(&MAIN, NULL, 98, TASK_MODE);

    pthread_create(&(thread1), NULL, teste1, NULL);
    pthread_create(&(thread2), NULL, teste2, NULL);
    
    //test_qosp_api();

    return (EXIT_SUCCESS);
}


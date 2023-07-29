/*
 * Programa a ser executado no computador responsavel por monitorar especificadamente um roteador.
 * Necessariamente este computador so executa esta funcao, ou seja, nenhum processo aplicativo que usufrui
 * dos servicos do QoS Provider pode executar neste computador.
 */

#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <list>
#include <string>
#include "qospm_init.h"
#include <iostream>

using namespace std;

#define ROUTER_IP   "192.168.1.2"

void catch_signal() {
    qospM_finish();
}

int main(int argc, char** argv) {
    #define TASK_MODE  0
    char *router_ip = ROUTER_IP";
    string str;
    list<string> ips;
    RT_TASK MAIN;

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);
    signal(SIGHUP, catch_signal);

    mlockall(MCL_CURRENT|MCL_FUTURE);
    rt_task_shadow(&MAIN, NULL, 98, TASK_MODE);

    str.assign(router_ip);
    ips.push_back(str);
    qospM_init(ips);

    cout << "Quando quiser encerrar o programa, tecle CTRL+C." << endl;

    pause();

    return (EXIT_SUCCESS);
}


/*
 * Contem todas os processos utilizadas pelo QoSPA alem das funcoes que necessitam estar
 * sincronizadas com alguns processos.
 */

#include "qospA_tasks.h"
#include "tools.h" //MARSHALLING
#include <list>
#include "mt_support.h"
#include "qosp_tasks.h" //MAX_REQUESTS_AT_A_TIME
#include "qospm_protocol.h"//MON_MSG_LENGTH
#include "monitor.h" //Monitor
#include "network.h"
#include <netinet/in.h> //struct sockaddr_in

extern Monitor *glob_node_monitor_obj;
extern Network *glob_network_obj;
extern MtSupporter *glob_mt_supporter_obj;
extern qosp_sock_t sock_handler;

qosp_lock_t         glob_explicit_mon_mutex;
qosp_cond_t         glob_explicit_mon_cond;

int                 glob_explicit_mon_flag;         //utilizado por mon_skeleton para avisar que necessita de informacoes de monitoramento
int                 glob_qos_maintained;            //informa se QoS contratada junto ao roteador continua sendo mantida

qosp_lock_t         glob_interested_mutex;
int                 glob_qt_interested;             //quantidade de modulos do QoSP que estao aguardando receber a resposta do monitoramento
list<struct sockaddr_in>  glob_interested;      //contem o endereco dos modulos do QoSP que estao aguardando receber a resposta do monitoramento

int glob_mon_skeleton_task_started;                 //informa se a tarefa mon_skeleton_task esta em execucao

char glob_mon_requests[MAX_REQUESTS_AT_A_TIME][MON_MSG_LENGTH]; //armazena todas as requisicoes de monitoramento que chegam

qosp_thread_t   glob_msg_listener_A_task;
qosp_thread_t   glob_period_router_mon_task;

/*
 * Processo responsavel por tratar as requisicoes acerca da QoS junto ao roteador.
 */
void mon_skeleton(void *arg) {
    int qtAddr;
    list<struct sockaddr_in>::iterator itr, end;
    struct sockaddr_in addr;

    glob_mt_supporter_obj->lockAcquire(glob_explicit_mon_mutex);
    glob_explicit_mon_flag = 1;
    glob_mt_supporter_obj->condWait(glob_explicit_mon_cond, glob_explicit_mon_mutex, -1);
    glob_mt_supporter_obj->lockAcquire(glob_interested_mutex);
    glob_explicit_mon_flag = 0;
    glob_mt_supporter_obj->lockRelease(glob_explicit_mon_mutex);

    itr = glob_interested.begin();

    end = glob_interested.end();
    for (qtAddr = 0; itr != end; itr++, qtAddr++) {
        glob_mon_requests[qtAddr][0] = (char)MON_REPLY_MSG_ID;
        glob_mon_requests[qtAddr][1] = (char)glob_qos_maintained;
        addr = *itr;
        glob_network_obj->sendMsg(sock_handler, *itr, glob_mon_requests[qtAddr], (int)MON_REPLY_MSG_LENGTH);
    }
    glob_qt_interested = 0;
    glob_mon_skeleton_task_started = 0;
    glob_mt_supporter_obj->lockRelease(glob_interested_mutex);
}

/*
 * Processo responsavel por verificar periodicamente se a QoS do roteador esta mantida.
 */
void periodic_router_mon(void *arg) {
    list<struct sockaddr_in> addrs;
    list<struct sockaddr_in>::iterator itr, end;
    char buffer[QOS_DEGRAD_MSG_LENGTH];
    buffer[0] = (char)QOS_DEGRAD_MSG_ID;
    buffer[1] = '\0';
    while(1) {
        if (!(glob_node_monitor_obj->isQoSMaintained())) {
            if (glob_explicit_mon_flag) {
                glob_mt_supporter_obj->lockAcquire(glob_explicit_mon_mutex);
                glob_qos_maintained = 0;
                glob_mt_supporter_obj->condSignal(glob_explicit_mon_cond);
                glob_mt_supporter_obj->lockRelease(glob_explicit_mon_mutex);
            }

            //informar aos modulos do QoSP interessados da degradacao
            glob_node_monitor_obj->getSubscribedQoSPs(addrs);
            itr = addrs.begin();
            end = addrs.end();
            for (; itr != end; itr++)// {
                glob_network_obj->sendMsg(sock_handler, *itr, buffer, (int)QOS_DEGRAD_MSG_LENGTH);
            glob_node_monitor_obj->releaseQoSPs();
            glob_node_monitor_obj->unSubscribeAll();
        } else {
            if (glob_explicit_mon_flag) {
                glob_mt_supporter_obj->lockAcquire(glob_explicit_mon_mutex);
                glob_qos_maintained = 1;
                glob_mt_supporter_obj->condSignal(glob_explicit_mon_cond);
                glob_mt_supporter_obj->lockRelease(glob_explicit_mon_mutex);
            }
        }
    }
}

/*
 * Processo responsavel por ficar escutando por mensagens de subscrição ou de verificação da qos
 * que está sendo provida por um roteador.
 */
void msg_listener_A(void *arg) {
    int ret, msgType;
    received_msg_str receivedMsg;
    int msg_id;
    qosp_thread_t monSkeletonParam;
    Monitor *glob_node_monitor_obj;

    glob_qt_interested = 0;
    while(1) { //fica eternamente esperando por mensagens
        ret = glob_network_obj->receiveMsg(sock_handler, receivedMsg.recMsg, receivedMsg.msgData, MAX_QOSPA_REC_MSG_LENGTH, &(receivedMsg.addr), SOCK_DGRAM);
        if (ret <= 0)//problema na mensagem recebida
            return;
        else {
            msgType = (int)receivedMsg.msgData[0];
            switch (msgType) {
                case MON_MSG_ID:
                    glob_mt_supporter_obj->lockAcquire(glob_interested_mutex);
                    glob_mon_requests[glob_qt_interested][2] = receivedMsg.msgData[2];
                    UNMARSHALLING(&msg_id, &(receivedMsg.msgData[3]), sizeof(int));
                    MARSHALLING(&(glob_mon_requests[glob_qt_interested][3]), &msg_id, sizeof(int));
                    glob_interested.push_back(receivedMsg.addr);
                    if (!glob_mon_skeleton_task_started)
                        glob_mt_supporter_obj->threadCreate(monSkeletonParam, QOSP_LINUX_DOMAIN, (void *)&mon_skeleton, NULL, -1);
                    glob_mt_supporter_obj->lockRelease(glob_interested_mutex);
                    break;
                case SUBS_MSG_ID:
                    glob_node_monitor_obj->subscribe(receivedMsg.addr);
                    break;
                case UNSUBS_MSG_ID:
                    glob_node_monitor_obj->unSubscribe(receivedMsg.addr);
                    break;
            }
        }
    }
}

/*
 * Inicializa tudo que eh necessario para ser utilizado pelas tarefas do QoSPA,
 * incluindo as proprias tarefas.
 */
void qospA_tasks_init() {
    glob_mt_supporter_obj->mutexCreate(glob_explicit_mon_mutex, QOSP_LINUX_DOMAIN);
    glob_mt_supporter_obj->condCreate(glob_explicit_mon_cond, QOSP_LINUX_DOMAIN);

    glob_mon_skeleton_task_started = 0;
    glob_mt_supporter_obj->mutexCreate(glob_interested_mutex, QOSP_LINUX_DOMAIN);
#ifndef HOLISTIC_QoSP
    glob_mt_supporter_obj->threadCreate(glob_msg_listener_A_task, QOSP_LINUX_DOMAIN, (void *)&msg_listener_A, NULL, -1);
#endif
   glob_mt_supporter_obj->threadCreate(glob_period_router_mon_task, QOSP_LINUX_DOMAIN, (void *)&periodic_router_mon, NULL, -1);
}

/*
 * Finaliza os recursos a ser utilizado pelas tarefas do QoSPA.
 */
void qospA_tasks_finish() {
    glob_mt_supporter_obj->mutexDestroy(glob_explicit_mon_mutex);
    glob_mt_supporter_obj->mutexDestroy(glob_explicit_mon_mutex);

    glob_mt_supporter_obj->mutexDestroy(glob_interested_mutex);
#ifndef HOLISTIC_QoSP
    glob_mt_supporter_obj->threadDestroy(glob_msg_listener_A_task);
#endif
    glob_mt_supporter_obj->threadDestroy(glob_period_router_mon_task);
}

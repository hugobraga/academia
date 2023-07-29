/*
 * Contem todas os processos utilizadas pelo QoSP alem das funcoes que necessitam estar
 * sincronizadas com alguns processos.
 */

#include "qosp_tasks.h"
#include "monitoringapplication.h"
#include "channelmanager.h"
#include "monitoring.h"
#include "tools.h"//MARSHALLING
#ifdef HOLISTIC_QoSP
#include "qospA_tasks.h"
#include "monitor.h"
#endif
#include <vector>
#include <list>
#include "failuredetector.h"
#include "qospm_protocol.h"
#include "network.h"
#include "mt_support.h"
#include "conf.h" //FD_MON_PERIOD
#include <native/task.h>
#include <native/event.h>
#include <native/types.h>
#include "delays.h"
#include "qospa.h" //as macros utilizadas em FD_STUB

//adicionado
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//ping
#include <netinet/ip_icmp.h> //struct icmp
#include <native/timer.h>
#define NOT_RECEIVED        0
#define RECEIVED            1
#define EXPLICIT_PING       0
#define EVENT_INIT          0x0           /* No flags present at init */
#define EVENT_MODE          EV_PRIO       /* Tasks will wait by priority order */
#define EVENT_WAIT_MASK     (0x1) /* List of monitored events */
#define EVENT_SIGNAL_MASK   (0x1)         /* List of events to send */

//variaveis de main que representam os objetos globais
extern ChannelManager *glob_manager_obj;
extern ChannelVerifier *glob_cv_obj;
extern MonitoringApplication *glob_ma_obj;
extern FailureDetector *glob_fd_obj;
extern Network *glob_network_obj;
extern MtSupporter *glob_mt_supporter_obj;
#ifdef HOLISTIC_QoSP
extern Monitor *glob_node_monitor_obj;
#endif

//ping
RT_EVENT    glob_pkt_rec_desc;       //utilizado pelos processos periodic_fd, fd_listener
RT_EVENT    glob_explicit_ping[MAX_FLOWS];      //utilizado pelos processos fd_stub, fd_listener
int glob_pkt_rec_status[2*MAX_ROUTERS+MAX_QOSP]; //armazena os status de recebimento das mensagens enviadas por FD
u_int16_t glob_expected_msg_id;                                        //armazena o id da proxima mensagem que eh esperada ser recebida por FD
u_int16_t glob_fd_msg_seq;                                              //identifica uma mensagem particular enviada pelo FD para um roteador

//variáveis utilizadas pelas tarefas de tempo real
int         glob_msg_id = 0;         //identifica uma mensagem particular de monitoramento enviada para um QoSPA
int         glob_ver_id = 0;         //identifica uma mensagem particular de monitoramento enviada para um QoSPA
int         glob_flows_msg_id[MAX_FLOWS];
int         glob_flows_ver_id[MAX_FLOWS];
int         glob_num_of_replies[MAX_FLOWS];     //número de QoSP para os quais foram enviadas mensagens de monitoramento
int         glob_qos_ok[MAX_FLOWS];             //informa se QoS de um canal esta ok
int         glob_remote_qos[MAX_FLOWS];         //informa se QoS de um canal remoto esta ok
qosp_lock_t glob_qos_mutex[MAX_FLOWS];          //utilizado pelos processos msg_listener, fd_stub, periodic_fd e pela funcao sendQoSEnqMsg
qosp_lock_t glob_remote_ver_mutex[MAX_FLOWS];   //utilizado pelos processos msg_listener e pela funcao sendRemoteVerMsg
qosp_cond_t glob_qos_cond[MAX_FLOWS];           //utilizado pelos processos msg_listener, fd_stub, periodic_fd e pela funcao sendQoSEnqMsg
qosp_cond_t glob_remote_ver_cond[MAX_FLOWS];    //utilizado pelos processos msg_listener e pela funcao sendRemoteVerMsg
extern qosp_sock_t sock_handler;       //socket para as mensagems UDP
extern qosp_sock_t sock_ping;          //socket para as mensagens ICMP
qosp_lock_t glob_first_reply_mutex;         //utilizado pelos processos fd_listener e periodic_fd
qosp_cond_t glob_first_reply_cond;          //utilizado pelos processos fd_listener e periodic_fd
qosp_lock_t glob_param_mutex;               //utilizado para evitar que o buffer utilizado
                                                //por verifier_skeleton seja sobrescrito

qosp_thread_t   glob_msg_listen_task;
qosp_thread_t   glob_period_fd_task;
qosp_thread_t   glob_fd_listen_task;

#ifdef HOLISTIC_QoSP
/******Variaveis globais do arquivo qospA_rt_tasks.c*******/
extern qosp_lock_t              glob_interested_mutex;           //utilizado pelos processos msg_listener_A e mon_skeleton
extern int                      glob_qt_interested;              //quantidade de modulos do QoSP que estao aguardando receber a resposta do monitoramento
extern list<struct sockaddr_in> glob_interested;//contem o endereco dos modulos do QoSP que estao aguardando receber a resposta do monitoramento
extern int                      glob_mon_skeleton_task_started;
extern char                     glob_mon_requests[MAX_REQUESTS_AT_A_TIME][MON_MSG_LENGTH];
/******Fim das variaveis globais do arquivo qospA_rt_tasks.c*******/
#endif

/*
 * Processo responsável por tratar uma requisição de remota de verificação de canal
 * e enviar a resposta para o processo solicitante
 * arg: ponteiro para uma estrutura do tipo received_msg_str
 */
void verifier_skeleton(void *arg) {
    received_msg_str *msg = (received_msg_str *)arg;
    qosp_thread_t thread_desc = msg->threadDesc;
    struct sockaddr_in  pxAddr, pyAddr, replyAddr;
    int size = sizeof(pxAddr), msgId;
    int result;
    char flowDesc;
    char buffer[VER_REPLY_MSG_LENGTH];

    flowDesc = (char)msg->msgData[1];
    UNMARSHALLING(&(pxAddr), &(msg->msgData[2]), size);
    UNMARSHALLING(&(pyAddr), &(msg->msgData[2+size]), size);
    UNMARSHALLING(&msgId, &(msg->msgData[2+(2*size)]), sizeof(msgId));

    glob_network_obj->copySockAddr(msg->addr, replyAddr);
    glob_mt_supporter_obj->lockRelease(glob_param_mutex);

    result = glob_cv_obj->qos(pxAddr, pyAddr);

    //preparando mensagem de resposta
    size = sizeof(msgId);
    buffer[0] = (char)VER_REPLY_MSG_ID;
    buffer[1] = (char)flowDesc;
    buffer[2] = (char)result;
    MARSHALLING(&(buffer[3]), &msgId, size);
    buffer[3+size] = '\0';
    glob_network_obj->sendMsg(sock_handler, replyAddr, buffer, (int)VER_REPLY_MSG_LENGTH);
    glob_mt_supporter_obj->threadDestroy(thread_desc);
}

/*
 * Processo responsavel por tratar a mensagem que informa que um determinado roteador
 * nao esta mais fornecendo a qos contratada.
 * arg: ponteiro para uma variavel do tipo received_msg_str.
 */
void notify_degrad(void *arg) {
    Router *router;
    received_msg_str *msg = (received_msg_str *)arg;
    qosp_thread_t threadDesc = msg->threadDesc;
    struct sockaddr_in faultyRouterAddr;
    glob_network_obj->copySockAddr(msg->addr, faultyRouterAddr);
    glob_mt_supporter_obj->lockRelease(glob_param_mutex);
    router = glob_manager_obj->getRouterByAddr(msg->addr);
    glob_ma_obj->notifyRouterFailure(*router);
    glob_mt_supporter_obj->threadDestroy(threadDesc);
}

/*
 * Processo responsavel por ficar escutando por mensagens relacionadas ao monitoramento.
 */
void msg_listener(void *arg) {
    int ret, msgType;
    int localMsgId, ok, flowDesc;
    received_msg_str receivedMsg;
#ifdef HOLISTIC_QoSP
    int msgId;
    qosp_thread_t monSkeletonParam;
#endif

    while(1) { //fica eternamente esperando por mensagens
        glob_mt_supporter_obj->lockAcquire(glob_param_mutex);
        ret = glob_network_obj->receiveMsg(sock_handler, receivedMsg.recMsg, receivedMsg.msgData, MAX_QOSP_REC_MSG_LENGTH, &(receivedMsg.addr), SOCK_DGRAM);
        glob_mt_supporter_obj->lockRelease(glob_param_mutex);
        if (ret > 0) {
            msgType = (int)receivedMsg.msgData[0];
            switch (msgType) {
                case MON_REPLY_MSG_ID:
                    UNMARSHALLING(&localMsgId, &(receivedMsg.msgData[3]), 0);
                    flowDesc = (int)receivedMsg.msgData[2];
                    if (localMsgId == (glob_flows_msg_id[flowDesc])) {//mensagens de monitoramento antigas serão desconsideradas
                        ok = (int)receivedMsg.msgData[1];
                        if (ok) {
                            if (!(--glob_num_of_replies[flowDesc])) {//todos os roteadores já foram monitorados e suas qos foram mantidas
                                glob_mt_supporter_obj->lockAcquire(glob_qos_mutex[flowDesc]);
                                glob_qos_ok[flowDesc]++;
                                if (glob_qos_ok[flowDesc] == 2)
                                    glob_mt_supporter_obj->condSignal(glob_qos_cond[flowDesc]);
                                glob_mt_supporter_obj->lockRelease(glob_qos_mutex[flowDesc]);
                            }
                        } else {
                            glob_mt_supporter_obj->lockAcquire(glob_qos_mutex[flowDesc]);
                            glob_qos_ok[flowDesc] = 0; //precisa inicializar ?
                            glob_mt_supporter_obj->condSignal(glob_qos_cond[flowDesc]);
                            glob_mt_supporter_obj->lockRelease(glob_qos_mutex[flowDesc]);
                        }
                    }
                    break;
                case VER_REPLY_MSG_ID:
                    UNMARSHALLING(&localMsgId, &(receivedMsg.msgData[3]), 0);
                    flowDesc = (int)receivedMsg.msgData[1];
                    if (localMsgId == (glob_flows_ver_id[flowDesc])) {//mensagens de monitoramento antigas serão desconsideradas
                        glob_mt_supporter_obj->lockAcquire(glob_remote_ver_mutex[flowDesc]);
                        glob_remote_qos[flowDesc] = (int)receivedMsg.msgData[2];
                        glob_mt_supporter_obj->condSignal(glob_remote_ver_cond[flowDesc]);
                        glob_mt_supporter_obj->lockRelease(glob_remote_ver_mutex[flowDesc]);
                    }
                    break;
                case VER_MSG_ID:
                    glob_mt_supporter_obj->lockAcquire(glob_param_mutex);
                    glob_mt_supporter_obj->threadCreate(receivedMsg.threadDesc, QOSP_XENOMAI_DOMAIN, (void *)&verifier_skeleton, (void *)&receivedMsg, VERIFIER_SKEL_PRIO);
                    break;
                case QOS_DEGRAD_MSG_ID:
                    glob_mt_supporter_obj->lockAcquire(glob_param_mutex);
                    glob_mt_supporter_obj->threadCreate(receivedMsg.threadDesc, QOSP_LINUX_DOMAIN, (void *)&notify_degrad, (void *)&receivedMsg, -1);
                    break;
#ifdef HOLISTIC_QoSP
                case MON_MSG_ID:
                    glob_mt_supporter_obj->lockAcquire(glob_interested_mutex);
                    glob_mon_requests[glob_qt_interested][2] = receivedMsg.msgData[2];
                    UNMARSHALLING(&msgId, &(receivedMsg.msgData[3]), sizeof(int));
                    MARSHALLING(&(glob_mon_requests[glob_qt_interested][3]), &msgId, sizeof(int));
                    glob_qt_interested++;
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
#endif
            }
        }
    }
}

/*
 * Processo responsavel por enviar um ping para um módulo do QoSP, para verificar se o mesmo está vivo.
 * Ajuda a descobrir a QoS final de um canal.
 * arg: ponteiro para uma variavel do tipo explicit_ping_str.
 */
void checkQoSP(void *arg) {
    unsigned long maskRet;
    int err;
    explicit_ping_str *param = (explicit_ping_str *)arg;
    QoSP *qosp = param->qosp;
    int flowDesc = param->flowDesc;

    glob_network_obj->ping(&sock_ping, qosp->getAddress(), flowDesc, (u_int16_t)EXPLICIT_PING);
    #ifdef DELAY
    err = rt_event_wait(&(glob_explicit_ping[flowDesc]), EVENT_WAIT_MASK, &maskRet, EV_ANY, PING_QOSP_DELAY);
    #else
    err = rt_event_wait(&glob_explicit_ping[flowDesc], EVENT_WAIT_MASK, &maskRet, EV_ANY, glob_fd_obj.delay->delay(*qosp));
    #endif
    rt_event_clear(&(glob_explicit_ping[flowDesc]), EVENT_WAIT_MASK, NULL);
    glob_mt_supporter_obj->lockAcquire(glob_qos_mutex[flowDesc]);
    if (err == -ETIMEDOUT) {//a resposta do ping não chegou a tempo, logo o QoSP falhou
        glob_qos_ok[flowDesc] = 0; //precisa inicializar ?
        glob_mt_supporter_obj->condSignal(glob_qos_cond[flowDesc]);
    }
    else {
        glob_qos_ok[flowDesc]++;
        if (glob_qos_ok[flowDesc] == 2)
            glob_mt_supporter_obj->condSignal(glob_qos_cond[flowDesc]);
    }
    glob_mt_supporter_obj->lockRelease(glob_qos_mutex[flowDesc]);
}

/*
 * Processo do FD que executa periodicamente a detecção de falhas dos componentes do QoSPM.
 * Simula o envio e recepção de mensagens Are you alive e I am alive.
 */
void periodic_fd(void *arg) {
    unsigned long maskRet;
    int err;
    long long msgSentTime[2*MAX_ROUTERS+MAX_QOSP];
    RTIME now;
    QoSPEntity *element;
    fd_element str;
    set<fd_element, FDElementsSortCriterion>::iterator elements, end;
    list<int>::iterator flows, flowsEnd;
    QoSP *qosp;
    QoSPA *agent;
    Router *router;
    struct sockaddr_in addr;
    int qtElements;
    u_int16_t index;
    int qtDependents, flowDesc;

    rt_task_set_periodic(NULL, TM_NOW, FD_MON_PERIOD);
    while (1) {
        glob_fd_obj->releaseElements();
        rt_task_wait_period(NULL);//a cada FD_MON_PERIOD um conjunto de elementos é pingado
        qtElements = glob_fd_obj->getElements(elements, end);
        glob_expected_msg_id = 0;
        if (!qtElements) //nao existe elementos para detectar falhas
            continue;
         glob_mt_supporter_obj->lockAcquire(glob_first_reply_mutex);
         glob_fd_msg_seq++;//eu incremento antes pois as respostas podem chegar antes do loop finalizar
         for (index = 0; elements!= end; elements++) {
             glob_pkt_rec_status[index] = NOT_RECEIVED;
             str = *elements;
             element = str.element;
             msgSentTime[index] = rt_timer_read();
             addr = element->getAddress();
             glob_network_obj->ping(&sock_ping, addr, (glob_fd_msg_seq-1), (index+1));
         }

         glob_mt_supporter_obj->condWait(glob_first_reply_cond, glob_first_reply_mutex, str.delay);
         rt_event_clear(&glob_pkt_rec_desc, EVENT_WAIT_MASK, NULL);
         glob_mt_supporter_obj->lockRelease(glob_first_reply_mutex);

         for (; elements != end; elements++) {
             str = *elements;
             element = str.element;
             if (glob_pkt_rec_status[index] == NOT_RECEIVED) {
                 if (index == glob_expected_msg_id) {//a próxima mensagem que era esperada não chegou no tempo esperado, logo o elemento falhou
                     glob_expected_msg_id++;
                     qosp = dynamic_cast<QoSP *>(element);
                     if (qosp) { //falhou um QoSP
                         if (element->isBeingMon()) {//verifier_stub não receberá uma das respostas, logo ele deve ser notificado
                             qtDependents = element->getDependents(flows, flowsEnd);

                             for (; flows != flowsEnd; flows++) {
                                 flowDesc = *flows;
                                 glob_mt_supporter_obj->lockAcquire(glob_remote_ver_mutex[flowDesc]);
                                 glob_remote_qos[flowDesc] = UNTIMELY;
                                 glob_mt_supporter_obj->condSignal(glob_remote_ver_cond[flowDesc]);
                                 glob_mt_supporter_obj->lockRelease(glob_remote_ver_mutex[flowDesc]);
                             }
                             element->releaseDependents();
                         } else
                             glob_ma_obj->notifyQoSPFailure(*qosp);
                     } else { //ROUTER_ELEM ou QOSPA_ELEM
                         if (element->isBeingMon()) {//mon_stub não receberá uma das respostas, logo ele deve ser notificado
                             qtDependents = element->getDependents(flows, flowsEnd);
                             for (;flows != flowsEnd; flows++) {
                                 flowDesc = *flows;
                                 glob_mt_supporter_obj->lockAcquire(glob_qos_mutex[flowDesc]);
                                 glob_qos_ok[flowDesc] = 0;
                                 glob_mt_supporter_obj->condSignal(glob_qos_cond[flowDesc]);
                                 glob_mt_supporter_obj->lockRelease(glob_qos_mutex[flowDesc]);
                             }
                             element->releaseDependents();
                         }

                         router = dynamic_cast<Router *>(element);
                         if (!router) {
                             agent = dynamic_cast<QoSPA *>(element);
                             router = agent->getRouter();
                         }
                         glob_ma_obj->notifyRouterFailure(*router);
                     }
		     if (glob_expected_msg_id == qtElements)
		     	continue;
		     now = rt_timer_read();
                     err = rt_event_wait(&glob_pkt_rec_desc, EVENT_WAIT_MASK, &maskRet, EV_ANY, (msgSentTime[glob_expected_msg_id] + str.delay) - now);
                     rt_event_clear(&glob_pkt_rec_desc, EVENT_WAIT_MASK, NULL);
                 } else {//o alarme deve ser programado para a possível próxima mensagem a ser recebida
                     glob_expected_msg_id = index;
                     now = rt_timer_read();
                     err = rt_event_wait(&glob_pkt_rec_desc, EVENT_WAIT_MASK, &maskRet, EV_ANY, (msgSentTime[index] + str.delay) - now);
                     rt_event_clear(&glob_pkt_rec_desc, EVENT_WAIT_MASK, NULL);
                 }
             }
         }
     }
}

/*
 * Processo responsavel por ficar escutando por respostas do ping do FD e notificar
 * chegada das mensagens. Alem disso, ela tambem eh responsavel por notificar a
 * chegada da mensagem enviada pelo processo checkQoSP.
 */
void fd_listener(void *arg) {
    int ret;
    struct icmp *icp_ptr;
    char buf[PING_REPLY_MSG_LENGTH];

    while(1) { //fica eternamente esperando por mensagens
        ret = glob_network_obj->receivePingReply(&sock_ping, buf, icp_ptr, PING_REPLY_MSG_LENGTH);
        if (ret > 0) {
            if (icp_ptr->icmp_type == ICMP_ECHOREPLY) {
                if (icp_ptr->icmp_id == EXPLICIT_PING) { //mensagem referente a solicitação explícita do ping
                        rt_event_signal(&(glob_explicit_ping[icp_ptr->icmp_seq]), EVENT_SIGNAL_MASK);
                } else { //mensagem referente ao envio automático (periódico) do ping
                    if (icp_ptr->icmp_seq == (glob_fd_msg_seq-1)) {//esta errado
                        glob_pkt_rec_status[icp_ptr->icmp_id-1] = RECEIVED;
                        if ((icp_ptr->icmp_id-1) == glob_expected_msg_id) {//a mensagem que estava sendo esperada chegou antes do timeout
                            if (icp_ptr->icmp_id > 1)
                                rt_event_signal(&glob_pkt_rec_desc, EVENT_SIGNAL_MASK);
                            else {//primeira mensagem enviada pelo FD
                                glob_mt_supporter_obj->lockAcquire(glob_first_reply_mutex);
                                glob_mt_supporter_obj->condSignal(glob_first_reply_cond);
                                glob_mt_supporter_obj->lockRelease(glob_first_reply_mutex);
                            }
                        }
                    }
                }
            }
        }
    }
}

/*
 * Inicializa tudo que eh necessario para ser utilizado pelas tarefas do QoSP,
 * incluindo as proprias tarefas.
 */
void qosp_tasks_init() {
    int index;

    glob_fd_msg_seq = 0;
    for (index = 0; index < MAX_FLOWS; index++) {
        rt_event_create(&(glob_explicit_ping[index]), NULL, EVENT_INIT, EVENT_MODE);

        glob_mt_supporter_obj->mutexCreate(glob_remote_ver_mutex[index], QOSP_XENOMAI_DOMAIN);
        glob_mt_supporter_obj->condCreate(glob_remote_ver_cond[index], QOSP_XENOMAI_DOMAIN);

        glob_mt_supporter_obj->mutexCreate(glob_qos_mutex[index], QOSP_XENOMAI_DOMAIN);
        glob_mt_supporter_obj->condCreate(glob_qos_cond[index], QOSP_XENOMAI_DOMAIN);
    }
    rt_event_create(&glob_pkt_rec_desc, NULL, EVENT_INIT, EVENT_MODE);

    index = glob_mt_supporter_obj->mutexCreate(glob_first_reply_mutex, QOSP_XENOMAI_DOMAIN);
    index = glob_mt_supporter_obj->condCreate(glob_first_reply_cond, QOSP_XENOMAI_DOMAIN);

    glob_mt_supporter_obj->mutexCreate(glob_param_mutex, QOSP_LINUX_DOMAIN);

    glob_mt_supporter_obj->threadCreate(glob_msg_listen_task, QOSP_LINUX_DOMAIN, (void *)&msg_listener, NULL, -1);
    glob_mt_supporter_obj->threadCreate(glob_period_fd_task, QOSP_XENOMAI_DOMAIN, (void *)&periodic_fd, NULL, PERIODIC_FD_PRIO);
    glob_mt_supporter_obj->threadCreate(glob_fd_listen_task, QOSP_XENOMAI_DOMAIN, (void *)&fd_listener, NULL, FD_LISTENER_PRIO);
}

/*
 * Finaliza os recursos a ser utilizado pelas tarefas do QoSP.
 */
void qosp_tasks_finish() {
    int index;

    for (index = 0; index < MAX_FLOWS; index++) {
        rt_event_delete(&(glob_explicit_ping[index]));

        glob_mt_supporter_obj->mutexDestroy(glob_remote_ver_mutex[index]);
        glob_mt_supporter_obj->condDestroy(glob_remote_ver_cond[index]);

        glob_mt_supporter_obj->mutexDestroy(glob_qos_mutex[index]);
        glob_mt_supporter_obj->condDestroy(glob_qos_cond[index]);
    }
    rt_event_delete(&glob_pkt_rec_desc);
    glob_mt_supporter_obj->mutexDestroy(glob_first_reply_mutex);
    glob_mt_supporter_obj->condDestroy(glob_first_reply_cond);

    glob_mt_supporter_obj->mutexDestroy(glob_param_mutex);

    glob_mt_supporter_obj->threadDestroy(glob_msg_listen_task);
    glob_mt_supporter_obj->threadDestroy(glob_period_fd_task);
    glob_mt_supporter_obj->threadDestroy(glob_fd_listen_task);
}

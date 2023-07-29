#include "qos_monitor.h"
#include "monitoring.h"
#include "tools.h"//MARSHALLING
#include "qospm_protocol.h"
#include "qosp_tasks.h"
#include "mt_support.h"
#include "network.h"

extern LocalHost *glob_local_host_obj;
extern MtSupporter *glob_mt_supporter_obj;
extern Network *glob_network_obj;

extern int         glob_msg_id;
extern int         glob_flows_msg_id[MAX_FLOWS];
extern int         glob_num_of_replies[MAX_FLOWS];
extern int         glob_qos_ok[MAX_FLOWS];
extern qosp_lock_t glob_qos_mutex[MAX_FLOWS];
extern qosp_cond_t glob_qos_cond[MAX_FLOWS];

extern qosp_sock_t sock_handler;

QoSMonitor::QoSMonitor(MonitoringApplication *consultant, int ma_desc) {
    this->flowBehaviour = new Flow(ma_desc);
    this->qosConsultant = consultant;
}

/*
 * Retorna a QoS de um canal de comunicacao para uma determinada thread de execucao,
 * ou seja, esta funcao pode ser chamada concorrentemente.
 * channel: canal cuja QoS sera retornada.
 * retorno: QoS do canal.
 */
int QoSMonitor::qos(Channel &channel) {
    int qt_addr;
    CommunicationEntity *temp_py;
    QoSP *qosp;
    list<Router *> routers;

    qt_addr = channel.getRouters(routers);

    if (!glob_local_host_obj->whichRemoteProcess(channel, temp_py))
        return 1;//TIMELY

    qosp = this->qosConsultant->getLocalQoSP(*temp_py);
    return this->sendQoSEnqMsg(qt_addr, routers, qosp);
}


/*
 * Envia mensagens para os agentes do QoSP solicitando a qos de um canal
 * Envia também um ping para o outro módulo do QoSP, para verificar se o mesmo ainda está vivo.
 * qt_routers: numero de roteadores para os quais as mensagens serao enviadas.
 * routers: lista dos roteadores para os quais a mensagem sera enviada.
 * qosp: modulo do QoSP a ser pingado.
 * retorno: QoS do canal.
 */
int QoSMonitor::sendQoSEnqMsg(int qt_routers, list<Router *> &routers, QoSP *qosp) {
    int dependents_desc[MAX_ROUTERS];//, qt_routers;//, qt_addr = elements->getQtAddr(elements);
    char buffer[MON_MSG_LENGTH];
    Router *router;
    QoSPA *monitor;
    int flow_desc = this->flowBehaviour->getFlowDesc();
    int index, max_attempt = MAX_MON_ATTEMPT;
    long timeout = MON_TIMEOUT;
    int ret, qos = UNTIMELY; //UNTIMELY
    explicit_ping_str param;
    qosp_thread_t explicit_ping_task;
    list<Router *>::iterator routers_iter;

    glob_qos_ok[flow_desc] = 0;
    buffer[0] = (char)MON_MSG_ID;
    buffer[1] = 1;
    buffer[2] = (char)flow_desc;
    MARSHALLING(&(buffer[3]), &glob_msg_id, 0);
    glob_flows_msg_id[flow_desc] = glob_msg_id;
    glob_msg_id++;

    glob_num_of_replies[flow_desc] = qt_routers;

    param.qosp = qosp;
    param.flowDesc = flow_desc;
    glob_mt_supporter_obj->threadCreate(explicit_ping_task, QOSP_XENOMAI_DOMAIN, (void *)&checkQoSP, (void *)&param, FD_STUB_PRIO);

    glob_mt_supporter_obj->lockAcquire(glob_qos_mutex[flow_desc]);
    while (max_attempt--) {/*mesmo considerando o canal confiavel, na pratica as coisas
     nao sao bem assim. Mesmo roteadores e agentes nao falhando, as mensagens podem nao chegar.
     Por isso eh necessario estabelecer timeout e ficar reenviando as mensagens.*/
        routers_iter = routers.begin();
        for (index = 0; index < qt_routers; index++) {
            router = *routers_iter;
            dependents_desc[index] = router->setDependent(*(this->flowBehaviour));
            monitor = router->getMonitor();
            glob_network_obj->sendMsg(sock_handler, monitor->getAddress(), buffer, (int)MON_MSG_LENGTH);
        }
        //como calcular o timeout ?
        ret = glob_mt_supporter_obj->condWait(glob_qos_cond[flow_desc], glob_qos_mutex[flow_desc], timeout);
        if (ret == -1) {
            MARSHALLING(&(buffer[3]), &glob_msg_id, 0);
            glob_flows_msg_id[flow_desc] = glob_msg_id;
            glob_msg_id++;
        } else {
            max_attempt = 0;
            if (glob_qos_ok[flow_desc])//tá tudo blz
                qos = 1; //TIMELY
        }
    }
    routers_iter = routers.begin();
    for (index = 0; index < qt_routers; index++) {
        router = *routers_iter;
        router->releaseDependent(dependents_desc[index]);
    }
    glob_mt_supporter_obj->lockRelease(glob_qos_mutex[flow_desc]);
    return qos;
}

QoSMonitor::~QoSMonitor() {
    delete this->flowBehaviour;
}

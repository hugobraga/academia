#include "verifier.h"
#include "channelmanager.h"
#include "conf.h"
#include <arpa/inet.h> //htons
#include <native/types.h>
#include <native/task.h>
#include <native/timer.h>
#include "qospm_protocol.h"
#include "monitoring.h"
#include "tools.h"//MARSHALLING
#include "mt_support.h"
#include "network.h"

using namespace std;

extern ChannelManager   *glob_manager_obj;
extern MtSupporter      *glob_mt_supporter_obj;
extern FailureDetector  *glob_fd_obj;
extern Network          *glob_network_obj;

extern int              glob_msg_id;
extern int              glob_ver_id;
extern int              glob_flows_ver_id[MAX_FLOWS];
extern int              glob_flows_ver_id[MAX_FLOWS];
extern qosp_lock_t      glob_remote_ver_mutex[MAX_FLOWS];
extern qosp_cond_t      glob_remote_ver_cond[MAX_FLOWS];
extern int              glob_remote_qos[MAX_FLOWS];

extern qosp_sock_t      sock_handler;

/*
 * Envia uma mensagem de verificacao remota para um QoSP.
 * pxAddr, pyAddr: enderecos dos processos que fazem parte do canal a ser verificado.
 * retorno: QoS do canal.
 */
int Verifier::sendRemoteVerMsg(struct sockaddr_in pxAddr, struct sockaddr_in pyAddr) {
    int dependentDesc;
    QoSP *remoteHost;
    int ret, qos = UNTIMELY;
    char buffer[VER_MSG_LENGTH];
    int flowDesc = this->flowBehv->getFlowDesc();
    int maxAttempt = MAX_MON_ATTEMPT;
    int addrSize;
    long timeout = (long)VER_TIMEOUT;

    buffer[0] = (char)VER_MSG_ID;
    buffer[1] = (char)flowDesc;
    addrSize = sizeof(pxAddr);
    MARSHALLING(&(buffer[2]), &(pxAddr), addrSize);
    MARSHALLING(&(buffer[2+addrSize]), &(pyAddr), addrSize);
    MARSHALLING(&(buffer[2+(2*addrSize)]), &glob_ver_id, sizeof(glob_ver_id));
    pyAddr.sin_port = htons(QoSP_PORT);
    glob_flows_ver_id[flowDesc] = glob_ver_id;
    glob_ver_id++;

    glob_mt_supporter_obj->lockAcquire(glob_remote_ver_mutex[flowDesc]);
    glob_fd_obj->registerRemoteQoSP(pyAddr, remoteHost);
    dependentDesc = remoteHost->setDependent(*(this->flowBehv));

    while (maxAttempt--) {/*mesmo considerando o canal confiavel, na pratica as coisas
     nao sao bem assim. Mesmo roteadores e agentes nao falhando, as mensagens podem nao chegar.
     Por isso eh necessario estabelecer timeout e ficar reenviando as mensagens.*/
        glob_network_obj->sendMsg(sock_handler, pyAddr, buffer, (int)VER_MSG_LENGTH);
        ret = glob_mt_supporter_obj->condWait(glob_remote_ver_cond[flowDesc], glob_remote_ver_mutex[flowDesc], timeout);
        if (ret == -1) {
            MARSHALLING(&(buffer[2+(2*addrSize)]), &glob_ver_id, 0);
            glob_flows_ver_id[flowDesc] = glob_ver_id;
            glob_ver_id++;
        } else {
            maxAttempt = 0;
            qos = glob_remote_qos[flowDesc];
        }
    }
    remoteHost->releaseDependent(dependentDesc);
    glob_fd_obj->unRegisterRemoteQoSP(remoteHost);
    glob_mt_supporter_obj->lockRelease(glob_remote_ver_mutex[flowDesc]);
    return qos;
}

Verifier::Verifier(MonitoringApplication *consultant, TrafficListener *listener, int cvDesc) {
    this->flowBehv = new Flow(cvDesc);
    this->qosConsultant = consultant;
    this->listener = listener;
}

/*
 * Verifica um canal para uma determinada thread de execucao, ou seja, esta funcao
 * pode ser chamada concorrentemente.
 * A verificação consiste em verificar a QoS do canal e se houve
 * troca de mensagens pelo canal durante um intervalo de tempo.
 * p1_addr e p2_addr: processos que compoem o canal a ser verificado.
 * retorno: TIMELY caso a QoS do canal seja TIMELY e houve troca de trafego no canal.
 * UNTIMELY caso contrario.
 */
int Verifier::qos(struct sockaddr_in pxAddr, struct sockaddr_in pyAddr) {
    int ret;
    RTIME startTime;
    CommunicationEntity px(pxAddr), py(pyAddr);
    Channel *channel;

    if (!(glob_manager_obj->getChannel(px, py, channel)))//deve ser feita uma consulta remota ao QoSP
        return this->sendRemoteVerMsg(pxAddr, pyAddr);
    //caso o canal tenha tido sua QoS degradada entre duas chamadas da funcao qos_MA
    if (channel->getQoS() == 0) //UNTIMELY
        return 0;
    ret = this->qosConsultant->qos(*channel);
    if (ret == 0)
        return ret;
    else {
        startTime = rt_timer_read();
        rt_task_sleep((RTIME)CHANNEL_DORMANCY_PERIOD);
        if (this->listener->lastMessageTime(channel->getListenerDesc()) < startTime)
            return 0;
    }
    return 1;
}

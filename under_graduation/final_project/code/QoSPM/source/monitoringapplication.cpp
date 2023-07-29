#include "monitoringapplication.h"
#include <algorithm>
#include <netinet/in.h> //struct sockadd_in
#include <arpa/inet.h> //htons
#include "local_host.h"
#include "qos_monitor.h"
#include "domain.h"
#include "channelmanager.h"
#include "network.h"
#include "qospm_protocol.h"
#include "conf.h"

extern MtSupporter *glob_mt_supporter_obj;
extern ChannelManager *glob_manager_obj;
extern Network *glob_network_obj;

extern qosp_sock_t sock_handler;

/*
 * Retorna um descritor valido para um QoSMonitor.
 * retorno: -1 - caso nao seja possivel retornar um descritor valido,
 * caso contrario retorna um descritor valido.
 * Este descritor e necessario para a funcao releaseMonDesc.
 */
int MonitoringApplication::getMonitorDesc() {
    vector<bool>::iterator iter, end;
    int index;
    if (!(glob_mt_supporter_obj->lockAcquire(this->monitorsLock)))
        return -1;
    for (index = 0, iter = this->monitorsDesc->begin(), end = this->monitorsDesc->end();
    iter != end; iter++, index++) {
        if ((*iter) == false) {
            this->monitorsDesc->insert(iter, true);
            return index;
        }
    }
    if (!(glob_mt_supporter_obj->lockRelease(this->monitorsLock)))
        return -1;
    return -1;
}

/*
 * Libera um descritor alocado para um QoSMonitor.
 * desc: descritor a ser liberado. Esta descritor
 * corresponde ao retorno da funcao getMonitorDesc.
 */
void MonitoringApplication::releaseMonDesc(int desc) {
    this->monitorsDesc->at(desc) = false;
}

/*
 * Envia mensagens de subscrição (ou cancelamento) aos QoSPA para o recebimento
 * automático de mensagens de monitoramento.
 * monitor: agente para o qual a mensagem sera enviada.
 * msg: mensagem a ser enviada.
 */
void MonitoringApplication::sendSubsMsg(QoSPA& monitor, char* msg) {
    glob_network_obj->sendMsg(sock_handler, monitor.getAddress(), msg, (int)SUBS_MSG_LENGTH);
}

/*
 * Retorna um QoSP (CommunicationEntity) com o mesmo IP/porta de um processo (CommunicationEntity).
 * Observe que qosp não é uma referência mantida por MonitoringApplication.
 * Ela é uma referência qualquer externa à chamada da função, que aponta para um
 * "objeto" com um mesmo endereço IP de um módulo do QoSP que se localiza no mesmo host de process.
 * process: processo que tera seu endereco socket clonado.
 * qosp: contera o mesmo endereco de process.
 */
void MonitoringApplication::getQoSPByProc(CommunicationEntity &process, QoSP &qosp) {
    uint16_t port = QoSP_PORT;
    struct sockaddr_in  addr;

    addr = process.getAddress();
    addr.sin_port = htons(port);
    qosp.setAddress(addr);
}

/*
 * Incrementa o numero de canais timely que passam pelo mesmo host de qosp.
 * qosp: qosp cujo host tera o numero de canais TIMELY incrementado.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int MonitoringApplication::incrementTimelyPerQoSP(QoSP* &qosp) {
    monitored_qosp str;
    list<monitored_qosp>::iterator element;

    if (!(glob_mt_supporter_obj->lockAcquire(this->lockDesc)))
        return 0;

    element = find_if(this->qospList.begin(), this->qospList.end(), bind1st(MAQoSPMatchCriterion(), *qosp));
    if (element != this->qospList.end()) {
        str = *element;
        str.qosp_dependents++;
        this->qospList.insert(element, str);
    } else {
        str.qosp_dependents = 1;
        str.qosp = new QoSP(qosp->getAddress());
        this->qospList.push_back(str);
    }
    qosp = str.qosp;
    return 1;

   if (!(glob_mt_supporter_obj->lockRelease(this->lockDesc)))
        return 0;
    return 1;
}

/*
 * Decrementa o número de canais timely que passam pelo mesmo host de qosp.
 * Esta funcao eh necessária para que MA saiba quando um módulo do QoSP não deve ser mais monitorado.
 * qosp: qosp cujo host tera o numero de canais TIMELY decrementado.
 * retorno: 1 caso o numero de canais TIMELY que passavam pelo host era 1, 0 caso contrario.
 */
int MonitoringApplication::decrementTimelyPerQoSP(QoSP* qosp) {
    monitored_qosp str;
    list<monitored_qosp>::iterator element;

    if (!(glob_mt_supporter_obj->lockAcquire(this->lockDesc)))
        return 0;

    element = find_if(this->qospList.begin(), this->qospList.end(), bind1st(MAQoSPMatchCriterion(), *qosp));
    str = *element;
    str.qosp_dependents--;
    if (str.qosp_dependents == 0) //está no hora de retirar o elemento da lista
        this->qospList.erase(element);
    else
        this->qospList.insert(element, str);

   if (!(glob_mt_supporter_obj->lockRelease(this->lockDesc)))
        return 0;
    return 1;
}

/*
 * Verifica se existe apenas um canal que passa pelo host que hospeda qosp.
 * Caso seja verdade, qosp apontará para o verdadeiro "objeto" que representa o módulo
 * QoSP localizado no host mencionado acima.
 * Isto é importante para saber quando um determinado módulo do QoSP não deve mais ser monitorado pelo FD.
 * qosp: contera a verdadeira referencia do qosp cujo host possui apenas um canal TIMELY.
 * retorno: true caso apenas um canal TIMELY passe pelo host, false caso contrario.
 */
bool MonitoringApplication::oneChannelPerQoSP(QoSP* &qosp) {
    list<monitored_qosp>::iterator element;
    monitored_qosp str;

    element = find_if(this->qospList.begin(), this->qospList.end(), bind1st(MAQoSPMatchCriterion(), *qosp));
    str = *element;
    if (str.qosp_dependents == 1) {
        qosp = str.qosp;
        return true;
    }
    return false;
}

/*
 * Verifica se um conjunto de canais esta contido em outro.
 * sub: subconjunto de canais que sera verificado se esta contido.
 * set: conjunto de canais que sera verificado se contem outro vetor.
 * retorno: true caso um conjunto esteja contido em outro, false caso contrario.
 */
bool MonitoringApplication::contains(list<Channel *> &set, list<Channel *> &sub) {
    int flag;
    list<Channel *>::iterator setItr, setEnd, subItr, subEnd;
    Channel *setChannel, *subChannel;
    
    for (subItr = sub.begin(), subEnd = sub.end(); subItr != subEnd; subItr++) {
        subChannel = *subItr;
        for (flag = 0, setItr = set.begin(), setEnd = set.end(); setItr != setEnd; setItr++) {
            setChannel = *setItr;
            if (subChannel->equal(*setChannel))
                flag = 1;
                break;
            }
        if (!flag)
            return false;
    }
    return true;
}

void MonitoringApplication::sendSubsMsg(QoSPA *monitor, char *msg) {
    glob_network_obj->sendMsg(sock_handler, monitor->getAddress(), msg, (int)SUBS_MSG_LENGTH);
}

/*
 * Construtor da classe MonitoringApplication.
 * fd: ponteiro para a instancia do detector de falhas.
 * local_host: ponteiro para a instancia de LocalHost.
 */
MonitoringApplication::MonitoringApplication(FailureDetector *fd, LocalHost *local_host) {
    this->failureDetector = fd;
    this->localHost = local_host;
    this->monitorsDesc = new vector<bool>(MAX_FLOWS, false);
    glob_mt_supporter_obj->mutexCreate(this->lockDesc, QOSP_XENOMAI_DOMAIN);
    glob_mt_supporter_obj->mutexCreate(this->monitorsLock, QOSP_XENOMAI_DOMAIN);
}

/*
 * Retorna a QoS de um canal de comunicacao.
 * channel: canal cuja QoS sera retornada.
 * retorno: QoS do canal (1 - TIMELY, 2 - UNTIMELY).
 */
int MonitoringApplication::qos(Channel &channel) {
    QoSMonitor *monitor;
    int ret, descPos;
    
    //caso o canal tenha tido sua QoS degradada entre duas chamadas da funcao qos_MA
    if (channel.getQoS() == 0)
        return 0;
    
    descPos = this->getMonitorDesc();
    //criando instancia para tratar a chamada
    monitor = new QoSMonitor(this, descPos);
    ret = monitor->qos(channel);
    
    delete monitor;
    this->releaseMonDesc(descPos);
    return ret;
}

/*
 * Notifica que um canal tornou-se timely. Esta função é chamada pelo Mediator.
 * channel: canal cuja QoS tornou-se TIMELY.
 */
void MonitoringApplication::notifyTimelyChannel(Channel &channel) {
    list<Router *> routers;
    Router *router;
    list<Router *>::iterator iter;
    CommunicationEntity *py;
    QoSPA *monitor;
    int index, qtRouters;
    char buffer[SUBS_MSG_LENGTH];
    struct sockaddr_in addr;
    QoSP qosp(addr), *localQoSP;

    glob_manager_obj->notifyQoSUpgrade(channel);
    qtRouters = channel.getRouters(routers);
    //caso os dois processos do canal estejam localizados neste host, nao ha nada a fazer
    if (!(this->localHost->whichRemoteProcess(channel, py)))
        return;

    //capturando os agentes do qosp que monitoram os roteadores de channel
    for (index = 0, iter = routers.begin(); index < qtRouters; index++, iter++) {
        //todo roteador que está sendo monitorado automaticamente por um nodeMonitor deve ser monitorado pelo FD.
        //Todo roteador que está sendo monitorado pelo //FD deve ser monitorado automaticamente por um nodeMonitor
        router = *iter;
        if (!(this->failureDetector->isElementBeingMon(router))) {
            monitor = router->getMonitor();
            buffer[0] = (char)SUBS_MSG_ID;
            this->sendSubsMsg(*monitor, buffer);
            this->failureDetector->registerElement(router);
        }
    }

    this->getQoSPByProc(*py, qosp);

    localQoSP = &qosp;
    //informa que mais um canal (ou o primeiro canal) está passando pelo mesmo host de elements.qospM.
    //Isto é importante para saber o momento em que um módulo do QoSP não deve ser mais monitorado.
    this->incrementTimelyPerQoSP(localQoSP);
    //se o módulo do QoSP ainda não estiver sendo monitorado, a partir de agora ele deve ser monitorado pelo FD
    if (!(this->failureDetector->isElementBeingMon(localQoSP))) {
        this->failureDetector->registerElement(localQoSP);
        //cout << "depois de registrar modulo do qosp\n";
    }
}

/*
 * Notifica sobre a falha de um módulo do QoSP. MA notifica Mediator sobre a degradação de
 * todos os canais timely cujos processos residam no mesmo host deste QoSP que falhou.
 * qosp: elemeno que falhou.
 */
void MonitoringApplication::notifyQoSPFailure(QoSP &qosp) {
    Channel *channel;
    CommunicationEntity *proc;
    vector<Channel *>::iterator channels, end;
    list<Channel *> downChannels;

    glob_manager_obj->getChannels(channels, end);
    //está verificando quais canais serão afetados devido a falha do qosp.
    //Notifica os clientes cujo canal teve sua qos degradada.
    for (; channels != end; channels++) {//isto poderia ser otimizado para percorrer apenas os canais TIMELY
        channel = *channels;
        if (channel->getQoS() == 1) {//TIMELY
            this->localHost->whichRemoteProcess(*channel, proc);
            if (proc->equalIP(qosp))//py de channels[a] passa a ser considerado untimely
                downChannels.push_back(channel);
        }
    }

    this->mediator->notifyChannelsDegrad(downChannels);
}

/*
 * Notifica sobre a falha de um roteador. MA notifica Mediator sobre a degradação
 * da QoS de todos os canais timely que passam por este roteador.
 * faultyRouter: roteador que falhou.
 */
void MonitoringApplication::notifyRouterFailure(Router& faultyRouter) {
    list<Channel *> downChannels;

    glob_manager_obj->getTChannelsPerRouter(faultyRouter, downChannels);
    this->mediator->notifyChannelsDegrad(downChannels);
}

/*
 * Notifica sobre a degradaçao de um conjunto de canais. Solicita o cancelamento
 * da inscrição do recebimento de informações de monitoramento. Verifica todos os roteadores cujas
 * informações não são mais necessárias (não existe mais canal timely passando por aquele roteador),
 * cancelando a inscrição do recebimento das mesmas.
 * downChannels: Contem a referencia de todos os canais cuja QoS deve ser degradada.
 */
void MonitoringApplication::notifyChannelsDegrad(list<Channel *> &downChannels) {
    Channel *channel;
    Router *router;
    vector<Router *>::iterator routers, endRouters;
    list<Channel *>::iterator downIter, endDown;
    list<Channel *> timelyChannels;
    char *buffer;
    CommunicationEntity *py;
    QoSP *qospPtr;
    QoSPA *nodeMonitor;

    glob_manager_obj->getRouters(routers, endRouters);

    downIter = downChannels.begin();
    endDown = downChannels.end();
    //removendo os módulos do QoSP que não precisam mais ser monitorados
    //informa ao manager sobre a degradação da qos dos canais
    for (; downIter != endDown; downIter++) {
        channel = *downIter;
        if (!(this->localHost->whichRemoteProcess(*channel, py)))
            continue;

        this->getQoSPByProc(*py, *qospPtr);

        if (this->oneChannelPerQoSP(qospPtr))/*um módulo do QoSP não precisa mais ser monitorado, visto
                                                que não existe mais canais timely passando pelo seu host*/
            this->failureDetector->unRegisterElement(qospPtr);
        this->decrementTimelyPerQoSP(qospPtr);
        /*********URGENTE***********/
        /*NOTIFICAR DEGRADACAO AO PROCESSO
         * CORRESPONDE A NOTIFICAR O PROCESSO LOCAL DO CANAL DE COMUNICACAO
         * (O QOSP QUE SE LOCALIZA NO MESMO HOST DO PROCESSO REMOTO DO CANAL DE COMUNICACAO
         * SERA NOTIFICADO DA DEGRADACAO PELO AGENTE E PODERA NOTIFICAR A FALHA DESTE PROCESSO REMOTO)
         * ATRAVES DO ENVIO DA MENSAGEM changeqos, DEFINIDA NA TESE DO PROFESSOR GORENDER.
         */
        /***************************/
    }

    //verifica quais roteadores não precisam mais serem monitorados pelo FD além de cancelar o monitoramento automático dos mesmos
    for (; routers != endRouters; routers++) {
        router = *routers;
        glob_manager_obj->getTChannelsPerRouter(*router, timelyChannels);
        //o conjunto dos canais timely de rAux está contido no subconjunto dos canais timely do roteador que falhou,
        //logo os canais timely deste roteador serão afetados e a informação de monitoramento dos mesmos não é mais necessária
        if (this->contains(downChannels, timelyChannels)) {
            //informa ao FD que não necessita mais detectar falhas do FD.
            this->failureDetector->unRegisterElement(router);
            nodeMonitor = router->getMonitor();
            buffer[0] = (char)UNSUBS_MSG_ID;
            this->sendSubsMsg(nodeMonitor, buffer);
        }
    }
}

/*
 * Retorna um QoSP (CommunicationEntity) gerenciado por MonitoringApplication com
 * o mesmo IP/porta de process.
 * process: processo que tera seu IP comparado.
 * retorno: a referencia para o qosp gerenciado.
 */
QoSP* MonitoringApplication::getLocalQoSP(CommunicationEntity &process) {
    list<monitored_qosp>::iterator element;
    monitored_qosp str;

    element = find_if(this->qospList.begin(), this->qospList.end(), bind1st(MAQoSPMatchCriterion(), process));
    if (element == this->qospList.end())
        return NULL;
    str = *element;
    return str.qosp;
}

MonitoringApplication::~MonitoringApplication() {
    delete this->monitorsDesc;
    glob_mt_supporter_obj->mutexDestroy(this->lockDesc);
    glob_mt_supporter_obj->mutexDestroy(this->monitorsLock);
    this->qospList.~list<monitored_qosp>();
}

/*
 * interface.c - API para o QoSPM. Alem disso, possui as funcoes de envio e recepcao
 * de mensagens pelos canais gerenciados pelo QoSP.
 */

#include <list>
#include "qospm_interface.h"
#include <netinet/in.h> //struct sockaddr_in
#include <sys/socket.h>
#include "mediator.h"
#include "channelmanager.h"

using namespace std;

extern Mediator                 *glob_mediator_obj;
extern MonitoringApplication    *glob_ma_obj;
extern ChannelVerifier          *glob_cv_obj;
extern TrafficListener          *glob_listener_obj;
extern ChannelManager           *glob_manager_obj;

/*Métodos de API*/
/*
 * Solicita a QoS de um canal. Eh apenas um involucro para a funcao de MonitoringApplication.
 * channel_desc: ponteiro para o descritor do canal.
 * retorno: a QoS do canal.
 */
int qos(qosp_channel_t &channelDesc) {
    Channel *channel;
    channel = glob_manager_obj->getChannelByDesc(channelDesc.channel_manager_desc);
    return glob_ma_obj->qos(*channel);
}

/*
 * Solicita a verificacao de um canal. Eh apenas um involucro para a funcao de ChannelVerifier.
 * p1_addr, p2_addr: enderecos dos processos que compoem o canal.
 * retorno: a QoS do canal.
 */
int verifyChannel(struct sockaddr_in pxAddr, struct sockaddr_in pyAddr) {
    return glob_cv_obj->qos(pxAddr, pyAddr);
}

/*
 * Notifica que um canal tornou-se timely.
 * Esta função é apenas um invólucro para a função de Mediator.
 */
int notifyTimelyChannel(qosp_channel_t &channelDesc) {
    Channel *channel;
    channelDesc.qos = TIMELY;
    channel = glob_manager_obj->getChannelByDesc(channelDesc.channel_manager_desc);
    return glob_mediator_obj->notifyTimelyChannel(channel);
}

/*
 * Notifica que a QoS de um canal foi alterada para untimely.
 * Esta função é apenas um invólucro para a função de Mediator.
 */
void notifyUntimelyChannel(qosp_channel_t &channelDesc) {
    Channel *channel;
    list<Channel *> channels;
    channelDesc.qos = UNTIMELY;
    channel = glob_manager_obj->getChannelByDesc(channelDesc.channel_manager_desc);
    channels.push_back(channel);
    glob_mediator_obj->notifyChannelsDegrad(channels);
}

/*
 * Notifica a existencia de um novo roteador.
 * routerAddr: endereco do roteador.
 * qospaAddr: endereco do monitor (agente) do roteador.
 * routerDesc: ponteiro para o descritor do roteador.
 */
void notifyNewRouter(struct sockaddr_in  routerAddr, struct sockaddr_in  qospaAddr, qosp_router_t *routeDesc) {
    *routeDesc = glob_manager_obj->notifyNewRouter(routerAddr, qospaAddr);
}

/*
 * Notifica a existencia de um canal.
 * px_addr, py_addr: enderecos dos processos que compoem o canal.
 * channel: ponteiro para o descritor do canal.
 */
void notifyNewChannel(struct sockaddr_in  pxAddr, struct sockaddr_in  pyAddr, qosp_channel_t *channel) {
    channel->qos = UNTIMELY;
    channel->channel_manager_desc = glob_manager_obj->notifyNewChannel(pxAddr, pyAddr);
}

/*
 * Informa que um roteador faz parte de um canal.
 * router: ponteiro para o descritor do roteador.
 * channel: ponteiro para o descritor do roteador.
 */
void insertRouterInChannel(qosp_router_t routerDesc, qosp_channel_t *channel) {
    Channel *chann;
    Router *router;
    chann = glob_manager_obj->getChannelByDesc(channel->channel_manager_desc);
    router = glob_manager_obj->getRouterByDesc(routerDesc);
    chann->insertRouter(router);
}
/*Fim dos Métodos de API*/

/*
 * Funcao a ser utilizada pelos processos aplicativos para enviar uma mensagem.
 * desc: ponteiro para o descritor do canal.
 * demais parametros e retorno: ver sendmsg (socket.h).
 */
int qosp_sendmsg (qosp_channel_t &desc, int sock, struct msghdr msg, int flags) {
    if (desc.qos == TIMELY) {
        glob_listener_obj->notifyMsgAppear(desc.listener_desc);
        //return (rt_dev_sendmsg(sock, &msg, flags));
        return (sendmsg(sock, &msg, flags));
    } else
        return (sendmsg(sock, &msg, flags));
}

/*
 * Funcao a ser utilizada pelos processos aplicativos para receber uma mensagem.
 * desc: ponteiro para o descritor do canal.
 * demais parametros e retorno: ver recvmsg (socket.h).
 */
int qosp_recvmsg (qosp_channel_t &desc, int sock, struct msghdr msg, int flags) {
    int ret;
    if (desc.qos == TIMELY) {
        //ret = rt_dev_recvmsg(sock, &msg, flags);
        ret = recvmsg(sock, &msg, flags);
        glob_listener_obj->notifyMsgAppear(desc.listener_desc);
    } else
        ret = recvmsg(sock, &msg, flags);
    return ret;
}

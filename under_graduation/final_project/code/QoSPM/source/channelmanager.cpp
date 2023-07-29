#include "domain.h"
#include "channelmanager.h"
#include "channel.h"
#include "communicationentity.h"
#include "mt_support.h"
#include <list>

extern MtSupporter *glob_mt_supporter_obj;

/*
 * Construtor da classe ChannelManager.
 * result(parametro de saida): contera 1 caso o objeto seja construido corretamente,
 * 0 caso contrario.
 */
ChannelManager::ChannelManager() {
    this->channelProcess.reserve(2*MAX_CHANNELS);
    this->channels.reserve(MAX_CHANNELS);
    this->routers.reserve(MAX_ROUTERS);
    this->nodeMonitors.reserve(MAX_ROUTERS);

    glob_mt_supporter_obj->mutexCreate(this->lockDesc, QOSP_XENOMAI_DOMAIN);
}

/*
 * Notifica que a QoS de um canal foi alterada para UNTIMELY.
 * channel: referencia que teve sua QoS degradada.
 */
void ChannelManager::notifyQoSDowngrade(Channel &channel) {
    channel.setQoS(0);
}

/*
 * Notifica que a QoS de um canal foi alterada para TIMELY.
 * channel: referencia que teve sua QoS alterada.
 */
int ChannelManager::notifyQoSUpgrade(Channel &channel) {
    if (!glob_mt_supporter_obj->lockAcquire(this->lockDesc))
        return 0;
    channel.setQoS(1);
    if (!glob_mt_supporter_obj->lockRelease(this->lockDesc))
        return 0;
    return 1;
}

/*
 * Retorna a referencia de um canal gerenciado pelo QoSP, cujos processos sao iguais
 * (em termos de enderecos socket) a px e py.
 * px e py: processos que servirao como base da comparacao, com relacao aos enderecos socket.
 * retorno: 1 caso o canal exista, 0 caso contrario.
 */
int ChannelManager::getChannel(CommunicationEntity &px, CommunicationEntity &py, Channel* &channel) {
    int a, qt_channels;
    vector<Channel *>::iterator channels, end;
    Channel *temp_channel, chan(&px, &py, 0);

    qt_channels = this->getChannels(channels, end);
    for (a = 0; a < qt_channels; a++, channels++) {
        temp_channel = *channels;
        
        if (temp_channel->equal(chan)) {
            channel = temp_channel;
            return 1;
        }
    }
    channel = NULL;
    return 0;
}

/*
 * Retorna todos os canais TIMELY passam por um determinado roteador.
 * router: roteador cujos canais TIMELY que passam pelo mesmo deverao ser retornados.
 * timelyChannels (parametro de saida): contera a lista de todos os canais TIMELY que passam por router.
 * obs: timelyChannels ja devera estar apontando para uma area de memoria valida.
 * retorno: quantidade de canais timely que passam por router.
 */
int ChannelManager::getTChannelsPerRouter(Router &router, list<Channel *> &timelyChannels) {
    int a, b, ret = 0, qtRouters, qtChannels;
    vector<Channel *>::iterator channels, end;
    Channel *channel;
    list<Router *> routers;
    list<Router *>::iterator routersItr;
    Router *tempRouter;

    //timely_channels = new list<Channel *>;
    qtChannels = this->getChannels(channels, end);
    for (a = 0; a < qtChannels; a++, channels++) {
        channel = *channels;
        if (channel->getQoS() == 1) {
            qtRouters = channel->getRouters(routers);
            routersItr = routers.begin();
            for (b = 0; b < qtRouters; b++, routersItr++) {
                tempRouter = *routersItr;
                if (router.equalIP(*tempRouter)) {
                    timelyChannels.push_back(channel);
                    ret++;
                    break; //ja descobriu que o canal em questao passa por router
                }
            }
        }
    }
}

/*
 * Retorna todos os roteadores gerenciados pelo QoSP.
 * routers: contera um iterador para o inicio da lista dos roteadores.
 * end: contera um iterador para o final da lista dos roteadores.
 * retorno: quantidade de roteadores gerenciados por QoSP.
 */
int ChannelManager::getRouters(vector<Router *>::iterator &routers, vector<Router *>::iterator &end) {
    routers = this->routers.begin();
    end = this->routers.end();
    return this->routers.size();
}

/*
 * Retorna um vetor contendo todos os canais (TIMELY e UNTIMELY) gerenciados pelo QoSP.
 * routers: contera um iterador para o inicio da lista dos canais.
 * end: contera um iterador para o final da lista dos canais.
 * retorno: quantidade de canais gerenciados pelo QoSP.
 */
int ChannelManager::getChannels(vector<Channel *>::iterator &channels, vector<Channel *>::iterator &end) {
    channels = this->channels.begin();
    end = this->channels.end();
    return this->channels.size();
}

/*
 * Dado um endereco socket, retorna o roteador corrrespondente gerenciado pelo QoSP.
 * sockAddr: endereco socket do roteador que devera ser retornado.
 * retorno: contera a referencia do roteador requisitado.
 */
Router *ChannelManager::getRouterByAddr(struct sockaddr_in  sockAddr) {
    int index, qtRouters;
    Router temp(sockAddr, NULL), *router;
    vector<Router *>::iterator routers, end;

    qtRouters = this->getRouters(routers, end);
    for (index = 0; index < qtRouters; index++, routers++) {
        router = *routers;
        if (temp.equalIP(*router))
            return router;
    }
}

/*
 * Notifica a existencia de um novo roteador.
 * routerAddr: endereco socket do novo roteador.
 * qospaAddr: endereco socket do monitor do novo roteador.
 * retorno: o descritor do novo roteador.
 */
int ChannelManager::notifyNewRouter(struct sockaddr_in  routerAddr, struct sockaddr_in  qospaAddr) {
    int ret = this->routers.size();
    vector<QoSPA *>::iterator monitorIt;
    vector<Router *>::iterator routerIt;
    QoSPA *agent;

    this->nodeMonitors.push_back(new QoSPA(qospaAddr));

    monitorIt = this->nodeMonitors.end();
    monitorIt--;
    this->routers.push_back(new Router(routerAddr, *monitorIt));
    routerIt = this->routers.end();
    routerIt--;
    agent = *monitorIt;
    agent->setRouter(*routerIt);
    this->nodeMonitors.insert(monitorIt, agent);

    return ret;
}

/*
 * Notifica a existencia de um novo canal.
 * pxAddr: endereco socket do processo local ao canal.
 * pyAddr: endereco socket do processo remoto ao canal.
 * retorno: o descritor do novo canal.
 */
int ChannelManager::notifyNewChannel(struct sockaddr_in  pxAddr, struct sockaddr_in  pyAddr) {
    int ret = this->channels.size();
    CommunicationEntity *px, *py;
    Channel *channel;

    px = new CommunicationEntity(pxAddr);
    py = new CommunicationEntity(pyAddr);
    channel = new Channel(px, py, 0);
    this->channelProcess.push_back(px);
    this->channelProcess.push_back(py);
    this->channels.push_back(channel);
    return ret;
}

Router* ChannelManager::getRouterByDesc(int desc) {
    return this->routers[desc];
}

Channel* ChannelManager::getChannelByDesc(int desc) {
    return this->channels[desc];
}

ChannelManager::~ChannelManager() {

    this->channelProcess.~vector<CommunicationEntity *>();
    this->channels.~vector<Channel *>();
    this->routers.~vector<Router *>();
    this->nodeMonitors.~vector<QoSPA *>();
    glob_mt_supporter_obj->mutexDestroy(this->lockDesc);
}

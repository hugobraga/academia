
#include "channel_verifier.h"

#include "channel.h"
#include "domain.h" //MAX_ROUTERS_PER_CHANNEL

Channel::Channel(CommunicationEntity* px, CommunicationEntity* py, int qos) {
    this->px = px;
    this->py = py;
    this->qos = qos;
}

/*
 * Cadastra um roteador como parte de um canal.
 * router: roteador a ser cadastrado.
 */
bool Channel::insertRouter(Router* &router) {
    int qt_routers = this->routers.size();
    if (qt_routers == MAX_ROUTERS_PER_CHANNEL)
        return false;

    this->routers.push_back(router);
    return true;
}

/*
 * Retorna o processo local do canal.
 * px: contera a referencia ao processo local.
 */
CommunicationEntity* Channel::getPx() {
    return this->px;
}

/*
 * Retorna o processo remoto do canal.
 * py: contera a referencia ao processo remoto.
 */
CommunicationEntity* Channel::getPy() {
    return this->py;
}

/*
 * Retorna os roteadores que compoem o canal.
 * routers: contera uma lista dos roteadores.
 * retorno: quantidade de roteadores do canal.
 */
int Channel::getRouters(list<Router *> &routers) {
    routers = this->routers;
    return this->routers.size();
}

/*
 * Retorna a QoS do canal.
 * retorno: 1 - TIMELY, 0 UNTIMELY.
 */
int Channel::getQoS() {
    return this->qos;
}

/*
 * Seta a QoS do canal.
 * qos: nova QoS (1 - TIMELY ou 0 - UNTIMELY) do canal.
 */
void Channel::setQoS(int qos) {
    this->qos = qos;
}

/*
 * Verifica se dois canais sao iguais, atraves da verificacao dos respectivos processos.
 * retorno: true se forem iguais, false caso contrario.
 */
bool Channel::equal(Channel &channel) {
    CommunicationEntity *px, *py;
    px = channel.getPx();
    py = channel.getPy();
    
    if (((this->px->equal(*px)) && (this->py->equal(*py))) ||
            ((this->px->equal(*py)) && (this->py->equal(*px))))
        return true;
    else
        return false;
}

/*
 * Retorno o descritor do canal utilizado pelo TrafficListener.
 * retorno: descritor do canal utilizado pelo TrafficListener.
 */
int Channel::getListenerDesc() {
    return this->listenerDesc;
}

/*
 * Seta o descritor do canal utilizado por TrafficListener.
 * desc: descritor do canal.
 */
void Channel::setListenerDesc(int desc) {
    this->listenerDesc = desc;
}

#include "mediator.h"

Mediator::Mediator(MonitoringApplication *ma, ChannelVerifier *cv, LocalHost *localHost) {
    this->ma = ma;
    this->cv = cv;
    this->localHost = localHost;
}

void Mediator::notifyChannelsDegrad(list<Channel *> &channels) {
    CommunicationEntity *px, *py;
    list<Channel *>::iterator iter, end;
    Channel *channel;

    this->ma->notifyChannelsDegrad(channels);
    for (iter = channels.begin(), end = channels.end(); iter != end; iter++) {
        channel = *iter;
        this->localHost->whichLocalProcess(*channel, px);
        this->localHost->whichRemoteProcess(*channel, py);
        this->cv->unRegister(channel->getListenerDesc());
    }
}

/*
 * Notifica MonitoringApplication que um canal tornou-se TIMELY alem de registrar
 * este canal junto ao ChannelVerifier.
 * channel: canal que teve sua QoS alterada.
 * retorno: um descritor a ser utilizado para identificar o canal junto a ChannelVerifier.
 */
int Mediator::notifyTimelyChannel(Channel *channel) {
    CommunicationEntity *px, *py;
    int ret;

    this->ma->notifyTimelyChannel(*channel);
    this->localHost->whichLocalProcess(*channel, px);
    this->localHost->whichRemoteProcess(*channel, py);
    ret = this->cv->registerChannel(px, py);
    channel->setListenerDesc(ret);
    return ret;
}

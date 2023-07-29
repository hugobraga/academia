#include "local_host.h"

/*
 * Verifica se um processo eh local (com relacao ao endereco socket) ao QoSP local.
 * otherProcess: processo a ser verificado se eh local.
 * retorno: true caso o processo seja local, false caso contrario.
 */
bool LocalHost::isProcessInLocalHost(CommunicationEntity &otherProcess) {
    if (this->localHost->equalIP(otherProcess))
        return true;
    return false;
}

LocalHost::LocalHost(struct sockaddr_in addr) {
    this->localHost = new CommunicationEntity(addr);
}

/*
 * Retorna o processo de channel que se localiza neste host.
 * channel: canal a ser verificado.
 * proc: contera o processo de channel que se localiza neste host.
 * retorno: 1 caso tenha encontrado um processo local, 0 caso contrario.
 */
int LocalHost::whichLocalProcess(Channel &channel, CommunicationEntity* &proc) {
    proc = channel.getPx();
    if (!(this->isProcessInLocalHost(*proc))) {
        proc = channel.getPy();
        if (!(this->isProcessInLocalHost(*proc)))
            return 0;
    }
    return 1;
}

/*
 * Retorna o processo de channel que se localiza em outro host.
 * channel: canal a ser verificado.
 * proc: contera o processo de channel que se localiza em outro host.
 * retorno: 1 caso tenha encontrado um processo remoto, 0 caso contrario.
 */
int LocalHost::whichRemoteProcess(Channel &channel, CommunicationEntity* &proc) {
    proc = channel.getPy();
    if (this->isProcessInLocalHost(*proc)) {
        proc = channel.getPx();
        if (this->isProcessInLocalHost(*proc))
            return 0;
    }
    return 1;
}

LocalHost::~LocalHost() {
    delete this->localHost;
}

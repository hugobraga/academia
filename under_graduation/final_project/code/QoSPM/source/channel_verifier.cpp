#include <vector>
#include "channel_verifier.h"
#include "verifier.h"
#include "domain.h"

using namespace std;

extern MtSupporter *glob_mt_supporter_obj;

/*
 * Retorna um descritor valido para um Verifier.
 * retorno: -1 - caso nao seja possivel retornar um descritor valido,
 * caso contrario retorna um descritor valido.
 * Este descritor e necessario para a funcao releaseVerDesc.
 */
int ChannelVerifier::getVerifierDesc() {
    vector<bool>::iterator iter, end;
    int index;
    
    if (!(glob_mt_supporter_obj->lockAcquire(this->verifiersLock)))
        return -1;
    for (index = 0, iter = this->verifiersDesc->begin(), end = this->verifiersDesc->end();
    iter != end; iter++, index++) {
        if ((*iter) == false) {
            this->verifiersDesc->insert(iter, true);
            return index;
        }
    }
    if (!(glob_mt_supporter_obj->lockRelease(this->verifiersLock)))
        return -1;
    return -1;
}

/*
 * Libera um descritor alocado para um Verifier.
 * desc: descritor a ser liberado. Esta descritor
 * corresponde ao retorno da funcao getVerifierDesc.
 */
void ChannelVerifier::releaseVerDesc(int desc) {
    this->verifiersDesc->at(desc) = false;
}

ChannelVerifier::ChannelVerifier(MonitoringApplication *ma, TrafficListener *listener) {
    this->ma = ma;
    this->listener = listener;
    this->verifiersDesc = new vector<bool>(MAX_FLOWS, false);
    glob_mt_supporter_obj->mutexCreate(this->verifiersLock, QOSP_LINUX_DOMAIN);
}

int ChannelVerifier::qos(struct sockaddr_in pxAddr, struct sockaddr_in pyAddr) {
    Verifier *verifier;
    int descPos;
    int ret;

    //criando instancia para tratar a chamada
    descPos = this->getVerifierDesc();
    verifier = new Verifier(this->ma, this->listener, descPos);
    ret = verifier->qos(pxAddr, pyAddr);
    delete verifier;
    this->releaseVerDesc(descPos);
    return ret;
}

/*
 * Registra um novo canal para ser escutado.
 * Esta função é apenas um invólucro para a função do TrafficListener.
 * px e py: representa os processos que compoem o canal a ser escutado.
 * retorno: um descritor que identifica unicamente o canal junto ao escutador.
 */
int ChannelVerifier::registerChannel(CommunicationEntity *px, CommunicationEntity *py) {
    return this->listener->registerChannel(px, py);
}

/*
 * Informa que um canal não necessita ser mais escutado.
 * Esta função é apenas um invólucro para a função do TrafficListener.
 * channel_desc: descritor do canal, retornado pela funcao registerChannel_CV.
 */
void ChannelVerifier::unRegister(int channelDesc) {
    this->listener->unRegister(channelDesc);
}

ChannelVerifier::~ChannelVerifier() {
    delete this->verifiersDesc;
    glob_mt_supporter_obj->mutexDestroy(this->verifiersLock);
}

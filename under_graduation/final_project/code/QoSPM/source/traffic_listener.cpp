#include "traffic_listener.h"
#include "domain.h"
#include <native/task.h>
#include "native/timer.h"

extern MtSupporter *glob_mt_supporter_obj;

TrafficListener::TrafficListener() {
    qosp_listener_str str;

    str.lmTime = 0;
    str.px = NULL;
    str.py = NULL;
    glob_mt_supporter_obj->mutexCreate(this->descLock, QOSP_XENOMAI_DOMAIN);
    this->channels = new vector<qosp_listener_str>(MAX_CHANNELS);
}

/*
 * Registra um novo canal para ser escutado.
 * px e py: representa os processos que compoem o canal a ser escutado.
 * retorno: um descritor que identifica unicamente o canal junto ao escutador,
 * ou -1 caso ocorra algum problema.
 */
int TrafficListener::registerChannel(CommunicationEntity *px, CommunicationEntity *py) {
    vector<qosp_listener_str>::iterator iter;
    qosp_listener_str str;
    int index, ret = -1;

    if (!(glob_mt_supporter_obj->lockAcquire(this->descLock)))
        return -1;
    for (index = 0, iter = this->channels->begin(); index < this->channels->size(); index++, iter++) {
        str = *iter;
        if (str.px == NULL) {
            str.px = px;
            str.py = py;
            this->channels->insert(iter, str);
            ret = index;
            break;
        }
    }
    if (!(glob_mt_supporter_obj->lockRelease(this->descLock)))
        return -1;
    return ret;
}

/*
 * Informa que um canal não necessita ser mais escutado.
 * channelDesc: descritor do canal, retornado pela funcao registerChannel.
 */
int TrafficListener::unRegister(int channelDesc) {
    qosp_listener_str str;
    
    if (!(glob_mt_supporter_obj->lockAcquire(this->descLock)))
        return 0;
    str = this->channels->at(channelDesc);
    str.px = str.py = NULL;
    this->channels->at(channelDesc) = str;
    if (!(glob_mt_supporter_obj->lockRelease(this->descLock)))
        return 0;
    return 1;
}

/*
 * Retorna a hora da última mensagem enviada ou recebida por um canal.
 * channelDesc: descritor que identifica o canal.
 * retorno: a hora da ultima mensagem trocada.
 */
RTIME TrafficListener::lastMessageTime(int channelDesc) {
    qosp_listener_str str;

    str = this->channels->at(channelDesc);
    return str.lmTime;
}

/*
 * Notifica que uma mensagem foi enviada ou recebida em um canal.
 * channelDesc: descritor que identifica o canal.
 */
void TrafficListener::notifyMsgAppear(int channelDesc) {
    qosp_listener_str str;

    str = this->channels->at(channelDesc);
    str.lmTime = rt_timer_read();
    this->channels->at(channelDesc) = str;
}

TrafficListener::~TrafficListener() {
    delete this->channels;
}

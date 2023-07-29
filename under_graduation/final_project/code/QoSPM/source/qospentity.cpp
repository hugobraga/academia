#include "qospentity.h"
#include "mt_support.h" //QOSP_XENOMAI_DOMAIN
#include <algorithm>

extern MtSupporter *glob_mt_supporter_obj;

/*
 * Construtor da classe QoSPEntity.
 * addr: endereco socket do QoSPEntity.
 */
QoSPEntity::QoSPEntity(struct sockaddr_in addr): CommunicationEntity(addr){
    glob_mt_supporter_obj->mutexCreate(this->lockDesc, QOSP_XENOMAI_DOMAIN);
}

/*
 * Verifica se o QoSPEntity esta sendo monitorado, visto que existem fluxos que
 * dependem da monitoracao deste QoSPEntity.
 * retorno: true - se o QoSPEntity esta sendo monitorado.
 *          false - caso contrario.
 */
bool QoSPEntity::isBeingMon() {
    return this->beingMonitored;
}

/*
 * Cadastra um determinado fluxo como dependente deste QoSPEntity. A dependencia
 * se deve ao fato de Failure Detector monitorar periodicamente QoSPentity e, caso
 * observe que este QoSPEntity falhou, necessita saber quais fluxos dependem desta
 * informacao de falha.
 * dependent: fluxo dependente deste QoSPEntity.
 * retorno: -1 - caso nao seja possivel cadastrar este fluxo como dependente. Caso
 * contrario, retorno um descritor para dependent.
 */
int QoSPEntity::setDependent(Flow &dependent) {
    int ret = dependent.getFlowDesc();
    if (!glob_mt_supporter_obj->lockAcquire(this->lockDesc))
        return -1;
    this->dependentsDesc.push_back(ret);
    this->beingMonitored = true;
    if (!glob_mt_supporter_obj->lockRelease(this->lockDesc))
        return -1;
    return ret;
}

/*
 * Retorna todos os fluxos dependentes deste QoSPEntity.
 * dependents: vetor de todos os fluxos dependentes deste QoSPEntity.
 * retorno: numero de fluxos dependentes deste QoSPEntity.
 */
int QoSPEntity::getDependents(list<int>::iterator &dependents, list<int>::iterator &end) {
    if (!glob_mt_supporter_obj->lockAcquire(this->lockDesc))
        return -1;
    dependents = this->dependentsDesc.begin();
    end = this->dependentsDesc.end();
    return this->dependentsDesc.size();
}

/*
 * Informa que um determinado fluxo nao eh mais dependente deste QoSPEntity.
 * dependent_desc: descritor do fluxo cuja dependencia nao existe mais.
 * retorno: 1 - se o procedimento ocorreu com sucesso.
 *          0 - caso constrario.
 */
int QoSPEntity::releaseDependent(int dependent_desc) {
    if (!glob_mt_supporter_obj->lockAcquire(this->lockDesc))
        return 0;
    remove(this->dependentsDesc.begin(), this->dependentsDesc.end(), dependent_desc);
    if (this->dependentsDesc.empty())
        this->beingMonitored = false;
    if (!glob_mt_supporter_obj->lockRelease(this->lockDesc))
        return 0;
    return 1;
}

/*
 * Informa que nao existe mais nenhum fluxo dependente deste QoSPEntity.
 * retorno: 1 - se o procedimento ocorreu com sucesso.
 *          0 - caso contrario.
 */
int QoSPEntity::releaseDependents() {
    if (!glob_mt_supporter_obj->lockRelease(this->lockDesc))
        return 0;
    this->dependentsDesc.clear();
    this->beingMonitored = 0;
    return 1;
}

QoSPEntity::~QoSPEntity() {
    this->dependentsDesc.~list<int>();
}

void QoSPEntity::dynamic_cast_method() {
}

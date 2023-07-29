
#include <list>

#include "failuredetector.h"
#include "router.h"
#include "qospa.h"
#include "channelmanager.h"
#include <algorithm>

extern MtSupporter *glob_mt_supporter_obj;

bool FailureDetector::setRemoveIf(set<fd_element, FDElementsSortCriterion> &elements, fd_element element_str) {
    set<fd_element, FDElementsSortCriterion>::iterator itr;

    itr = elements.find(element_str);
    if (itr != elements.end())
        elements.erase(itr);
}

FailureDetector::FailureDetector(Delay* delay) {
    this->delayCalc = delay;
    glob_mt_supporter_obj->mutexCreate(this->lockDesc, QOSP_XENOMAI_DOMAIN);
}

void FailureDetector::setMA(MonitoringApplication *client) {
    this->client = client;
}

/*
 * Registra junto ao FD um QoSPEntity para ser monitorado.
 * element: ponteiro para o QoSPEntity cuja monitoracao deve ser iniciada.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int FailureDetector::registerElement(QoSPEntity* element) {
    fd_element element_str;
    QoSPA *monitor;
    Router *router;

    if (!glob_mt_supporter_obj->lockAcquire(this->lockDesc))
        return 0;
    
    element_str.delay = this->delayCalc->delay(*element);
    element_str.element = element;
    this->elements.insert(element_str);

    router = dynamic_cast<Router *> (element);
    if (router) {//o elemento eh um roteador, logo seu monitor deve ser registrado
        monitor = router->getMonitor();
        element_str.delay = this->delayCalc->delay(*monitor);
        element_str.element = monitor;
        this->elements.insert(element_str);
    }

    if (!glob_mt_supporter_obj->lockRelease(this->lockDesc))
        return 0;
    return 1;
}

/*
 * Solicita ao FD para cancelar a monitoracao de um elemento.
 * element: ponteiro para o elemento cuja monitoracao deve ser cancelada.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int FailureDetector::unRegisterElement(QoSPEntity* element) {
    fd_element element_str;
    QoSPA *monitor;
    Router *router;
    set<fd_element, FDElementsSortCriterion>::iterator itr;
    if (!glob_mt_supporter_obj->lockAcquire(this->lockDesc))
        return 0;

    element_str.element = element;
    this->setRemoveIf(this->elements, element_str);

    router = dynamic_cast<Router *> (element);
    if (router) {//o elemento eh um roteador, logo seu monitor tambem deve ser removido
        monitor = router->getMonitor();
        element_str.element = monitor;
        this->setRemoveIf(this->elements, element_str);
    }

    if (!glob_mt_supporter_obj->lockRelease(this->lockDesc))
        return 0;
    return 1;
}

/*
 * Verifica se um determinado elemento já está sendo monitorado pelo FD.
 * element: referencia do elemento a ser verificado.
 * retorno: true caso o qosp ja esteja sendo monitorado, false caso contrario.
 */
bool FailureDetector::isElementBeingMon(QoSPEntity *element) const {
    fd_element element_str;
    element_str.element = element;
    return (binary_search(this->elements.begin(), this->elements.end(), element_str, FDElementsSortCriterion()));
}

/*
 * Retorna todos os elementos que estão sendo monitorados. Quando os elementos nao
 * forem mais necessarios, a funcao releaseElements deste objeto deve ser chamado.
 * elements: contera um iterador para o inicio dos elementos monitorados.
 * end: contera um iterador para o final dos elementos monitorados.
 * retorno: quantidade de elementos monitorados, ou -1 caso o procedimento nao seja
 * executado com sucesso.
 */
int FailureDetector::getElements(set<fd_element, FDElementsSortCriterion>::iterator &elements, set<fd_element, FDElementsSortCriterion>::iterator &end) {
    if (!glob_mt_supporter_obj->lockAcquire(this->lockDesc))
        return -1;
    elements = this->elements.begin();
    end = this->elements.end();
    return this->elements.size();
}

/*
 * Funcao a ser chamada posteriormente a chamada da funcao getElements, quando os
 * elementos ja nao sao mais necessarios.
 * retorno: 1 caso o procedimento seja executado corretamente, 0 caso contrario.
 */
int FailureDetector::releaseElements() {
    if (!glob_mt_supporter_obj->lockRelease(this->lockDesc))
        return 0;
    return 1;
}

/*
 * Registra junto ao FD um QoSP para ser monitorado temporariamente. Este QoSP nao
 * esta hospedado no mesmo host de um processo que faz parte de um canal gerenciado
 * pelo modulo do QoSP local. Este QoSP a ser registrado eh necessario para que uma
 * verificacao remota possa ser executada.
 * qospAddr: endereco do QoSP a ser registrado.
 * qosp: contera a referencia para o QoSP registrado.
 * retorno: 0 caso o procedimento seja executado com sucesso, -1 caso contrario.
 */
int FailureDetector::registerRemoteQoSP(struct sockaddr_in qospAddr, QoSP* &qosp) {
    if (!glob_mt_supporter_obj->lockAcquire(this->lockDesc))
        return 0;
    this->remoteQoSP.push_back(new QoSP(qospAddr));
    qosp = this->remoteQoSP.back();
    if (!glob_mt_supporter_obj->lockRelease(this->lockDesc))
        return 0;
    return 1;
}

/*
 * Solicita ao FD para cancelar a monitoracao de um QoSP temporario (ver registerTempQoSP).
 * qosp: qosp a ser removido.
 */
void FailureDetector::unRegisterRemoteQoSP(QoSP *qosp) {
    remove_if(this->remoteQoSP.begin(), this->remoteQoSP.end(), bind1st(QoSPMatchCriterion(), qosp));
}

FailureDetector::~FailureDetector() {
    glob_mt_supporter_obj->mutexDestroy(this->lockDesc);
    this->elements.~set();
    this->remoteQoSP.~list<QoSP *>();
}

/* 
 * File:   failuredetector.h
 * Author: hugo
 *
 * Created on 8 de Fevereiro de 2009, 16:32
 */

#ifndef _FAILUREDETECTOR_H
#define	_FAILUREDETECTOR_H

class MonitoringApplication;

#include "monitoringapplication.h"
#include "qospentity.h"
#include "qosp.h"
#include "delay.h"
#include "mt_support.h"
#include "qospentity.h"
#include <set>
#include <list>
#include <functional>

using namespace std;

typedef struct fd_element {
    QoSPEntity  *element;
    long long   delay;
} fd_element;

/*Function Object a ser utilizado como criterio de ordenacao para os elementos do FailureDetector*/
class FDElementsSortCriterion : public binary_function<fd_element, fd_element, bool> {
public:
    result_type operator() (const first_argument_type &elem1, const second_argument_type &elem2) const {
        return (elem1.delay < elem2.delay);
    }
};

class FDElementsMatchCriterion : public binary_function<fd_element, fd_element, bool> {
public:
    result_type operator() (first_argument_type &elem1, second_argument_type &elem2) const {
        return (elem1.element->equalIP(*(elem2.element)));
    }
};

class QoSPMatchCriterion : public binary_function<QoSP *, QoSP *, bool> {
public:
    result_type operator() (first_argument_type elem1, second_argument_type elem2) const {
        return (elem1->equalIP(*(elem2)));
    }
};

class FailureDetector {
    MonitoringApplication                       *client;
    Delay                                       *delayCalc;
    set<fd_element, FDElementsSortCriterion>    elements;
    list<QoSP *>                                remoteQoSP;
    qosp_lock_t                                 lockDesc;
private:
    bool setRemoveIf(set<fd_element, FDElementsSortCriterion> &elements, fd_element element_str);
public:
    FailureDetector(Delay *delay);
    void setMA(MonitoringApplication *client);
    int registerElement(QoSPEntity *element);
    int unRegisterElement(QoSPEntity* element);
    bool isElementBeingMon(QoSPEntity *element) const;
    int getElements(set<fd_element, FDElementsSortCriterion>::iterator &elements, set<fd_element, FDElementsSortCriterion>::iterator &end);
    int releaseElements();
    int registerRemoteQoSP(struct sockaddr_in qospAddr, QoSP* &qosp);
    void unRegisterRemoteQoSP(QoSP *qosp);
    virtual ~FailureDetector();
};

#endif	/* _FAILUREDETECTOR_H */


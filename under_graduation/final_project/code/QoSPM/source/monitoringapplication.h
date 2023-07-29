/* 
 * File:   monitoringapplication.h
 * Author: hugo
 *
 * Created on 15 de Fevereiro de 2009, 21:32
 */

#ifndef _MONITORINGAPPLICATION_H
#define	_MONITORINGAPPLICATION_H

#include <list>
#include <vector>
#include "mt_support.h"
#include "failuredetector.h"
#include "mediator.h"
#include "channel.h"
#include "qosp.h"
#include "qospa.h"
#include "router.h"
#include "communicationentity.h"
#include <functional>

using namespace std;

class Mediator; //resolver problema de referencia cruzada
class FailureDetector; //resolver problema de referencia cruzada

typedef struct monitored_qosp {
    QoSP    *qosp;
    int     qosp_dependents;
} monitored_qosp;

class MAQoSPMatchCriterion : public binary_function<CommunicationEntity, monitored_qosp, bool> {
public:
    result_type operator() (const first_argument_type &qosp, second_argument_type &str) const {
        return (qosp.equalIP(*(str.qosp)));
    }
};

class MonitoringApplication {
    LocalHost               *localHost;
    FailureDetector         *failureDetector;
    Mediator                *mediator;
    list<monitored_qosp>    qospList;
    vector<bool>            *monitorsDesc;
    qosp_lock_t             lockDesc;
    qosp_lock_t             monitorsLock;
private:
    int getMonitorDesc();
    void releaseMonDesc(int desc);
    void sendSubsMsg(QoSPA &monitor, char *msg);
    void getQoSPByProc(CommunicationEntity &process, QoSP &qosp);
    int incrementTimelyPerQoSP(QoSP* &qosp);
    int decrementTimelyPerQoSP(QoSP *qosp);
    bool oneChannelPerQoSP(QoSP* &qosp);
    bool contains(list<Channel *> &set, list<Channel *> &sub);
    void sendSubsMsg(QoSPA *monitor, char *msg);
public:
    MonitoringApplication(FailureDetector *fd, LocalHost *local_host);
    int qos(Channel &channel);
    void notifyTimelyChannel(Channel &channel);
    void notifyQoSPFailure(QoSP &qosp);
    void notifyRouterFailure(Router &faultyRouter);
    void notifyChannelsDegrad(list<Channel *> &downChannels);
    QoSP* getLocalQoSP(CommunicationEntity &process);
    virtual ~MonitoringApplication();
};

#endif	/* _MONITORINGAPPLICATION_H */


/* 
 * File:   qos_monitor.h
 * Author: hugo
 *
 * Created on 15 de Fevereiro de 2009, 22:11
 */

#ifndef _QOS_MONITOR_H
#define	_QOS_MONITOR_H

//#include "flow.h"
#include "monitoringapplication.h"

class QoSMonitor {
    Flow                    *flowBehaviour;
    MonitoringApplication   *qosConsultant;
private:
    int sendQoSEnqMsg(int qt_routers, list<Router *> &routers, QoSP *qosp);
public:
    QoSMonitor(MonitoringApplication *consultant, int ma_desc);
    int qos(Channel &channel);
    virtual ~QoSMonitor();
};

#endif	/* _QOS_MONITOR_H */


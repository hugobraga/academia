/* 
 * File:   qospentity.h
 * Author: hugo
 *
 * Created on 25 de Janeiro de 2009, 16:27
 */

#ifndef _QOSPENTITY_H
#define	_QOSPENTITY_H

#include "communicationentity.h"
#include <netinet/in.h> //struct sockaddr_in
#include "flow.h"
#include "mt_support.h"
#include <list>

using namespace std;

class QoSPEntity : public CommunicationEntity {
    bool beingMonitored;
    list<int> dependentsDesc;
    qosp_lock_t lockDesc;
public:
    QoSPEntity(struct sockaddr_in addr);
    bool isBeingMon();
    int setDependent(Flow &dependent);
    int releaseDependent(int dependent_desc);
    int getDependents(list<int>::iterator &dependents, list<int>::iterator &end);
    int releaseDependents();
    virtual ~QoSPEntity();
    virtual void dynamic_cast_method();
};

#endif	/* _QOSPENTITY_H */


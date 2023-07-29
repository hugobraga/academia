/* 
 * File:   flow.h
 * Author: hugo
 *
 * Created on 25 de Janeiro de 2009, 16:19
 */

#ifndef _FLOW_H
#define	_FLOW_H

class Flow {
    int flowDesc;
public:
    Flow(int flowDesc);
    void setFlowDesc(int flowDesc);
    int getFlowDesc();
};

#endif	/* _FLOW_H */


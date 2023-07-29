/* 
 * File:   monitoring.h
 * Author: hugo
 *
 * Created on 22 de Fevereiro de 2009, 16:42
 */

#ifndef _MONITORING_H
#define	_MONITORING_H

#define QOSP_ELEM_ID            0   //utilizado para indicar que um determinado elemento em FailureDetector.elements
                                    //eh do tipo QoSP
#define ROUTER_ELEM_ID          1   //utilizado para indicar que um determinado elemento em FailureDetector.elements
                                    //eh do tipo Router
#define QOSPA_ELEM_ID           2

#define MAX_MON_ATTEMPT         5   //numero maximo de tentativas para realizar um monitoramento
#define MON_TIMEOUT             1000000000
#define VER_TIMEOUT             2000000000

#endif	/* _MONITORING_H */


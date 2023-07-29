/* 
 * File:   conf.h
 * Author: hugo
 *
 * Created on 24 de Janeiro de 2009, 15:32
 */

#ifndef _CONF_H
#define	_CONF_H

//#define QoSP_MODULE     //comente esta linha se o host local nao hospedar o modulo do QoSP
#define QoSPA_MODULE    //comente esta linha se o host local nao hospedar o modulo do QoSPA

#define QoSP_PORT   5000    //porta utilizada pelos canais do QoSP/QoSPA
#define FD_PORT     5001    //porta utilizada pelo detector de falhas do do modulo do QoSP

#define FD_MON_PERIOD 50000000    //periodicidade (em nanosegundos) de execucao do detector de falhas do QoSP
#define CHANNEL_DORMANCY_PERIOD 1000000  //periodo (em nanosegundos) em que um canal deve ser executado
                                            //para verificar a existencia de mensagens

#define TIMELY_CLASS_NAME   "premium"       //nome da classe que recebe o servico expresso
                                            //configurada nos roteadores cisco
#define COMMUNITY_STR       "public"        //community string dos roteadores cisco
                                            //para troca de mensagens SNMP

#ifdef QoSP_MODULE
    #define RT_CONTEXT
    #ifdef QoSPA_MODULE
        #define HOLISTIC_QoSP
    #endif
#endif

#endif	/* _CONF_H */


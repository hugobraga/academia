/* 
 * File:   mt_support.h
 * Author: hugo
 *
 * Created on 26 de Janeiro de 2009, 22:47
 */

#ifndef _MT_SUPPORT_H
#define	_MT_SUPPORT_H

#include "conf.h"
#include <pthread.h>
#ifdef RT_CONTEXT
#include <native/cond.h>
#include <native/mutex.h>
#include <native/task.h>
#endif

#define QOSP_LINUX_DOMAIN           0
#define QOSP_XENOMAI_DOMAIN         1

typedef struct qosp_lock_t {
    #ifdef RT_CONTEXT
    RT_MUTEX        xeno_lock;
    #endif
    pthread_mutex_t lin_lock;
    int             lock_domain;
}qosp_lock_t;

typedef struct qosp_cond_t {
    #ifdef RT_CONTEXT
    RT_COND         xeno_cond;
    #endif
    pthread_cond_t  lin_cond;
    int             cond_domain;
}qosp_cond_t;

typedef struct qosp_thread_t {
    #ifdef RT_CONTEXT
    RT_TASK         xeno_thread;
    #endif
    pthread_t       lin_thread;
    int             thread_domain;
}qosp_thread_t;

class MtSupporter {
public:
    /*funcoes relacionadas as variaveis condicionais*/
    int condCreate(qosp_cond_t &cond, int context);
    int condDestroy(qosp_cond_t &cond);
    int condWait(qosp_cond_t &cond, qosp_lock_t &mutex, long timeout);
    int condSignal(qosp_cond_t &cond);

    /*funcoes relacionadas a manipulacao de mutex*/
    int mutexCreate(qosp_lock_t &mutex, int context);
    int mutexDestroy(qosp_lock_t &mutex);
    int lockAcquire(qosp_lock_t &lock);
    int lockRelease(qosp_lock_t &lock);

    int threadCreate(qosp_thread_t &thread, int context, void *proc, void *arg, int prio);
    void threadDestroy(qosp_thread_t &thread);
};

#endif	/* _MT_SUPPORT_H */


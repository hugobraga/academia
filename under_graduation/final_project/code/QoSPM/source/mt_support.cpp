#include "mt_support.h"
#include "tools.h"
#include <time.h> //struct timespec
#include <posix/errno.h>

#define TASK_MODE  0
#define TASK_STKSZ 0

/*
 * Destroi uma variavel condicional.
 * cond: ponteiro para a variavel condicional a ser destruida.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int MtSupporter::condDestroy(qosp_cond_t &cond) {
    if (cond.cond_domain == QOSP_XENOMAI_DOMAIN) {
        #ifdef RT_CONTEXT
        if (rt_cond_delete(&(cond.xeno_cond)))
            return 0;
        #endif
    } else {
        if (pthread_cond_destroy(&(cond.lin_cond)))
            return 0;
    }
    return 1;
}

/*
 * Cria uma variavel condicional.
 * cond: ponteiro para a variavel condicional a ser criada.
 * context: informa se a variavel condicional sera de tempo real ou do linux.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int MtSupporter::condCreate(qosp_cond_t &cond, int context) {
    if (context == QOSP_XENOMAI_DOMAIN) {
        #ifdef RT_CONTEXT
        if (rt_cond_create(&(cond.xeno_cond), NULL))
            return 0;
        #endif
    } else if (pthread_cond_init(&(cond.lin_cond), NULL))
            return 0;
    cond.cond_domain = context;
    return 1;
}

/*
 * Bloqueia em uma variavel condicional ate que ela seja sinalizada, ou um timeout ocorra.
 * cond: variavel condicional a ser bloqueada.
 * mutex: mutex para garantir exclusao mutua.
 * timeout: tempo maximo em nanossegundos a ser esperado para uma variavel condicional
 * ser sinalizada. Caso nao exista tempo maximo, deve-se passar como parametro -1.
 * retorno: 1 caso o procedimento seja executado com sucesso,
 * -1 caso o timeout tenha ocorrido antes da variavel ser sinalizada,
 * 0 caso contrario.
 */
int MtSupporter::condWait(qosp_cond_t &cond, qosp_lock_t &mutex, long timeout) {
    int ret;
    struct timespec timeout_str;
    if (cond.cond_domain == QOSP_XENOMAI_DOMAIN) {
        #ifdef RT_CONTEXT
        if ((ret = rt_cond_wait(&(cond.xeno_cond), &(mutex.xeno_lock), ((timeout != -1) ? (RTIME)timeout : TM_INFINITE)))) {
            if (ret == -ETIMEDOUT)
                return -1;
            return 0;
        }
        #endif
    } else {
        if (timeout != -1) {
            timeout_str.tv_nsec = timeout;
            timeout_str.tv_sec = 0;
            if ((ret = pthread_cond_timedwait(&(cond.lin_cond), &(mutex.lin_lock), &timeout_str))) {
                if (ret == ETIMEDOUT)
                    return -1;
                return 0;
            }
        } else if (pthread_cond_wait(&(cond.lin_cond), &(mutex.lin_lock)))
            return 0;
    }
    return 1;
}

/*
 * Sinaliza uma variavel condicional.
 * cond: ponteiro para a variavel condicional a ser sinalizada.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int MtSupporter::condSignal(qosp_cond_t &cond) {
    if (cond.cond_domain == QOSP_XENOMAI_DOMAIN) {
        #ifdef RT_CONTEXT
        if (rt_cond_signal(&(cond.xeno_cond)))
            return 0;
        #endif
    } else if (pthread_cond_signal(&(cond.lin_cond)))
            return 0;
    return 1;
}

/*
 * Cria um mutex.
 * mutex: ponteiro para o mutex.
 * context: informa se o mutex sera de tempo real ou do linux.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int MtSupporter::mutexCreate(qosp_lock_t &mutex, int context) {
    if (context == QOSP_XENOMAI_DOMAIN) {
        #ifdef RT_CONTEXT
        if (rt_mutex_create(&(mutex.xeno_lock), NULL))
            return 0;
        #endif
    } else {
        if (pthread_mutex_init(&(mutex.lin_lock), NULL))
            return 0;
    }
    mutex.lock_domain = context;
    return 1;
}

/*
 * Destroi um mutex.
 * mutex: ponteiro para o mutex a ser destruido.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int MtSupporter::mutexDestroy(qosp_lock_t &mutex) {
    if (mutex.lock_domain == QOSP_XENOMAI_DOMAIN) {
        #ifdef RT_CONTEXT
        if (rt_mutex_delete(&(mutex.xeno_lock)))
            return 0;
        #endif
    } else {
        if (pthread_mutex_destroy(&(mutex.lin_lock)))
            return 0;
    }
    return 1;
}

/*
 * Adquire um mutex.
 * lock: ponteiro para o mutex a ser adquirido.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int MtSupporter::lockAcquire(qosp_lock_t &lock) {
    if (lock.lock_domain == QOSP_XENOMAI_DOMAIN) {
        #ifdef RT_CONTEXT
        if (rt_mutex_acquire(&(lock.xeno_lock), TM_INFINITE))
            return 0;
        #endif
    } else {
        if (pthread_mutex_lock(&(lock.lin_lock)))
            return 0;
    }
    return 1;
}

/*
 * Libera um mutex.
 * lock: ponteiro para o mutex a ser liberado.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int MtSupporter::lockRelease(qosp_lock_t &lock) {
    if (lock.lock_domain == QOSP_XENOMAI_DOMAIN) {
        #ifdef RT_CONTEXT
        if (rt_mutex_release(&(lock.xeno_lock)))
            return 0;
        #endif
    } else {
        if (pthread_mutex_unlock(&(lock.lin_lock)))
            return 0;
    }
    return 1;
}

/*
 * Cria uma thread de execucao.
 * thread: ponteiro para o tipo thread.
 * context: informa se a thread sera de tempo real ou do linux.
 * proc: ponteiro para o procedimento que contem o codigo da thread.
 * arg: ponteiro para o argumento a ser passado para a thread.
 * prio: caso a thread seja de tempo real, sua prioridade devera ser informada.
 * Caso contrario, este parametro eh ignorado.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int MtSupporter::threadCreate(qosp_thread_t &thread, int context, void *proc, void *arg, int prio) {
    void (*rt_proc)(void *);
    void *(*nrt_proc)(void *);

    if (context == QOSP_XENOMAI_DOMAIN) {
        #ifdef RT_CONTEXT
        rt_proc = (void (*)(void  *))proc;
        if (rt_task_spawn(&(thread.xeno_thread), NULL, TASK_STKSZ, prio, TASK_MODE, rt_proc, arg))
            return 0;
        #endif
    } else {
        nrt_proc = (void *(*)(void  *))proc;
        if (pthread_create(&(thread.lin_thread), NULL, nrt_proc, arg))
            return 0;
    }
    thread.thread_domain = context;
    return 1;
}

/*
 * Destroi uma thread.
 * thread: ponteiro para o descritor da thread a ser destruida.
 */
void MtSupporter::threadDestroy(qosp_thread_t &thread) {
    if (thread.thread_domain == QOSP_XENOMAI_DOMAIN) {
        #ifdef RT_CONTEXT
        rt_task_delete(&(thread.xeno_thread));
        #endif
     } else
        pthread_cancel(thread.lin_thread);
}

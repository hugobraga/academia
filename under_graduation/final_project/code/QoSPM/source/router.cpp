#include "router.h"

/*
 * Construtor da classe Router.
 * sock_addr: endereco socket do roteador.
 * monitor: ponteiro do QoSPA responsavel por monitorar este roteador.
 */
Router::Router(struct sockaddr_in addr, QoSPA *monitor) : QoSPEntity(addr) {
    this->monitor = monitor;
}

/*
 * Retorna o QoSPA responsavel por monitorar este roteador.
 * retorno: o ponteiro do QoSPA que monitora este roteador.
 */
QoSPA* Router::getMonitor() {
    return this->monitor;
}

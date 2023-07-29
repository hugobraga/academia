#include "qospa.h"

QoSPA::QoSPA(struct sockaddr_in addr) : QoSPEntity(addr) {
}

void QoSPA::setRouter(Router* router) {
    this->router = router;
}

Router *QoSPA::getRouter() {
    return this->router;
}

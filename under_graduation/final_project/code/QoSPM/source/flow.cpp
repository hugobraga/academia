#include "flow.h"

Flow::Flow(int flowDesc) {
    this->flowDesc = flowDesc;
}

void Flow::setFlowDesc(int flowDesc) {
    this->flowDesc = flowDesc;
}

int Flow::getFlowDesc() {
    return this->flowDesc;
}

#include "monitor.h"
#include "conf.h"
#include <cstring>
#include <arpa/inet.h>
#include "network.h"

extern Network *glob_network_obj;
extern MtSupporter *glob_mt_supporter_obj;

Monitor::Monitor(string routerIP, list<string> &interfacesIP) {
    list<string>::iterator itr, end;
    vector<string>::iterator cbItr;
    vector<struct sockaddr_in>::iterator addrItr;
    struct sockaddr_in addr;
    string str, cbQoSPolicyIndex_str, cbQoSObjectsIndex_str, cbQosCMDropPkt_str;
    int index = 0;

    glob_mt_supporter_obj->mutexCreate(this->lockDesc, QOSP_LINUX_DOMAIN);
    this->routerInterfaces.reserve(MAX_ROUTER_IF);
    this->cbQosCMDropPkt.reserve(MAX_ROUTER_IF);
    this->prevDropPkt.reserve(MAX_ROUTER_IF);

    this->communityStr.assign(COMMUNITY_STR);
    this->timelyClass.assign(TIMELY_CLASS_NAME);

    this->snmp = new SNMPCisco(this->communityStr, routerIP);

    itr = interfacesIP.begin();
    end = interfacesIP.end();
    addrItr = this->routerInterfaces.begin();
    cbItr = this->cbQosCMDropPkt.begin();
    for (; itr != end; itr++, addrItr++, cbItr++, index++) {
        this->prevDropPkt[index] = 0;
        cbQosCMDropPkt_str.assign(cbQosCMDropPkt_STR);
        str = *itr;
        inet_aton(str.c_str(), &(addr.sin_addr));
        this->routerInterfaces.insert(addrItr, addr);
        this->snmp->getClassIndexes(str, this->timelyClass,
                cbQoSPolicyIndex_str, cbQoSObjectsIndex_str);
        cbQosCMDropPkt_str.append(cbQoSPolicyIndex_str.c_str());
        cbQosCMDropPkt_str.append(cbQoSObjectsIndex_str.c_str());
        this->cbQosCMDropPkt.insert(cbItr, cbQosCMDropPkt_str);
    }
}

/*
 * Verifica se a QoS contratada junto ao roteador continua sendo mantida.
 * retorno: 1 - caso a QoS continua sendo mantida. 0 - caso contrario.
 */
bool Monitor::isQoSMaintained() {
    int ret = true;
    vector<int> dropPkt(this->cbQosCMDropPkt.size());
    vector<int>::iterator itr, end, prevItr;

    this->snmp->getDropPkts(this->cbQosCMDropPkt.begin(), this->cbQosCMDropPkt.end(), dropPkt);

    itr = dropPkt.begin();
    end = dropPkt.end();
    prevItr = this->prevDropPkt.begin();
    for (; itr != end; itr++, prevItr++) {
        if ((*itr) > (*prevItr)) {
            this->prevDropPkt.insert(prevItr, (*itr));
            ret = false;
        }
    }
    return ret;
}

/*
 * Retorna o endereco de todos os QoSP inscritos junto ao agente. Quando o vetor
 * nao for mais necessario, a funcao releaseQoSPs deve ser chamada.
 * addrs: lista contendo todos os enderecos.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int Monitor::getSubscribedQoSPs(list<struct sockaddr_in> &addrs) {
    list<struct sockaddr_in>::iterator interestedItr, end, itr;

    if (!(glob_mt_supporter_obj->lockAcquire(this->lockDesc)))
        return 0;

    interestedItr = this->interestedQoSP.begin();
    end = this->interestedQoSP.end();
    itr = addrs.begin();
    for (; interestedItr != end; interestedItr++, itr++)
        addrs.insert(itr, *interestedItr);
    
    return 1;
}

/*
 * Libera o vetor que havia sido adquirido atraves da chamada getSubscribedQoSPs.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int Monitor::releaseQoSPs() {
    if (!(glob_mt_supporter_obj->lockRelease(this->lockDesc)))
        return 0;
    return 1;
}

/*
 * Inscreve um módulo do QoSP para receber informações sobre degeradacao.
 * qospAddr: endereco do QoSP a ser inscrito.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int Monitor::subscribe(struct sockaddr_in qospAddr) {
    if (!glob_mt_supporter_obj->lockAcquire(this->lockDesc))
        return 0;
    this->interestedQoSP.push_back(qospAddr);
    if (!glob_mt_supporter_obj->lockRelease(this->lockDesc))
        return 0;
    return 1;
}

/*
 * Cancela a inscrição de um módulo do QoSP. Este não receberá mais informações acerda da degradacao.
 * qospAddr: endereco do QoSP cuja inscricao sera cancelada.
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int Monitor::unSubscribe(struct sockaddr_in qospAddr) {
    list<struct sockaddr_in>::iterator itr, end;
    
    if (!(glob_mt_supporter_obj->lockAcquire(this->lockDesc)))
        return 0;

    for (; itr != end; itr++) {
        if (glob_network_obj->equalAddr(qospAddr, *itr)) {
            this->interestedQoSP.erase(itr);
            break;
        }
    }
    
    if (!glob_mt_supporter_obj->lockRelease(this->lockDesc))
        return 0;
    return 1;
}

/*
 * Cancela a inscricao de todos os QoSP inscritos
 * retorno: 1 caso o procedimento seja executado com sucesso, 0 caso contrario.
 */
int Monitor::unSubscribeAll(){
    if (!(glob_mt_supporter_obj->lockAcquire(this->lockDesc)))
        return 0;
    this->interestedQoSP.clear();
    if (!glob_mt_supporter_obj->lockRelease(this->lockDesc))
        return 0;
    return 1;
}

Monitor::~Monitor() {
    glob_mt_supporter_obj->mutexDestroy(this->lockDesc);
    this->interestedQoSP.~list<struct sockaddr_in>();
    this->routerInterfaces.~vector<struct sockaddr_in>();
    this->cbQosCMDropPkt.~vector<string>();
    this->prevDropPkt.~vector<int>();
    delete this->snmp;
}

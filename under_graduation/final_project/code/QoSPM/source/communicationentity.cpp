/*
 * File:   teste.cpp
 * Author: hugo
 *
 * Created on 24 de Janeiro de 2009, 16:39
 */

#include "communicationentity.h"


CommunicationEntity::CommunicationEntity(){
    
}

CommunicationEntity::CommunicationEntity(struct sockaddr_in sockAddr) {
    this->sockAddr = sockAddr;
}

/*
 * Compara se uma CommunicationEntity possui o mesmo endereco IP (levando em consideracao
 * o endereco IP da camada de rede e a porta da camada de transporte) de outra
 * CommunicationEntity.
 * element: a CommunicationEntity com a qual sera comparada o endereco IP.
 * retorno: true - caso os enderecos IP sejam iguais.
 *          false - caso contrario.
 */
bool CommunicationEntity::equal(CommunicationEntity &element) {
    if ((this->sockAddr.sin_addr.s_addr == element.getAddress().sin_addr.s_addr) &&
            (this->sockAddr.sin_port == element.getAddress().sin_port))
        return true;
    return false;
}

/*
 * Compara se uma CommunicationEntity possui o mesmo endereco IP de rede de outra
 * CommunicationEntity.
 * element: a CommunicationEntity com a qual sera comparada o endereco IP de rede.
 * retorno: true - caso os enderecos IP sejam iguais.
 *          false - caso contrario.
 */
bool CommunicationEntity::equalIP(CommunicationEntity &element) const{
    if (this->sockAddr.sin_addr.s_addr == element.getAddress().sin_addr.s_addr)
        return true;
    else
        return false;
}

/*
 * Retorna o endereco socket da CommunicationEntity.
 * retorno: endereco socket da CommunicationEntity.
 */
struct sockaddr_in CommunicationEntity::getAddress() {
    return this->sockAddr;
}

void CommunicationEntity::setAddress(struct sockaddr_in addr) {
    this->sockAddr = addr;
}

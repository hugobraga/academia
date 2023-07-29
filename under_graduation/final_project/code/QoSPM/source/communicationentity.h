/* 
 * File:   communicationentity.h
 * Author: hugo
 *
 * Created on 24 de Janeiro de 2009, 15:54
 */

#ifndef _COMMUNICATIONENTITY_H
#define	_COMMUNICATIONENTITY_H

#include <netinet/in.h> //struct sockaddr_in

class CommunicationEntity {
    struct sockaddr_in  sockAddr;
public:
    CommunicationEntity();
    CommunicationEntity(struct sockaddr_in sockAddr);
    bool equal(CommunicationEntity &element);
    bool equalIP(CommunicationEntity &element) const;
    struct sockaddr_in getAddress();
    void setAddress(struct sockaddr_in addr);
private:

};

#endif	/* _COMMUNICATIONENTITY_H */


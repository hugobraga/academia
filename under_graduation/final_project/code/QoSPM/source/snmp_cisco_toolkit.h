/* 
 * File:   snmp_cisco_toolkit.h
 * Author: hugo
 *
 * Created on 14 de Março de 2009, 21:34
 */

#ifndef _SNMP_CISCO_TOOLKIT_H
#define	_SNMP_CISCO_TOOLKIT_H

/** @def SNMP_FREE(s)
    Frees a pointer only if it is !NULL and sets its value to NULL */
//para resolver problemas de compilacao, foi necessario copiar esta linha de um arquivo cabecalho
#define SNMP_FREE(s)    do { if (s) { free((void *)s); s=NULL; } } while(0)

#include <net-snmp/net-snmp-config.h>
#include <net-snmp/net-snmp-includes.h>
#include <string>
#include <vector>

#define COMMUNITY_LENGTH    10
#define IP_LENGTH           15

using namespace std;

class SNMPCisco {
    struct snmp_session snmpSession;
    struct snmp_session *ss;
    char                communityStr[COMMUNITY_LENGTH];
    string              peername;
private:
    int snmpwalk(netsnmp_variable_list **walkList, oid *mibOid, size_t oidLen);
public:
    SNMPCisco(string community, string peerAddr);
    int getDropPkts(vector<string>::iterator cbQosCMDropPkt, vector<string>::iterator end, vector<int> &output);
    int getClassIndexes(string interfaceIp, string className, string &cbQoSPolicyIndex, string &cbQoSObjectsIndex);
};


#endif	/* _SNMP_CISCO_TOOLKIT_H */


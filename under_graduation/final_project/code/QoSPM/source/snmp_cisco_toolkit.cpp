/*
 * snmp_cisco_toolkit.c - Funcoes para capturar valores de objetos MIB da CISCO-CLASS-BASED-QOS-MIB
 * nos roteadores Cisco. Caso necessario, acesse esta MIB para maiores detalhes.
 *
 * Ultima modificacao: 21/11/08
 * Autor: Hugo Braga
 */

#include "snmp_cisco_toolkit.h"
#include <string.h> //strcpy

using namespace std;

/*
 * Executa uma caminhada em uma arvore MIB (semelhante ao snmpwalk)
 * mibOid: oid do objeto MIB para o qual sera executada a caminhada
 * oidLen: tamanho do mib_oid
 * ss: descritor da sessao na qual sera executado o procedimento.
 * retorno: 1 - caso a caminhada seja executada com sucesso. 0 - caso contrario
 */
int SNMPCisco::snmpwalk(netsnmp_variable_list **walkList, oid *mibOid, size_t oidLen) {
    *walkList = NULL;
    if (!snmp_varlist_add_variable(walkList, mibOid, oidLen, ASN_NULL, NULL, 0))
        return 0;
    if (netsnmp_query_walk(*walkList, this->ss) != SNMP_ERR_NOERROR)
        return 0;
    return 1;
}

/*
 * Representa o construtor da classe SNMPCISCO.
 * community: community string do elemento para efetuar trocas de mensagens SNMP.
 * peerAddr: endereco do elemento com o qual sera trocado mensagens SNMP.
 */
SNMPCisco::SNMPCisco(string community, string peerAddr) {
    char peerName[IP_LENGTH];
    strcpy(this->communityStr, community.c_str());
    //this->communityStr.assign(community.c_str());
    this->peername.assign(peerAddr.c_str());
    init_snmp("");
    snmp_sess_init(&(this->snmpSession));
    this->snmpSession.version = SNMP_VERSION_2c;
    this->snmpSession.community = (unsigned char *)this->communityStr;
    this->snmpSession.community_len = strlen((char *)this->snmpSession.community);
    strcpy(peerName, this->peername.c_str());
    this->snmpSession.peername = peerName;
    this->ss = snmp_open(&(this->snmpSession));
}

/*
 * Retorna a quantidade de pacotes descartados em uma determinada classe de cada interface.
 * Recupera o valor do objeto MIB cbQoSCMDropPkt definido na MIB CISCO-CLASS-BASED-QOS-MIB.
 * cbQoSCMDropPkt: iterador para os cbQoSCMDropPkt das classes em formato string.
 * end: iterador que aponta para o fim.
 * output: contera a quantidade de pacotes descartados em cada interface.
 * retorno: 1 caso o procedimento tenha sido executado corretamente, 0 caso contrario.
 */
int SNMPCisco::getDropPkts(vector<string>::iterator cbQosCMDropPkt, vector<string>::iterator end, vector<int> &output) {
    netsnmp_pdu *pdu, *response;
    netsnmp_variable_list *vars;
    oid     cbQosCMDropPkt_oid[MAX_OID_LEN];
    string oidStr;
    size_t  oidLen;
    int     status;

    if (!(pdu = snmp_pdu_create(SNMP_MSG_GET)))
        return 0;
    for (; cbQosCMDropPkt != end; cbQosCMDropPkt++) {
        oidStr = *cbQosCMDropPkt;
        if (!read_objid(oidStr.c_str(), cbQosCMDropPkt_oid, &oidLen))
            return 0;
        if (!snmp_add_null_var(pdu, cbQosCMDropPkt_oid, oidLen))
            return 0;
    }
    status = snmp_synch_response(this->ss, pdu, &response);
    if (status != STAT_SUCCESS || !response) {
        snmp_sess_perror("snmpget", this->ss);
        return -1;
    }
    for (vars = response->variables; vars; vars = vars->next_variable)
        output.push_back((int)*(vars->val.integer));
    snmp_free_pdu(response);
    return 1;
}

/*
 * Recupera os indexes necessarios para identificar uma classe na politica vinculada a saida da interface
 * identificada por interface_ip.
 * Estes indexes sao necessarios para acessar as informacoes estatisticas relativas
 * as classes na MIB CISCO-CLASS-BASED-QOS-MIB.
 * interfaceIp: ip da interface na qual citada anteriormente.
 * className: nome da classe para o qual os indexes serao recuperados.
 * cbQoSObjectsIndex: contera o index cbQoSPolicyIndex da politica vinculada
 * a saida da interface citada anteriormente.
 * cbQoSObjectsIndex: contera o index cbQoSObjectsIndex da classe na politica
 * vinculada a saida da interface citada anteriormente.
 * ss: descritor da sessao na qual sera executado o procedimento.
 * retorno: 1 - caso o procedimento seja executado com sucesso. 0 - caso contrario.
 */
int SNMPCisco::getClassIndexes(string interfaceIp, string className, string &cbQoSPolicyIndex, string &cbQoSObjectsIndex) {
    netsnmp_pdu *pdu, *response;
    netsnmp_variable_list *vars, *vars2;
    size_t  oidLen;
    int     status, index;
    char    str[MAX_OID_LEN];

    long     if_index; //ifIndex da interface de interesse
    char    ipAddEntIfIndex_str[MAX_OID_LEN]  = { ".1.3.6.1.2.1.4.20.1.2." };
    oid     ipAddEntIfIndex_oid[MAX_OID_LEN];

    char    cbQoSIfIndex_str[MAX_OID_LEN] = { ".1.3.6.1.4.1.9.9.166.1.1.1.1.4" };
    oid     cbQoSIfIndex_oid[MAX_OID_LEN];
    oid     possible_cbQoSPolicyIndex[2];

    char    cbQoSPolicyDirection_str[MAX_OID_LEN] = { ".1.3.6.1.4.1.9.9.166.1.1.1.1.3" };
    oid     cbQoSPolicyDirection_oid[MAX_OID_LEN];

    char    cbQoSCMName_str[MAX_OID_LEN] = { ".1.3.6.1.4.1.9.9.166.1.7.1.1.1" };
    oid     cbQoSCMName_oid[MAX_OID_LEN];
    int     class_config_index;
    char    class_config_index_str[MAX_OID_LEN];

    char    cbQoSConfigIndex_str[MAX_OID_LEN] = { ".1.3.6.1.4.1.9.9.166.1.5.1.1.2" };
    oid     cbQoSConfigIndex_oid[MAX_OID_LEN];

    //descobrir o ifIndex da interface de interesse
    if (!(pdu = snmp_pdu_create(SNMP_MSG_GET)))
        return 0;
    oidLen = MAX_OID_LEN;
    strcat(ipAddEntIfIndex_str, interfaceIp.c_str());
    if (!read_objid(ipAddEntIfIndex_str, ipAddEntIfIndex_oid, &oidLen)) {
        return 0;
    }
    if (!snmp_add_null_var(pdu, ipAddEntIfIndex_oid, oidLen))
        return 0;
    status = snmp_synch_response(this->ss, pdu, &response);
    if (status != STAT_SUCCESS || !response)
        return 0;
    if_index = *(response->variables->val.integer);
    snmp_free_pdu(response);

    //obter o cbQoSPolicyIndex da politica vinculada a interface em questao
    oidLen = MAX_OID_LEN;
    if (!read_objid(cbQoSIfIndex_str, cbQoSIfIndex_oid, &oidLen))
        return 0;
    if (!this->snmpwalk(&vars, cbQoSIfIndex_oid, oidLen))
        return 0;
    for (index = 0, vars2 = vars; vars2; vars2 = vars2->next_variable) {
        if (*vars2->val.integer == if_index) {
            possible_cbQoSPolicyIndex[index++] = vars2->name[vars2->name_length - 1];
        }
    }
    snmp_free_var(vars);

    if (index > 1) { /*existem duas politicas vinculadas a interface
     (uma na entrada outra na saida). Temos que descobrir o cbQoSPolicyIndex da saida*/
        oidLen = MAX_OID_LEN;
        if (!read_objid(cbQoSPolicyDirection_str, cbQoSPolicyDirection_oid, &oidLen))
            return 0;
        if (!this->snmpwalk(&vars, cbQoSPolicyDirection_oid, oidLen))
            return 0;
        for (index = 0, vars2 = vars; vars2; vars2 = vars2->next_variable) {
            if (*(vars2->val.integer) == 2) { //esta sendo aplicada a saida da interface
                if (vars2->name[vars2->name_length - 1] == possible_cbQoSPolicyIndex[0] ||
                        vars2->name[vars2->name_length - 1] == possible_cbQoSPolicyIndex[1]) {
                    snprint_objid(str, MAX_OID_LEN, &(vars2->name[vars2->name_length - 1]), 1);
                    cbQoSPolicyIndex.assign(str);
                    break;
                }
            }
        }
        snmp_free_var(vars);
    } else {
        snprint_objid(str, MAX_OID_LEN, &(possible_cbQoSPolicyIndex[0]), 1);
        cbQoSPolicyIndex.assign(str);
    }

    //descobrir o cbQoSConfigIndex da classe
    oidLen = MAX_OID_LEN;
    if (!read_objid(cbQoSCMName_str, cbQoSCMName_oid, &oidLen))
        return 0;
    if (!this->snmpwalk(&vars, cbQoSCMName_oid, oidLen))
        return 0;
    for (index = 0, vars2 = vars; vars2; vars2 = vars2->next_variable) {
        if (!memcmp(vars2->val.string, className.c_str(), strlen(className.c_str()))) {/*eh um armengue,
         visto que a string esta vindo com sujeira, ou seja, eu nao posso comparar apenas a string mas
         tambem tenho que levar em consideracao o tamanho da string*/
            snprint_objid (class_config_index_str, MAX_OID_LEN, &(vars2->name[vars2->name_length - 1]), 1);
            class_config_index = atoi(strtok((char *)class_config_index_str, "."));
            break;
        }
    }
    snmp_free_var(vars);

    //descobrir o cbQoSObjectsIndex da classe
    oidLen = MAX_OID_LEN;
    strcat(cbQoSConfigIndex_str, cbQoSPolicyIndex.c_str());
    if (!read_objid(cbQoSConfigIndex_str, cbQoSConfigIndex_oid, &oidLen))
        return 0;
    if (!this->snmpwalk(&vars, cbQoSConfigIndex_oid, oidLen))
        return 0;
    for (index = 0, vars2 = vars; vars2; vars2 = vars2->next_variable) {
        if ((int)*(vars2->val.integer) == class_config_index) {
            snprint_objid(str, MAX_OID_LEN, &(vars2->name[vars2->name_length - 1]), 1);
            cbQoSObjectsIndex.assign(str);
            break;
        }
    }
    snmp_free_var(vars);
    return 1;
}


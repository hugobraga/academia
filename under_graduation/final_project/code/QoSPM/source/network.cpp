#include "network.h"
#include "qospm_protocol.h"
#include "mt_support.h"
#include <string.h>
#include <arpa/inet.h>
#include <sys/types.h> //u_int16_t e u_short
#include <netinet/ip.h> //struct ip
#include <sys/socket.h> //struct msghdr
#include <netdb.h> //struct protoent
#include <netinet/ip_icmp.h> //struct icmp
#ifdef RT_CONTEXT
#include <rtdm/rtdm.h>
//#include <rtnet.h> //RTNET_RTIOC_EXTPOOL
#define RTNET_RTIOC_EXTPOOL 2
#endif

using namespace std;

/*
 * Inicializa um socket.
 * sock: ponteiro para um descritor de socket.
 * context: indica se o socket sera de tempo real ou nao.
 * type: socket bruto (RAW_SOCKET) ou um socket datagrama (SOCK_DGRAM).
 * sockPort: indica a porta na qual sera vinculada o socket.
 * retorno: 1 se o procedimento foi realizado com sucesso, 0 caso contrario.
 */
int Network::initializeSocket(qosp_sock_t &sock, int context, int type, int sockPort) {
    int ret;
    uint16_t port = sockPort;
    struct protoent *proto;
    unsigned int addRtskbs = 75;
    struct sockaddr_in localAddr; //utilizado apenas para vincular um socket a uma porta

    if (type == SOCK_RAW) {
        //capturando o valor do protocolo
        if ((proto = getprotobyname("icmp")) == NULL) {
            return 0;
        }
        if (context == QOSP_XENOMAI_DOMAIN) {
            #ifdef RT_CONTEXT
            sock.sock_desc = rt_dev_socket(AF_INET,SOCK_RAW,proto->p_proto);
            #endif
        } else
            sock.sock_desc = socket(AF_INET, SOCK_RAW, proto->p_proto);
    }else {
        if ((proto = getprotobyname("udp")) == NULL) {
            return 0;
        }
        if (context == QOSP_XENOMAI_DOMAIN) {
            #ifdef RT_CONTEXT
            sock.sock_desc = rt_dev_socket(AF_INET,SOCK_DGRAM,0);
            #endif
        } else
            sock.sock_desc = socket(AF_INET, SOCK_DGRAM, proto->p_proto);
    }

    if (sock.sock_desc < 0)
        return 0;
    sock.sock_domain = context;

    if (context == QOSP_XENOMAI_DOMAIN) {
        /* extend the socket pool */
        #ifdef RT_CONTEXT
        ret = rt_dev_ioctl(sock.sock_desc, RTNET_RTIOC_EXTPOOL, &addRtskbs);
        if (ret != (int)addRtskbs) {
            rt_dev_close(sock.sock_desc);
            return 0;
        }

        /* bind the rt-socket to a port */
        bzero((char *) &localAddr, sizeof(localAddr));
        localAddr.sin_family = AF_INET;
        localAddr.sin_port = htons(port);
        localAddr.sin_addr.s_addr = htons(INADDR_ANY);
        ret = rt_dev_bind(sock.sock_desc, (struct sockaddr *)&localAddr, sizeof(struct sockaddr_in));
        if (ret < 0) {
            rt_dev_close(sock.sock_desc);
            return 0;
        }
        #endif
    } else {
        bzero((char *) &localAddr, sizeof(localAddr));
        localAddr.sin_family = AF_INET;
        localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        localAddr.sin_port = htons(sockPort);
        if (bind(sock.sock_desc, (struct sockaddr *) &localAddr, sizeof(localAddr)) < 0)
            return 0;
    }
    return 1;
}

/*
 * Envia uma mensagem para um endereco qualquer por um socket.
 * sock: ponteiro para o descritor do socket.
 * destAddr: endereco para o qual sera enviada a mensagem.
 * opMsg: vetor contendo a mensagem.
 * msgSize: tamanho da mensagem a ser enviada.
 * retorno: retorna o numero de bytes enviado ou -1 caso contrario.
 */
int Network::sendMsg(qosp_sock_t &sock, struct sockaddr_in destAddr, char *opMsg, int msgSize) {
    int ret;
    struct msghdr msg; //mensagem que sera enviada
    struct iovec iov;//buffer de dados

    iov.iov_base = opMsg; //end da area de dados
    iov.iov_len = msgSize; //tamanho da area de dados

    bzero((char *) &msg, sizeof(msg));
    msg.msg_name    = &destAddr; //socket
    msg.msg_namelen = sizeof(destAddr); //tamanho da estrutura do socket
    msg.msg_iov     = &iov; //endereço do buffer dos dados
    msg.msg_iovlen  = 1; //numero de buffer de dados
    msg.msg_control = 0;
    msg.msg_controllen = 0;
    msg.msg_flags = 0;

    if (sock.sock_domain == QOSP_XENOMAI_DOMAIN) {
        #ifdef RT_CONTEXT
        ret = rt_dev_sendmsg(sock.sock_desc, &msg, 0);
        #endif
    } else
        ret = sendmsg(sock.sock_desc, &msg, 0);

    //if (ret == -EBADF) //falhou em enviar a mensagem
    if (ret < 0)
        return 0;
    else if (ret != msgSize)
        return 0;
    else
        return ret;
}

u_short _checksum(u_short *addr, int len) {
    u_short *w = addr, answer;
    int sum = 0;

    while( len > 0 )  {
        sum += *w++;
        len -= 2;
    }
    answer= ~sum;
    return (answer);
}

/*
 * Envia um ping (msg ECHO_REQUEST) para um determinado elemento de rede.
 * pingSock: descritor do raw socket pelo qual sera enviado a mensagem.
 * seqId: um numero de sequencia qualquer.
 * msgId: um id qualquer que serve para identificar o tipo de mensagem.
 */
int Network::ping(qosp_sock_t *pingSock, struct sockaddr_in destAddr, u_int16_t seqId, u_int16_t msgId) {
    struct icmp tempIcmp;
    int msgLength = sizeof(tempIcmp.icmp_type)+sizeof(tempIcmp.icmp_code)+sizeof(tempIcmp.icmp_cksum)+sizeof(tempIcmp.icmp_seq)+sizeof(tempIcmp.icmp_id);
    char pingMsg[msgLength];
    struct icmp *icp = (struct icmp *) pingMsg;

    icp->icmp_type = ICMP_ECHO;
    icp->icmp_code = 0;
    icp->icmp_cksum = 0;
    icp->icmp_seq = seqId;
    icp->icmp_id = msgId;
    icp->icmp_cksum = _checksum((u_short *)icp, msgLength);
    return (this->sendMsg(*pingSock, destAddr, pingMsg, msgLength));
}

/*
 * Recebe uma mensagem em um socket.
 * sock: descritor do socket.
 * msgAddr: contera a mensagem recebida incluindo cabecalhos.
 * msgData: contera apenas a mensagem recebida.
 * maxMsgLength: tamanho maximo da mensagem a ser recebida.
 * hostAddr: contera o endereco do host que enviou a mensagem.
 * sockType: tipo do socket (SOCK_RAW, SOCK_DGRAM) pelo qual recebera a mensagem.
 * retorno: o mesmo de recvmsg.
 */
int Network::receiveMsg(qosp_sock_t &sock, char *msgAddr, char* &msgData, int maxMsgLength, struct sockaddr_in *hostAddr, int sockType) {
    struct msghdr msg; //mensagem que sera recebida
    struct iovec iov;//buffer de dados
    struct ip *ip;
    int ret;

    iov.iov_base = msgAddr; //end da area de dados
    ip = (struct ip *) msgAddr;
    iov.iov_len = maxMsgLength; //tamanho da area de dados

    bzero((char *) &msg, sizeof(msg));
    msg.msg_name    = hostAddr; //endereço do host que enviou a resposta
    msg.msg_namelen = sizeof(*hostAddr); //tamanho da estrutura do endereço
    msg.msg_iov     = &iov; //endereço do buffer dos dados
    msg.msg_iovlen  = 1; //numero de buffer de dados

    if (sock.sock_domain == QOSP_XENOMAI_DOMAIN) {
        #ifdef RT_CONTEXT
        ret = rt_dev_recvmsg(sock.sock_desc, &msg, 0);
        msgData = (msgAddr + ip->ip_hl);
        #endif
        return (ret);
    } else {
        ret = recvmsg(sock.sock_desc, &msg, 0);
        //printf("erro: %d\n", errno);
        //cout << "erro: " << msg.msg_flags << "\n";
        //cout << "uhhhhhhhhhhhh\n";
        if (sockType == SOCK_DGRAM)
            msgData = msgAddr;
        else
            msgData = (msgAddr + IP_HEADER_LENGTH);
        return (ret);
    }
}

/*
 * Recebe a responsta de um ping (enviado pela funcao ping).
 * pingSock: descritor do raw socket pelo qual sera enviado a mensagem.
 * buf: contera o mensagem que corresponde a resposta do ping.
 * icp: contera a resposta de acordo com a estrutura icmp.
 * maxMsgLength: tamanho maximo da mensagem a ser recebida.
 * retorno: o mesmo de receive_msg.
 */
int Network::receivePingReply(qosp_sock_t *sock, char *buf, struct icmp* &icp, int maxMsgLength) {
    char *bufData;
    struct sockaddr_in addr;
    int ret;

    //cout << "receivePingReply\n";
    ret = this->receiveMsg(*sock, buf, bufData, maxMsgLength, &addr, SOCK_RAW);
    icp = (struct icmp *)(bufData);
    return ret;
}

/*
 * Copia os valores da estrutura addr para new_addr.
 * addr: estrutura da qual serao copiados os valores.
 * newAddr: ponteiro para a estrutura que contera os valores.
 */
void Network::copySockAddr(struct sockaddr_in addr, struct sockaddr_in &newAddr) {
    newAddr.sin_addr.s_addr = addr.sin_addr.s_addr;
    newAddr.sin_family = addr.sin_family;
    newAddr.sin_port = addr.sin_port;
}

/*
 * Verifica se dois enderecos socket sao iguais.
 * addr e addr2: enderecos socket que serao copiados.
 * retorno: 1 caso sejam iguais, 0 caso contrario.
 */
int Network::equalAddr(struct sockaddr_in addr, struct sockaddr_in addr2) {
    if ((addr.sin_addr.s_addr == addr2.sin_addr.s_addr) && (addr.sin_port == addr2.sin_port))
        return 1;
    else
        return 0;
}

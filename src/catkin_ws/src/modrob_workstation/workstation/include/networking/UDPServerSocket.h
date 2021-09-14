//
// Created by lukas on 28.11.19.
//

#ifndef RCI_UDPSERVERSOCKET_H
#define RCI_UDPSERVERSOCKET_H

#include <netinet/in.h>

class UDPServerSocket {
private:
    int sockfd, connfd;
    socklen_t len;
    struct sockaddr_in servaddr, cliaddr;

public:
    explicit UDPServerSocket(uint16_t port);

    ~UDPServerSocket();

    void send(void *data, size_t count);

    void receive(void *data, size_t count);
};


#endif //RCI_UDPSERVERSOCKET_H
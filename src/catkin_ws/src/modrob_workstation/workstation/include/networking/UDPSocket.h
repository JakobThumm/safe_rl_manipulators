//
// Created by lukas on 24.11.19.
//

#ifndef RCI_UDPSOCKET_H
#define RCI_UDPSOCKET_H

#include <netinet/in.h>

class UDPSocket {
private:
    int sockfd;
    struct sockaddr_in servaddr;
    socklen_t len;

public:
    explicit UDPSocket(const char target_ip[], uint16_t port);

    ~UDPSocket();

    void send(void *data, size_t count);

    void receive(void *data, size_t count);


};


#endif //RCI_UDPSOCKET_H

//
// Created by lukas on 20.11.19.
//

#ifndef RCI_TCP_SERVER_SOCKET_H
#define RCI_TCP_SERVER_SOCKET_H

#include <netinet/in.h>


class TCPServerSocket {
    private:
        int sockfd, connfd;
        socklen_t len;
        struct sockaddr_in servaddr, cli;

    public:
        explicit TCPServerSocket(uint16_t port);

        ~TCPServerSocket();

        void send(void *data, size_t count);

        void receive(void *data, size_t count);


};


#endif

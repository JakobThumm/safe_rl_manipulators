#ifndef RCI_TCP_SOCKET_H
#define RCI_TCP_SOCKET_H

#include <netinet/in.h>


class TCPSocket {
    private:
        int sockfd;
        struct sockaddr_in servaddr;

    public:
        explicit TCPSocket(const char connection_ip[], uint16_t port);

        ~TCPSocket();

        void send(void *data, size_t count);

        void receive(void *data, size_t count);
};


#endif

//
// Created by lukas on 20.11.19.
//

#include "TCPServerSocket.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
//#include <libnet.h>
#include <string.h>

#define ERROR -1
#define SUCCESS 0
#define STANDARD_PROTOCOL 0
#define NO_BACKLOG 1

TCPServerSocket::TCPServerSocket(uint16_t port) {
    //Create socket
    sockfd = socket(AF_INET, SOCK_STREAM, STANDARD_PROTOCOL);
    if (sockfd == ERROR) {
        fprintf(stderr, "Socket creation failed");
        exit(ERROR);
    }

    //Assign IP / Port internally
    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(port);

    // Bind socket to IP
    if ((bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr))) != 0) {
        fprintf(stderr, "Socket bind failed \n");
        exit(0);
    }

    //Listen for attempted connections
    if (listen(sockfd, NO_BACKLOG) != SUCCESS) {
        fprintf(stderr, "Listen failed \n");
        exit(0);
    }

    //Accept incoming connection
    len = sizeof(cli);
    connfd = accept(sockfd, (struct sockaddr*) &cli, &len);
    if (connfd == ERROR) {
        fprintf(stderr, "Server acccept failed \n");
        exit(ERROR);
    }
}

TCPServerSocket::~TCPServerSocket() {
    close(sockfd);
}

void TCPServerSocket::send(void *data, size_t count) {
    write(connfd, data, count);
}

void TCPServerSocket::receive(void *data, size_t count) {
    read(connfd, data, count);
}

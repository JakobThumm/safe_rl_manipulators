#include "networking/TCPSocket.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
//#include <libnet.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <iostream>

#define ERROR -1
#define SUCCESS 0
#define STANDARD_PROTOCOL 0
#define CONNECTION_TIMEOUT 5
#define CONNECT_INTERVAL 1

TCPSocket::TCPSocket(const char target[], uint16_t PORT) {
    //Create Socket
    sockfd = socket(AF_INET, SOCK_STREAM, STANDARD_PROTOCOL);
    if (sockfd == ERROR) {
        fprintf(stderr, "Socket creation failed\n");
        exit(EXIT_FAILURE);
    }
    
    //Assign IP / Port internally
    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(target);
    servaddr.sin_port = htons(PORT);
    
    //Establish connection
    int timeout = 0;
    if (connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == ERROR) {
        std::cout << "TCP Connection cant be established" << std::endl;
       //exit(EXIT_FAILURE);
    }
}

TCPSocket::~TCPSocket() {
    close(sockfd);
}

void TCPSocket::send(void *data, size_t count) {
   if (write(sockfd, data, count) == ERROR) {
       std::cout << "TCP Connection could not write" << std::endl;
   };
}

void TCPSocket::receive(void *data, size_t count) {
    if (read(sockfd, data, count) == ERROR){
        std::cout << "TCP Connection could not read" << std::endl;
    };
}

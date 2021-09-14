#include "networking/UDPServerSocket.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define ERROR -1
#define SUCCESS 0
#define DEFAULT 0
#define STANDARD_PROTOCOL 0
#define CONNECTION_TIMEOUT 60
#define CONNECT_INTERVAL 5

UDPServerSocket::UDPServerSocket(uint16_t PORT) {
    len = sizeof(cliaddr);

    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, STANDARD_PROTOCOL)) == ERROR) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Filling server information
    servaddr.sin_family = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    // Bind the socket with the server address
    if (bind(sockfd, (const struct sockaddr *) &servaddr,
             sizeof(servaddr)) != SUCCESS) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
}

UDPServerSocket::~UDPServerSocket() {
    close(sockfd);
}

void UDPServerSocket::send(void *data, size_t count) {
    sendto(sockfd, data, count, MSG_CONFIRM, (const struct sockaddr *) &cliaddr, sizeof(cliaddr));
}

void UDPServerSocket::receive(void *data, size_t count) {
    recvfrom (sockfd, data, count, DEFAULT, (struct sockaddr *) &cliaddr, &len);
}
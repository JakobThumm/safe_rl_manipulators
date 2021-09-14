#include "networking/UDPSocket.h"
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

UDPSocket::UDPSocket(const char target[], uint16_t PORT) {
    len = sizeof(servaddr);

    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, STANDARD_PROTOCOL)) == ERROR) {
        fprintf(stderr, "Socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, DEFAULT, sizeof(servaddr));

    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = inet_addr(target);
}

UDPSocket::~UDPSocket() {
    close(sockfd);
}

void UDPSocket::send(void *data, size_t count) {
    sendto(sockfd, data, count, MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr));
}

void UDPSocket::receive(void *data, size_t count) {
    recvfrom(sockfd, data, count, DEFAULT, (struct sockaddr *) &servaddr, &len);
}
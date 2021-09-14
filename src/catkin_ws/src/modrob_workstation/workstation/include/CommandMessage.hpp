//
// Created by lukas on 28.01.20.
//
#include "RobotCommanded.hpp"
#include <cstdint>

#ifndef WORKSTAION_COMMANDMESSAGE_H
#define WORKSTAION_COMMANDMESSAGE_H


class CommandMessage {
private:
    uint8_t msgType;
    uint16_t payloadLength;
    RobotCommanded *robotCommanded;
    uint16_t privateSeqNum;
    static uint16_t seqNum;
public:

    CommandMessage(RobotCommanded *robotCommanded);

    void serialize(char *buffer);

    int getSize();
};


#endif //WORKSTAION_COMMANDMESSAGE_H

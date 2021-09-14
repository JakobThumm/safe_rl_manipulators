
#include "CommandMessage.hpp"
uint16_t CommandMessage::seqNum = 0;

void CommandMessage::serialize(char *buffer) {
    *((uint8_t *) buffer++) = this->msgType;
    *((uint16_t *) buffer) = this->payloadLength;
    buffer+=sizeof(uint16_t);
    *((uint16_t *) buffer) = this->privateSeqNum;
    buffer+=sizeof(uint16_t);
    this->robotCommanded->serialize(buffer);
}

int CommandMessage::getSize() {
    return sizeof(uint8_t) + 2 * sizeof(uint16_t) + this->robotCommanded->getSize();
}

CommandMessage::CommandMessage(RobotCommanded *robotCommanded) {
    this->msgType = robotCommanded->getType();
    this->payloadLength = robotCommanded->getSize();
    this->robotCommanded = robotCommanded;
    this->privateSeqNum = CommandMessage::seqNum++;
}
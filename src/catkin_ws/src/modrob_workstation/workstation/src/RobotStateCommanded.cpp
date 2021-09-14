#include "RobotStateCommanded.hpp"

#define STATE_MESSAGE_TYPE 6

RobotStateCommanded::RobotStateCommanded(uint8_t state) {
    this->state = state;
    this->type = STATE_MESSAGE_TYPE;
}

uint16_t RobotStateCommanded::getSize() {
    return sizeof(uint8_t);
}

char * RobotStateCommanded::serialize(char * result) {
    *result++ = this->state;
    return result;
}

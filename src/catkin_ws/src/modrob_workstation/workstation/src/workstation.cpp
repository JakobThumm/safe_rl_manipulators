
#include "workstation.hpp"
#include "CommandMessage.hpp"
#include "RobotConfigMeasured.hpp"
#include "RobotModuleOrder.hpp"



#define DEFAULT_PORT_UDP 25000
#define DEFAULT_PORT_TCP 25001
#define USE_TCP_MESSAGES false
#define STATE_MESSAGE_TYPE 6
#define BYTES_PER_JOINT 36
#define MEASUREMENT_MESSAGE_TYPE 5
#define MODULE_ORDER_MESSAGE_TYPE 4


Workstation::Workstation(const char *targetIP, const std::function<void(RobotConfigMeasured)>& receiveConf, const std::function<void(RobotModuleOrder)>& receiveMod):
 udpsocket (UDPSocket(targetIP,DEFAULT_PORT_UDP)), tcpsocket (TCPSocket(targetIP,DEFAULT_PORT_TCP)), receiveConfigMeasured(receiveConf), receiveModuleOrder(receiveMod){

    // init receiver Thread
    receiverThread = std::thread (&Workstation::receiverRoutine,this);
    
    if(!USE_TCP_MESSAGES){
        std::cout << "send State via udp" << std::endl;
    }else{
        std::cout << "send State via tcp" << std::endl;
    }
    

}

void Workstation::send(RobotCommanded *command) {
    // init CommandMessage msg that shall be sent
    CommandMessage msg(command);
    // serialize the CommandMessage
    char serializedMsg[1024];
    msg.serialize(serializedMsg);

    std::cout << "Sending message: ";
    for (int i = 0; i < msg.getSize(); i++) {
        std::cout << (int) serializedMsg[i] << " ";
    }
    std::cout << std::endl;

    // if USE_TCP_MESSAGES is set to true, send RobotStateCommanded Messages over TCP and all other Messages over UDP
    // else send all messages over UDP
    if (command->getType() == STATE_MESSAGE_TYPE && USE_TCP_MESSAGES) {
        tcpsocket.send(serializedMsg, msg.getSize());
    } else {
        udpsocket.send(serializedMsg, msg.getSize());
    }
}


void Workstation::receiverRoutine() {
    
	UDPServerSocket socket(25000);

    std::cout << "Listening to speedgoat.." << std::endl;
	while(true){

	    // Create buffer and receive message via UDP
		char rec[1024];
        char *cur = rec;
        memset(rec, 0, 1024);
    	socket.receive(&rec, 1024);

    	// Read out type of incoming message
    	int msgType = (int) *cur++;

    	// Calculate number of joints from payload length
    	int payloadLength = *((uint16_t *) cur);

    	// Move pointer to sequence number
    	cur += 2;

    	// Read out sequence number
    	uint16_t receivedSequenceNumber = *((uint16_t *) cur);

    	// Move pointer to start of robot properties
    	cur += 2;

        if (msgType == MEASUREMENT_MESSAGE_TYPE){
            // calculate the number of joints
            int numberOfJoints = (payloadLength - 2) / BYTES_PER_JOINT;
            // deserialize the input message
            RobotConfigMeasured config = RobotConfigMeasured::deserialize(cur, numberOfJoints);
            // do something with the received data (defined by user)
            receiveConfigMeasured(config);
		} else if (msgType == MODULE_ORDER_MESSAGE_TYPE){
            // calculate the number of joints
            int numberOfModules = payloadLength;
            // deserialize the input message
			RobotModuleOrder moduleOrder = RobotModuleOrder::deserialize(cur, numberOfModules);
            // do something with the received message (defined by user)
            receiveModuleOrder(moduleOrder);
		}
	}
    
    
}

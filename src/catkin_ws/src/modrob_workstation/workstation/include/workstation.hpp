#include <iostream>
#include <thread>
#include <functional>
#include <cstring>

#include "networking/UDPSocket.h"
#include "networking/TCPSocket.h"
#include "networking/UDPServerSocket.h"
#include "RobotCommanded.hpp"
#include "RobotStateCommanded.hpp"
#include "RobotAngleCommanded.hpp"
#include "RobotTorqueCommanded.hpp"
#include "RobotConfigMeasured.hpp"
#include "RobotModuleOrder.hpp"




class Workstation{

private:
    /**
     * Thread that waits for incoming messages from the Robot
     * The receiverThread runs the receiverRoutine()
     */
    std::thread receiverThread;
    /**
     * TCP Connection to the Robot Computer
     * If activated RobotStateCommanded Messages will be sent over the TCP Connection
     * The TCP Connection is not used by default. to use it set the USE_TCP_MESSAGES Marcro in workstation.cpp to true
     * Port: 25001
     * 
     */
    TCPSocket tcpsocket;

    /**
     * UDP Connection to the Robot Computer
     * The UDP Connection is used to send RobotConfigCommanded and RobotStateCommanded messages to the Robot
     * The UDP Connection is used to receive RobotConfigMeasured and RobotModuleOrder messages from the Robot
     * If the TCP Connection is activated RobotStateCommanded messages will not be sent over the UDP Connection
     * Port: 25000
     * 
     */
    UDPSocket udpsocket;
    
    /**
     * Method that is called if a RobotConfigMeasured is received from the Robot
     * This method needs to be defined by the User
     * It needs to be provided as an input Argument for Worstation()
     * 
     * @param robotConfigMeasured the RobotConfigMeasured that is received from the Robot
     * 
     */
    const std::function<void(RobotConfigMeasured)>& receiveConfigMeasured;

    /**
     * Method that is called if a RobotModuleOrder is received from the Robot
     * This method needs to be defined by the User
     * It needs to be provided as an input Argument for Worstation()
     * 
     * @param robotModuleOrder the RobotModuleOrder that is received from the Robot
     * 
     */
    const std::function<void(RobotModuleOrder)>& receiveModuleOrder;

    

public:
    /**
     * Initializes a Workstation Adapter
     * 
     * @param targetIP the IP adress of the Robot Computer, that the Library should connect to
     * @param receiveConfigMeasured the receiver method that gets called if a RobotConfigMeasured comes in from the Robot Computer
     * @param receiveModuleOrder the reciever method that gets called if a RobotModuleOrder comes in from the Robot Computer
     * 
     */
    Workstation(const char *targetIP, const std::function<void(RobotConfigMeasured)>& receiveConf, const std::function<void(RobotModuleOrder)>& receiveMod);

    /**
     * Sends a RobotCommand to the Robot
     * RobotCommand is either a RobotAngleCommanded or RobotTorqeCommanded
     * 
     * @param command the RobotCommand that shall be sent to the Robot
     * 
     */
    void send(RobotCommanded *command);

    /**
     * Waits for incoming messages from the Robot
     * This routine runs on the receiverThread
     * If a RobotConfigMeasured comes in, it calls receiveConfigMeasured
     * If a RobotModuleOrder comes in, it calls receiveModuleOrder
     * 
     */
    void receiverRoutine();
   

};

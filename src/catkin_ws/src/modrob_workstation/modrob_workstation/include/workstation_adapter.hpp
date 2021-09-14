
#include "workstation.hpp"
#include "modrob_workstation/JointConfigCommanded.h"
#include "modrob_workstation/RobotConfigCommanded.h"
#include "modrob_workstation/RobotStateCommanded.h"
#include "modrob_workstation/RobotConfigMeasured.h"
#include "modrob_workstation/JointConfigMeasured.h"
#include "modrob_workstation/ModuleOrder.h"

class WorkstationAdapter{
    private:
        // the internal workstation that is used by the WorkstationAdapter
        Workstation workstation;

         /**
         * parses a RobotConfigCommanded ROS Message to a RobotAngleCommand
         * 
         * @param config the RobotConfigCommanded that shall be parsed
         * 
         */
        RobotAngleCommanded parseToRobotAngleCommanded(const modrob_workstation::RobotConfigCommanded& config);

         /**
         * parses a RobotConfigCommanded ROS Message to a RobotTorqueCommand
         * 
         * @param config the RobotConfigCommanded that shall be parsed
         * 
         */
        RobotTorqueCommanded parseToRobotTorqueCommanded(const modrob_workstation::RobotConfigCommanded& config);

        /**
         * parses a RobotStateCommanded ROS Message to a RobotStateCommand
         * 
         * @param stateCommanded the RobotStateCommanded that shall be parsed
         * 
         */
        RobotStateCommanded parseToRobotStateCommanded(const modrob_workstation::RobotStateCommanded& stateCommanded);
	    
    public:
        /**
         * Initializes a Workstation Adapter
         * 
         * @param targetIP the IP adress of the Robot Computer, that the Library should connect to
         * @param receiveConfigMeasured the receiver method that gets called if a RobotConfigMeasured comes in from the Robot Computer
         * @param receiveModuleOrder the reciever method that gets called if a RobotModuleOrder comes in from the Robot Computer
         * 
         */
        WorkstationAdapter(const char *targetIP, const std::function<void(RobotConfigMeasured)>& receiveConfigMeasured, const std::function<void(RobotModuleOrder)>& receiveModuleOrder);
        
        /**
         * send a RobotConfigCommanded message to the Robot
         * 
         * @param config the configuration that shall be sent to the Robot Computer
         *
         */
        void sendConfig(const modrob_workstation::RobotConfigCommanded& config);

        /**
         * send a RobotStateCommanded message to the Robot
         * 
         * @param config the state that shall be sent to the Robot Computer
         *
         */
        void sendState(const modrob_workstation::RobotStateCommanded& state);

        /**
         * parses a RobotModuleOrder to a ModuleOrder ROS Message
         * 
         * @param moduleOrder the RobotModuleOrder that shall be parsed
         * 
         */
        static modrob_workstation::ModuleOrder parseToModuleOrder(RobotModuleOrder moduleOrder);

        
        /**
         * parses a RobotConfigMeasured to a RobotConifigMeasured ROS Message
         * 
         * @param config the Robot configuration that shall be parsed
         * 
         */
	    static modrob_workstation::RobotConfigMeasured parseToConfigMeasured(RobotConfigMeasured config);
};

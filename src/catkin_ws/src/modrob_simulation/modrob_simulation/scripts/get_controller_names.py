#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def get_contoller_names():
    rospy.init_node('controller_name_parser', anonymous=True)
    parameter_names = rospy.get_param_names()
    all_controller_string = ""
    for parameter_name in parameter_names:
        if "controller" in parameter_name:
            all_controller_string += parameter_name + " "
    print(all_controller_string[:-1])

if __name__ == '__main__':
  try:
    get_contoller_names()
  except rospy.ROSInterruptException:
    pass
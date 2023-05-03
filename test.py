#!/usr/bin/env python

import sys
import rospy
from qt_robot_interface.srv import *
from qt_vosk_app.srv import *



if __name__ == '__main__':
    
    # Initialisation du noeud ROS
    rospy.init_node('qt_robot_speech')
    rospy.loginfo("Hi") 
    
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    recognize = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/speech/recognize')


    speechSay("Bonjour! Tu veux faire une recherche? ")
    resp = recognize("fr_FR", [], 0)
    print(resp.transcript)
    rospy.loginfo("I got: %s", resp.transcript)
    speechSay("Tu as dis: %s " % resp.transcript)

    try:
        
        #The code is repeated until the user taps ctrl+C 
        rospy.spin()
    except KeyboardInterrupt:
        pass
    
    rospy.loginfo("Finished!")

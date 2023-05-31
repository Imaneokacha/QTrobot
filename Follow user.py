#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from qt_motors_controller.srv import *
from audio_common_msgs.msg import AudioData
from qt_vosk_app.srv import *
from qt_robot_interface.srv import *


# Definition of the speech service and waiting for the service
speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
rospy.wait_for_service('/qt_robot/speech/recognize')

# Definition of a ROS publisher to publish new positions of the head motors
head_pub = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=10)

def head_movement(direction):
    rospy.loginfo('Sound direction is: {}'.format(direction))
    if 215 <= direction <= 320:
        motor_angle = 360 - direction
        head_command = Float64MultiArray()
        head_command.data = [motor_angle, 0]
        head_pub.publish(head_command)
    else:
        rospy.loginfo("I cannot see you. Can you come forward?")


def sound_direction_callback(msg):
    # This function will be called whenever a new sound direction message is received
    sound_direction = msg.data
    rospy.loginfo("Sound direction: %d degrees", sound_direction)
    head_movement(sound_direction)


def is_speaking_callback(msg):
    rospy.loginfo("Currently speaking")


if __name__ == '__main__':
    rospy.init_node('sound_direction', disable_signals=True)

    rospy.Subscriber('/qt_respeaker_app/sound_direction', Int32, sound_direction_callback)
    rospy.Subscriber('/qt_respeaker_app/is_speaking', Bool, is_speaking_callback)
    rospy.sleep(3)

    # Indicate when the code starts
    rospy.loginfo("Let's exchange greetings")
    rospy.spin()

    # Reset head position to default after program finishes
    head_command = Float64MultiArray()
    head_command.data = [0, 0]
    head_pub.publish(head_command)

    # The code will keep running until the user presses Ctrl+C

    # Indicate to the user when the script stops
    rospy.loginfo("Finished!")
    # You can stop the program by typing Ctrl+C in the terminal

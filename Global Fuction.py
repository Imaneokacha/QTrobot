#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import random
import numpy as np
from qt_motors_controller.srv import *
from qt_robot_interface.srv import *
from qt_vosk_app.srv import *
import wikipediaapi
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import threading
from qt_gesture_controller.srv import gesture_play

def random_moves():
    # Choose random values for the positions of the motors while respecting the constraints on the limit values of the motors
    RightShoulderPitch_ref = random.randrange(-75, 0)
    RightShoulderRoll_ref = random.randrange(-75, -40)
    LeftShoulderPitch_ref = random.randrange(0, 75)
    LeftShoulderRoll_ref = random.randrange(-75, -40)

    # Wait for ros service
    rospy.wait_for_service('/qt_robot/motors/setVelocity')

    # Define new Float64MultiArray messages for the motors
    r_ref = Float64MultiArray()
    l_ref = Float64MultiArray()

    # Add data that we want to publish to them
    r_ref.data = [RightShoulderPitch_ref, RightShoulderRoll_ref, -20]
    l_ref.data = [LeftShoulderPitch_ref, LeftShoulderRoll_ref, -20]

    # Publish the data and order the head motors
    right_pub.publish(r_ref)
    left_pub.publish(l_ref)

    # Add a random sleep time to make the body's motion smoother
    rospy.sleep(np.random.uniform(0.5, 1.5))

    # Reduce the velocity of the motors' movement
    setVelocity(['left_arm', 'right_arm'], 3)


def search_wikipedia(key_word):
    # Create an instance of WikipediaAPI for the French language
    wiki = wikipediaapi.Wikipedia('fr')
    page = wiki.page(key_word)

    if page.exists():
        summary = page.summary  # Complete summary

        # Divide the summary into sentences using punctuation
        sentences = []
        current_sentence = ""
        punctuations = {".", "!", "?"}  # Common punctuations to end a sentence

        for char in summary:
            current_sentence += char
            if char in punctuations:
                sentences.append(current_sentence.strip())
                current_sentence = ""

        # Limit the number of sentences
        max_sentences = 4
        selected_sentences = sentences[:max_sentences]

        result = ' '.join(selected_sentences)  # Combine the sentences into a single text
        return result
    else:
        return None


# Function to call the speech synthesis service asynchronously
def speechSay(text):
    rospy.loginfo("Sending speech synthesis request: %s", text)

    # Function executed in a thread for random moves
    active = True

    def random_moves_thread():
        i = 0
        while i < len(text) - 1 and active:
            random_moves()
            i += 1

    # Launch the thread for random moves
    moves_thread = threading.Thread(target=random_moves_thread)
    moves_thread.start()

    # Call the speech synthesis service
    response = speech_say_client(text)
    active = False
    if response.status:
        rospy.loginfo("Speech synthesis succeeded")
    else:
        rospy.logwarn("Speech synthesis failed")


def discussion_savante():
    # Ask the user for a keyword to search
    speechSay("Donne-moi un mot clÃ©")

    resp_kw = recognize("fr_FR", [], 15)
    key_word = resp_kw.transcript
    if len(key_word) >= 2 and ("mm" in key_word or "hm" in key_word):
        key_word = key_word[2:]

    rospy.loginfo(key_word)

    # Perform the search on Wikipedia
    try:
        result = search_wikipedia(key_word)
        print(result)

        speechSay("Voici ce que j'ai trouvÃ© sur Wikipedia : {}".format(result))

    except:
        speechSay("Aucun rÃ©sultat trouvÃ© sur Wikipedia pour la recherche : {}".format(key_word))


def retour_position_initiale():
    href = Float64MultiArray()
    r_ref = Float64MultiArray()
    l_ref = Float64MultiArray()
    href.data = [0, 0]
    r_ref.data = [-90, -70, -20]
    l_ref.data = [90, -70, -20]
    head_pub.publish(href)
    right_pub.publish(r_ref)
    left_pub.publish(l_ref)


if __name__ == '__main__':
    # Initialize the ROS node for the process
    rospy.init_node('fonction_globale', disable_signals=True)

    # Indicate that the script was launched successfully
    rospy.loginfo("le robot va interagir avec vous")

    # Define Subscribers, Publisher, and Services that we'll need
    head_pub = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=10)
    right_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=10)
    left_pub = rospy.Publisher('/qt_robot/left_arm_position/command', Float64MultiArray, queue_size=10)

    # Define a ROS service which we will use to control the velocity of the motors
    setVelocity = rospy.ServiceProxy('/qt_robot/motors/setVelocity', set_velocity)

    recognize = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)

    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/speech/recognize')

    # Define the speech synthesis service and wait for the service
    speech_say_client = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)

    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
    gesturePlay("QT/up_right", 0)
    speechSay("Bonjour !")

    sound_subscriber = None
    while not rospy.is_shutdown():

        rospy.sleep(1)
        try:
            speechSay("Tu veux faire une recherche ? ")

            resp = recognize("fr_FR", [], 0)
            rospy.loginfo(resp.transcript)
            rospy.loginfo("I got: %s", resp.transcript)

            if "d'accord" in resp.transcript:
                discussion_savante()
            else:
                # If the user indicates that they don't want to do a search, exit the loop
                speechSay("D'accord! A bientÃ´t! Au revoir!")
                retour_position_initiale()
                rospy.sleep(1)
                gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
                gesturePlay("QT/up_right", 0)
                break

        # You can stop the program by typing Ctrl+C in the terminal
        except KeyboardInterrupt:
            # The robot returns to its initial position
            retour_position_initiale()

            # Stop the program
            rospy.loginfo("Stopping the current node")
            rospy.signal_shutdown("Interruption of use")

    retour_position_initiale()

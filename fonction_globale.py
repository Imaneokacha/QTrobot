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


           
def head_mouvement(direction):
    rospy.loginfo('La direction du son est : {}'.format(direction))
    
    
    # Convertir la direction du son en un angle relatif à la tête
    angle_tete = abs(direction - 180)-90
        
    # Limiter l'angle de la tête dans les limites [-60, 60] degrés
    if angle_tete>=0 :
        angle_tete=min(angle_tete, 60)
        #if angle_tete==60 :
            #speechSay('Pardon ! je ne peux pas vous voir')
            
    else:
    
        angle_tete = max(angle_tete, -60)
        #if angle_tete==-60 :
            #speechSay('Pardon ! je ne peux pas vous voir')
        
    # Inverser l'angle pour tenir compte du mouvement du microphone avec la tête
    angle_moteur = angle_tete
        
    # Publier les nouvelles positions des moteurs de la tête
    href = Float64MultiArray()
    href.data = [angle_moteur, 0]
    head_pub.publish(href)






def sound_direction_callback(msg):
    # Cette fonction sera appelée chaque fois qu'un nouveau message de direction du son sera reçu
    sound_direction = msg.data
    rospy.loginfo("Direction du son : %d degrés", sound_direction)
    head_mouvement(sound_direction)


def random_moves() :
    
   
    # Choose random values ​​for the positions of the motors while respecting the constraints on the limit values ​​of the motors       
    RightShoulderPitch_ref = random.randrange(-75,0)
    RightShoulderRoll_ref = random.randrange(-75,-40)
    LeftShoulderPitch_ref = random.randrange(0,75)
    LeftShoulderRoll_ref = random.randrange(-75,-40)
    
    # wait for ros service        
    rospy.wait_for_service('/qt_robot/motors/setVelocity')
              
    # We define a new Float64MultiArray message for the motors
    r_ref = Float64MultiArray()
    l_ref = Float64MultiArray()
        
    # We add data that we want to publish to it
    r_ref.data = [RightShoulderPitch_ref, RightShoulderRoll_ref,-20]
    l_ref.data = [LeftShoulderPitch_ref, LeftShoulderRoll_ref,-20]

    # We publish the data and order the head motors            
   
    right_pub.publish(r_ref)
    left_pub.publish(l_ref)

    # We add a random sleep time so that the body's motion becomes more smooth            
    rospy.sleep(np.random.uniform(0.5,1.5))

    # We reduce the velocity of the motors' movement
    setVelocity(['left_arm', 'right_arm'],3)
        
def search_wikipedia(key_word):
    # Crée une instance de WikipediaAPI pour la langue française
    wiki = wikipediaapi.Wikipedia('fr')
    page = wiki.page(key_word)
    
    if page.exists():
        summary = page.summary  # Résumé complet

        # Divise le résumé en phrases en utilisant la ponctuation
        sentences = []
        current_sentence = ""
        punctuations = {".", "!", "?"}  # Ponctuations courantes pour terminer une phrase

        for char in summary:
            current_sentence += char
            if char in punctuations:
                sentences.append(current_sentence.strip())
                current_sentence = ""

        # Limite le nombre de phrases
        max_sentences = 4
        selected_sentences = sentences[:max_sentences]

        result = ' '.join(selected_sentences)  # Combine les phrases en un seul texte
        return result
    else:
        return None
        
        
def discussion_savante ():
     
    speechSay("Bonjour !")   
    # Boucle principale pour effectuer des recherches multiples
    while True:
        # Demande à l'utilisateur s'il souhaite effectuer une recherche
        speechSay("Tu veux faire une recherche ? ")
        resp = recognize("fr_FR", [], 0)
        rospy.loginfo(resp.transcript)
        rospy.loginfo("I got: %s", resp.transcript)

        if "d'accord" in resp.transcript:
            # Demande le mot-clé à rechercher
            speechSay("Donne-moi un mot clé")
            resp_kw = recognize("fr_FR", [], 15)
            
            key_word = resp_kw.transcript
            if len(key_word)>=2 and ("mm" in key_word or "hm" in key_word):
                key_word = key_word[2:]
            
            rospy.loginfo(key_word)

            # Effectue la recherche sur Wikipedia
            result = search_wikipedia(key_word)
            if result is not None:
                print(result)
                random_moves()
                speechSay("Voici ce que j'ai trouvé sur Wikipedia : {}".format(result))
                
            else:
                speechSay("Aucun résultat trouvé sur Wikipedia pour la recherche : {}".format(key_word))
        else:
            # Si l'utilisateur indique qu'il ne souhaite pas faire de recherche, sort de la boucle
            speechSay("D'accord! A bientôt! Au revoir!")
            break
       
     
if __name__ == '__main__':

    # We initialize the ROS node for the process
    rospy.init_node('fonction_globale', disable_signals = True)

    # We indicate that the script was lauched with success
    rospy.loginfo("le robot va interagir avec vous")


    #Define Subscribers, Publisher and Services that we'll need   
    head_pub = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=10)
    right_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=10)
    left_pub = rospy.Publisher('/qt_robot/left_arm_position/command', Float64MultiArray, queue_size=10)
     # Define a ros service which we will use to controll the velocity of the motors
    setVelocity = rospy.ServiceProxy('/qt_robot/motors/setVelocity', set_velocity)  

    recognize = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
    #rospy.Subscriber('/qt_respeaker_app/sound_direction', Int32, sound_direction_callback)
    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/speech/recognize')
   

    # Définition du service de speech et attente du service
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
   
    
    while not rospy.is_shutdown():
    
        rospy.sleep(1)
        try:
            #rospy.Subscriber('/qt_respeaker_app/sound_direction', Int32, sound_direction_callback)
            #rospy.sleep(2)
           
            discussion_savante()
            
         
     
        # You can stop the program by typing Ctrl+C in the terminal      
        except KeyboardInterrupt:
            
            # The robot returns to its initial position
            href = Float64MultiArray()
            r_ref = Float64MultiArray()
            l_ref = Float64MultiArray()
            href.data = [0, 0]
            r_ref.data = [-90, -70,-20]
            l_ref.data = [90, -70,-20]
            head_pub.publish(href)
            right_pub.publish(r_ref)
            left_pub.publish(l_ref)
            
            # Stop the program
            rospy.loginfo("Stopping the current node")
            rospy.signal_shutdown("Interruption of use")
        rospy.loginfo("Finished") 


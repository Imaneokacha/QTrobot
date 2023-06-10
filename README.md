# QTrobot
QTrobot is a humanoid robot created by LuxAI S.A. It has been employed for diverse applications, including emotional training for children with autism.  
![a](https://i.imgur.com/9sc6CwK.jpg)  


This repository is a collaborative project developed by a group of 4 students from IMT Atlantique. The objective of this project is to improve the interaction between the robot and the user by implementing three functions :   
* Random Moves  
This function generates random movements of the robot's head and arms at different levels of speed, in order to provide an engaging interaction. By varying the code, the robot can perform movements with different velocities.
* Wikipedia Search  
This function allows the robot to perform searches in Wikipedia. It utilizes speech recognition to extract keywords, performs the search, and provides a summary of the article that the robot can read to the user. 
* Follow user  
By utilizing sound localization techniques, the robot can accurately determine the direction of the user's voice and rotate its head towards the speaker. The motor HeadYaw is moving in order to follow the user's position.
![a](https://i.imgur.com/qUb16bv.jpg)  

Finally, the Global Function integrates the functionalities of Wikipedia search, random moves, and follow user. It enables the robot to perform Wikipedia searches while simultaneously executing random movements and tracking the user's direction. 
These functionalities contribute to creating a more immersive and interactive experience with QTrobot, fostering a sense of natural interaction and reducing the perception of robotic behavior.  
For a demonstration of these functionalities in action, here is the link to the video: [![Demonstration video](https://img.youtube.com/vi/HgkzYq131vg/0.jpg)](https://youtu.be/HgkzYq131vg)


## Launching a function
* Place the Python file in the following directory : catkin_ws/src  
* Open a terminal and tap the command : rosrun Name_of_the_folder Name_of_the_function.py 
* To stop a script, you have two options:
Press Ctrl+C as there is an "except KeyboardInterrupt" statement in the script, or in another terminal, enter the command rosnode kill node_name.
If you don't know the node name, you can find it in the script where rospy.init_node(node_name) is written, or enter rosnode list in the same terminal. This command displays the list of active nodes.
## Contributors
Victoria ANDRE  
Maissa BEJI  
Gabriel CHASTANET  
Imane OKACHA 

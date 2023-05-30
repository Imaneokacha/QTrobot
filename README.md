# QTrobot
QTrobot is a humanoid robot created by LuxAI S.A. It has been employed for diverse applications, including emotional training for children with autism.  
This repository is a collaborative project developed by a group of 4 students from IMT Atlantique. The objective of this project is to improve the interaction between the robot and the user by implementing three functions :   
* Random Moves  
* Wikipedia Search  
* Follow user    
These functionalities contribute to creating a more immersive and interactive experience with QTrobot, fostering a sense of natural interaction and reducing the perception of robotic behavior.

## Launching a function
* Place the Python file in the following directory : catkin_ws/src  
* Open a terminal and tap the command : rosrun Name_of_the_folder Name_of_the_function.py 
* To stop a script, you have two options:
Press Ctrl+C as there is an "except KeyboardInterrupt" statement in the script, or in another terminal, enter the command rosnode kill node_name.
If you don't know the node name, you can find it in the script where rospy.init_node(node_name) is written, or enter rosnode list in the same terminal. This command displays the list of active nodes.
## Contributors
Victoria ANDRE  
Maissa BEJI  
Imane OKACHA  
Gabriel CHASTANET





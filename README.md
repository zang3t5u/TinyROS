# TinyROS

Repo to integrate TinyOS based networks with ROS Publish and Subscribe Methods

##Experimental Setup
###Motes
Upload the TinyOS App at

Bot_Net/tinyOS_App/

To your WSN mote with

    $ make <mote> install.<botID> bsl,<port>
    
Here

    <mote> = Type of Mote (telosb, xm1000)
    
    <botID> = The ID of the robot in your robot system. This should be a non-zero positive number
    OR
    <botID> = 0 for the Central Computer connected to Optitrack
    
    <port> = USB port to which your mote is connected. This can be checked by running the command $ motelist on your terminal
  
Move the Files from ROS scripts to the scripts directory of your ROS Package

###Bot Tracking
Make sure the tracked Robot coordinates, sent by your Optitrack or Central Computer system are of ROS msg type:

geometry_msgs/Pose2D

These should be published to the topic named:

Robot_\<botID\>/ground_pose

##Execution

Once you have configured your motes and the central tracked topics, build the ROS workspace by using

    $catkin_make
    $catkin_make install

Export the setup file with

    $. devel/setup.bash
  
###Robot
VERY IMPORTANT TO BE CONFIGURED FIRST
On your Robotś Controlling Computer (in our case an iRobot Create), execute the following command:

    $rosrun <your_ROS_package> tinyROS.py <Num_of_Bots> <botID> serial@<USB Port of Mote>:<baudRate>
    

Here, make sure that

    <Num_of_Bots> = Maximum of the IDs configured on your Robots.
    Eg: With a robot system with IDs: [1, 5, 7]
        <Num_of_Bots> = 7
    
    <botID> = ID with which you burned the TinyOS App on your robot
    <USB Port of Mote> = The USB Port where your mote is connected
    <baudRate> = The baud rate at which the mote can communicate. (115200 for TelosB and XM1000)

###Central Tracking System

Make sure you have started the tinyROS script on your Robot First

Connect the Mote with botID = 0, when you installed the TinyOS App

On your Central Tracking System (in our case an Optitrack Motive system), execute the following command

    $rosrun <your_ROS_Package> tinyROS.py <Num_of_Bots> 0 serial@<USB Port of Mote>:<baudRate>
    
Here, make sure that

    <Num_of_Bots> = Maximum of the IDs configured on your Robots.
    Eg: With a robot system with IDs: [1, 5, 7]
        <Num_of_Bots> = 7
    
    <USB Port of Mote> = The USB Port under motelist that has the mote with ID 0
    <baudRate> = The baud rate at which the mote can communicate. (115200 for TelosB and XM1000)
    

##Result

On your respective robots, you should see a list of topics named:
  /create<~botID>/ground_pose
  /create<~botID>/cmd_vel
  /ground_pose
  /cmd_vel

where \<~botID\> is the botID of the robots other than yourself ranging from 1 to \<Num_of_Bots\>

On your Central Tracking System, you should see a list of topics named:
  /create<~botID>/ground_pose
  /create<~botID>/cmd_vel

where \<~botID\> is the botID of the robots other than 0 ranging from 1 to \<Num_of_Bots\>

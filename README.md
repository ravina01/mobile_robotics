# mobile_robotics
LAB#2 : Q.3
1. Terminal 1 : Run - roslaunch turtlebot3_mr turtlebot3_lab2.launch
2. Terminal 2 : Run 
              1. cd ~/catkin_ws/src/turtlebot3_mr
              2. rosrun rviz rviz -d config/rviz.rviz
3. Terminal 3 : Run - roslaunch turtlebot3_mr apriltag_gazebo.launch
4. Once the setup is running, we are good to execute python file. I have calculated inverse kinematics using Matlab code and used the same values inside code to set linear velocity along x axis. matlab code snippet is attached in the uploaded solution paper.
6. Terminal 4 : rosrun turtlebot3_mr aprilTagFinal.py
7. Observe the outputs of Rviz and gazebo models.
8. Thank You ! 

Q.2 c part : 
1. export TURTLEBOT3_MODEL=burger
2. roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
3. rostopic pub /cmd vel geometry msgs/Twist -r 10 '[0.2, 0, 0]' '[0, 0, 0.133]'

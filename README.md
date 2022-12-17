# Shellby The Shell Shuffler

Read the report [here](https://matthewsahim.notion.site/matthewsahim/Shellby-the-Shell-Shuffler-074f832cbb2a4328a283dbf2438d9ee6#f724c893e8784427a3909033201e4768)

After setting up the .bashrc file and running `roscore`, here is how to run both the guesser and the shuffler. 

## Guesser 
1. `roslaunch guesser run_cam.launch`
2. `rosrun rviz rviz` (to monitor the output and input images) 
3. `rosrun guesser play.py`

The guesser will ask for the number of cups and which cup (from the left) the ball is under. 

## Shuffler 
1. `rosrun intera_interface enable_robot.py -e`
2. `rosrun intera_interface joint_trajectory_action_server.py` 
3. `rosrun shuffler main.py` 

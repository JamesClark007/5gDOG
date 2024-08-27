Dog Door Setup Documentation
	1	Updating CMakeLists.txt
First, we need to add some lines to the CMakeLists.txt file located in the src/dog_door directory.
Navigate to the directory:
/home/turbineone/Desktop/FinalDog/DogDoor/dog_door/CMakeLists.txt

Open the CMakeLists.txt file in a text editor. Add the following lines to the file (please replace with the actual lines that need to be added):
Add these lines to CMakeLists.txt
Example:
<node name=“<scriptName_node.py>“ pkg="dog_door" type=“<scriptName_node.py>“ output="screen" />
<node name="gps_waypoint_server" pkg="dog_door" type="gps_waypoint_server_node.py" output="screen" />
    <node name="achieve_gps_waypoint_node" pkg="dog_door" type="achieve_gps_waypoint_node.py" output="screen" />
    <node name="openthread_bridge_node" pkg="dog_door" type="openthread_bridge_node.py" output="screen" />
    <node name="tak_client_bridge_node" pkg="dog_door" type="tak_client_bridge_node.py" output="screen" />
</launch>

Save and close the file.
	2	Updating dog_door.launch
Next, we need to update the dog_door.launch file and add some lines there as well.
Open the dog_door.launch file in a text editor. Add the following lines to the file (please replace with the actual lines that need to be added): /home/turbineone/Desktop/FinalDog/DogDoor/dog_launcher/launch/dog_door.launch
Add these lines to dog_door.launch
Example:
scripts/<scriptName.py>
catkin_install_python(PROGRAMS
  scripts/dog_door_node.py
  scripts/gps_waypoint_server_node.py
  scripts/odom_cov_compute_node.py
  scripts/achieve_gps_waypoint_node.py
  scripts/openthread_bridge_node.py
  scripts/tak_client_bridge_node.py
  #add node script here
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


Save and close the file.
	3	Running the Code
Lastly, to run the code and view the terminal:
a. Execute the SITL (Software In The Loop) script: 
./start_sitl.sh
b. Once the SITL is running, open a new terminal window to view the dog_door output: 
screen -r dog_door
This will connect you to the dog_door screen session where you can view the terminal output of the running code.
To detach from the screen session without stopping it, press Ctrl+A followed by D.


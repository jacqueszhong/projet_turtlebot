# projet_turtlebot

## Getting started

1) launch tele operation :
```
	#Joystick version
	roslaunch teleop_python joy_teleop.launch

	#Keyboard version
	roslaunch teleop_python key_teleop.launch

```

2) launch robot simulation on Gazebo :
```
	roslaunch minilab_simulation minilab_simulation.launch
```

3) launch slam using gmapping :
```
	rosrun gmapping slam_gmapping
```
  
4) display the map and lasers shooting area :
```
	rosrun rviz rviz
```
In the rviz interface, add /map and /scan topics.

It is possible to run steps 2,3,4 at the same time with :
```
	roslaunch minilab_simulation minilab_gmap.launch
```


## Things to do
Next :
* Write prm graph function (store connections, find connections)
* Write graph search (Djikstra / Astar)

In general :
* Path finding in simulation (14/12/2018)
* Mapping in real world (21/12/2018)
* Path following (16/01/2018)
* Going back to start point (17/01/2018)
* (Bonus) Autonomous exploration and obstacle avoidance (18/01/2018)
* Demonstration video, presentation, demo (28/01/2018)

## Little problems
* Logging ? (key_teleop)
* How map_saver knows which map to save ?

## Useful ressources
* gmapping :
```
http://wiki.ros.org/gmapping
http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData
http://wiki.ros.org/map_server
```
* websites :
```
http://rco.fr.nf/index.php/2016/07/10/tuto-trajectoire-interpol/
http://www.theconstructsim.com/ros-qa-how-to-convert-quaternions-to-euler-angles/?fbclid=IwAR0gGc5jpxlZ_3hWxxt43tHKz_uUBAXb5xFVLqxS_Njb6VaRSU333Sp0x5s
https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_TRAJ
```


## Author
Jacques Zhong [Jaquapi](https://github.com/jacqueszhong)

Maeva Arlandis [Barbe Bleue](https://github.com/BarbeBleue)

<launch>
	
	<param name="joy_node/dev" value="/dev/input/js1" />
	<node name="joy_node" pkg="joy" type="joy_node" />

	<node name="Teleop" pkg="teleop_python" type="joy_teleop.py" />

</launch>
# Home Service Robot
## Used ROS package
1. turtlebot_gazebo
    1. `turtlebot_world.launch`: It can deploy a turtlebot in a gazebo environment by linking the `UdacityOffice.wrold` file to it.
    1. `amcl_demo.launch`: `Adaptive Montre Carlo Localization` accurately localizes a mobile robot inside a map in the Gazebo simulation environments.
1. turtlebot_rviz_launchers
    1. `view_navigation.launch`: It can load a preconfigured rviz workspace. 
1. `add_markers`: It will mark the virtual object's location on the rviz
1. `pick_objects`: It will drive the robot to pick the virtual object and will drop off the object at the drop off place.

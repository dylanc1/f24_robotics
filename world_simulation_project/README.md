# Bryce Lawn Apartment simulation

## Running the Bryce Lawn Apartment world

From the f24_robotics directory, run the following in the terminal:

```
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch world_simulation_project simulation_launch.py
```

## Changing light settings

There are two floor lights that can be altered, one in the living room and one in one of the bedrooms. To change the lights, either in the webots 
or in the world file, change the intensity of the light.

## Opening and closing doors

There are 7 different doors, which can all be opened or closed. By default, the exterior doors are closed and the interior doors are open. To change 
the status of a door, in webots or in the world file, change the position value for the door. Note that the value is in radians, with a value of zero
(or very close to zero) indicating that the door is shut.

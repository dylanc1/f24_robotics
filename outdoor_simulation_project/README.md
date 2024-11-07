# Recreation Center Area simulation

By: Dylan Canipe

## Running the Recreation Center Area world

To run the daytime world simulation, navigate to the f24_robotics directory in the terminal, then run the following:

```
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch outdoor_simulation_project daytime_launch.py
```

To run the nightime word simulation, use the launch command:
```
ros2 launch outdoor_simulation_project dusk_launch.py
```

## Traffic lights

There are three sets of 4-way traffic lights, each of these are controlled through the webots controller.

## Street light issues

There is a limit in Webots of 48 of each kind of light (PointLight, DirectionalLight, and SpotLight). Because of this, some of the street lights in the dusk world will not work unless the limit is changed. This is not essential, but it can be done by following the instructions [here](https://cyberbotics.com/doc/reference/light#limitation).

## Resources Used

The basic framework for the world was generated using information from OpenStreetMap. There is a tool that converts between osm files and wbt files found in the Cyberbotics documentation [here](https://cyberbotics.com/doc/automobile/openstreetmap-importer), and I used this as a starting point. In additional images were taken from [Shutterstock](https://www.shutterstock.com/image-photo/aerial-view-blue-tennis-court-1629948055), [iStock](https://www.istockphoto.com/photo/empty-green-soccer-football-pitch-aerial-view-gm1176781177-328250818), and [Google Earth](https://earth.google.com/web/@33.21363939,-87.53194348,68.36633874a,106.74001852d,35y,94.64128991h,0t,0r/data=CgRCAggBOgMKATBCAggASg0I____________ARAA).

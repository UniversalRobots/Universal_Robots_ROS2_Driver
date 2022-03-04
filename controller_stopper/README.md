# controller_stopper

A small helper node that stops and restarts ROS controllers based on a boolean status topic. When the status goes to `false`, all running controllers except a set of predefined *consistent_controllers* gets stopped. If status returns to `true` the stopped controllers are restarted.
This is done by Subscribing to a robot's running state topic. Ideally this topic is latched and only publishes on changes. However, this node only reacts on state changes, so a state published each cycle would also be fine.

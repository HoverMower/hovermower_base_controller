# hovermower_base_controller
Base controller firmware for HoverMower

As stated in documentation of HoverMower project, firmware depends on your needs. 
However, my build of HoverMower uses ROS (robot operating system). Tasks like path planning, move commands and behavior gets managet by ROS nodes.

But some really basic parts can't be handeled by ROS, this is where hovermower_bare_controller comes in play. It is a firmware which runs on Arduino Nano and will perform thinks like:

- battery monitor, sense battery voltage and init shutdown/disconnect battery before drawing it to death
- charge monitor, sense if robot is attached to charger and charge battery, if needed. Also disconnect charger when battery is full
- perimeter sensor, detect perimeter fence and detect, if robot leaves the mowing area
- bump sensor, simple bumper to detect collisions
- mow motor, control mow motor, sense power usage of mow motor
- user switches, control MOSFET switches to toggle headlights or other payload on/off
- button, user controlled button, analyze if button has been pressed and for how long

All values gets reported to a ROS node,m which will publish corresponding messages. This firmware will not take any decisions beside toggle charger on/off. Instead, it is just a wrapper to abstract hardware specific implementation to ROS
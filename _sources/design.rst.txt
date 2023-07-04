System Overview
----------------

In :ref:`topview-image-label`. An overview of the architecture of the entire software is shown. 
For this implementation a centralised architecture is chosen. 
This means that the robots are controlled by a central node, the VS manager.
This approach is chosen for the following reasons:
   * It is easier to implement
   * Minimal variability in delay

A Task is an object which contains information about the task that should be executed. For example, the number of robots and the diameter of the VS.
The VS manager communicates with the robots via ROS topics.

.. figure:: images/TopOverview.png
   :name: topview-image-label
   :align: center

   figure 1: Topview of the software architecture

In figure :ref:`overviewmanager-image-label` the architecture of the virtual structure manager is shown.

The vs_manager is the main node, it is responsible for receiving the task and giving the other nodes the correct information based on the incomming task.

The pathplanner is responsible for calculating the route for each of the robots. 
The pathplanner can for example use nav2 (or any other pathplanner) the vs path, based on this path the pathplanner calculates the route for each of the robots.

The Assembler is responsible for assembling the robots. 
It receives the task and chooses the correct robots for the task and assembles them at their starting position.
 
The VS_Controller is responsible for sending the velocity commands based on the route and the current position of the robot.
It receives the routes created by the pathplanner and the current position of the robots, based on this information it calculates the velocity commands for the robots.

.. figure:: images/overviewVSManager.png
   :name: overviewManager-image-label
   :align: center

   figure 2: Overview of the virtual structure manager

Project Overview
----------------

Project description
^^^^^^^^^^^^^^^^^^^
The project is an applied research project, which aims to develop a system capable of controlling a fleet of robots to perform a task. 
The system will be able to control the robots to form a virtual structure (VS) and transport a load from A to B. 
The aim of this project is to take an initial step into the realm of collaborative load transportation. 
Through research and experimentation, we will explore the most promising and feasible methods for creating virtual structure formations.


System Requirements
^^^^^^^^^^^^^^^^^^^
    * The implementation must be realized in a physical setup (as opposed to virtual).
    * The members of the VS shall use a differential drive system for moving.
    * The VS navigates in the shortest trajectory to the immediate 2D pose of the provided path until it reaches the last one.
    * The VS shall be formed by 2, 3 or 4 robots.
    * The behavior of the VS shall not change regardless of the number of members.
    * The starting poses of each robot are random.
    * The formation must be a regular polygon where the number of faces is equal to the number of members.

Scope
^^^^^

It is outside of the scope of the project to:
    * Develop a commercially viable product
    * Achieve specific performance marks
    * Provide training to use the system
    * The simultaneous control of multiple VS
    * Request virtual structures with a different number of members than robots available in the scenario
    * Perform the demonstrations in any other environment than the accorded one
    * Implementing a system capable of dealing with non pre-mapped environments
    * Account for dynamic obstacles
    * Task management

Future work
^^^^^^^^^^^
    * AMCL
    * Nav2 path planning
    * ORMF integration
    * Assembler
    * Load transportation
    * Interfacing

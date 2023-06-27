System Overview
----------------

In :ref:`topview-image-label`. you see the topview architecture of the entire software. 
Task is the task that it will get that it needs to execute. 
This will enter the VS manager where the paths for the robots are calculated and the robots are assembled. 
The VS manager communicates with the robots through MQTT.

.. figure:: images/TopOverview.png
   :name: topview-image-label
   :align: center

   figure 1: Topview of the software architecture

In figure :ref:`overviewmanager-image-label` you can see the architecture of the virtual structure manager.
The vs\_manager is the main node, this is responsible for calling services and action to complete the incoming task. 
The pathplanner is node that can be called by a service.

.. figure:: images/overviewVsManager.png
   :name: overviewManager-image-label
   :align: center

   figure 2: Overview of the virtual structure manager

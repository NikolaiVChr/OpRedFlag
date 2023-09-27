==============
OPRF Libraries
==============

This directory contains libraries shared by several OPRF aircrafts.
They are *not* mandatory for OPRF compatibility.

-------
iff.nas
-------
IFF system, allows aircrafts to set a shared code to recognise each other as friendly.
See comments for installation and usage instructions.

------------
datalink.nas
------------
Simulates aircrafts sharing their position through a datalink connection.
Allows sending additional data through an extension system.
See comments for installation and usage instructions.

------------
airbases.nas
------------
A database of military airbases around the world.

------------
airbases.xml
------------
A gui dialog that uses the airbase database to find military airbases.

--------------------
crash-and-stress.nas
--------------------
Simulate wingstress and collisions with terrain objects.
Especially useful for jsbsim aircraft.

----------------
fire-control.nas
----------------
A fire control system that makes use of station-manager and missile-code.
Not up to date right now. See F-16 for the most upto date version.

--------------
hackCanvas.nas
--------------
Replace some FG canvas core functions to speed up canvas loops.
Mainly for aircraft with glass cockpits or HUD.
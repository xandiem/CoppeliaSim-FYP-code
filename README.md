# CoppeliaSim-FYP-code
Final year project to create software capable of creating human aware navigation data for robots
This repository provides all the essentials to run and test this software. The instructions only apply for running the software in Linux.

Setup:

To run the software it requires the downloading of the CoppeliasSim robot simulator from here: https://www.coppeliarobotics.com/downloads

From here it then requires changes to the .bashrc file. The path needs to be changed for the first line based on the file path

The following lines need to be added:

export COPPELIASIM_ROOT=/home/user/Downloads/CoppeliaSim-FYP-code-master

alias coppeliasim="cd $COPPELIASIM_ROOT; bash coppeliaSim.sh"

export B0_RESOLVER="tcp://127.0.0.1:22000"


The following commands can be used to handle minor bugs in CoppeliaSim:

cd $COPPELIASIM_ROOT

sed -i 's/function LoadModel(/function LoadModelFromFile(/g' lua/b0RemoteApiServer.lua

rm libsimExtDynamicsVortex.so


After this, the following can be added to run the API when CoppeliaSim is run, so do the following commands:

cd $COPPELIASIM_ROOT

mv lua/b0RemoteApiServer.lua simAddOnScript_b0RemoteApiServer.lua


Once all of the previous has been undertaken. The code can now been run from a new terminal using:

"coppeliasim"


This will run the simulator and set it up.


Running the software:

Once the simulator is running, we can now run the software. First open a new terminal(ctrl+shift+c) and move to the directory downloaded.
If in Downloads, the following commands would be used:

cd Downloads

cd CoppeliaSim-FYP-code-master


Once inside the directory, now using the python version downloaded either "python3" or "python":

"python3 fyp_code.py"


Which should now produce a scenario in CoppeliaSim


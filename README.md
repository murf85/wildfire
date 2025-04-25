Asynchronous Cell-DEVS Model of a wildfire based on the BehavePlus burn rate

Introduction

This repository contains a Cell-DEVS model of a wildfire, it builds on the work of Soulier and Tratnik - https://github.com/jsoulier/wildfire_simulator

The 

Dependencies

Cadmium_v2 - https://github.com/SimulationEverywhere/cadmium_v2/tree/64f61d96e7296ea5d20d45a6543cfcca2dcbc68a

BehavePlus - https://github.com/jsoulier/behave/tree/9ae08863da34b066848c011d7071f6084fb8f544

QGIS - https://qgis.org/download/

Python rasterio library - https://rasterio.readthedocs.io/en/stable/

QGIS setup 

Copy the wildfire_simulator_plugin into the QGIS folder (if you already have the plugin installed, simply replace the file called plugin.py):

C:\Users\[username]\AppData\Roaming\QGIS\QGIS3\profiles\default\python\plugins\wildfire_simulator_plugin

In the QGIS Plugins tab access the Python Console and type "import rasterio"
Under "Plugins -> Manage and Install Plugins" search for the Wildfire Simulator plugin, check to add it and then restart QGIS
There will now be a button for the Wildfire Simulation

Build

To build this project, run:

source build_sim.sh

NOTE: Every time you run build_sim.sh, the contents of build/ and bin/ will be replaced.

Execute

In order to run the model a .json configuration file is required. The examples discussed in the report can be found in the config folder.

To run the models in this project, run the following command, replacing the with name of .json file containing the simulation parameters and the desired output filename:

./bin/wildfire config/[filename].json [filename].csv

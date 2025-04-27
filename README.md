**Asynchronous Cell-DEVS Model of a wildfire based on the BehavePlus burn rate**

**Introduction**

This repository contains a Cell-DEVS model of a wildfire, it builds on the work of Soulier and Tratnik - https://github.com/jsoulier/wildfire_simulator

**Dependencies**

Cadmium_v2 - https://github.com/SimulationEverywhere/cadmium_v2/

BehavePlus - https://github.com/jsoulier/behave/

QGIS - https://qgis.org/download/

Python rasterio library - https://rasterio.readthedocs.io/en/stable/

**QGIS setup **

Copy the wildfire_simulator_plugin into the QGIS folder (if you already have the plugin installed, simply replace the file called plugin.py):

C:\Users\[username]\AppData\Roaming\QGIS\QGIS3\profiles\default\python\plugins\wildfire_simulator_plugin

In the QGIS Plugins tab access the Python Console and type "import rasterio"
Under "Plugins -> Manage and Install Plugins" search for the Wildfire Simulator plugin, check to add it and then restart QGIS
There will now be a button for the Wildfire Simulation.

**Maps**

Example maps can be found in the "maps" folder. Landcover for all of Canada can be found here:
https://open.canada.ca/data/en/dataset/ee1580ab-a23d-4f86-a09b-79763677eb47/resource/81252d30-5102-46db-a9c5-6ab1ccd5dcd7

Elevation maps for all regions can be found here:
https://ftp.maps.canada.ca/pub/elevation/dem_mne/highresolution_hauteresolution/dtm_mnt/


**Build**

In order to properly run this project clone the main repo and all of the submodules using the clone and recurese submodules commands:

git clone --recurse-submodules https://github.com/murf85/wildfire
 
Open the project in VSCode:

code wildfire

Inside VSCode build the project:

source build_sim.sh

NOTE: Every time you run build_sim.sh, the contents of build/ and bin/ will be replaced.

**Execute**

In order to run the model a .json configuration file is required. The examples discussed in the report can be found in the config folder.

To run the models in this project, run the following command, replacing the with name of .json file containing the simulation parameters, the desired output filename and the start date and time for the simulation:

./bin/wildfire config/map_1.json output_1.csv "2025-04-28 10:30:00"

**Viewer**

To view the output in QGIS go to Layer-> Add Layer -> Add Delimited Text Layer then choose the output file
Once the layer is available right click and choose Properties
In the Temporal menu click the box Dynamic Temporal Control and then set the Configuration to Single Field with Date/Time, and click the box Accumulate features of time. 
On the Temporal controller click the blue arrows and select Full Range, and set the step to seconds or minutes. The slide bar will reveal the burn progression. 

For more detailed instruction on QGIS or maps see https://github.com/jsoulier/wildfire_simulator/blob/main/README.md





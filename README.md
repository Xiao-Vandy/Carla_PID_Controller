Hi this is a readme file for how to run this program.
This is my final project for CS6376 Hybrid/Embedded System, Vanderbilt University, Prof.Abhishek Dubey
Xiao Li

Working Environment
Python 3.7, Ubuntu 22.04LTS with RTX4090.

1 Install CARLA
#Carla is the framework I have used to simulate the autonomous vehicle.
#Here is the installation script, make it executable.
chmod +x setup_carla.sh

#Install CARLA using this .sh file:

./setup_carla.sh
or bash ./setup_carla.sh

#Then you should have the carla framework.
#Activate CARLA:
./CarlaUE4.sh -carla-port=2000 -trafficManager=8000

2 Install Necessary Package for Python
#Install python version 3.7
sudo apt install python3.7 python3.7-dev python3.7-venv

#Install build dependencies
sudo apt update
sudo apt install build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev libreadline-dev libffi-dev wget

#Configure and install
./configure --enable-optimizations
sudo make altinstall

#Verify installation
python3.7 --version

#Create the virtual environment
python3.7 -m venv venv

#Activate the virtual environment
source venv/bin/activate  # On Linux/Mac
#or
.\venv\Scripts\activate  # On Windows

#Install my project directory, run this pipeline
pip install -r /carla/requirements.txt

#Also there is another package IMPORTANT for carla v0.9.10
#This is because we cannot install this package with pip or conda
easy_install /carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg

3 Run project
#For controllers
python car.py
#The controller file is in vehicle_controller.py

#Other practical script to debug
#To generate town plots interactive plots
python 001_generate_town_plots_interactive_plots.py

#To create special turning points plots
python 002_create_town_plots_special_turning_point_plots.py

#To create manual waypoints
python 003_create_plot_manual_waypoints.py

#To get camera waypoints for current position of Spectator
python 004_get_current_camera_waypoint.py

#To get waypoints xml format
#I also create a waypoints-waypoint XML label for controller script to read as a xml strcture file
python 005_get_waypoints_xml_format.py

#If you want to draw the routine of the vehicle
python 006_draw_waypoints.py

#You can use the waypoints number to generate the routine
#See Carla documents
#What I have done is to make all the points files in dir "Town_required_waypoints"
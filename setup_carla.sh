#!/usr/bin/env bash

# Download and install CARLA
mkdir carla
cd carla
# Changes were made here by same to download carla. This is the latest way to download carla.
#wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.10.1.tar.gz
wget https://carla-releases.b-cdn.net/Linux/CARLA_0.9.10.1.tar.gz
#wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_0.9.10.1.tar.gz
wget https://carla-releases.b-cdn.net/Linux/AdditionalMaps_0.9.10.1.tar.gz
tar -xf CARLA_0.9.10.1.tar.gz
tar -xf AdditionalMaps_0.9.10.1.tar.gz
rm CARLA_0.9.10.1.tar.gz
rm AdditionalMaps_0.9.10.1.tar.gz
cd ..

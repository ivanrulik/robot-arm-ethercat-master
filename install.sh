#!/bin/bash
# Install script for linux

function install(){
    echo "Starting Installation"
    sudo apt-get update
    sudo apt-get upgrade
    echo "export RA_ECAT=$PWD" >> ~/.bashrc
    echo "Cloning SOEM"
    cd ..
    git clone https://github.com/OpenEtherCATsociety/SOEM.git
    cd SOEM
    echo "export SOEM=$PWD" >> ~/.bashrc
    mkdir build && cd build
    cmake .. && make install
    echo "Installation completed"
    echo "Try to run ./scripts/build.sh to test the code"
}

install | tee install.log
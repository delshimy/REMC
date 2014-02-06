#!/bin/bash

mkdir Dependencies
cd Dependencies

#linux-lowtency kernel installation
sudo apt-get install linux-lowlatency

#installing JACK
sudo apt-get install jackd qjackctl libjack-dev

#installing ardour
sudo apt-get install ardour

#installing jmess
sudo apt-get install libasound2-dev g++ qt4-dev-tools
wget http://jmess-jack.googlecode.com/files/jmess-1.0.1.tar.gz
tar xvf jmess-1.0.1.tar.gz
cd jmess-1.0.1/src
sudo ./build
sudo make install
cd ../..

#installing SuperCollider
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys FABAEF95
sudo add-apt-repository ppa:supercollider/ppa
sudo apt-get update
sudo apt-get install supercollider supercollider-gedit supercollider-dev libsclang1 

#installing Java RunTime Environment
#sudo add-apt-repository ppa:webupd8team/java
#sudo apt-get update
#sudo apt-get install oracle-java8-installer

#installing SwingOSC
wget http://iweb.dl.sourceforge.net/project/swingosc/swingosc/0.70/SwingOSC-0.70-Linux.zip
mkdir SwingOSC
unzip SwingOSC-0.70-Linux.zip -d SwingOSC
cd SwingOSC
sudo sh install_linux_local.sh
cd ..

#installing OpenNI
sudo apt-get install git freeglut3-dev libusb-1.0-0-dev doxygen graphviz
git clone https://github.com/OpenNI/OpenNI
cd OpenNI/Platform/Linux/CreateRedist
sudo ./RedistMaker
cd ../Redist/OpenNI-Bin-Dev*
sudo ./install.sh
cd ../../../../..


#install SensorKinect
git clone https://github.com/avin2/SensorKinect.git
cd SensorKinect
git checkout master
cd Platform/Linux/CreateRedist
sudo ./RedistMaker
cd ../Redist/*
sudo ./install.sh
cd ../../../../..


#install NITE
MACHINE_TYPE=`uname -m`
if [ ${MACHINE_TYPE} == 'x86_64' ]; then
  wget http://www.openni.org/wp-content/uploads/2012/12/NITE-Bin-Linux-x64-v1.5.2.21.tar.zip
else
  wget http://www.openni.org/wp-content/uploads/2012/12/NITE-Bin-Linux-x86-v1.5.2.21.tar.zip
fi

unzip NITE-Bin-Linux-*.zip
tar xvf NITE-Bin-Linux-*.tar.bz2
cd NITE-Bin-Dev-Linux-*
sudo ./install.sh

#installing OSCeleton_for_REMC
cd ../../OSCeleton_for_REMC
make



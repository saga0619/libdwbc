#!/bin/bash

echo "LIBDWBC: Installing prerequisites..."

# if [ "$EUID" -ne 0 ]
#   then echo "Please run with sudo"
#   exit
# fi


# store current directory
CURRENT_DIR=$(pwd)
# store current user
CURRENT_USER=$(whoami)


while true; do
    echo "Select Installation method";
    echo "0 : Install all prereq for LIBDWBC";
    echo "1 : Install RBDL";
    echo "2 : Install qpOASES";
    read -p "Select Number : " yn
    case $yn in
        [0]* ) echo "Starting Install ... all prerequistes";
              mkdir Temp
              cd Temp

              git clone --recursive https://github.com/saga0619/rbdl-orb
              cd rbdl-orb
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../..

              git clone https://github.com/saga0619/qpoases
              cd qpoases
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../..

              rm -rf Temp
              exit;;
       
        [1]* ) echo "Starting Install ... RBDL";
              mkdir Temp
              cd Temp
              git clone --recursive https://github.com/saga0619/rbdl-orb
              cd rbdl-orb
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../../..
              rm -rf Temp
              exit;;
        [2]* ) echo "Starting Install ... qpOASES";
              mkdir Temp
              cd Temp
              git clone https://github.com/saga0619/qpoases
              cd qpoases
              mkdir build
              cd build
              cmake ..
              make all
              sudo make install
              cd ../../..
              rm -rf Temp
              exit;;

        [Nn]* ) echo "Aborting ...";
                exit;;
        * ) echo "Please select proper number.";;
    esac
done
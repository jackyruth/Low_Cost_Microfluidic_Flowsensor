#!/bin/bash -i

# ---------------------------------------------------------------------
# Author: Charles Ju (https://github.com/CharlesJu)                     |
# Revision: v1.0                                                        |                                      
# Description:                                                          |
#   - Workspace setup for Zephyr environment                            |          
#   - Made to easily setup working environment on any Ubuntu machine    |                                  
#   - Future revisions should also update your environment              |                       
# ---------------------------------------------------------------------


echo "///// Setting up Microvalve Zephyr workspace \\\\\\\\\\"
echo " "
echo "Checking dependencies..."
WORKSPACE_DIR=$(pwd)

# update all existing libs
sudo apt update 
sudo apt upgrade

# --- START: Manually installed dependencies ---
cmake_requirement="3.13.1"
python_requirement="3.6"
dtc_requirement="1.4.6"

# cmake
cmake_version="$(cmake --version | head -n1 | cut -d ' ' -f3)"
echo "cmake ${cmake_version} "

if ! [ "$(printf '%s\n' "$cmake_requirement" "$cmake_version" | sort -V | head -n1)" = "$cmake_requirement" ]; then 
    echo "ERROR: Cmake version less than ${cmake_requirement}"
    echo "Try one of the following:"
    echo "  $ sudo apt-get -y install cmake"
    echo "  Visit https://cmake.org/download/ and download the latest bash script."
    exit 1
fi

# python
python_version="$(python --version | cut -d ' ' -f2)"
echo "python ${python_version}"

if ! [ "$(printf '%s\n' "$python_requirement" "$python_version" | sort -V | head -n1)" = "$python_requirement" ]; then 
    echo "ERROR: Python version less than ${python_requirement}"
    echo "Try one of the following: "
    echo "  $ sudo apt install python3"
    echo "  $ sudo apt install python-is-python3"
    exit 1
fi


# dtc
dtc_version="$(dtc --version | head -n1 | cut -d ' ' -f3)"
echo "device-tree-compiler ${dtc_version}"

if ! [ "$(printf '%s\n' "$dtc_requirement" "$dtc_version" | sort -V | head -n1)" = "$dtc_requirement" ]; then 
    echo "ERROR: device-tree-compiler version less than ${dtc_requirement}"
    echo "Try the following: "
    echo "  $ sudo apt-get install -y device-tree-compiler"
    exit 1
fi

# --- END: Manually installed dependencies ---


sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget \
  python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
  make gcc gcc-multilib g++-multilib libsdl2-dev


# --- START: Zephyr environment installation ---
echo "Done"
echo " "
echo "Checking for West tool..."

# Checks to see if west is a PATH variable
if ! [ -x "$(command -v west)" ]; then 
    echo "  No West tool found, installing..."
    pip3 install --user -U west
    echo 'export PATH=~/.local/bin:"$PATH"' >> ~/.bashrc
    . ~/.bashrc
    echo "  Done"
else
    echo "  West tool already installed."
fi
echo "Done"
echo " "
# echo "Checking Zephyr-project install..."

# if [ ! -d "${HOME}/zephyrproject" ]; then
# echo "  Zephyr-project not found, installing..."
#     west init ~/zephyrproject --mr v2.6.0
# else
#     echo "  Zephyr-project already installed"
# fi
# cd ~/zephyrproject
# west update
# west zephyr-export
# pip3 install --user -r ~/zephyrproject/zephyr/scripts/requirements.txt
# echo "Done"
cd ~
# --- END: Zephyr environment installation ---


# --- START: Zephyr SDK installation --- 
echo " "
echo "Checking Zephyr SDK installation at ~/zephyr-sdk-0.12.4"
if [ ! -d "${HOME}/zephyr-sdk-0.12.4" ] ;then
    echo "  Installing Zephyr SDK at ~/zephyr-sdk-0.12.4 ..."
    wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.12.4/zephyr-sdk-0.12.4-x86_64-linux-setup.run
    chmod +x zephyr-sdk-0.12.4-x86_64-linux-setup.run
    ./zephyr-sdk-0.12.4-x86_64-linux-setup.run -- -d ~/zephyr-sdk-0.12.4
    sudo cp ~/zephyr-sdk-0.12.4/sysroots/x86_64-pokysdk-linux/usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d
    sudo udevadm control --reload
    echo "  Done"
else
    echo "  Zephyr SDK already installed"
fi
echo "Done"
cd ~
# --- END: Zephyr SDK installation --- 

# T2 Workspace initilization
cd ${WORKSPACE_DIR}
echo " "
echo "Initializing workspace..."
west config zephyr.base
west init --mr v2.6.0
west update
west zephyr-export
pip3 install --user -r ${WORKSPACE_DIR}/zephyr/scripts/requirements.txt
echo "Done"

echo "\\\\\\\\\\Workspace setup complete/////"

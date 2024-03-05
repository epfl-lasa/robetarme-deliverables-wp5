#!/usr/bin/bash
# This script sets up the environment for the use of control-libraries in the project

checkError () {
    if [ $? -ne 0 ]; then
    echo "An Error occured, setup aborted."
    exit 1
    fi
}

mkdir temp
cd temp

# Install control Lasa
echo "* CLONING CONTROL LIBRAIRIES LASA..."
git clone https://github.com/epfl-lasa/control-libraries.git --branch v6.3.1  --single-branch
cd control-libraries/source
echo "* CONFIGURING CONTROL LIBRAIRIES LASA..."
sudo bash install.sh -y
checkError
cd ../..

# Clean up
cd ..
echo "* CLEANING WORKSPACE..."
rm -rf temp

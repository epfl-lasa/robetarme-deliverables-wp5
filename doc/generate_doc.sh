#!/bin/bash
# Script to generate the documentation using Doxygen and create a symbolic link to the index.html file
#
# Author: Louis Munier - lmunier@protonmail.com
# Update: 240507
SCRIPT_DIR=$(dirname "$0")
cd "$SCRIPT_DIR"

# Generate the documentation
doxygen Doxyfile

# Create a symbolic link to the index.html file
ln -s ./html/index.html ./index.html
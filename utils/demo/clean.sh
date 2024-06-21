#!/bin/bash

# Check if the 'deployes' directory exists and remove it
if [ -d "deployes" ]; then
    echo "Removing 'deployes' directory..."
    rm -rf deployes
    echo "'deployes' directory removed."
else
    echo "'deployes' directory does not exist."
fi

# Check if the 'files' directory exists and remove it
if [ -d "files" ]; then
    echo "Removing 'files' directory..."
    rm -rf files
    echo "'files' directory removed."
else
    echo "'files' directory does not exist."
fi

echo "Cleanup complete."

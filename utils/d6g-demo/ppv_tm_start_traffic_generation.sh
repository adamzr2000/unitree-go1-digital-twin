#!/bin/bash

# Set default values for variables
SCRIPT_NAME_PATTERN="script_name_c.cc0_X_port30000*cli"
FT_VALUE="GC0"
F_VALUE="100"
P_VALUE="6000"
B_VALUE="20000"

# Navigate to the pyperf directory
PYTHON_SCRIPT_DIR=~/ppv/pyperf

# Check if the directory exists
if [ ! -d "$PYTHON_SCRIPT_DIR" ]; then
    echo "Error: Directory '$PYTHON_SCRIPT_DIR' does not exist."
    exit 1
fi

cd "$PYTHON_SCRIPT_DIR"

# Check if the pyperf.py script exists
if [ ! -f "./pyperf.py" ]; then
    echo "Error: pyperf.py script not found in '$PYTHON_SCRIPT_DIR'."
    exit 1
fi

# Run the script with the default parameters
./pyperf.py "$SCRIPT_NAME_PATTERN" -ft "$FT_VALUE" -f "$F_VALUE" -p "$P_VALUE" -B "$B_VALUE"

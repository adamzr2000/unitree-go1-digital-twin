#!/bin/bash

# Function to display usage information
usage() {
    echo "Usage: $0 [-v <vxlan_id>]"
    echo "  -v <vxlan_id>        VXLAN ID (default: 200)"
    exit 1
}

# Parse input arguments
vxlan_id="200"
while getopts "v:" opt; do
    case ${opt} in
        v ) vxlan_id=$OPTARG ;;
        * ) usage ;;
    esac
done

vxlan_iface="vxlan$vxlan_id"
echo -e "\nRemoving VXLAN network interface '$vxlan_iface'..."
sudo ip link set $vxlan_iface down
sudo ip link del $vxlan_iface

echo -e "\nCleanup completed successfully."
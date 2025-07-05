#!/bin/bash

# Check for required arguments
if [[ $# -lt 4 ]]; then
  echo "Usage: $0 --k3s-interface <interface> --k3s-ip <ip-address>"
  exit 1
fi

# Parse arguments
while [[ "$#" -gt 0 ]]; do
  case $1 in
    --k3s-interface) K3S_INTERFACE="$2"; shift ;;
    --k3s-ip) K3S_IP="$2"; shift ;;
    *) echo "Unknown parameter: $1"; exit 1 ;;
  esac
  shift
done

# Validate that the required arguments are set
if [[ -z "$K3S_INTERFACE" || -z "$K3S_IP" ]]; then
  echo "Error: Both --k3s-interface and --k3s-ip must be provided."
  exit 1
fi

# File to modify
K3S_SERVICE_FILE="/etc/systemd/system/k3s-agent.service"

# Check if the file exists
if [[ ! -f "$K3S_SERVICE_FILE" ]]; then
  echo "Error: $K3S_SERVICE_FILE does not exist."
  exit 1
fi

# Define the sudo password
SUDO_PASSWORD="netcom;"

# Update the file with new values
echo "Updating $K3S_SERVICE_FILE with interface '$K3S_INTERFACE' and IP '$K3S_IP'..."
echo "$SUDO_PASSWORD" | sudo -S sed -i "s/--node-ip [^ ]*/--node-ip $K3S_IP/" "$K3S_SERVICE_FILE"
echo "$SUDO_PASSWORD" | sudo -S sed -i "s/--node-external-ip [^ ]*/--node-external-ip $K3S_IP/" "$K3S_SERVICE_FILE"
echo "$SUDO_PASSWORD" | sudo -S sed -i "s/--flannel-iface [^ ]*/--flannel-iface $K3S_INTERFACE/" "$K3S_SERVICE_FILE"

# Reload systemd and restart K3s
echo "Reloading systemd and restarting K3s..."
echo "$SUDO_PASSWORD" | sudo -S systemctl daemon-reload
echo "$SUDO_PASSWORD" | sudo -S systemctl restart k3s-agent

# Annotate the node "robot" to overwrite the public IP used by Flannel
echo "$SUDO_PASSWORD" | sudo -S kubectl annotate node robot flannel.alpha.coreos.com/public-ip-overwrite=$K3S_IP

# Confirm changes
if [[ $? -eq 0 ]]; then
  echo "K3s service updated and restarted successfully."
else
  echo "Error: Failed to restart K3s service."
  exit 1
fi


#!/bin/bash

# Default values
REMOTE_USER=""
ROBOT_IP=""
FLANNEL_INTERFACE=""
PASSWORD="netcom;"

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --user)
      REMOTE_USER="$2"
      shift 2
      ;;
    --host)
      ROBOT_IP="$2"
      shift 2
      ;;
    --flannel-interface)
      FLANNEL_INTERFACE="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

# Validate required arguments
if [[ -z "$REMOTE_USER" || -z "$ROBOT_IP" || -z "$FLANNEL_INTERFACE" ]]; then
  echo "Usage: $0 --user <remote_user> --host <robot_ip> --flannel-interface <interface>"
  exit 1
fi

# Step 1: Execute local command
echo "Executing local script: ./delete_k3s_node.sh robot"
./delete_k3s_node.sh robot

# Step 2: Execute remote commands via SSH
echo "Executing remote commands on $ROBOT_IP..."

ssh -T "${REMOTE_USER}@${ROBOT_IP}" << EOF
cd /home/${REMOTE_USER}/d6g-demo-k3s-cluster-setup-5tonic
./update_k3s_agent_service.sh --k3s-interface ${FLANNEL_INTERFACE} --k3s-ip ${ROBOT_IP}
EOF

echo "Deployment completed successfully!"


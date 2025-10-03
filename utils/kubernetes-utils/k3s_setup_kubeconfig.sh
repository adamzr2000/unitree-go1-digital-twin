#!/bin/bash

# Set KUBECONFIG environment variable
export KUBECONFIG=~/.kube/config

# Create the .kube directory if it doesn't exist
mkdir -p ~/.kube

# Retrieve and store the Kubernetes config from k3s
echo "Retrieving k3s kubeconfig..."
sudo k3s kubectl config view --raw > "$KUBECONFIG"

# Secure the kubeconfig file
chmod 600 "$KUBECONFIG"

# Ensure the export command is added to .bashrc if not already present
if ! grep -q "export KUBECONFIG=~/.kube/config" ~/.bashrc; then
    echo "export KUBECONFIG=~/.kube/config" >> ~/.bashrc
    echo "Added KUBECONFIG export to ~/.bashrc"
else
    echo "KUBECONFIG export already present in ~/.bashrc"
fi

# Reload .bashrc to apply changes immediately
source ~/.bashrc

echo "KUBECONFIG setup completed successfully!"

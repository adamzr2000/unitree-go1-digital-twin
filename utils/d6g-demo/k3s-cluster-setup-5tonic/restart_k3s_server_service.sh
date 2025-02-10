#!/bin/bash

# Reload systemd and restart K3s
echo "Reloading systemd and restarting K3s server..."
sudo systemctl daemon-reload
sudo systemctl restart k3s
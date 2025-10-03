#!/bin/bash

# Step 1: Delete the Multus DaemonSet
kubectl delete -f multus-daemonset.yml

# Step 2: Delete the NetworkAttachmentDefinition CRD (if exists)
kubectl delete crd network-attachment-definitions.k8s.cni.cncf.io --ignore-not-found

# Step 3: Remove Multus CNI configuration files
sudo rm -rf /var/lib/rancher/k3s/agent/etc/cni/net.d/00-multus.conflist
sudo rm -rf /var/lib/rancher/k3s/agent/etc/cni/net.d/multus.d

# Step 4: Remove Multus binaries (if exists)
sudo rm -f /var/lib/rancher/k3s/data/cni/multus

# Step 5: Restart K3s to apply changes
sudo systemctl restart k3s

# Step 6: Verify Multus is removed
echo "Verifying Multus removal..."
kubectl get pods -n kube-system | grep multus
ls -l /var/lib/rancher/k3s/agent/etc/cni/net.d/
ls -l /var/lib/rancher/k3s/data/cni/

echo "Multus removal process completed."

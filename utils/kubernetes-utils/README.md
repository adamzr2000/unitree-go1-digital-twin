## K3s Management Commands

### Start/Stop Servers
```bash
sudo systemctl stop k3s     
sudo systemctl start k3s     
```

### Start/Stop Agents
```bash
sudo systemctl stop k3s-agent     
sudo systemctl start k3s-agent   
```

### Reset K3s Container State
```bash
/usr/local/bin/k3s-killall.sh
```

### Fix Kubectl Permissions
```bash
sudo chmod 644 /etc/rancher/k3s/k3s.yaml
```

### Manage K3s Docker Images

List images:
```bash
sudo k3s ctr images list | grep docker.io   
```

Import images:
```bash
docker save <docker-image-name> | sudo k3s ctr images import - 
```

---

## Install Multus

[Multus](https://github.com/k8snetworkplumbingwg/multus-cni) is a CNI (Container Network Interface) plugin for Kubernetes that allows pods to connect to multiple networks. This is particularly useful for advanced networking configurations where pods need to interact with multiple interfaces, such as overlay networks or external systems.

1. Add the Helm repository for RKE2 charts and update it:
```bash
helm repo add rke2-charts https://rke2-charts.rancher.io
helm repo update
```

2. Install Multus CNI using Helm in the `kube-system` namespace:
```bash
helm install multus rke2-charts/rke2-multus -n kube-system --kubeconfig ~/.kube/config --values ./utils/multus-values.yaml
```

3. Verify that Multus is running:
```sh
kubectl get pods -n kube-system | grep multus
```

Expected output:
```
kube-multus-ds-xxxxxxx    Running
```

4. Confirm Multus Installation
Check if Multus is installed and registered as a CNI plugin:

```sh
sudo ls /var/lib/rancher/k3s/agent/etc/cni/net.d/
```

You should see:
```
00-multus.conf
```

---

# VXLAN Setup Script

```sh
sudo ./vxlan_setup_multi_hosts.sh -l 10.3.202.67 -r 10.5.1.21 -i ue0 -v 200 -p 4747 -a 10.10.10.1/24

sudo ./vxlan_setup_multi_hosts.sh -l 10.5.1.21 -r 10.3.202.67 -i br10 -v 200 -p 4747 -a 10.10.10.2/24
```

```sh
sudo ./remove_vxlan_setup_hosts.sh -v 200
```


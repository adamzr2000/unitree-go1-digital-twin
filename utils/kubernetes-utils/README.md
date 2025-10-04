# Build a K8S cluster

To effortlessly set up a fully-functional, single-node Kubernetes cluster, execute the following command:
```bash
curl -sfL https://get.k3s.io | INSTALL_K3S_EXEC="--cluster-cidr=10.42.0.0/16 --service-cidr=10.43.0.0/16 --" sh -s -
```
> Note: This single-node will function as a server, including all the `datastore`, `control-plane`, `kubelet`, and `container runtime` components necessary to host workload pods. 

After installing k3s, run `./utils/k3s_setup_kubeconfig.sh` to set up the `KUBECONFIG` environment, allowing the user to manage the Kubernetes cluster with `kubectl` without requiring `sudo`, with proper ownership and permissions.

To check the CIDR allocations for pods and services in the cluster, run:
```bash
kubectl get nodes -o jsonpath='{range .items[*]}{.metadata.name}{" - "}{.spec.podCIDR}{"\n"}{end}'
kubectl get svc -A -o wide
```

---

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

### Manage Container Registry

List images:
```bash
sudo k3s ctr images list | grep docker.io   
```

Import images:
```bash
docker save <docker-image-name> | sudo k3s ctr images import - 
```

---

## Install Helm CLI

You can install Helm, the Kubernetes package manager, following this [tutorial](https://helm.sh/docs/intro/install/)

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
sudo ./vxlan_setup_multi_hosts.sh -l 10.5.1.21 -r 10.3.202.67 -i br10 -v 200 -p 4747 -a 172.20.50.1/24

sudo ./vxlan_setup_multi_hosts.sh -l 10.3.202.67 -r 10.5.1.21 -i ue0 -v 200 -p 4747 -a 172.20.50.2/24
```

## Capture VXLAN Traffic with TShark
```bash
sudo tshark -i br10 -f "udp port 4747" -d udp.port==4747,vxlan -V --color
```

```sh
sudo ./remove_vxlan_setup_hosts.sh -v 200
```


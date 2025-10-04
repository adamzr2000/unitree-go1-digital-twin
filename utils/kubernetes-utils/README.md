# Build a K8S cluster

To effortlessly set up a fully-functional, single-node Kubernetes cluster, execute the following command:
```bash
curl -sfL https://get.k3s.io | sh -
```
> Note: This single-node will function as a server, including all the `datastore`, `control-plane`, `kubelet`, and `container runtime` components necessary to host workload pods. 

After installing k3s, run `./k3s_setup_kubeconfig.sh` to set up the `KUBECONFIG` environment, allowing the user to manage the Kubernetes cluster with `kubectl` without requiring `sudo`, with proper ownership and permissions.

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

### Kustomization files 
```bash
kubectl apply -k services/

kubectl delete -k services/
```

[Documentation](https://kubectl.docs.kubernetes.io/references/kustomize/glossary/#kustomization)

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
helm install multus rke2-charts/rke2-multus --version 4.1.001 -n kube-system --kubeconfig ~/.kube/config --values multus-values.yaml
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

## Remove Multus

```bash
# Uninstall the release
helm uninstall multus -n kube-system

# Remove the CRD that Multus added (if it still exists)
kubectl delete crd network-attachment-definitions.k8s.cni.cncf.io 2>/dev/null || true

# Optionally remove the repo entry you added
helm repo remove rke2-charts

# Multus autoconfig & kubeconfig (only if they exist)
sudo rm -rf /var/lib/rancher/k3s/agent/etc/cni/net.d/multus.d
sudo rm -f  /var/lib/rancher/k3s/agent/etc/cni/net.d/*multus*.conf

# (Optional) If you created a custom binDir for Multus plugins, remove it
sudo rm -rf /var/lib/rancher/k3s/data/cni/ 2>/dev/null || true
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

## Test multus
```bash
kubectl apply -f test-multus-pods-ipvlan-1.yaml
kubectl exec -it pod-1 -- sh
```

```bash
kubectl apply -f test-multus-pods-ipvlan-2.yaml
kubectl exec -it pod-4 -- sh
```

## Remove VXLAN Setup

```sh
sudo ./remove_vxlan_setup_hosts.sh -v 200
```


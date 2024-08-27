# Unitree Go1 Digital Twin service deployment on end-to-end scenario

## What is this?

This repository contains the necessary files to run the Digital Twin stack on a `Kubernetes` cluster,deploying the containers in the form of `Pods`, `Services`, and `Deployments` across different hosts.

## Pre-requisites

Each machine running the containers must have the following components installed: 
* K3s Kubernetes Distribution

Also, in absence of a centralized registry, the Docker images composing the service need to be already present in the machine (therefore you need to build each service image manually for every host machine before running the stack, and then import them into the `k3s registry`, which utilizes containerd.). 

Execute the following command to import the docker images into your k3s registry:
```bash
docker save <docker-image-name> | sudo k3s ctr images import -
```

## Hosts configuration

This is the multi-host setup: 

- **Server Node (EDGE)**: 
  - IP: `192.168.40.4`
  - Pods/Services: `roscore-edge`, `digital-twin-app`, `gesture-control-app`, `go1-navigation`, `rviz-vnc`

- **Agent Node (ROBOT)**: 
  - IP: `192.168.40.70`
  - Pods/Services: `lidar`, `camera`, `go1-base`

Here is a diagram that represents visually the architecture of the scenario:

![E2E Scenario 5TONIC](../../images/e2e-scenario-kubernetes.png)

To participate in the ROS network, ROS nodes need to know the IP address of the ROS Master. However,in Kubernetes, we don't know the IP address of the `roscore-edge` container until the application is deployed. To address this issue, a solution is implemented using a `Headless Service` so that application pods can DNS the hostname based on that service name, whose backend is corresponding application pods.

### Cluster installation on the edge host as server

To effortlessly set up a fully-functional, single-node Kubernetes cluster, execute the following command:
```bash
curl -sfL https://get.k3s.io | sh -
```

> Note: The installation should be executed on the `edge` host. This single-node will function as a server, including all the `datastore`, `control-plane`, `kubelet`, and `container runtime` components necessary to host workload pods. 

After installing k3s, use the `export KUBECONFIG="/etc/rancher/k3s/k3s.yaml"` environment variable to specify to `kubectl` the location of the [kubeconfig](https://kubernetes.io/docs/concepts/configuration/organize-cluster-access-kubeconfig/) file required for cluster configuration.

To ensure permanent application of this environment variable during startup, add it to either `~/.bash_profile` or `~/.bashrc` files:
```bash
echo 'export KUBECONFIG="/etc/rancher/k3s/k3s.yaml"' >> ~/.bashrc
```

### Adding the robot host to the cluster as agent

To add the `robot` host as an additional agent node and include it in the cluster, run the installation script with the `K3S_URL` and `K3S_TOKEN` environment variables. 

```bash
curl -sfL https://get.k3s.io | K3S_URL=https://192.168.40.4:6443 K3S_TOKEN=mynodetoken sh -
```

You can find the token value required for `K3S_TOKEN` at `/var/lib/rancher/k3s/server/node-token` on your server node.

Next, copy the `/etc/rancher/k3s/k3s.yaml` file from the server to the agent node. Ensure to update the IP address in the file to match the edge IP address as the server.

The `/etc/rancher/k3s/k3s.yaml` file should look like this:
```bash
apiVersion: v1
clusters:
- cluster:
    certificate-authority-data: 
    ...
    server: https://192.168.40.4:6443
  name: default
contexts:
- context:
    cluster: default
    user: default
  name: default
current-context: default
kind: Config
preferences: {}
users:
- name: default
  user:
    client-certificate-data: 
    ...
    client-key-data:
    ...
```

Finally, set it as an environmental variable:
```bash
echo 'export KUBECONFIG="/etc/rancher/k3s/k3s.yaml"' >> ~/.bashrc
```

### Applying labels to cluster nodes

To label nodes, execute the following commands from the server. 
```bash
# Show the hostnames of the nodes
kubectl get nodes --show-labels

# Add labels
kubectl label nodes <server-node-hostname> nodetype=edge
kubectl label nodes <agent-node-hostname> nodetype=robot
```

You can use the following command to eliminate labels from nodes:
```bash
kubectl label node <node-hostname> nodetype-
```

## Deployment

Run this command in the `edge` node: 
```bash
kubectl apply -f digital-twin-service.yml
```








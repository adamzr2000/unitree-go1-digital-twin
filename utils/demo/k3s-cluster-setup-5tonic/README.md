# Cluster Configuration

## 1. Edge Server
Delete the K3s node named `robot`:
```bash
./delete_k3s_node.sh robot
```

## 2. Rpi 5G HAT
Connect to 5G network and set up NAT for the robot:
```bash
python3 connect_5g_5tonic_nat_robot.py --robot-ip 192.168.2.2
```

## 3. Robot (miniPC)
Update the K3s agent service with the correct network interface and IP:
```bash
./update_k3s_agent_service.sh --k3s-interface ue0 --k3s-ip 10.11.15.X
```
---
## Quick-Start Guide

```bash
./ssh_k3s_cluster_config.sh --user netcom --host 10.11.15.X --flannel-interface ue0
```
---
# Utility Commands

## Verify Flannel Public IP Annotation
```bash
kubectl describe node robot | grep flannel.alpha.coreos.com/public-ip
```

## Set Flannel Public IP Annotation
```bash
kubectl annotate node robot flannel.alpha.coreos.com/public-ip-overwrite=10.11.15.X
```

## Alpine Deployment Test
```bash
kubectl apply -f alpine-test.yaml
```

## Access Running Alpine Deployments
```bash
kubectl exec -it deploy/alpine1-deployment -- sh
kubectl exec -it deploy/alpine2-deployment -- sh
```

## Capture VXLAN Traffic with TShark
```bash
sudo tshark -i <physical-interface> -f "udp port 8472" -d udp.port==8472,vxlan -V --color
```

## Edge Server Configuration
```bash
desire6g@xtreme:~/adam/kubernetes_utils$ sudo cat /etc/systemd/system/k3s.service
[Unit]
Description=Lightweight Kubernetes
Documentation=https://k3s.io
Wants=network-online.target
After=network-online.target

[Install]
WantedBy=multi-user.target

[Service]
Type=notify
EnvironmentFile=-/etc/default/%N
EnvironmentFile=-/etc/sysconfig/%N
EnvironmentFile=-/etc/systemd/system/k3s.service.env
KillMode=process
Delegate=yes
# Having non-zero Limit*s causes performance problems due to accounting overhead
# in the kernel. We recommend using cgroups to do container-local accounting.
LimitNOFILE=1048576
LimitNPROC=infinity
LimitCORE=infinity
TasksMax=infinity
TimeoutStartSec=0
Restart=always
RestartSec=5s
ExecStartPre=/bin/sh -xc '! /usr/bin/systemctl is-enabled --quiet nm-cloud-setup.service 2>/dev/null'
ExecStartPre=-/sbin/modprobe br_netfilter
ExecStartPre=-/sbin/modprobe overlay

ExecStart=/usr/local/bin/k3s \
    server \
    --write-kubeconfig-mode 644 \
    --node-label nodetype=edge \
    --node-ip 10.5.1.21 \
    --flannel-iface br10 \
    --node-external-ip 10.5.1.21
```

## Robot Configuration
```bash
netcom@robot:~$ sudo cat /etc/systemd/system/k3s-agent.service.env 
K3S_TOKEN='K107ac68c77b9c7485c3ee1dee86f7e52c0bb142e64cd685d4334b88d0da90d6913::server:f55150e2346b1629791b019a6a56ce19'
K3S_URL='https://10.5.1.21:6443'

netcom@robot:~$ sudo cat /etc/systemd/system/k3s-agent.service
[Unit]
Description=Lightweight Kubernetes
Documentation=https://k3s.io
Wants=network-online.target
After=network-online.target

[Install]
WantedBy=multi-user.target

[Service]
Type=notify
EnvironmentFile=-/etc/default/%N
EnvironmentFile=-/etc/sysconfig/%N
EnvironmentFile=-/etc/systemd/system/k3s-agent.service.env
KillMode=process
Delegate=yes
# Having non-zero Limit*s causes performance problems due to accounting overhead
# in the kernel. We recommend using cgroups to do container-local accounting.
LimitNOFILE=1048576
LimitNPROC=infinity
LimitCORE=infinity
TasksMax=infinity
TimeoutStartSec=0
Restart=always
RestartSec=5s
ExecStartPre=/bin/sh -xc '! /usr/bin/systemctl is-enabled --quiet nm-cloud-setup.service 2>/dev/null'
ExecStartPre=-/sbin/modprobe br_netfilter
ExecStartPre=-/sbin/modprobe overlay

ExecStart=/usr/local/bin/k3s \
        agent \
	    --node-label nodetype=robot \
        --node-ip 10.11.15.132 \
        --node-external-ip 10.11.15.132
```

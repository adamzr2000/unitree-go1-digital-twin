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
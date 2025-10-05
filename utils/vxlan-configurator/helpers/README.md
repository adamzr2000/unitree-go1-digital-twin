## Configure VXLAN

```sh
# edge
sudo ./vxlan_setup_multi_hosts.sh -r 10.3.202.68 -i br10 -v 200 -p 4747 -a 172.20.50.1/24
sudo ./add_vxlan_peers.sh -d vxlan200 -r 10.5.99.30

# robot
sudo ./vxlan_setup_multi_hosts.sh -r 10.5.1.21 -i ue0 -v 200 -p 4747 -a 172.20.50.2/24
sudo ./add_vxlan_peers.sh -d vxlan200 -r 10.5.99.30

# edge2
sudo ./vxlan_setup_multi_hosts.sh -r 10.5.1.21,10.3.202.68 -i eth0 -v 200 -p 4747 -a 172.20.50.3/24
```

## Remove VXLAN

```sh
sudo ./remove_vxlan_setup_hosts.sh -v 200
```


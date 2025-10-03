# PTPD clock synchronization

## Edge server node (Master)
```bash
sudo ptpd --masteronly --interface enp9s0f1 --unicast --unicast-destinations 10.3.202.67 -V
```

## Robot node (Slave)
```bash
sudo ptpd --slaveonly --interface ue0 --unicast --unicast-destinations 10.11.7.4 -V
```
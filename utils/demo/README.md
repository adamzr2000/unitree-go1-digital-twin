# Demo Workflow

## 1. SMO

### Deploy the Service

```bash
curl -X POST -H "Content-Type: application/json" -d '{"json_data": {"type": "service_graph", "name": "final_dt_demo_nsd.sg.yml", "site_id": "desire6g-site"}}' http://localhost:32008/main_id/ | jq
```

### Check Deployed Services

Verify the status of deployed services and retrieve the `service_id`:

```bash
curl http://localhost:32008/deployed_services 
```

### Delete the Service

```bash
curl -X DELETE "http://localhost:32008/main_id/<service_id>" | jq
```
	
## 2. Edge Server 1

### Start iperf server

```bash
cd ~/ppv/pyperf
./pyperf.py script_name_s.cc0*srv -ft GC0 -p 6000
```

## 3. Traffic Generator

### Simulate Traffic for 20 Users

```bash
cd ~/ppv/pyperf
./pyperf.py script_name_c.cc0_X_port30000*cli -ft GC0 -f 10 -p 6000 -B 20000
```

> Note: The first port (20000) is used by the robot network service, while ports 20001-20100 are used by the background service. The number of users/flows can be adjusted with the `-f` option.

### Remove Simulated Traffic

To stop the simulated traffic, launch the GUI and select `kill all`:

```bash
./pyperf.py gui
```

## 4. IML

### Load Equal Sharing Policy

```bash
./iml_reset_policies.sh
```

### Load QoS Policy for robot users belonging to the low-latency network service

```bash
./iml_qos_policies.sh
```

# Demo Workflow

## 1. SMO

### Deploy the Service

```bash
curl -X POST -H "Content-Type: application/json" -d '{"json_data": {"type": "service_graph", "name": "dt_demo_nsd.sg.yml", "site_id": "desire6g-site"}}' http://localhost:32008/main_id/ | jq
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
	
## 2. Remote Operator

### Send video frames to the 5TONIC server:

```bash
./video_sender.sh --server_url http://10.5.1.21:8888/upload_frame
```

## 3. Edge Server 1

### Start iperf server

```bash
cd ~/ppv/pyperf
./pyperf.py script_name_s.cc0*srv -ft GC0 -p 6000
```

## 4. Traffic Generator

### Simulate Traffic for 100 Users

```bash
cd ~/ppv/pyperf
./pyperf.py script_name_c.cc0_X_port30000*cli -ft GC0 -f 5 -p 6000 -B 20000
```

> Note: The first port (20000) is used by the robot network service, while ports 20001-20100 are used by the background service. The number of users/flows can be adjusted with the `-f` option.

### Remove Simulated Traffic

To stop the simulated traffic, launch the GUI and select `kill all`:

```bash
./pyperf.py gui
```

## 5. IML

### Load Equal Sharing Policy

Apply performance isolation policies to ensure equal resource sharing among network slices.

Apply Policy to Network Slice 1
```bash
curl -F file=@policies/demo_pol_2_perf_isolation_same_pol_ns-slice_1.yml http://localhost:5000/iml/update-policy
```

Apply Policy to Network Slice 2
```bash
curl -F file=@policies/demo_pol_2_perf_isolation_same_pol_ns-slice_2.yml http://localhost:5000/iml/update-policy
```


### Modify Resource Usage Policies

Adjust resource usage policies for the robot network service to ensure higher guaranteed bandwidth for the robot and reduce resource allocation for background flows.

Increase Bandwidth for Robot User in Network Slice 1
```bash
curl -F file=@policies/demo_pol_3_perf_isolation_different_ns-slice_1-robot.yml http://localhost:5000/iml/update-policy
```

Reduce Resource Usage for Background Flow in Network Slice 1
```bash
curl -F file=@policies/demo_pol_3_perf_isolation_different_ns-slice_1-bg.yml http://localhost:5000/iml/update-policy
```

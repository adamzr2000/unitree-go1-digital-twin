
## Overview

The provided scripts allow for applying different performance policies to network slices, ensuring equal performance sharing, or modifying resource usage to prioritize specific users or flows.

## Scripts

### performance_isolation.sh

This script applies a policy to ensure equal performance sharing among services and users. It loads the performance isolation policy for two network slices.

**Usage:**
```bash
./performance_isolation.sh
```

**Script Details:**
```bash
#!/bin/bash

# This script applies a policy to ensure equal performance sharing among services and users.
# The policy files ensure that network slices have performance isolation.

# Applying the performance isolation policy to network slice 1
curl -F file=@demo_pol_2_perf_isolation_same_pol_ns-slice_1.yml http://localhost:5000/iml/update-policy

# Applying the performance isolation policy to network slice 2
curl -F file=@demo_pol_2_perf_isolation_same_pol_ns-slice_2.yml http://localhost:5000/iml/update-policy
```

### no_performance_isolation.sh

This script removes performance isolation, allowing resources to be shared without restrictions. It applies the no performance isolation policy for two network slices.

**Usage:**
```bash
./no_performance_isolation.sh
```

**Script Details:**
```bash
#!/bin/bash

# This script removes performance isolation, allowing resources to be shared without restrictions.
# The policy files ensure that network slices do not have performance isolation.

# Applying no performance isolation policy to network slice 1
curl -F file=@demo_pol_1_no_perf_isolation_ns-slice_1.yml http://localhost:5000/iml/update-policy

# Applying no performance isolation policy to network slice 2
curl -F file=@demo_pol_1_no_perf_isolation_ns-slice_2.yml http://localhost:5000/iml/update-policy
```

### modify_resource_usage.sh

This script modifies resource usage policies in the robot network service. It ensures higher guaranteed bandwidth for the robot user and reduces the resource usage of the background (bg) flow.

**Usage:**
```bash
./modify_resource_usage.sh
```

**Script Details:**
```bash
#!/bin/bash

# This script modifies resource usage policies in the robot network service.
# It ensures higher guaranteed bandwidth for the robot user and reduces the resource usage of the background (bg) flow.

# Apply the policy to increase guaranteed bandwidth for the robot user in network slice 1
curl -F file=@demo_pol_3_perf_isolation_different_ns-slice_1-robot.yml http://localhost:5000/iml/update-policy

# Apply the policy to reduce resource usage for the background flow in network slice 1
curl -F file=@demo_pol_3_perf_isolation_different_ns-slice_1-bg.yml http://localhost:5000/iml/update-policy
```

## Service Management and Orchestration

### SMO Endpoints

#### Upload Service Graph

Upload the service graph if it does not exist.
```bash
curl -X POST -F "file=@demo_nsd.sg.yaml" http://10.5.15.55:32006/upload/
```

#### Add Site

Add a site if it does not exist.
```bash
curl -X POST http://10.5.15.55:32007/nodes/ -H 'accept: application/json' -H 'Content-Type: application/json' -d '{
   "site_id": "desire6g-site",
   "cpu": 8,
   "mem": 32,
   "storage": 1024
}'
```

#### Deploy Service

Each deployment is unique, so you can deploy the same service multiple times.
```bash
curl -X POST -H "Content-Type: application/json" -d '{"json_data": {"type": "service_graph", "name": "demo_nsd.sg.yml", "site_id": "desire6g-site"}}' http://localhost:32008/main_id/ | jq
```

#### Check Deployed Services

Check the status of deployed services.
```bash
curl http://localhost:32008/deployed_services
```

#### Delete Service

Delete a deployed service by ID.
```bash
curl -X DELETE http://localhost:32008/main_id/11
```

### IML Endpoints

#### Deploy

Deploy a YAML configuration. The response will include the ID of the deployment.
```bash
curl -F file=@demo_nsd.yml http://localhost:5000/iml/yaml/deploy
```

#### Delete

Delete a deployment by ID. The response will indicate success or failure.
```bash
curl -X DELETE http://localhost:5000/iml/yaml/deploy/1
```

#### Update Policy

Update a policy. The response will indicate success or failure.
```bash
curl -F file=@demo_policy.yml http://localhost:5000/iml/update-policy
```

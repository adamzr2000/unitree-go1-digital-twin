#!/bin/bash

# This script removes performance isolation, allowing resources to be shared without restrictions.
# The policy files ensure that network slices do not have performance isolation.

echo "Applying no performance isolation policy to network slice 1..."
curl -F file=@demo_pol_1_no_perf_isolation_ns-slice_1.yml http://localhost:5000/iml/update-policy

echo "Applying no performance isolation policy to network slice 2..."
curl -F file=@demo_pol_1_no_perf_isolation_ns-slice_2.yml http://localhost:5000/iml/update-policy

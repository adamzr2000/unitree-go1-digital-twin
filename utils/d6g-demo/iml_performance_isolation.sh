#!/bin/bash

# This script applies a policy to ensure equal performance sharing among services and users.
# The policy files ensure that network slices have performance isolation.

echo "Applying performance isolation policy to network slice 1..."
curl -F file=@policies/demo_pol_2_perf_isolation_same_pol_ns-slice_1.yml http://localhost:5000/iml/update-policy

echo "Applying performance isolation policy to network slice 2..."
curl -F file=@policies/demo_pol_2_perf_isolation_same_pol_ns-slice_2.yml http://localhost:5000/iml/update-policy

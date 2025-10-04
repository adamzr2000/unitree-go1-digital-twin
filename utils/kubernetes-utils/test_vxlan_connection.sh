#!/usr/bin/env bash
# Ping all VXLAN peers (including self) via a fixed interface.
# No params; extend the NODES list as you add hosts.

set -u -o pipefail

IFACE="vxlan200"
NODES=(
  "172.20.50.1:xtreme"
  "172.20.50.2:robot"
  # "172.20.50.3:edge2"
  # "172.20.50.4:sensor1"
)

# Optional: show local IP for context
LOCAL_IP=$(ip -o -4 addr show dev "$IFACE" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -n1)
echo "Interface: $IFACE  Local IP: ${LOCAL_IP:-unknown}"
echo "Pinging all configured nodes (including self if listed):"
echo

fail=0
for entry in "${NODES[@]}"; do
  ip="${entry%%:*}"
  name="${entry#*:}"

  printf " - %-12s (%s): " "$name" "$ip"
  if ping -I "$IFACE" -c 3 -W 1 -q "$ip" >/dev/null; then
    echo "OK"
  else
    echo "FAILED"
    fail=1
  fi
done

echo
if [[ $fail -eq 0 ]]; then
  echo "All pings OK via $IFACE."
else
  echo "Some pings FAILED. Check VXLAN, routes, or firewall."
fi

exit $fail

#!/usr/bin/env bash
set -euo pipefail

# === Config (edit here as you grow) ===
IFACE="vxlan200"
# List of "IP:NAME" entries (add more as needed)
NODES=(
  "172.20.50.1:xtreme"
  "172.20.50.2:robot"
  # "172.20.50.3:edge2"
  # "172.20.50.4:sensor1"
)

# === Detect local IP on the VXLAN interface ===
LOCAL_IP=$(ip -o -4 addr show dev "$IFACE" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -n1)
if [[ -z "${LOCAL_IP:-}" ]]; then
  echo "ERROR: Interface $IFACE has no IPv4 address. Is VXLAN up?"
  exit 2
fi

echo "Local iface: $IFACE, IP: $LOCAL_IP"
echo "Pinging VXLAN peers..."

fail=0
for entry in "${NODES[@]}"; do
  ip="${entry%%:*}"
  name="${entry#*:}"

  if [[ "$ip" == "$LOCAL_IP" ]]; then
    echo " - Skipping self: $name ($ip)"
    continue
  fi

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
  echo "All peers reachable via $IFACE."
else
  echo "Some peers FAILED. Check VXLAN, routes, or firewall."
fi

exit $fail

#!/bin/bash
# add-vxlan-peers.sh
# Adds remote peers to an existing vxlan interface's FDB.
# Usage:
#   sudo ./add-vxlan-peers.sh -d vxlan200 -r 10.0.0.2,10.0.0.3
#
# Notes:
# - Idempotent: skips peers already present.
# - Works with Linux 'bridge' tool (iproute2).

set -euo pipefail

usage() {
  echo "Usage: $0 -d <vxlan_iface> -r <remote_ip1,remote_ip2,...>"
  echo "  -d <vxlan_iface>   Existing VXLAN device (e.g., vxlan200)"
  echo "  -r <remote_ips>    Comma-separated list of remote underlay IPs"
  exit 1
}

vxlan_dev=""
remote_ips=""

while getopts "d:r:" opt; do
  case "$opt" in
    d) vxlan_dev="$OPTARG" ;;
    r) remote_ips="$OPTARG" ;;
    *) usage ;;
  esac
done

[ -n "$vxlan_dev" ] && [ -n "$remote_ips" ] || usage

# Ensure interface exists
if ! ip link show "$vxlan_dev" &>/dev/null; then
  echo "[!] Interface '$vxlan_dev' not found. Aborting."
  exit 1
fi

echo "[*] Adding peers to $vxlan_dev"
IFS=',' read -ra peers <<< "$remote_ips"

# Helper: check if an FDB entry for this 'dst' already exists
has_fdb_entry() {
  local dev="$1" dst="$2"
  # Grep for 'dst <ip>' on this dev
  bridge fdb show dev "$dev" 2>/dev/null | grep -qE " dst[[:space:]]+$dst(\b|$)"
}

added=0 skipped=0
for p in "${peers[@]}"; do
  peer="$(echo "$p" | xargs)"  # trim spaces
  [ -n "$peer" ] || continue

  if has_fdb_entry "$vxlan_dev" "$peer"; then
    echo "  ↷ $peer (already present, skipping)"
    ((skipped++)) || true
    continue
  fi

  # Add a flood entry to the multicast/broadcast MAC for this remote dst
  # This matches the pattern used in your setup script.
  if bridge fdb append to 00:00:00:00:00:00 dev "$vxlan_dev" dst "$peer" 2>/dev/null; then
    echo "  → $peer (added)"
    ((added++)) || true
  else
    echo "  ! $peer (failed to add)" >&2
  fi
done

echo "[✔] Done. Added: $added, Skipped: $skipped"
echo
echo "[i] Current FDB entries on $vxlan_dev:"
bridge fdb show dev "$vxlan_dev"

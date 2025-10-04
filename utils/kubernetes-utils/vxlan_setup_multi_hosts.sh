#!/bin/bash
set -euo pipefail

usage() {
    echo "Usage: $0 -r <remote_ip1,remote_ip2,...> -i <interface> -v <vxlan_id> -p <port> -a <vxlan_ip>"
    echo "  -r <remote_ips>     Comma-separated list of remote underlay IPs"
    echo "  -i <interface>      Physical NIC (e.g., ens3)"
    echo "  -v <vxlan_id>       VXLAN VNI (e.g., 200)"
    echo "  -p <port>           VXLAN UDP port (e.g., 4789)"
    echo "  -a <vxlan_ip>       IP address for VXLAN interface (must be within the specified range)"
    exit 1
}

# Derive a locally-administered unicast MAC using the (IPv4) ip's octets + random last byte
generate_mac_from_ip() {
    local ip=$1
    IFS='.' read -r -a octets <<< "$ip"
    printf "02:%02x:%02x:%02x:%02x:%02x\n" \
        "${octets[0]}" "${octets[1]}" "${octets[2]}" "${octets[3]}" "$(( RANDOM % 256 ))"
}

# Strip /prefix if present (e.g., 172.20.50.10/24 -> 172.20.50.10)
strip_cidr() {
    echo "${1%%/*}"
}

while getopts "r:i:v:p:a:" opt; do
    case ${opt} in
        r ) remote_ips=$OPTARG ;;
        i ) iface=$OPTARG ;;
        v ) vni=$OPTARG ;;
        p ) port=$OPTARG ;;
        a ) vxlan_ip=$OPTARG ;;
        * ) usage ;;
    esac
done

# Check if all required arguments are provided
if [ -z "${remote_ips:-}" ] || [ -z "${iface:-}" ] || [ -z "${vni:-}" ] || [ -z "${port:-}" ] || [ -z "${vxlan_ip:-}" ]; then
    usage
fi

vxlan_iface="vxlan${vni}"

# Use the VXLAN IP (without /prefix) to seed MAC generation
vxlan_ip_nopfx="$(strip_cidr "$vxlan_ip")"
mac="$(generate_mac_from_ip "$vxlan_ip_nopfx")"

echo -e "\nCreating VXLAN network interface '$vxlan_iface' with parameters:"
echo "  VXLAN ID: $vni"
echo "  Remote IP(s): $remote_ips"
echo "  Destination Port: $port"
echo "  Device Interface: $iface"
echo "  VXLAN IP: $vxlan_ip"
echo "  Derived MAC: $mac"

if ip link show "$vxlan_iface" &>/dev/null; then
    echo "[!] Interface $vxlan_iface already exists. Aborting to avoid conflict."
    exit 1
fi

# Create VXLAN interface
# ip link add "$vxlan_iface" type vxlan id "$vni" dstport "$port" dev "$iface" local "$local_ip"
ip link add "$vxlan_iface" type vxlan id "$vni" dstport "$port" dev "$iface"

# Add FDB entries
echo "[*] Adding FDB entries for remote peers"
IFS=',' read -ra remotes <<< "$remote_ips"
for remote in "${remotes[@]}"; do
    echo "  → $remote"
    bridge fdb append to 00:00:00:00:00:00 dev "$vxlan_iface" dst "$remote" 2>/dev/null || true
done

# Assign IP address
ip addr add "$vxlan_ip" dev "$vxlan_iface"

# Bring interface up
ip link set "$vxlan_iface" up
ip link set "$vxlan_iface" address "$mac"

echo "[✔] Multi-host VXLAN setup complete."

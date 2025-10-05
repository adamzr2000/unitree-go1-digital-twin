#!/bin/bash

filename=".env"

# Function to validate IPv4 address format
validate_ip() {
    local ip=$1
    if [[ ! $ip =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        echo "Error: Invalid IP address format: $ip"
        exit 1
    fi

    # Split IP into octets and check each is within 0-255 range
    IFS='.' read -r a b c d <<< "$ip"
    if ((a > 255 || b > 255 || c > 255 || d > 255)); then
        echo "Error: IP address out of range (0-255): $ip"
        exit 1
    fi
}

# Ensure --local-ip argument is provided
if [[ "$1" != "--local-ip" || -z "$2" ]]; then
    echo "Usage: $0 --local-ip <IP_ADDRESS>"
    exit 1
fi

ip=$2  # Assign provided IP address

# Validate IP format
validate_ip "$ip"

echo "Using Local IP: $ip"

# Write IP to .env file
echo "LOCAL_IP=${ip}" > "$filename"

# Start Docker Compose with the specified YAML file
docker compose -f d6gsite1.yml up -d

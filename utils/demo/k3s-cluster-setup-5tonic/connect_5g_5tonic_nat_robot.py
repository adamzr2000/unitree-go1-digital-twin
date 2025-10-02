import serial
import subprocess
import time
import argparse
import netifaces

# Configuration
SERIAL_PORT = '/dev/ttyUSB3'  # Replace with your port
BAUD_RATE = 9600             # Adjust based on your device
DELAY = 2                    # Delay (in seconds) between commands
MAX_RETRIES = 5              # Maximum number of retries for qmicli
SUDO_PASSWORD = "netcom;"  # Replace with your actual sudo password

def get_wwan0_ip():
    """
    Retrieves and prints the IP address assigned to wwan0.
    """
    try:
        if "wwan0" in netifaces.interfaces():
            addresses = netifaces.ifaddresses("wwan0")
            if netifaces.AF_INET in addresses:
                ip_address = addresses[netifaces.AF_INET][0]["addr"]
                print(f"wwan0 IP Address: {ip_address}")
                return ip_address
    except Exception as e:
        print(f"Error retrieving wwan0 IP: {e}")
    return None

def configure_raw_ip():
    """
    Configures the network interface wwan0 for raw IP mode.
    """
    try:
        command = ["sudo", "tee", "/sys/class/net/wwan0/qmi/raw_ip"]
        process = subprocess.run(command, input="Y\n", text=True, capture_output=True)
        if process.returncode == 0:
            print("Raw IP mode configured for wwan0.")
        else:
            print(f"Error configuring raw IP mode: {process.stderr}")
    except Exception as e:
        print(f"Exception occurred: {e}")
    time.sleep(DELAY)

def execute_command(command, retries=1):
    """
    Executes a shell command with sudo and returns the result.
    Retries the command up to `retries` times if it fails.
    """
    for attempt in range(retries):
        try:
            # Adjust command handling to avoid quoting issues
            process = subprocess.run(
                ["sudo", "sh", "-c", command],
                text=True,
                capture_output=True
            )
            if process.returncode == 0:
                print(f"Command executed successfully: {command}")
                return True  # Command succeeded
            else:
                print(f"Attempt {attempt + 1}/{retries}: Error executing command: {command}\n{process.stderr}")
        except Exception as e:
            print(f"Exception occurred while executing command: {command}\n{e}")
        time.sleep(DELAY)  # Wait before retrying
    return False  # All retries failed

def send_at_command(command, serial_conn):
    """
    Sends an AT command and reads the response.
    """
    serial_conn.write((command + '\r\n').encode())
    time.sleep(0.5)
    response = serial_conn.read_all().decode(errors='ignore')
    return response

def setup_nat_iptables(robot_ip, ports):
    """
    Configures NAT and IPTABLES rules.
    """
    print(f"Configuring NAT for robot at {robot_ip} with port range {ports}...")
    iptables_commands = [
        "iptables -t nat -A POSTROUTING -o wwan0 -j MASQUERADE",
        "iptables -A FORWARD -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT",
        f"iptables -t nat -A PREROUTING -p tcp -i wwan0 --dport {ports} -j DNAT --to-destination {robot_ip}",
        f"iptables -t nat -A PREROUTING -p udp -i wwan0 --dport {ports} -j DNAT --to-destination {robot_ip}"
    ]
    
    for cmd in iptables_commands:
        execute_command(cmd)

def main():
    parser = argparse.ArgumentParser(description="Connect to 5G and configure NAT/IPTABLES.")
    parser.add_argument("--robot-ip", required=True, help="IP address of the robot.")
    parser.add_argument("--ports", default="1:65535", help="Port range to forward (default: all ports).")
    parser.add_argument("--apn", type=str, default="5tonic-flamingo", help="Access Point Name (APN)")
    args = parser.parse_args()
    apn = args.apn
    
    try:
        configure_raw_ip()
        
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
            
            at_commands = [
                f'AT+CGDCONT=1,"IP","{apn}"',
                'AT+CNBP=0x7FFFFFFFFFFFFFFF,0x00000000000000000000000000000000,0x000000000000003F,0x00000000003FFE63000601E2090808D7,0x00000000003FFE63000601E2090808D7',
                'AT+CNMP=71',
                'AT+COPS=2',
                'AT+COPS=1,2,"21405",13'
            ]
            
            for command in at_commands:
                print(f"Sending command: {command}")
                response = send_at_command(command, ser)
                print(f"Response: {response}")
        
        time.sleep(DELAY)
        
        qmicli_command = (
            f"qmicli -d /dev/cdc-wdm0 --device-open-net='net-raw-ip|net-no-qos-header' "
            f"--wds-start-network=\"apn='{apn}',ip-type=4\" --client-no-release-cid"
        )
        if not execute_command(qmicli_command, retries=MAX_RETRIES):
            print(f"Failed to execute qmicli command after {MAX_RETRIES} attempts.")
            return
        
        execute_command("udhcpc -q -f -n -i wwan0")

        # Check and print the assigned IP
        wwan0_ip = get_wwan0_ip()
        if not wwan0_ip:
            print("Warning: No IP assigned to wwan0.")
                
        setup_nat_iptables(args.robot_ip, args.ports)
        print("5G connection established and NAT/IPTABLES configured successfully.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()

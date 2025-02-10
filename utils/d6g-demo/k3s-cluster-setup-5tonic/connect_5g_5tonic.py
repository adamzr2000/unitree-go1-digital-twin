import serial
import subprocess
import time

# Configuration
SERIAL_PORT = '/dev/ttyUSB3'  # Replace with your port
BAUD_RATE = 9600             # Adjust based on your device
SUDO_PASSWORD = "netcom;"    # Sudo password for the system
DELAY = 2                    # Delay (in seconds) between commands
MAX_RETRIES = 5              # Maximum number of retries for qmicli

def configure_raw_ip():
    """
    Configures the network interface wwan0 for raw IP mode.
    Equivalent to: echo 'Y' | sudo tee /sys/class/net/wwan0/qmi/raw_ip
    """
    try:
        command = ["sudo", "tee", "/sys/class/net/wwan0/qmi/raw_ip"]
        process = subprocess.run(
            command,
            input="Y\n",
            text=True,
            capture_output=True
        )
        if process.returncode == 0:
            print("Raw IP mode configured for wwan0.")
        else:
            print(f"Error configuring raw IP mode: {process.stderr}")
    except Exception as e:
        print(f"Exception occurred: {e}")
    time.sleep(DELAY)  # Wait before proceeding

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
    serial_conn.write((command + '\r\n').encode())  # Send AT command
    time.sleep(0.5)  # Wait for the response
    response = serial_conn.read_all().decode(errors='ignore')  # Read all response
    return response

def main():
    try:
        # Configure raw IP mode
        configure_raw_ip()

        # Open the serial connection
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
            
            # List of AT commands to send
            at_commands = [
                'AT+CGDCONT=1,"IP",""',
                'AT+CNBP=0x7FFFFFFFFFFFFFFF,0x00000000000000000000000000000000,0x000000000000003F,0x00000000003FFE63000601E2090808D7,0x00000000003FFE63000601E2090808D7',
                'AT+CNMP=71',
                'AT+COPS=2',
                'AT+COPS=1,2,"21405",13'
            ]

            # Loop through and execute each AT command
            for command in at_commands:
                print(f"Sending command: {command}")
                response = send_at_command(command, ser)
                print(f"Response: {response}")

        # Wait after AT commands
        time.sleep(DELAY)

        # Retry qmicli command up to MAX_RETRIES times
        qmicli_command = (
            'qmicli -d /dev/cdc-wdm0 --device-open-net="net-raw-ip|net-no-qos-header" '
            '--wds-start-network="apn=\'Internet\',ip-type=4" --client-no-release-cid'
        )
        if not execute_command(qmicli_command, retries=MAX_RETRIES):
            print(f"Failed to execute qmicli command after {MAX_RETRIES} attempts.")
            return

        # Execute udhcpc command (no retries needed)
        execute_command("udhcpc -q -f -n -i wwan0")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()


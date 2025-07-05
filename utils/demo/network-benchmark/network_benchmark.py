import iperf3
import re
import csv
import subprocess
import os
import argparse
import numpy as np
import json

def parse_arguments():
    parser = argparse.ArgumentParser(description='📶 Network benchmark tool for RTT, jitter, and bandwidth.')
    parser.add_argument('--server-ip', default='127.0.0.1', help='IP address of the iperf3 server.')
    parser.add_argument('--csv-file', default='network_benchmark.csv', help='Output CSV filename.')
    parser.add_argument('--output-dir', default='./results', help='Output directory for CSV and log files.')
    parser.add_argument('--duration', type=int, default=10, help='Test duration for all tests (ping and iperf3) in seconds.')
    parser.add_argument('--no-ping', action='store_true', help='Skip ping test.')
    parser.add_argument('--no-udp', action='store_true', help='Skip jitter (UDP) test.')
    parser.add_argument('--no-downlink', action='store_true', help='Skip downlink bandwidth test.')
    parser.add_argument('--no-uplink', action='store_true', help='Skip uplink bandwidth test.')
    return parser.parse_args()


def execute_ping_command(server_ip, duration):
    command = f'ping -w {duration} {server_ip}'
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
    return result.stdout.decode('utf-8')


def extract_ping_values(output):
    rtt_avg = rtt_stddev = packet_loss = None
    match = re.search(r'rtt min/avg/max/mdev = [\d.]+/([\d.]+)/[\d.]+/([\d.]+) ms', output)
    if match:
        rtt_avg = float(match.group(1))
        rtt_stddev = float(match.group(2))
    loss_match = re.search(r'(\d+)% packet loss', output)
    if loss_match:
        packet_loss = float(loss_match.group(1))
    return rtt_avg, rtt_stddev, packet_loss


def perform_iperf3_test(server_ip, duration, protocol='tcp', reverse=False):
    client = iperf3.Client()
    client.server_hostname = server_ip
    client.duration = duration
    client.protocol = protocol
    client.reverse = reverse
    result = client.run()
    return result


def compute_jitter_stats(result):
    if not result or not hasattr(result, 'text') or not result.text:
        return None, None
    try:
        import json
        data = json.loads(result.text)

        # Try to collect per-interval jitter (preferred)
        intervals = data.get('intervals', [])
        jitters = [
            stream['jitter_ms']
            for interval in intervals
            for stream in interval.get('streams', [])
            if 'jitter_ms' in stream
        ]

        if jitters:
            return float(np.mean(jitters)), float(np.std(jitters))

        # Fallback: use end summary jitter
        jitter_fallback = data.get("end", {}).get("sum", {}).get("jitter_ms", None)
        return float(jitter_fallback), 0.0 if jitter_fallback is not None else (None, None)

    except Exception as e:
        print(f"⚠️ Failed to parse jitter: {e}")
        return None, None

def compute_bandwidth_stats(result, reverse=False):
    if not result or not hasattr(result, 'text') or not result.text:
        return None, None
    try:
        import json
        data = json.loads(result.text)
        intervals = data.get('intervals', [])
        bws = [
            interval['sum']['bits_per_second'] / 1e6
            for interval in intervals
            if 'sum' in interval and 'bits_per_second' in interval['sum']
        ]
        if not bws:
            return None, None
        return float(np.mean(bws)), float(np.std(bws))
    except Exception as e:
        print(f"⚠️ Failed to parse bandwidth intervals: {e}")
        return None, None


def export_to_csv(folder_name, filename, data_dict):
    os.makedirs(folder_name, exist_ok=True)
    file_path = os.path.join(folder_name, filename)
    with open(file_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=data_dict.keys())
        writer.writeheader()
        writer.writerow(data_dict)
    print(f"✅ Clean results saved to {file_path}")


def write_log(folder_name, filename, log_content):
    os.makedirs(folder_name, exist_ok=True)
    log_path = os.path.join(folder_name, filename)
    with open(log_path, 'w') as f:
        f.write(log_content)
    print(f"📝 Raw log saved to {log_path}")


def main():
    args = parse_arguments()
    results = {}
    raw_logs = ""

    if not args.no_ping:
        print("📡 Running Ping Test...")
        ping_output = execute_ping_command(args.server_ip, args.duration)
        raw_logs += f"\n### PING OUTPUT ###\n{ping_output}\n"
        rtt_avg, rtt_stddev, packet_loss = extract_ping_values(ping_output)
        results['rtt_latency_ms'] = rtt_avg if rtt_avg is not None else 'N/A'
        results['rtt_latency_stddev_ms'] = rtt_stddev if rtt_stddev is not None else 'N/A'
        results['packet_loss_percent'] = packet_loss if packet_loss is not None else 'N/A'

    if not args.no_udp:
        print("📊 Measuring Jitter (UDP)...")
        udp_result = perform_iperf3_test(args.server_ip, args.duration, protocol='udp')
        raw_logs += f"\n### IPERF3 UDP OUTPUT ###\n{udp_result.text if udp_result and udp_result.text else 'No output'}\n"
        jitter_avg, jitter_stddev = compute_jitter_stats(udp_result)
        results['jitter_ms'] = jitter_avg if jitter_avg is not None else 'N/A'
        results['jitter_stddev_ms'] = jitter_stddev if jitter_stddev is not None else 'N/A'

    if not args.no_downlink:
        print("⬇️ Measuring Downlink Bandwidth (TCP)...")
        downlink_result = perform_iperf3_test(args.server_ip, args.duration)
        raw_logs += f"\n### IPERF3 DOWNLINK OUTPUT ###\n{downlink_result.text if downlink_result and downlink_result.text else 'No output'}\n"
        downlink_avg, downlink_stddev = compute_bandwidth_stats(downlink_result)
        results['downlink_bandwidth_mbps'] = downlink_avg if downlink_avg is not None else 'N/A'
        results['downlink_bandwidth_stddev_mbps'] = downlink_stddev if downlink_stddev is not None else 'N/A'

    if not args.no_uplink:
        print("⬆️ Measuring Uplink Bandwidth (TCP, reverse)...")
        uplink_result = perform_iperf3_test(args.server_ip, args.duration, reverse=True)
        raw_logs += f"\n### IPERF3 UPLINK OUTPUT ###\n{uplink_result.text if uplink_result and uplink_result.text else 'No output'}\n"
        uplink_avg, uplink_stddev = compute_bandwidth_stats(uplink_result, reverse=True)
        results['uplink_bandwidth_mbps'] = uplink_avg if uplink_avg is not None else 'N/A'
        results['uplink_bandwidth_stddev_mbps'] = uplink_stddev if uplink_stddev is not None else 'N/A'

    export_to_csv(args.output_dir, args.csv_file, results)

    log_filename = os.path.splitext(args.csv_file)[0] + '.log'
    write_log(args.output_dir, log_filename, raw_logs)


if __name__ == '__main__':
    main()


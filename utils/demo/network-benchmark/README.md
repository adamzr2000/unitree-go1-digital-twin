# Network Benchmark Tool

This repository contains the [network_benchmark.py](./network_benchmark.py) Python script for measuring various network metrics like latency RTT (Round-Trip Time), paquet loss, jitter, and bandwidth (both downlink and uplink) using [iperf3](https://iperf.fr/iperf-download.php) and [ping](https://www.paessler.com/it-explained/ping) tools. The results are exported to a CSV file within the [results](./results) directory.

## How to Use

```bash
python3 network_benchmark.py --server-ip 10.11.15.49 --duration 30 --csv-file test_run_1.csv
```

If no arguments are provided, the script defaults to `127.0.0.1` for the server IP and `network_benchmark.csv` for the CSV file name.

## Simulating Network Conditions

The tc qdisc command can be used to simulate various network conditions. Here are some examples for common scenarios:

### Adding Latency

To simulate additional latency (e.g., 10ms):
```bash
sudo tc qdisc add dev <interface> root netem delay 10ms
```

### Simulating Packet Loss 

To introduce packet loss (i.e., 10%):
```bash
sudo tc qdisc add dev <interface> root netem loss 10%
```

### Adding Jitter

To simulate jitter: 
```bash
sudo tc qdisc add dev <interface> root netem delay 10ms 5ms 25%
```

In this case, each packet experiences a base delay of 10 milliseconds, with additional variability (jitter) of up to 5 milliseconds, distributed around these values with a 25% probability following a normal distribution.

Replace <interface> with the name of your network interface (e.g., eth0 or wlan0). You can find your interface name by running ip addr.


## Removing Simulated Conditions

To remove any network condition you have applied:
```bash
sudo tc qdisc del dev <interface> root netem
```

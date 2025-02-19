#!/usr/bin/env python3
import socket
import json
import time
import random
import datetime

SERVER_HOST = '127.0.0.1'
SERVER_PORT = 9999

def get_current_timestamp():
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def run_simulation():
    """
    Simulate a complete simulation run.
    In your actual implementation, this function would integrate MININET-WiFi and SUMO,
    and collect real simulation data.
    """
    num_vehicles = 10
    simulation_run_duration = 10  # seconds; adjust as needed
    simulation_data = []

    # Simulate data collection every second
    for t in range(simulation_run_duration):
        current_time = get_current_timestamp()
        for i in range(1, num_vehicles + 1):
            record = {
                "timestamp": current_time,
                "car_id": f"car{i}",
                "position": [round(random.uniform(0, 1000), 2), round(random.uniform(0, 1000), 2)],
                "speed": round(random.uniform(0, 40), 2),
                "power_transmission": round(random.uniform(10, 30), 2),
                "data_rate": round(random.uniform(6, 54), 2),  # in Mbps
                "modulation": random.choice(["BPSK", "QPSK", "16-QAM", "64-QAM"]),
                "beacon_rate": random.choice([5, 10, 15]),
                "channel_busy_ratio": round(random.uniform(0, 1), 2)
            }
            simulation_data.append(record)
        time.sleep(1)  # simulate passage of simulation time

    return simulation_data

def send_data_to_server(data):
    """
    Connect to the server and send the complete simulation dataset as JSON.
    """
    json_data = json.dumps(data)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((SERVER_HOST, SERVER_PORT))
        s.sendall(json_data.encode("utf-8"))
    print("Data sent to server.")

if __name__ == "__main__":
    print("Starting simulation...")
    simulation_data = run_simulation()
    print(f"Simulation complete. Collected {len(simulation_data)} records.")
    print("Connecting to server and sending data...")
    send_data_to_server(simulation_data)

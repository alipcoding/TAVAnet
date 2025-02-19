#!/usr/bin/env python3
import socket
import json
import csv
import logging

logging.basicConfig(level=logging.INFO)

HOST = '127.0.0.1'
PORT = 9999

def process_data(data_list):
    """
    Process the received simulation data using a Q-learning algorithm.
    Here we simulate optimization by computing new (optimized) values for power transmission
    and data rate.
    In your final version, replace this with your actual Q-learning process.
    """
    processed = []
    for record in data_list:
        try:
            original_power = float(record.get("power_transmission", 15))
            original_data_rate = float(record.get("data_rate", 18))
        except Exception:
            original_power = 15
            original_data_rate = 18

        # Placeholder optimization: reduce power by 10% and increase data rate by 10%
        record["optimized_power"] = round(original_power * 0.9, 2)
        record["optimized_data_rate"] = round(original_data_rate * 1.1, 2)
        processed.append(record)
    return processed

def write_csv(data_list, filename="network.csv"):
    fieldnames = ["timestamp", "car_id", "position", "speed",
                  "power_transmission", "optimized_power",
                  "data_rate", "optimized_data_rate",
                  "modulation", "beacon_rate", "channel_busy_ratio"]
    with open(filename, "w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for record in data_list:
            writer.writerow(record)
    logging.info(f"CSV file '{filename}' generated successfully.")

def handle_client(conn, addr):
    logging.info(f"Connected by {addr}")
    data_received = ""
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            data_received += data.decode("utf-8")
        # Expecting a JSON array from the client
        simulation_data = json.loads(data_received)
        logging.info(f"Received {len(simulation_data)} records from client.")
        return simulation_data
    except Exception as e:
        logging.error(f"Error receiving data: {e}")
        return []
    finally:
        conn.close()

def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        logging.info(f"Server listening on {HOST}:{PORT}")
        conn, addr = s.accept()  # Accept a single client connection
        return handle_client(conn, addr)

if __name__ == "__main__":
    received_data = start_server()
    processed_data = process_data(received_data)
    write_csv(processed_data, "network.csv")

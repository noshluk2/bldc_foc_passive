# Simple UDP listener: binds to all interfaces on PORT and prints incoming packets.
# Used to verify ESP32 UDP senders and check firewall/port reachability.
# Run with python3 and leave it running while the device transmits.
import socket

HOST = "0.0.0.0"   # listen on all interfaces
PORT = 4444        # match DEST_PORT in the sketch

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))
print(f"Listening on {HOST}:{PORT} …")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        print(f"{addr[0]}:{addr[1]} -> {data.decode(errors='replace')}")
except KeyboardInterrupt:
    pass

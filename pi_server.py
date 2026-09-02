import socket
import serial
import time
import sys

# ─────────────────────────────── CONFIG ────────────────────────────────
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 115200
HOST        = '0.0.0.0'       # Listen on all network interfaces
PORT        = 5000
# ────────────────────────────────────────────────────────────────────────

def main():
    # 1. Initialize Serial Connection to Arduino
    print(f"[INIT] Opening serial port {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
        time.sleep(2.0)  # Allow Arduino time to reset after serial connection initialization
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("[INIT] Serial port connected.")
    except Exception as e:
        print(f"[ERROR] Failed to connect to Arduino serial: {e}")
        sys.exit(1)

    # 2. Initialize TCP Socket Server
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server_sock.bind((HOST, PORT))
        server_sock.listen(1)
        print(f"[NET] Server listening on port {PORT}...")
    except Exception as e:
        print(f"[ERROR] Socket bind failed: {e}")
        sys.exit(1)

    # 3. Client Connection Loop
    while True:
        print("\n[NET] Waiting for incoming connection from Pronterface...")
        conn, addr = server_sock.accept()
        print(f"[NET] Connected by {addr}")
        
        # Enable TCP_NODELAY to disable Nagle's algorithm for low latency
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        try:
            # Wrap socket file descriptor for clean line reading
            sock_file = conn.makefile('r', encoding='utf-8')
            
            while True:
                line = sock_file.readline()
                if not line:
                    print("[NET] Client disconnected.")
                    break
                
                cmd = line.strip()
                if not cmd:
                    continue

                # Forward command directly to Arduino via Serial
                ser.write((cmd + '\n').encode('utf-8'))
                ser.flush()  # Force immediate transmission to Arduino hardware

                # Rapid Response Loop for Arduino Serial Replies
                start_time = time.time()
                while True:
                    if ser.in_waiting > 0:
                        response_line = ser.readline().decode('utf-8', errors='replace')
                        if response_line:
                            # Echo response line immediately over TCP socket
                            conn.sendall(response_line.encode('utf-8'))
                            
                            # Break command wait state on completion signals
                            if response_line.startswith("ACK:") or response_line.strip().endswith("DONE"):
                                break

                    # Timeout safety for commands (2 seconds for polling, longer for movements)
                    timeout_val = 2.0 if cmd == 'S' else 120.0
                    if time.time() - start_time > timeout_val:
                        conn.sendall(b"LOG:ERROR Timeout waiting for Arduino serial response\nACK:DONE\n")
                        break

        except (ConnectionResetError, BrokenPipeError):
            print("[NET] Connection lost.")
        except Exception as e:
            print(f"[ERROR] Exception during client handling: {e}")
        finally:
            conn.close()
            print("[NET] Socket closed.")

if __name__ == "__main__":
    main()

import socket
import struct
import time
import collections
import logging
import threading

LOG_FILE = "coex_log.txt"

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# Console handler
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
console_fmt = logging.Formatter("%(message)s")
console_handler.setFormatter(console_fmt)

# File handler
file_handler = logging.FileHandler(LOG_FILE, mode="a", encoding="utf-8")
file_handler.setLevel(logging.INFO)
file_fmt = logging.Formatter("%(asctime)s | %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
file_handler.setFormatter(file_fmt)

logger.addHandler(console_handler)
logger.addHandler(file_handler)

# Thread-safe logging
log_lock = threading.Lock()

def safe_log(msg):
    with log_lock:
        logger.info(msg)

HOST = "0.0.0.0"
PORT = 5000
SERIAL_PORT = "COM10"  # ESP32-H2 USB serial
SERIAL_BAUD = 115200

# Match the C struct (packed):
PACKET_FORMAT = "<HHQBbBbBH64sH"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

MODE_NAMES = {1: "WIFI", 2: "ZIGBEE", 3: "BLE"}

WINDOW_SIZE = 1000
REPORT_EVERY = 1000


def process_stream(source_name, read_func, stop_event):
    """Generic packet processor for both TCP and serial sources."""
    safe_log(f"\n{'='*80}")
    safe_log(f"[{source_name}] Stream started")
    safe_log(f"{'='*80}")
    safe_log(
        f"{'Mode':<9} | {'Seq':>6} | {'RSSI':>5} | {'TxPwr':>5} | {'CH':>3} | "
        f"{'Latency':>10} | {'Jitter':>8} | {'Thruput':>10} | {'PER':>7}"
    )
    safe_log(f"{'-'*85}")

    expected_seq = None
    lost_packets = 0
    total_packets = 0
    detected_mode = source_name

    arrival_times = collections.deque(maxlen=WINDOW_SIZE + 1)
    inter_arrival_deltas = collections.deque(maxlen=WINDOW_SIZE)
    throughput_window = collections.deque(maxlen=WINDOW_SIZE)

    buf = b""
    while not stop_event.is_set():
        chunk = read_func()
        if not chunk:
            break
        buf += chunk

        while len(buf) >= PACKET_SIZE:
            data = buf[:PACKET_SIZE]
            buf = buf[PACKET_SIZE:]

            fields = struct.unpack(PACKET_FORMAT, data)
            header, seq, ts, mode, rssi, lqi, tx_power, channel, p_len, payload, crc = fields

            if header != 0xAA55:
                # Try to re-sync: find next 0xAA55 marker
                marker = buf.find(b'\x55\xAA')
                if marker >= 0:
                    buf = buf[marker:]
                continue

            mode_name = MODE_NAMES.get(mode, f"?{mode}")
            if total_packets == 0:
                detected_mode = mode_name

            now = time.time()
            total_packets += 1

            # PER
            if expected_seq is not None and seq > expected_seq:
                lost = seq - expected_seq
                lost_packets += lost
            expected_seq = seq + 1

            per = (lost_packets / (total_packets + lost_packets)) * 100

            # Jitter
            arrival_times.append(now)
            if len(arrival_times) >= 2:
                delta_ms = (arrival_times[-1] - arrival_times[-2]) * 1000.0
                inter_arrival_deltas.append(delta_ms)

            avg_latency_ms = 0.0
            jitter_ms = 0.0
            if len(inter_arrival_deltas) > 1:
                avg_latency_ms = sum(inter_arrival_deltas) / len(inter_arrival_deltas)
                mean = avg_latency_ms
                variance = sum((d - mean) ** 2 for d in inter_arrival_deltas) / len(inter_arrival_deltas)
                jitter_ms = variance ** 0.5

            # Throughput
            throughput_window.append((now, len(data)))
            throughput_kbps = 0.0
            if len(throughput_window) >= 2:
                time_span = throughput_window[-1][0] - throughput_window[0][0]
                if time_span > 0:
                    total_bytes = sum(b for _, b in throughput_window)
                    throughput_kbps = (total_bytes / 1024.0) / time_span

            # Report
            if total_packets % REPORT_EVERY == 0:
                safe_log(
                    f"[{mode_name:<6}] | {seq:>6} | {rssi:>5} | {tx_power:>4}dB | {channel:>3} | "
                    f"{avg_latency_ms:>8.1f}ms | {jitter_ms:>6.2f}ms | "
                    f"{throughput_kbps:>7.2f}KB/s | {per:>5.2f}%"
                )

    safe_log(f"\n{'='*80}")
    safe_log(f"[{detected_mode}] Stream closed.")
    safe_log(f"  Total packets: {total_packets}")
    safe_log(f"  Lost packets:  {lost_packets}")
    safe_log(f"  Final PER:     {(lost_packets / max(1, total_packets + lost_packets)) * 100:.2f}%")
    safe_log(f"{'='*80}")


def handle_tcp_client(conn, addr, stop_event):
    """Handle a single TCP client connection."""
    conn.settimeout(30.0)

    def tcp_read():
        try:
            return conn.recv(4096)
        except socket.timeout:
            safe_log(f"TCP connection from {addr} timed out")
            return b""
        except OSError:
            return b""

    try:
        process_stream(f"TCP:{addr[0]}:{addr[1]}", tcp_read, stop_event)
    finally:
        conn.close()


def serial_reader(stop_event):
    """Read coex_packet_t from ESP32-H2 via USB serial (COM10)."""
    try:
        import serial
    except ImportError:
        safe_log("[SERIAL] pyserial not installed. Run: pip install pyserial")
        safe_log("[SERIAL] Skipping serial reader for ESP32-H2")
        return

    safe_log(f"[SERIAL] Opening {SERIAL_PORT} at {SERIAL_BAUD} baud...")

    while not stop_event.is_set():
        try:
            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=5)
            safe_log(f"[SERIAL] Connected to {SERIAL_PORT}")

            def serial_read():
                try:
                    return ser.read(ser.in_waiting or 1)
                except Exception:
                    return b""

            process_stream("SERIAL:H2", serial_read, stop_event)
            ser.close()
        except serial.SerialException as e:
            safe_log(f"[SERIAL] {SERIAL_PORT} not available ({e}), retrying in 5s...")
            time.sleep(5)


def main():
    stop_event = threading.Event()

    safe_log(f"Coexistence Monitor — TCP on {HOST}:{PORT}, Serial on {SERIAL_PORT}")
    safe_log(f"Packet size: {PACKET_SIZE} bytes")
    safe_log(f"Modes: 1=WIFI, 2=ZIGBEE, 3=BLE")
    safe_log(f"{'-'*85}")

    # Start serial reader thread for H2
    serial_thread = threading.Thread(target=serial_reader, args=(stop_event,), daemon=True)
    serial_thread.start()

    # TCP server for S3 (Wi-Fi) and ESP32 (BLE)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(5)  # Allow multiple connections
        safe_log(f"TCP server listening on {HOST}:{PORT}...")

        try:
            while True:
                conn, addr = s.accept()
                safe_log(f"\nNew TCP connection from {addr}")
                t = threading.Thread(
                    target=handle_tcp_client,
                    args=(conn, addr, stop_event),
                    daemon=True
                )
                t.start()
        except KeyboardInterrupt:
            safe_log("\nShutting down...")
            stop_event.set()


if __name__ == "__main__":
    main()

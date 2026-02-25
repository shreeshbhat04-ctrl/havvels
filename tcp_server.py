import socket
import struct
import time
import collections
import logging

LOG_FILE = "coex_log.txt"

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# Console handler — prints to terminal
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
console_fmt = logging.Formatter("%(message)s")
console_handler.setFormatter(console_fmt)

# File handler — appends to log file with timestamps
file_handler = logging.FileHandler(LOG_FILE, mode="a", encoding="utf-8")
file_handler.setLevel(logging.INFO)
file_fmt = logging.Formatter("%(asctime)s | %(message)s", datefmt="%Y-%m-%d %H:%M:%S")
file_handler.setFormatter(file_fmt)

logger.addHandler(console_handler)
logger.addHandler(file_handler)

HOST = "0.0.0.0"
PORT = 5000

# Match the C struct (packed):
#   header(H), seq(H), ts(Q), mode(B), rssi(b), lqi(B),
#   tx_power(b), channel(B),  <-- NEW FIELDS
#   payload_len(H), payload(64s), crc(H)
PACKET_FORMAT = "<HHQBbBbBH64sH"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

# Rolling window size for jitter and throughput calculation
WINDOW_SIZE = 50
REPORT_EVERY = 10  # Print stats every N packets


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        logger.info(f"Listening on {HOST}:{PORT} ... (packet size: {PACKET_SIZE} bytes)")

        while True:
            conn, addr = s.accept()
            with conn:
                logger.info(f"\n{'='*80}")
                logger.info(f"Connected by {addr}")
                logger.info(f"{'='*80}")
                logger.info(
                    f"{'Seq':>6} | {'RSSI':>5} | {'TxPwr':>5} | {'CH':>3} | "
                    f"{'Latency':>10} | {'Jitter':>8} | {'Thruput':>10} | {'PER':>7}"
                )
                logger.info(f"{'-'*80}")

                expected_seq = None
                lost_packets = 0
                total_packets = 0

                # Jitter tracking: rolling window of inter-arrival times
                arrival_times = collections.deque(maxlen=WINDOW_SIZE + 1)
                inter_arrival_deltas = collections.deque(maxlen=WINDOW_SIZE)

                # Throughput tracking
                throughput_window = collections.deque(maxlen=WINDOW_SIZE)  # (timestamp, bytes)

                while True:
                    data = conn.recv(PACKET_SIZE)
                    if not data:
                        break

                    if len(data) != PACKET_SIZE:
                        continue

                    fields = struct.unpack(PACKET_FORMAT, data)
                    header, seq, ts, mode, rssi, lqi, tx_power, channel, p_len, payload, crc = fields

                    if header != 0xAA55:
                        continue

                    now = time.time()
                    total_packets += 1

                    # --- PER (sequence gap) ---
                    if expected_seq is not None and seq > expected_seq:
                        lost = seq - expected_seq
                        lost_packets += lost
                    expected_seq = seq + 1

                    per = (lost_packets / (total_packets + lost_packets)) * 100

                    # --- Inter-arrival jitter ---
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
                        jitter_ms = variance ** 0.5  # standard deviation

                    # --- Throughput ---
                    throughput_window.append((now, len(data)))
                    throughput_kbps = 0.0
                    if len(throughput_window) >= 2:
                        time_span = throughput_window[-1][0] - throughput_window[0][0]
                        if time_span > 0:
                            total_bytes = sum(b for _, b in throughput_window)
                            throughput_kbps = (total_bytes / 1024.0) / time_span

                    # --- Print every N packets ---
                    if total_packets % REPORT_EVERY == 0:
                        logger.info(
                            f"{seq:>6} | {rssi:>5} | {tx_power:>4}dB | {channel:>3} | "
                            f"{avg_latency_ms:>8.1f}ms | {jitter_ms:>6.2f}ms | "
                            f"{throughput_kbps:>7.2f}KB/s | {per:>5.2f}%"
                        )

                logger.info(f"\n{'='*80}")
                logger.info(f"Connection closed.")
                logger.info(f"  Total packets: {total_packets}")
                logger.info(f"  Lost packets:  {lost_packets}")
                logger.info(f"  Final PER:     {(lost_packets / max(1, total_packets + lost_packets)) * 100:.2f}%")
                logger.info(f"{'='*80}")


if __name__ == "__main__":
    main()

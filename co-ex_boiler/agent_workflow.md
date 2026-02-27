# Co-Ex Boiler - Agent Workflow & Knowledge Base

## PROJECT OVERVIEW

Multi-node IoT coexistence test system measuring WiFi, BLE, and Zigbee concurrent operation on ESP32 variants. Hub-and-spoke architecture with 1000-packet batch testing.

## NODE ARCHITECTURE

### Node A - ESP32-S3 (Master Hub)
- **File:** `NodeA_s3/main/main.c`
- **WiFi AP:** SSID=`ESP32_TEST_AP`, pass=`12345678`, channel 6
- **BLE Central:** Scans/connects to Node B (`NODE-B(classic)`) and Node C (`NODE-C(H2)`)
- **TCP Server:** Port 5001, receives `coex_packet_t` (86 bytes) from Node B
- **UART Bridge RX:** UART1, TX=GPIO17, RX=GPIO18, 115200 baud - receives `zb_relay_pkt_t` from Node D
- **Dashboard:** 5-second interval, reports WiFi/BLE/Zigbee RX/TX counts
- **Components:** `wifi_ap/`, `ble_central/`, `tcp_server/`, `coex_packet/`

### Node B - ESP32 Classic (WiFi+BLE Spoke)
- **File:** `NodeB_classic/main/main.c`
- **WiFi STA:** Connects to Node A AP (192.168.4.1:5001)
- **BLE Peripheral:** Advertises as `NODE-B(classic)`, echoes writes as notifications
- **TCP Client:** Sends 1000 `coex_packet_t` at 20ms intervals
- **Components:** `wifi_sta/`, `ble_peripheral/`, `tcp_client/`, `coex_packet/`

### Node C - ESP32-H2 (Zigbee ZED + BLE)
- **File:** `NodeC_h2/main/main.c`
- **Zigbee ZED source:** `components/zigbee_zed/zigbee_zed.c`
- **BLE Peripheral:** Advertises as `NODE-C(H2)`, echoes writes as notifications
- **Zigbee End Device:** Joins Node D network (channel 14), sends attribute reports
- **ZB TX Task:** 10 packets/sec (100ms), 1000-packet batches, 5s pause between
- **Custom Cluster:** 0xFF00, Attribute: 0xFFF0, payload=[len=2][seq_lo][seq_hi]
- **Reports to:** Coordinator at 0x0000

### Node D - ESP32-H2 (Zigbee Coordinator + UART Bridge)
- **File:** `NodeD_h2/main/main.c`
- **Zigbee Coordinator:** Forms network on channel 14, max 10 children
- **UART Bridge TX:** UART1, TX=GPIO4, RX=GPIO5, 115200 baud
- **Action handler:** On `ESP_ZB_CORE_REPORT_ATTR_CB_ID`, extracts seq from attr data, sends `zb_relay_pkt_t` via UART
- **Heartbeat:** Every 5s, seq=0xFFFF, addr=0xBEEF (Node A skips these)

## DATA FLOW DIAGRAM

```
Node B (ESP32)          Node A (ESP32-S3)           Node C (ESP32-H2)
  WiFi STA ------TCP:5001-----> WiFi AP
  BLE Periph <---BLE notify---> BLE Central <---BLE notify--- BLE Periph
                                   |
                                UART RX (GPIO18)
                                   ^
                                   |
                              UART TX (GPIO4)
                                   |
                              Node D (ESP32-H2)
                              ZB Coordinator
                                   ^
                                   | Zigbee reports (0xFF00)
                                   |
                              Node C (ESP32-H2)
                              ZB End Device
```

## KEY DATA STRUCTURES

### zb_relay_pkt_t (9 bytes, packed) - UART bridge packet
```c
uint16_t header;      // 0x55AA (wire: 0xAA 0x55 little-endian)
uint16_t seq;         // Zigbee sequence number
uint16_t short_addr;  // Sender's ZB short address
int8_t rssi;          // Signal strength (currently always 0)
uint8_t lqi;          // Link quality (currently always 0)
uint8_t payload_len;  // Always 0
```

### coex_packet_t (86 bytes, packed) - WiFi test packet
```c
uint16_t header;       // 0xAA55
uint16_t seq;
uint64_t tx_timestamp; // microseconds
uint8_t mode;          // 1=WiFi, 2=Zigbee, 3=BLE
uint8_t direction;     // 0x01=Request, 0x02=Response
int8_t rssi; uint8_t lqi; int8_t tx_power; uint8_t channel;
uint16_t payload_len; uint8_t payload[64]; uint16_t crc;
```

### test_session_t - Session tracking for 1000-packet batches
- Tracks: rx_count, lost_count, jitter_sum_us, latency_sum_us, rssi_min/max/sum
- Auto-resets after target_count (1000) reached
- Used by: WiFi (tcp_server), BLE (ble_central per-node), Zigbee (main.c)

## UART PARSING (Node A - uart_rx_task)
- State machine: sync on 0xAA then 0x55, buffer 9 bytes
- Heartbeat detection: seq=0xFFFF && addr=0xBEEF -> skip (ESP_LOGD only)
- Data packets: increment g_zigbee_rx, track session metrics
- Heartbeat logged at DEBUG level (not visible in default INFO output)

## PHYSICAL WIRING
- Node D TX (GPIO4) --wire--> Node A RX (GPIO18)
- Node D RX (GPIO5) --wire--> Node A TX (GPIO17) (unused currently)

## BUILD TARGETS
- Node A: `idf.py -p COMx -D IDF_TARGET=esp32s3 build flash`
- Node B: `idf.py -p COMx -D IDF_TARGET=esp32 build flash`
- Node C: `idf.py -p COMx -D IDF_TARGET=esp32h2 build flash`
- Node D: `idf.py -p COMx -D IDF_TARGET=esp32h2 build flash` (needs `ZB_COORDINATOR_ROLE` in menuconfig)

## KNOWN ISSUES & DEBUGGING NOTES

### Issue: Node C Zigbee reports not reaching Node A (2026-02-27)

**Symptoms from log:**
- Node D received only 1 Zigbee report (seq=435) during entire session
- Node D heartbeats continue normally (#447-#478), proving UART TX works
- Node A dashboard shows `Zigbee: RX=0`
- BLE NodeC session: 1000/1000 (100% PDR) - BLE works fine
- WiFi session: 129/1000 (12.9% PDR) - WiFi severely degraded

**Root Cause Analysis:**

1. **PRIMARY: Node C Zigbee TX is starved by BLE coexistence**
   - ESP32-H2 shares the same 802.15.4 radio for BLE and Zigbee
   - BLE session completed perfectly (100% PDR, 1000/1000)
   - Meanwhile Zigbee managed only ~1 report in the same window
   - `zb_tx_task` calls `esp_zb_zcl_report_attr_cmd_req()` every 100ms
   - But when BLE is actively exchanging data, the radio is unavailable for Zigbee
   - The Zigbee stack silently drops/queues reports when radio is busy

2. **SECONDARY: UART relay status unknown for Node A**
   - Heartbeat RX logged at DEBUG level on Node A - not visible in log
   - Cannot confirm Node A is actually parsing UART packets from the provided log
   - The 1 packet Node D did relay may or may not have reached Node A

3. **SECONDARY: WiFi degradation (12.9% PDR)**
   - All protocols running simultaneously causes radio contention on Node A (S3)
   - WiFi AP + BLE Central sharing the 2.4GHz radio

**Potential Fixes:**
- Stagger BLE and Zigbee tests (don't run simultaneously on Node C)
- Increase Zigbee TX interval to reduce contention
- Use Zigbee reporting configure (automatic reporting) instead of manual reports
- Add RSSI/LQI extraction in Node D action handler (currently hardcoded to 0)
- Change Node A heartbeat log from DEBUG to INFO for diagnostics
- Consider separate test phases: BLE-only, Zigbee-only, then combined

## FILE QUICK REFERENCE

```
co-ex_boiler/
  NodeA_s3/
    main/main.c              -- Hub entry, UART RX, dashboard
    wifi_ap/wifi_ap.c/h      -- WiFi Access Point
    ble_central/ble_central.c/h  -- BLE scanner+connector, session tracking
    tcp_server/tcp_server.c/h    -- TCP listener :5001
    coex_packet/coex_packet.c/h  -- Shared packet format + CRC16
  NodeB_classic/
    main/main.c              -- Spoke entry
    wifi_sta/wifi_sta.c/h    -- WiFi Station
    ble_peripheral/ble_peripheral.c/h  -- BLE advertiser
    tcp_client/tcp_client.c/h    -- TCP sender
    coex_packet/coex_packet.c/h  -- Same packet format
  NodeC_h2/
    main/main.c              -- ZED+BLE entry
    components/
      ble_peripheral/ble_peripheral.c/h  -- BLE advertiser
      zigbee_zed/zigbee_zed.c/h  -- ZB End Device, TX task, signal handler
  NodeD_h2/
    main/main.c              -- Coordinator + UART bridge
```

## FUNCTION CROSS-REFERENCE

| Function | File | Purpose |
|----------|------|---------|
| `uart_rx_task()` | NodeA main.c:133 | Parse UART relay packets from Node D |
| `dashboard_task()` | NodeA main.c:91 | Print WiFi/BLE/ZB stats every 5s |
| `print_zb_session_report()` | NodeA main.c:66 | Format Zigbee session report |
| `zb_session_reset()` | NodeA main.c:58 | Clear session counters |
| `ble_data_task()` | ble_central.c:376 | Send BLE writes to NodeB/C |
| `gap_event_cb()` | ble_central.c:189 | Handle BLE events (scan/connect/notify) |
| `print_ble_session_report()` | ble_central.c:82 | Format BLE session report |
| `zb_tx_task()` | zigbee_zed.c:47 | Send ZB reports every 100ms |
| `zb_action_handler()` (NodeC) | zigbee_zed.c:96 | Receive coordinator commands |
| `esp_zb_app_signal_handler()` (NodeC) | zigbee_zed.c:136 | Handle ZB join/leave signals |
| `zb_action_handler()` (NodeD) | NodeD main.c:125 | Receive reports, UART relay |
| `app_task()` (NodeD) | NodeD main.c:78 | Send UART heartbeats |
| `zigbee_task()` (NodeD) | NodeD main.c:249 | Configure coordinator, start stack |
| `log_file_init()` | log_file.c | Initialize SPIFFS and open log file |
| `log_file_redirect_esp_log()` | log_file.c | Redirect ESP_LOG output to file |
| `log_file_dump()` | log_file.c | Print saved logs to serial monitor |

## SPIFFS LOGGING FEATURE

Each node now saves all ESP_LOG output to a SPIFFS partition on flash.

### Partition Configuration
- NodeA/NodeB: 256KB `storage` partition
- NodeC/NodeD: 64KB `storage` partition (H2 has limited flash)

### Log File Location
- Mount point: `/spiffs`
- Log file: `/spiffs/coex_log.txt`
- Auto-rotation when file exceeds max size

### Usage
Logging is automatically initialized in `app_main()`. All ESP_LOG output is:
1. Printed to serial (as normal)
2. Simultaneously written to SPIFFS

### Extracting Logs
Call `log_file_dump()` to print saved logs to serial monitor:
```c
// Add to app or call via console command
log_file_dump();
```

Or use `esptool.py read_flash` to extract the SPIFFS partition:
```bash
# Read SPIFFS partition to file
esptool.py --port COMx read_flash <offset> <size> spiffs.bin
# Use mkspiffs to extract files from spiffs.bin
```

### Log Rotation
Logs rotate automatically when exceeding:
- 256KB for NodeA/NodeB
- 64KB for NodeC/NodeD

### API Functions
| Function | Description |
|----------|-------------|
| `log_file_init(node_name)` | Initialize SPIFFS, open log file |
| `log_file_redirect_esp_log()` | Redirect ESP_LOG to also write to file |
| `log_file_write(fmt, ...)` | Write custom formatted log entry |
| `log_file_dump()` | Print log file contents to serial |
| `log_file_clear()` | Delete and start fresh log file |
| `log_file_close()` | Flush and close log file |
| `log_file_get_size()` | Get current log file size in bytes |


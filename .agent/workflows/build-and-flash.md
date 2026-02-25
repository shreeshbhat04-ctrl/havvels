---
description: Build and flash the Haavvvels coexistence project for ESP32-H2 and ESP32-S3 targets, then run the TCP server
---

## Build and Flash

// turbo-all

### ESP32-H2 (Zigbee + USB Serial)

1. Set target:
```
idf.py -B build_h2 set-target esp32h2
```

2. Build:
```
idf.py -B build_h2 build
```

3. Flash and monitor (COM10):
```
idf.py -B build_h2 -p COM10 flash monitor
```

### ESP32-S3 (Wi-Fi Client)

4. Set target:
```
idf.py -B build_s3 -D SDKCONFIG=sdkconfig.s3 set-target esp32s3
```

5. Build:
```
idf.py -B build_s3 -D SDKCONFIG=sdkconfig.s3 build
```

6. Flash and monitor (COM9):
```
idf.py -B build_s3 -D SDKCONFIG=sdkconfig.s3 -p COM9 flash monitor
```

### ESP32 Classic (Wi-Fi + BLE)

7. Set target:
```
idf.py -B build_esp32 set-target esp32
```

8. Build:
```
idf.py -B build_esp32 build
```

9. Flash and monitor (COM3):
```
idf.py -B build_esp32 -p COM3 flash monitor
```

### TCP Server (Laptop)

10. Install dependencies:
```
pip install pyserial
```

11. Run the Python server:
```
python tcp_server.py
```

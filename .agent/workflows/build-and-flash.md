---
description: Build and flash the Haavvvels coexistence project for ESP32-H2 and ESP32-S3 targets, then run the TCP server
---

## Build and Flash

### ESP32-H2 (Zigbee/802.15.4 Interference)

// turbo-all

1. Clean (if needed):
```
idf.py fullclean
```

2. Set target:
```
idf.py -B build_h2 set-target esp32h2
```

3. Build:
```
idf.py -B build_h2 build
```

4. Flash and monitor (COM10):
```
idf.py -B build_h2 -p COM10 flash monitor
```

### ESP32-S3 (Wi-Fi Client)

5. Set target with sdkconfig:
```
idf.py -B build_s3 -D SDKCONFIG=sdkconfig.s3 set-target esp32s3
```

6. Build:
```
idf.py -B build_s3 -D SDKCONFIG=sdkconfig.s3 build
```

7. Flash and monitor (COM9):
```
idf.py -B build_s3 -D SDKCONFIG=sdkconfig.s3 -p COM9 flash monitor
```

### TCP Server (Laptop)

8. Run the Python server:
```
python tcp_server.py
```

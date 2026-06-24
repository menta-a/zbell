
# zbell — Zigbee Smart Plug Monitor

Monitor a Zigbee smart plug (Heiman HS2SK-EF-EU) from the command line, using an EFR32MG21 dongle (JetHome JetStick Z4) with the EZSP v14 protocol.

## Current status

- ✅ Voltage, current, and active power read from ElectricalMeasurement cluster (0x0B04)
- ✅ OnOff state read from OnOff cluster (0x0006)
- ✅ Measurement coefficients auto-loaded from the device
- ✅ CSV logging (plug_data.csv)
- ✅ Device discovery and pairing
- ❌ Remote on/off — **not implemented**
- ❌ Sensor attribute reading (temperature, humidity, IAS Zone) — **not implemented**
- ❌ Multi-device support — pending

## Requirements

- Raspberry Pi 3B (or similar) with Raspberry Pi OS 64-bit
- EFR32MG21 dongle running EZSP v14 firmware (e.g. JetHome JetStick Z4)
- Zigbee-compatible smart plug (tested with Heiman HS2SK-EF-EU)

## Build

```bash
g++ -std=c++17 -O2 -o zbell main.cpp -lrt
```

## Usage

```bash
./zbell                       # normal mode (electrical data only)
./zbell --verbose             # debug mode (full EZSP traffic)
./zbell /dev/ttyUSB0          # custom serial port
```

The program opens the network for 180 seconds to pair devices.
Press the pairing button on the smart plug (~5s). After pairing,
electrical data is displayed every ~5 seconds and logged to `plug_data.csv`.

## How it works

`zbell` communicates directly with the Zigbee dongle via the ASH (serial) +
EZSP v14 protocol. It does not use Zigbee2MQTT, zigpy, or Home Assistant.
All ZCL (Zigbee Cluster Library) logic is implemented from scratch in C++.

## Code structure

- `main.cpp` — Complete implementation (~1200 lines)
- `plug_data.csv` — Auto-generated electrical data log

## License

MIT

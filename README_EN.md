# asd (INCOMPLETE)

EZSP-over-ASH Reader for NCP Zigbee Silicon Labs (EFR32MG21).

## Objective

Receive and decode Zigbee sensor messages from a USB dongle (JetHome JetStick Z4) using only C++, without databases.

## Requirements

- Raspberry Pi OS 64-bit
- JetHome JetStick Z4 dongle (EFR32MG21, EmberZNet NCP firmware)
- `g++` with C++17 support
- Port `/dev/ttyUSB0`

## Compilation

```bash
make
```

## Usage

```bash
./zigbee_reader [port] [pairing_seconds]
```

Example:
```bash
./zigbee_reader /dev/ttyUSB0 180
```

## Current Status

- ✅ ASH EZSP v13 communication established
- ✅ EZSP version negotiation
- ✅ Zigbee network formation (channel 26, PAN ID 0x2D3F)
- ✅ Receiving callbacks (`trustCenterJoin`, `incomingMessage`)
- ✅ IAS Zone configuration for PIR/contact/smoke sensors
- ✅ Decoding incoming messages (ZCL, APS)
- ✅ Pairing temperature/humidity sensors (Heiman HS1HT-N), other sensors, and smart plugs (I think)
- ⚠️ PIR pairing (Heiman HS1MS-E) in progress

## License

None, steal it if you want, this is communism

# zbell

Lector EZSP-over-ASH para NCP Zigbee Silicon Labs (EFR32MG21).

## Objetivo

Recibir y decodificar mensajes de sensores Zigbee desde un dongle USB (JetHome JetStick Z4) usando solo C++, sin zigpy/bellows, MQTT ni bases de datos.

## Requisitos

- Raspberry Pi 3B con Raspberry Pi OS 64-bit
- Dongle JetHome JetStick Z4 (EFR32MG21, firmware EmberZNet NCP)
- `g++` con soporte C++17
- Puerto `/dev/ttyUSB0`

## Compilación

```bash
make
```

## Uso

```bash
./zigbee_reader [puerto] [segundos_pairing]
```

Ejemplo:
```bash
./zigbee_reader /dev/ttyUSB0 180
```

## Estado actual

- ✅ Comunicación ASH EZSP v13 establecida
- ✅ Negociación de versión EZSP
- ✅ Formación de red Zigbee (canal 26, PAN ID 0x2D3F)
- ✅ Recepción de callbacks (`trustCenterJoin`, `incomingMessage`)
- ✅ Configuración IAS Zone para sensores PIR/contacto/smoke
- ✅ Decodificación de mensajes entrantes (ZCL, APS)
- ✅ Pairing de sensores de temperatura/humedad (Heiman HS1HT-N)
- ⚠️ Pairing de PIR (Heiman HS1MS-E) en progreso

## Licencia

Ninguna robalo si quieres, esto es comunismo
[Spanish](README.md)/[English](README_en.md)

# zbell — Zigbee Smart Plug Monitor

Monitoriza un smart plug Zigbee (Heiman HS2SK-EF-EU) desde la consola, usando un dongle EFR32MG21 (JetHome JetStick Z4) con el protocolo EZSP v14.

## Estado actual

- ✅ Lectura de voltaje, corriente y potencia desde el cluster ElectricalMeasurement (0x0B04)
- ✅ Lectura del estado OnOff (cluster 0x0006)
- ✅ Coeficientes de medición cargados automáticamente desde el dispositivo
- ✅ Logging a CSV (plug_data.csv)
- ✅ Descubrimiento y emparejamiento de dispositivos
- ❌ Encendido/apagado remoto — **no implementado**
- ❌ Lectura de atributos de sensores (temperatura, humedad, IAS Zone) — **no implementado**
- ❌ Soporte para múltiples dispositivos simultáneos — pendiente

## Requisitos

- Raspberry Pi 3B (o similar) con Raspberry Pi OS 64-bit
- Dongle EFR32MG21 con firmware EZSP v14 (Ej: JetHome JetStick Z4)
- Smart plug compatible con Zigbee (probado con Heiman HS2SK-EF-EU)

## Compilar

```bash
make
```
NOTA: `Makefile` esta hecho para el codigo con nombre `main.cpp`, por lo que se necesitará modificarlo para que pueda funcionar con `main_en.cpp`, o en todo caso reenombrar el codigo a `main.cpp`.

## Usar

```bash
./zbell                       # modo normal (solo datos eléctricos)
./zbell --verbose             # modo debug (todo el tráfico EZSP)
./zbell /dev/ttyUSB0          # puerto personalizado
```

El programa abre la red por 180 segundos para emparejar dispositivos.
Presiona el botón pairing del smart plug (~5s). Tras emparejarse,
los datos eléctricos se muestran cada ~5 segundos y se guardan en `plug_data.csv`.

## Cómo funciona

`zbell` implementa comunicación directa con el dongle Zigbee a través del
protocolo ASH (serial) + EZSP v14. No utiliza Zigbee2MQTT, zigpy ni Home Assistant.
Toda la lógica ZCL (Zigbee Cluster Library) está implementada desde cero en C++.

## Estructura del código

- `make` — Gestion de dependencias para la compilación
- `main.cpp` — Implementación completa (~1200 líneas)
- `plug_data.csv` — Datos eléctricos generados automáticamente

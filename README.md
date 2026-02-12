# sintetizador-Corporal
Este proyecto contiene el código y modelos para el proyecto de sintetizador corporal.

## Objetivos
El Sintetizador corporal es un proyecto que consiste en un modulo de sensores conectado via WiFi a una PC que convierte las señales recibidas en notas MIDI y un modulo cámara WiFi que envía posiciones skeleton del cuerpo a un programa secundario para mover un modelo 3D.

## Hardware

Cada modulo corporal contiene los siguientes componentes
- 1 micro controlador ESP32 DevKit V1
- 1 sensor de gas MQ-6
- 1 IMU de 9 ejes ICM-20948V2
- 1 Sensor ambiental ENS160 + AHT2X
- 2 Sensor de efecto Hall 49E

- 1 Base Prototipo.stl
- 1 Tapa.stl
- 1 Strap Holder.stl

- 18 tornillos M3x6
- 1 PowerBank 1Hora 5000mAh

Adicionalmente, cada mano cuenta con los siguientes componentes.
- Hand Circuit.stl
- Strap Holder.stl
- Wrist Strap.stl

Para hacer la detección corporal se usa lo siguiente

- 1 ESP32CAM
- Carcasa ESP32CAM

Para realizar las conexiones se requiere:
- 1 WiFi Modem

Para correr el Software se requiere:
- 1 Equipo PC/MAC

## Direcciones I2C

Los sensores cuentan con las siguientes direcciones I2C

```
HW-290
0x68 → AD0 a GND (la más común)
0x69 → AD0 a VCC

ENS160 + AHT2X

ENS160
0x52 → ADDR a GND
0x53 → ADDR a VCC

AHT20 / AHT21 (AHT2X)
0x38 → fija, no configurable

ICM-20948

IMU principal
0x68 → AD0 a GND
0x69 → AD0 a VCC

Magnetómetro interno (AK09916)
0x0C
```


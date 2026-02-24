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

### Direcciones I2C

Los sensores cuentan con las siguientes direcciones I2C

```
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

## Software

Este proyecto requiere una colección de software necesaria para el funcionamiento de todos sus sistemas. Debido a que puede instalarse en multiples sistemas operativos y plataformas, se enlaza la documentación oficial de cada software. Las pruebas fueron realizadas en Fedora 43. Las versiones de los softwares utilizados se indican en cada etapa.

- [Arduino IDE](https://www.arduino.cc/en/software/). Probado con la versión 2.3.7
- [ESP32 para Arduino IDE](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html). Probado con la versón 2.0.18-arduino.5
- [Node.JS](https://nodejs.org/en/download). Probado con la versión 24.13.1 LTS
- [NPM](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm). Probado con la versión 11.8.0
- [Node-Red](https://nodered.org/docs/getting-started/local). Probado con la versión 4.1.5
- [Mosquitto](https://mosquitto.org/download/). Probado con la versión 2.0.22

Consulta cada carpeta correspondiente a cada software para verificar configuraciones específicas.
# ESP32

Este proyeceto hace uso de un ESP32 Devkit V1 para la lectura de los sensores.

## Bibliotecas

Para correr los programas, se requieren las siguientes bibliotecas.

- ArduinoJson de Benoit Blanchon. Versión probrada 7.4.2
- AHTX0 de Adafruit. Versión probada 2.0.5
- Adafruit Unified Sensor de Adafruit. Versión probada 1.1.15
- PubSubClient de Nick O'Leary. Versión probada 2.8

## Programas de prueba

Se realizaron una serie de programas de prueba para verificar progresivamente el funcionamiento de todos los sensores. Antes de probar estos programas, lee completa la lista, pues la lectura del sensor ENS+AHT se corrige hacia el final debido a que se requieren condiciones específicas de lectura.

Los siguientes programas corresponden a la etapa de desarrollo en la que los sensores analogos tienen su propipo pin, y los acelerómetros y el sensor ambiental comparten 2 buses I2C en los pines 21/22 y en los pines 16/17. En esta etapa, las lecturas del ENS+AHT no reportan correctamente valores.

- **Analog Tester**. Es un programa que realiza la lectura de los sensores magnéticos en los `pines 32 y 35` y la lectura del sensor de gases en el  `pin 34`. Los valores se reportan via monitor serial.
- **I2C Lister**. Es un progama que escanea todas las direcciones I2C usando 2 buses para evitar conflictos con las direcciones I2C de los sensores. Se usan 2 buses porque se usan 2 sensores IMU, uno para cada mano. Los buses se encuentran en los `pines 21 y 22` para el bus A y en los `pines 16 y 17` para el bus B.
- **I2C_Reader_0X**. Son programas que realizan la lectura de todos los sensores por diferentes métodos.
- **Full_Reader_01**. Es un programa que realiza la lectura de los sensores analógicos y digitales y reporta todos los valores vía Serial.
- **Full_MQTT_01**. Es un programa que realiza la lectura de todos los sensores y la reporta en un mensaje MQTT separado por comas al tema `sinte/cuerpo1`. Los datos se envían a una IP fija y a una red predeterminada, estos deben actualizarse en caso de cambiar de configuración.

Los siguientes programas corresponden a la etapa de desarrollo en la que se agrega un tercer bus específico para el sensor ENS+AHT en un bus separado en los pines 25/26.

- **ENS_AHT_Tester**. Es un programa que realiza sólo la lectura del sensor ENS+AHT para garantizar su lectura en un tercer bus.
- **Full_MQTT_02**. Ete programa realiza la lectura de todos los sensores y corrige el problema de lectura del ENS+AHT que ahora se encuentra en un tercer bus separado. Este cambio se debe a que este sensor requiere condiciones específicas de lectura.
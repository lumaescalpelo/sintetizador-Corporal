# Mosquitto

Este proyecto usa el broker MQTT Mosquitto para lograr realizar el intercambio de mensajes entre los dispositivos Sintetizador Corporal y los equipos PC/Mac involucrados en el funcionamiento general.

## Configuraciones

Para que un servidor de mosquitto funcione correctamente, debe permitir conexiones desde IPs externas.

Agrega lo siguiente al archivo `/etc/mosquitto/conf.d/remote.conf`

```
allow_anonymous false
listener 1883 0.0.0.0
```

Asegurate de abrir el puerto 1883 para conexiones TCP y UDP.
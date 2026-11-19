# Paquete de telemetría

Fuente de verdad: `docs/SPEC.md`. Implementaciones: `protocol/telemetry.py` y `firmware/include/packet.h`.

Tamaño fijo: **32 bytes**, little-endian, sin relleno.

| Offset | Tamaño | Tipo | Campo        | Unidad / escala        |
|-------:|-------:|------|--------------|------------------------|
| 0      | 1      | u8   | version      | versión del formato (1)|
| 1      | 1      | u8   | estado       | 0..6 (ver SPEC)        |
| 2      | 2      | u16  | secuencia    | contador, da la vuelta |
| 4      | 4      | u32  | t_ms         | ms desde el arranque   |
| 8      | 4      | u32  | presion_Pa   | Pa                     |
| 12     | 2      | i16  | temp         | 0.01 °C                |
| 14     | 2      | i16  | altura       | 0.1 m (sobre p0)       |
| 16     | 2      | i16  | vz           | cm/s                   |
| 18     | 2      | i16  | ax           | mg                     |
| 20     | 2      | i16  | ay           | mg                     |
| 22     | 2      | i16  | az           | mg                     |
| 24     | 2      | i16  | roll         | 0.01 °                 |
| 26     | 2      | i16  | pitch        | 0.01 °                 |
| 28     | 2      | i16  | yaw          | 0.01 ° (solo relativo) |
| 30     | 1      | u8   | banderas     | ver abajo              |
| 31     | 1      | u8   | bateria      | V × 10                 |

## Estados
0 AUTOPRUEBA, 1 TIERRA, 2 ASCENSO, 3 CAIDA, 4 DESPLIEGUE, 5 DESCENSO, 6 ATERRIZADO.

## Banderas
| Bit | Significado          |
|----:|----------------------|
| 0   | base lista           |
| 1   | liberación           |
| 2   | disparo              |
| 3   | disparo por respaldo |
| 4   | aterrizado           |
| 5   | fallo baro           |
| 6   | fallo imu            |
| 7   | NO-GO                |

En el firmware de demostración (`DEMO_SEPARACION`, entornos `heltec_demo` y `heltec_banco`) el bit 2
(disparo) marca la separación del modo demo. El log guarda un registro en el ciclo exacto
de la separación (el primero con ese bit); en la telemetría el bit aparece en el
siguiente paquete (<= 200 ms).

## Log de la placa (`/vuelo.bin`, comando `DUMP`)
Registros de 32 B: los paquetes de arriba (byte 0 = versión, 1) y **registros de evento**,
que no se transmiten y empiezan con `0xEE` en lugar de la versión:

| Offset | Tamaño | Tipo | Campo   | Valor |
|-------:|-------:|------|---------|-------|
| 0      | 1      | u8   | marca   | `0xEE` |
| 1      | 1      | u8   | tipo    | 1 = arranque, 2 = cambio de modo de despliegue |
| 2      | 1      | u8   | valor   | modo de despliegue en ASCII (`'A'`, `'B'` o `'C'`) |
| 3      | 1      | u8   | —       | 0 |
| 4      | 4      | u32  | t_ms    | ms desde el arranque |
| 8      | 24     | —    | —       | ceros |

Decodificación: `decode_log_event()` en `protocol/telemetry.py` (devuelve `None` si el
registro es un paquete).

## Salida USB del receptor (`rx_station`)
Una línea por paquete, a 115200 baudios:

```
<HEX>,<RSSI dBm>,<SNR dB>      p. ej. 0100...59,-48.5,9.75
ERR,<código RadioLib>,<RSSI>,<SNR>   paquete con error (CRC, etc.)
# texto                        mensajes informativos (ignorar)
```
El HEX es el paquete crudo tal como llegó (normalmente 64 caracteres = 32 B); el backend
debe verificar la longitud antes de decodificar. Formato supuesto: ver docs/PREGUNTAS.md P14.

## Notas
- Altura máx. representable: ±3276.7 m; vz: ±327.67 m/s; aceleración: ±32.767 g; los valores se saturan al codificar.
- Sin CRC en el paquete: LoRa ya aplica CRC de capa física.

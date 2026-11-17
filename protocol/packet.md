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

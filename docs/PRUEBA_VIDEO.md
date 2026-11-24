# Prueba en video: separación (modo DEMO_SEPARACION)

Procedimiento para grabar el video de aceptación de la separación con el firmware de
demostración. El servo se dispara `DEMO_DELAY_S` segundos (10 por defecto) después de
armar. Dudas y suposiciones: `docs/PREGUNTAS.md` (P28, P34).

## Qué firmware usar

| Entorno | Para qué |
|---|---|
| `heltec_demo` | el video (modo demo; mide la pila si `BATT_ADC_ENABLED = 1`) |
| `heltec_banco` | lo mismo en la mesa, sin medición de batería (siempre reporta 0 V) |
| `heltec_wifi_lora_32_V3` | **vuelo**: no tiene modo demo ni usa el botón PRG |

`heltec_demo` y `heltec_banco` **nunca son para vuelo**. Después del video hay que volver
a cargar `heltec_wifi_lora_32_V3`.

Desde `firmware/`:
```bash
# Linux (puerto /dev/ttyUSB0, /dev/ttyUSB1...)
../.venv/bin/pio run -e heltec_demo -t upload --upload-port /dev/ttyUSB1
../.venv/bin/pio device monitor -p /dev/ttyUSB1 -b 115200
```
```powershell
# Windows (puerto COM5, COM6...)
..\.venv\Scripts\pio.exe run -e heltec_demo -t upload --upload-port COM5
..\.venv\Scripts\pio.exe device monitor -p COM5 -b 115200
```
En Linux, si el puerto da "Permission denied", el usuario debe estar en el grupo del
puerto (`uucp` en Arch: `sudo usermod -aG uucp $USER` y volver a iniciar sesión).

## Pasos

1. **Encender** con el interruptor deslizable. **No presionar PRG al encender**: GPIO0 es
   pin de arranque y la placa entraría en modo de descarga. La placa arranca sola en
   AUTOPRUEBA y pasa a TIERRA (no hay que armar nada para eso). Mantener el CanSat
   quieto los primeros ~3 s (calibración del giroscopio).
2. En el monitor serial, revisar el arranque: `MODO DEMO_SEPARACION`, `IMU OK`,
   `BARO OK`, `LittleFS OK`, `Radio OK (codigo 0)`, `NVS OK`. Si aparece
   `# REINICIO EN VUELO`, mandar `MISION_BORRAR`.
3. Mandar **`MISION_BORRAR`** (borra el registro de misión de NVS y reinicia la placa).
   Hacerlo antes de cada prueba o vuelo: un corte de corriente o apagar a mitad de una
   misión deja el registro en NVS, y el siguiente arranque se cree un reinicio en vuelo.
4. Mandar **`MODO B`** (o revisar con `MODO`; B es el de por defecto y el del lanzamiento
   real). El firmware demo también corre la máquina de vuelo; en el modo C una caída
   breve del CanSat dispararía el servo por su cuenta. En B (y en A) no: la liberación
   solo cuenta por encima de 85 m.
5. Opcional: `BORRAR` para empezar con el log vacío (o revisar el espacio con `STAT`).
6. Mandar **`STAT`** y comprobar: `estado=1` (TIERRA), `despliegue=B`, `servo=OFF`,
   banderas sin fallos (0x00, o 0x01 cuando la base de 30 s está lista).
7. Comprobar que la estación terrena recibe paquetes (1 Hz en TIERRA).
8. **Empezar a grabar.** Armar con un **toque largo del botón PRG (1 s)** o con el comando
   **`ARMAR`**:
   - el LED de la placa parpadea rápido durante la cuenta;
   - el servo recibe PWM y se sostiene en -10°;
   - la telemetría sube a 5 Hz;
   - por serial: `# DEMO ARMADO: disparo en 10 s ...`.
9. A los 10 s el servo gira a -80° (con hasta 2 reintentos si no hay pico de
   aceleración). Por serial: `# SEPARACION t_ms=...`. El LED queda **encendido fijo**.
   Desde ese paquete la bandera de disparo (bit 2) va activa a 5 Hz; la estación
   terrena la ve.
10. **Cancelar** (antes de que termine la cuenta): otro toque largo de PRG o `DESARMAR`.
    Por serial: `# DEMO CANCELADO`; el LED se apaga y el servo queda sin PWM.
11. **Repetir la toma**: después del disparo, un toque largo de PRG (o `ARMAR`) vuelve a
    armar. `ARMAR` durante la cuenta la reinicia.
12. Al terminar: `DUMP` para sacar el log (el instante de separación queda en un
    registro con el bit 2), y volver a cargar el firmware de vuelo.

## Comandos serial (115200, terminar con Enter)

| Comando | Qué hace |
|---|---|
| `STAT` | estado, banderas, modo de despliegue, servo, disparos, radio, log, NVS |
| `MODO` | muestra el modo de despliegue activo (B, A o C) y sus umbrales |
| `MODO A` \| `MODO B` \| `MODO C` | cambia el modo (solo en estados 0 y 1); se guarda en NVS y en el log |
| `MISION_BORRAR` | borra el registro de misión de NVS y reinicia (no borra el modo) |
| `ARMAR` | arma (o re-arma) la cuenta regresiva del servo (solo firmware demo/banco) |
| `DESARMAR` | cancela la cuenta regresiva |
| `BORRAR` | borra el log |
| `DUMP` | vuelca el log en hex |

## Checklist antes de grabar

- [ ] Firmware `heltec_demo` cargado (el arranque dice `MODO DEMO_SEPARACION`).
- [ ] Pila de 9 V nueva o medida con multímetro (con `BATT_ADC_ENABLED = 0` el paquete
      reporta 0 V: no sirve para saber la carga).
- [ ] Arranque sin `FALLA` (IMU, BARO, LittleFS, Radio, NVS).
- [ ] `MISION_BORRAR` hecho y sin `# REINICIO EN VUELO` al arrancar.
- [ ] `MODO B` (no C).
- [ ] `STAT`: `estado=1`, `servo=OFF`, banderas sin fallos.
- [ ] Espacio en el log (`STAT` → `log=usado/capacidad`) o `BORRAR`.
- [ ] Estación terrena recibiendo paquetes; RSSI razonable.
- [ ] Mecanismo y autogiro montados, servo probado antes en banco (ángulos -10° y -80°,
      `docs/PREGUNTAS.md` P16). El servo tiene energía siempre (no hay pin de energía),
      pero no recibe PWM hasta armar.
- [ ] CanSat firme y manos lejos del mecanismo; nadie presiona PRG sin querer.
- [ ] Cámara con el LED de la placa, el mecanismo y, si se puede, el monitor serial o la
      estación terrena en cuadro.

## Checklist previo al vuelo (regla fija, en este orden)

**`MISION_BORRAR`, `BORRAR`, `MODO B`, `STAT`.**

1. `MISION_BORRAR` (borra la misión de NVS y reinicia; el arranque no debe decir
   `# REINICIO EN VUELO`). El arranque muestra el modo guardado; si dice
   `# AVISO: modo A activo; el lanzamiento real es el modo B`, el paso 3 lo corrige.
2. `BORRAR` (vacía el log completo; responde `OK log vaciado (X B liberados): log=0/... B (0 %)`).
   Es obligatorio si el arranque mostró `# AVISO: log al X %, usa BORRAR antes del vuelo`
   (sale cuando el log pasa del 50 %).
3. `MODO B` (lanzamiento real: suelta a ~100 m, autogiro a 80 m). Responde
   `OK modo B (guardado en NVS)` o `OK modo B (sin cambios)`.
4. `STAT`: la primera línea debe ser
   ```
   despliegue=B [lanzamiento real: suelta ~100 m] liberacion valida > 85 m, autogiro a 80 m (h_disp = 80 + |vz|*0.5), respaldo 70 m, compuerta 45 grados max 0.3 s
     deteccion: baja g 100 ms + vz < -5 m/s en 0.6 s | vz < -8 m/s 0.2 s | 8 m bajo h_max con vz < -5 m/s | sin baro: disparo 1.0 s tras liberar | liberacion tardia: disparo inmediato
   ```
   y además `estado=1`, `log=0/...`, banderas sin fallos (0x00 o 0x01) y `p0_pausa=0`
   con el CanSat quieto en el suelo.

## Antes de cada vuelo (recordatorio)

1. Cargar `heltec_wifi_lora_32_V3` (el arranque dice `firmware de vuelo` y
   `Modo guardado: despliegue=B ...` o `Modo por defecto: despliegue=B ...`).
2. Checklist: `MISION_BORRAR`, `BORRAR`, `MODO B`, `STAT` (`despliegue=B`, `estado=1`).
3. Encender en el sitio de despegue y esperar 30 s quieto en el suelo (base de p0 lista,
   bandera 0x01) antes del despegue. El p0 solo se promedia con el CanSat quieto en el
   suelo: si se mueve o se levanta, `STAT` dice `p0_pausa=1` (normal mientras se monta en
   el dron; vuelve a 0 al dejarlo quieto en el suelo).
4. El dron sube (≥ 1.5 m/s), se estaciona a ~100 m y suelta. La placa pasa a ASCENSO a
   ~20 m + 3 s de subida; el estacionamiento no cambia el estado. Sueltas entre 90 y
   105 m despliegan a 80 m; por debajo de 85 m **no** hay liberación (no despliega).

## Log lleno

El log (LittleFS, ~1.4 MB) **deja de guardar cuando se llena** (no sobrescribe). Para que
los datos de vuelo nunca se pierdan: al despegar (ASCENSO) o al armar el demo, si quedan
menos de 600 KB libres, la placa **vacía sola el log de arranque y pruebas** y avisa
`# AVISO: log vaciado al despegar/armar (...)`. Aun así, haz `BORRAR` en el checklist:
así el log del vuelo empieza limpio. `STAT` muestra `log=usado/cap (X %)` y
`sin_espacio=N` (registros que no se pudieron guardar; debe ser 0 en el vuelo).
Si un comando no se reconoce, la placa imprime `ERR comando desconocido: "<texto>"`
con lo que recibió.

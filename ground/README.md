# Estación terrena Project Hellmary

Backend (FastAPI + WebSocket + pyserial) e interfaz web local, sin internet (uPlot va en
`static/vendor/`). Recibe lo que imprime `rx_station` (`HEX,RSSI,SNR`), calcula las
estadísticas y guarda la toma de datos en `ground/data/` (un CSV y un archivo crudo por
sesión). Dudas y supuestos: `docs/PREGUNTAS.md` (P36 a P43).

## Instalar (PowerShell, desde la raíz del repositorio)
```powershell
py -m venv .venv                       # solo si no existe el entorno virtual
.\.venv\Scripts\python -m pip install -r requirements.txt
```

## Ejecutar
```powershell
# Vuelo simulado en tiempo real (tools/flight_sim.py): ensayo sin placa
.\.venv\Scripts\python ground\server.py --sim
#   opciones: --speed 4 (más rápido)  --sim-loss 10 (10 % de paquetes perdidos)
#             --sim-blackout 30,8 (sin paquetes de t=30 s a t=38 s: prueba el aviso de enlace perdido)

# Receptor real (Heltec con rx_station, 115200 baudios)
.\.venv\Scripts\python ground\server.py --port COM5

# Repetir una grabación
.\.venv\Scripts\python ground\server.py --replay ground\data\sesion_20260301_101500.csv
#   opcional: --speed 4
```
Luego abrir **http://127.0.0.1:8000** en el navegador (Edge o Chrome). Para otro puerto
web: `--web-port 8080`. `--auto-record` empieza a grabar al arrancar.

**Ver el puerto COM de la Heltec receptora:** Administrador de dispositivos → *Puertos
(COM y LPT)* → buscar "USB Serial Device" o "CP210x/USB-Enhanced-SERIAL" (aparece al
conectar la placa y desaparece al desconectarla). También:
`.\.venv\Scripts\python ground\server.py --list-ports`. El monitor serial de PlatformIO
u otro programa abierto en ese puerto lo ocupa: cerrarlo antes. Si el puerto no está
disponible, la interfaz lo avisa y el servidor reintenta cada 2 s.

## Uso de la interfaz
- **INICIAR TOMA DE DATOS / FINALIZAR TOMA DE DATOS** (botón verde/rojo del encabezado):
  empieza y termina la grabación. Cada sesión crea `ground/data/sesion_AAAAMMDD_HHMMSS.csv`
  (variables decodificadas + RSSI, SNR y hora) y `.raw.txt` (las líneas tal como llegaron).
  Lo que llega antes de pulsar INICIAR se ve en pantalla pero no se guarda.
- **Exportar CSV**: descarga el CSV de la sesión actual (o de la última grabada).
- **Generar reporte**: corre `tools/reporte_vuelo.py` sobre la sesión actual (o la última
  grabada), deja el reporte en `ground/data/reporte_<fecha>/` y abre esa carpeta en el
  explorador. Tarda unos segundos (el botón dice *Generando…*); la recepción no se detiene.
  Con la grabación en curso, el reporte lleva los datos hasta ese momento. El registro de
  eventos muestra la carpeta. `--no-abrir-reporte` solo la crea, sin abrirla.
- **Gráficas**: ventanas de 30 s, 2 min, 5 min o todo; **Pausar** (tecla **P**) congela las
  gráficas para revisarlas (el cursor muestra los valores); las tarjetas siguen en vivo.
- **Modo pantalla de video**: botón *VIDEO* (esquina del encabezado) o tecla **V**. Oculta
  controles, banderas y eventos; deja tarjetas, gráficas y actitud, con el logotipo
  visible. `?video=1` en la dirección abre directo en ese modo. Ajustado a 1920x1080 con
  el zoom del navegador al 100 % (F11 para pantalla completa).
- **Reiniciar**: borra estadísticas, gráficas y eventos de la pantalla (no los archivos).

## Tema visual: «plano técnico / bitácora de vuelo»
La interfaz tiene dos temas. El **predeterminado** es el de *plano técnico*, pensado para
usarse al aire libre a pleno sol (`static/tema-plano.css`): papel crema con rejilla sutil,
tinta azul muy oscura, líneas de 1 px, esquinas rectas, marcas de registro en las esquinas
de cada panel y rótulos tipo cuadro de títulos (`FIG. 1 · ALTURA`...). Hay **un solo color de
alerta, el naranja aeronáutico**: NO-GO y fallos de sensores, enlace perdido y velocidad de
descenso fuera de 8 a 12 m/s. Todo lo normal va en tinta; el estado actual de la tira va con
relleno de tinta y texto crema; la velocidad dentro de la banda se marca con trama de rayas.
Las gráficas distinguen las series por el trazo (continua, guiones, puntos), no solo por el
color, y la banda de 8 a 12 m/s va con rayas finas. El horizonte artificial usa papel y tinta.
Las cifras van en monoespaciada y los rótulos en sans condensada, con cifras tabulares.

El tema anterior (oscuro) se conserva intacto como **respaldo**:
- `http://127.0.0.1:8000/?tema=clasico` lo abre (y `?tema=plano` fuerza el nuevo);
- la tecla **T** alterna entre los dos y recuerda la elección en el navegador
  (`localStorage`, con `try/catch`: si el navegador lo bloquea, funciona igual sin recordar).
El modo video (tecla **V**) funciona con ambos temas.

**Sin red:** las fuentes están en `static/fonts/` (IBM Plex Mono e IBM Plex Sans Condensed,
latin 400/500/700, WOFF2, licencia SIL OFL: `fonts/LICENSE-IBM-Plex-OFL.txt`); uPlot ya iba
en `static/vendor/`. Ninguna fuente, ícono ni librería se carga de internet. Si faltaran las
fuentes, el tema cae a `DejaVu Sans Mono` / `Liberation Mono` y a `Arial Narrow`.

## Logotipo
Poner el archivo del equipo en **`ground/static/logo.png`** (en el tema plano se muestra
sobre un recuadro de tinta, porque el escudo actual es de fondo oscuro). Mientras no exista se
muestra un marcador amarillo que dice dónde ponerlo. Conviene un PNG con fondo
transparente, de proporción ancha y unos 400x120 px o más.

## Reporte de vuelo
Gráficas PNG (200 dpi, fases sombreadas por estado) de altura, velocidad vertical (banda
de 8 a 12 m/s en el descenso), presión y temperatura, aceleración, actitud y enlace, más
la tabla de números en `reporte.md` y `reporte.html` (un solo archivo, con las imágenes
adentro) y `resumen.json`. Se guarda en `reporte_<fecha de la sesión>/`, junto al CSV.
Linux o macOS (bash, en la raíz del repositorio):
```bash
.venv/bin/python tools/reporte_vuelo.py                                  # sesión más reciente
.venv/bin/python tools/reporte_vuelo.py ground/data/sesion_20261119_172700.csv --pdf --abrir
.venv/bin/python tools/reporte_vuelo.py ground/data/sesion_20261119_172700.csv --log vuelo_dump.txt
```
`--abrir` y el botón abren la carpeta con `xdg-open` (Linux), `open` (macOS) o
`os.startfile` (Windows); si no se puede, el reporte se genera igual y se avisa (consola,
ventana y registro de eventos). Desde PowerShell (Windows), en la raíz del repositorio:
```powershell
# Sesión más reciente de ground\data (la última que tenga paquetes)
.\.venv\Scripts\python tools\reporte_vuelo.py

# Una sesión concreta, con PDF de dos hojas, abriendo la carpeta al terminar
.\.venv\Scripts\python tools\reporte_vuelo.py ground\data\sesion_20261119_172700.csv --pdf --abrir

# Con el log interno de la placa (DUMP) superpuesto: marca lo que la estación no recibió
.\.venv\Scripts\python tools\reporte_vuelo.py ground\data\sesion_20261119_172700.csv --log vuelo_dump.txt
```
`--log` acepta el texto tal como lo imprime `DUMP` (copiado del monitor serial, con o sin
las líneas `LOG_INICIO`/`LOG_FIN`) o un CSV con las columnas del paquete (`t_ms`,
`altura_m`, ...). Otras opciones: `--salida CARPETA` (dónde crear `reporte_<fecha>/`),
`--dpi 300`. Al terminar imprime la tabla en la consola. Criterios de cada número (qué es
"aterrizaje", etc.). La velocidad de descenso principal es la **mediana sin los primeros
2 s de DESCENSO**; se muestran también la media sin ese tramo y el promedio de los últimos
5 s antes del aterrizaje, cada una con su Sí/No de 8 a 12 m/s. El impacto sin `--log` sale
como "no medible con telemetría (usa --log)":
`docs/PREGUNTAS.md` P50 a P56.

## API REST
`GET /api/status` · `POST /api/record/start` · `POST /api/record/stop` ·
`GET /api/export.csv` · `POST /api/report` · `POST /api/reset` · WebSocket `/ws`.

## Pruebas
```powershell
.\.venv\Scripts\python -m pytest ground\tests -q
.\.venv\Scripts\python -m pytest tools\tests\test_reporte_vuelo.py -q    # cálculos del reporte
```
Cubren el parser de líneas, la pérdida de paquetes, la altura máxima, el enlace perdido,
la grabación del CSV y el servidor (REST y WebSocket). No se puede probar el puerto serie
real sin la placa.

## Archivos
`server.py` (backend y argumentos), `gs_core.py` (parser, estadísticas, grabación),
`gs_sources.py` (puerto serie, simulador, repetición), `static/` (interfaz), `data/`
(grabaciones, no se sube a git).

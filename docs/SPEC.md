# CanSat Hellmary: sistemas
Concurso PEUUM CanSat 2026. Reglas que afectan al software: despliegue del autogiro a 200 m medidos desde el suelo de donde el dron inició el ascenso; transmitir presión, temperatura, orientación y aceleración en subida y en caída libre (liberación a ~450 m); calcular velocidad y altura máxima; seguir transmitiendo >=10 s tras aterrizar; los jueces ven la transmisión en tiempo real en la estación terrena; aceptación: transmitir a 250 m; velocidad de descenso con autogiro entre 8 y 12 m/s; pila cuadrada de 9 V; interruptor ON/OFF sin protuberancia (reed). Sin GPS. La secundaria cae libre y no se detecta.
Hardware: Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262), MPU6500 (6 ejes, sin magnetómetro), BMP280, servo MG90S, reed + enclavamiento por GPIO (lo diseña el electrónico), regulador MP1584 a 5 V.
Pines: NO inventar. Todos en firmware/include/config.h con valores TODO_CONFIRMAR hasta que el usuario los dé.
Velocidad: vertical (Kalman h,vz con barómetro + aceleración vertical rotada por la actitud). Actitud: filtro Mahony o Madgwick; el yaw solo es relativo.
Radio: LoRa 915 MHz, SF7, BW 250 kHz, CR 4/5, 14 dBm, sync word propio. Librería RadioLib.
## Estados
0 AUTOPRUEBA: IDs de sensores, sesgo del giroscopio, servo SIN energía. Si falla todo -> NO-GO en las banderas.
1 TIERRA: presión base p0 por promedio móvil de ~30 s; TX 1 Hz, sensores a baja frecuencia (ahorro de pila). Salida: altura>20 m y vz>+1 m/s durante 3 s -> 2 (congelar p0 y guardarlo en NVS).
2 ASCENSO: log 50 Hz, TX 2 Hz, guarda altura máxima. Liberación -> 3.
3 CAIDA: TX 5 Hz. Liberación válida solo si altura>240 m (parámetro). Se detecta por: |a|<0.35 g durante 150 ms confirmada por vz<-8 m/s en el siguiente 1 s; o vz<-12 m/s durante 0.4 s; o altura 25 m por debajo del máximo. Pasa a 4 cuando h <= h_disp y la compuerta de ángulo se cumple.
   h_disp = 200 m + |vz|*t_lat (t_lat inicial 0.5 s, parámetro). Compuerta: ángulo entre el eje del cuerpo y la vertical <=45° (eje con signo configurable en config.h). Respaldo: si no se cumple en 1.5 s o si h<=160 m, disparar igual.
4 DESPLIEGUE: alimentar el servo, girarlo de -10° a -80°. Confirmar con pico de aceleración (por validar); si no hay confirmación en 1 s reintentar hasta 2 veces. -> 5.
5 DESCENSO: TX 5 Hz, vz promedio de 1 s, bandera si sale de 8 a 12 m/s. Impacto y reposo -> 6.
6 ATERRIZADO: vaciar el log, TX 2 Hz durante 60 s, luego baliza cada 5 s.
## Fallas
Baro muerto o congelado (valor idéntico >1 s): liberación solo por IMU y disparo por tiempo desde la liberación (parámetro). IMU muerto: liberación solo por barómetro, sin compuerta de ángulo. Reinicio en vuelo: bandera de misión en NVS, recargar p0 sin recalibrar y entrar al estado según vz. Servo sin energía hasta el estado 3 y pull-down en su señal. El radio nunca bloquea el lazo de control.
## Modo demo (video de aceptación)
Compilación alterna DEMO_SEPARACION: el servo se dispara DEMO_DELAY_S segundos (por defecto 10) después de armar por comando serial o imán. El modo de vuelo normal no cambia.
## Paquete de telemetría (32 B, little-endian)
0 version u8 | 1 estado u8 | 2 secuencia u16 | 4 t_ms u32 | 8 presion_Pa u32 | 12 temp_0.01C i16 | 14 altura_0.1m i16 | 16 vz_cm/s i16 | 18 ax,ay,az mg 3xi16 | 24 roll,pitch,yaw 0.01° 3xi16 | 30 banderas u8 | 31 bateria V*10 u8
Banderas: bit0 base lista, bit1 liberacion, bit2 disparo, bit3 disparo por respaldo, bit4 aterrizado, bit5 fallo baro, bit6 fallo imu, bit7 NO-GO.
## Estación terrena
Receptor: segunda Heltec V3 que imprime por USB cada paquete en hex más RSSI y SNR. Backend Python (pyserial + FastAPI + WebSocket). Frontend web sin CDN (todo local). Guarda CSV y crudo.

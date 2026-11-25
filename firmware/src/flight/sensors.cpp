#include "sensors.h"

#include <Adafruit_BMP280.h>
#include <Wire.h>

#include "config.h"
#include "i2c_diag.h"

// ---------------------------------------------------------------------------
// Registros del MPU6500 (hoja de datos / mapa de registros de InvenSense)
// ---------------------------------------------------------------------------
namespace mpu {
constexpr uint8_t SMPLRT_DIV = 0x19;
constexpr uint8_t CONFIG = 0x1A;
constexpr uint8_t GYRO_CONFIG = 0x1B;
constexpr uint8_t ACCEL_CONFIG = 0x1C;
constexpr uint8_t ACCEL_CONFIG2 = 0x1D;
constexpr uint8_t ACCEL_XOUT_H = 0x3B;
constexpr uint8_t PWR_MGMT_1 = 0x6B;
constexpr uint8_t PWR_MGMT_2 = 0x6C;
constexpr uint8_t WHO_AM_I = 0x75;

// Rangos: ±16 g (impacto y despliegue) y ±2000 °/s (caída con giro). Ver PREGUNTAS P7.
constexpr uint8_t ACCEL_FS_16G = 0x18;
constexpr uint8_t GYRO_FS_2000 = 0x18;
constexpr float ACC_LSB_PER_G = 2048.0f;
constexpr float GYRO_LSB_PER_DPS = 16.4f;
// Filtros pasa bajas internos: giroscopio DLPF_CFG=2 (~92 Hz), acelerómetro A_DLPF_CFG=2 (~99 Hz),
// adecuados para muestrear a 100 Hz.
constexpr uint8_t DLPF_GYRO_92HZ = 0x02;
constexpr uint8_t DLPF_ACC_99HZ = 0x02;
}  // namespace mpu

static Adafruit_BMP280 bmp(&Wire);

static bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6500_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(MPU6500_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU6500_I2C_ADDR, (uint8_t)n) != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool Sensors::begin() {
#if (PIN_I2C_SDA >= 0) && (PIN_I2C_SCL >= 0)
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
#else
  // Pines pendientes (solo posible con PERMITIR_PINES_PENDIENTES): no se inicia el bus.
  return false;
#endif
  imu_ok_ = beginImu();
  baro_ok_ = beginBaro();
  return imu_ok_ || baro_ok_;
}

bool Sensors::beginImu() {
  if (!readRegs(mpu::WHO_AM_I, &who_am_i_, 1)) return false;
  if (who_am_i_ != MPU6500_WHO_AM_I_EXPECTED) return false;
  writeReg(mpu::PWR_MGMT_1, 0x80);  // reinicio del dispositivo
  delay(100);
  bool ok = writeReg(mpu::PWR_MGMT_1, 0x01);  // reloj: PLL automático
  delay(10);
  ok &= writeReg(mpu::PWR_MGMT_2, 0x00);       // todos los ejes activos
  ok &= writeReg(mpu::SMPLRT_DIV, 0x00);
  ok &= writeReg(mpu::CONFIG, mpu::DLPF_GYRO_92HZ);
  ok &= writeReg(mpu::GYRO_CONFIG, mpu::GYRO_FS_2000);
  ok &= writeReg(mpu::ACCEL_CONFIG, mpu::ACCEL_FS_16G);
  ok &= writeReg(mpu::ACCEL_CONFIG2, mpu::DLPF_ACC_99HZ);
  // Verifica que la configuración quedó escrita
  uint8_t v = 0;
  ok &= readRegs(mpu::GYRO_CONFIG, &v, 1) && v == mpu::GYRO_FS_2000;
  return ok;
}

bool Sensors::readImu(ImuSample &s) {
  if (!imu_ok_) return false;
  uint8_t b[14];
  if (!readRegs(mpu::ACCEL_XOUT_H, b, sizeof b)) return false;
  auto be16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
  for (int i = 0; i < 3; i++) {
    s.acc_g[i] = be16(2 * i) / mpu::ACC_LSB_PER_G;
    s.gyro_dps[i] = be16(8 + 2 * i) / mpu::GYRO_LSB_PER_DPS;  // 6..7 es temperatura
  }
  return true;
}

bool Sensors::beginBaro() {
  if (!bmp.begin(BMP280_I2C_ADDR, BMP280_CHIPID)) return false;
  // Modo normal, presión x4, temperatura x1, sin filtro IIR (evita retraso a 35 m/s),
  // standby 1 ms: ~70 Hz de datos nuevos; se lee a 25 Hz. Ver PREGUNTAS P8.
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X1,
                  Adafruit_BMP280::SAMPLING_X4, Adafruit_BMP280::FILTER_OFF,
                  Adafruit_BMP280::STANDBY_MS_1);
  return true;
}

bool Sensors::readBaro(BaroSample &s) {
  if (!baro_ok_) return false;
  s.temp_c = bmp.readTemperature();
  s.p_pa = bmp.readPressure();
  return !isnan(s.p_pa) && s.p_pa > 0;
}

// ---------------------------------------------------------------------------
// Diagnóstico del bus (comando I2C). Quien lo llame debe garantizar que nadie más usa Wire.
// ---------------------------------------------------------------------------
static I2cEscaneo escanea(Print &out, int sda, int scl) {
  I2cEscaneo e;
  Wire.end();
  Wire.begin(sda, scl, I2C_CLOCK_HZ);
  Wire.setTimeOut(10);
  for (uint8_t a = I2C_DIR_MIN; a <= I2C_DIR_MAX; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      e.agrega(a);
      out.printf("  0x%02X %s\n", a, i2cNombre(a));
    }
  }
  if (e.n == 0) out.println("  (nada responde)");
  return e;
}

void i2cDiagnostico(Print &out) {
#if (PIN_I2C_SDA >= 0) && (PIN_I2C_SCL >= 0)
  Wire.end();
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, INPUT_PULLUP);
  delay(5);
  const bool sda = digitalRead(PIN_I2C_SDA), scl = digitalRead(PIN_I2C_SCL);
  out.printf("I2C reposo: SDA(%d)=%s SCL(%d)=%s\n", PIN_I2C_SDA, sda ? "ALTO" : "BAJO",
             PIN_I2C_SCL, scl ? "ALTO" : "BAJO");
  if (!sda || !scl) out.println("  posible corto a GND o linea sin pull-up");

  out.printf("I2C escaneo normal (SDA=%d SCL=%d):\n", PIN_I2C_SDA, PIN_I2C_SCL);
  const I2cEscaneo normal = escanea(out, PIN_I2C_SDA, PIN_I2C_SCL);
  out.printf("I2C escaneo cruzado (SDA=%d SCL=%d):\n", PIN_I2C_SCL, PIN_I2C_SDA);
  const I2cEscaneo cruzado = escanea(out, PIN_I2C_SCL, PIN_I2C_SDA);
  if (cruzado.n > 0) out.println("  AVISO: responde algo con SDA/SCL invertidos: posible SDA/SCL CRUZADOS");

  char v[160];
  i2cVeredicto(normal, cruzado, sda, scl, v, sizeof v);
  out.println(v);

  Wire.end();  // restaura la configuración normal del bus
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
  Wire.setTimeOut(50);  // valor por defecto de Arduino-ESP32
#else
  out.println("ERR pines I2C pendientes");
#endif
}

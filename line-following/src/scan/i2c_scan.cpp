/**
 * ============================================================================
 * i2c_scan.cpp —— S3 台架诊断探针 v5（不参与正式固件）
 * ============================================================================
 * 已确认事实：
 *   · SDA=GPIO1/SCL=GPIO2 上确有 PCA9685(0x40)+MPU6050(0x68)，100k/400k 都 10/10 成功，
 *     accel/gyro/temp 读数正常 ⇒ 硬件没问题。
 *   · v4 复刻正式固件开机时序（但前面多了 600ms）→ 全部检测成功。
 *   · 正式固件本体却报 imuOk=0。
 * 怀疑：正式固件在复位后 ~100ms 就做 I2C 探测，**太早**（外扩板/器件尚未就绪）。
 *
 * v5：完全按正式固件时序（Serial.begin → Wire.begin → delay(50) → 探测），
 *     并在 50/100/200/400/800/1600 ms 各测一次，把结果存下来，在 loop 里打印。
 *     若前面几个失败、后面成功 ⇒ 确认"开机太早"，修法是加延时或重试。
 *
 * 烧录：pio run -e s3-i2cscan -t upload
 * ============================================================================
 */
#include <Arduino.h>
#include <Wire.h>

#define MPU_A 0x68
#define PCA_A 0x40
#define N 6

static bool i2cPresent(uint8_t addr)
{
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

static uint8_t rd8(uint8_t a, uint8_t reg)
{
    Wire.beginTransmission(a);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xEE;
    if (Wire.requestFrom((int)a, 1) != 1) return 0xEE;
    return (uint8_t)Wire.read();
}

static const uint32_t DELAYS[N] = {50, 50, 100, 200, 400, 800};  // 相对上一次的等待
static int      rMpu[N];      // i2cPresent(0x68)
static int      rPca[N];      // i2cPresent(0x40)
static uint8_t  rWho[N];      // WHO_AM_I(0x68, 0x75)
static uint32_t rT[N];        // 采样时刻 millis()

void setup()
{
    Serial.begin(921600);

    // ---- 完全按正式固件：立刻 begin + delay(50) 就探测 ----
    Wire.begin(1, 2, 400000UL);

    for (int i = 0; i < N; i++) {
        delay(DELAYS[i]);
        rT[i]   = millis();
        rMpu[i] = i2cPresent(MPU_A) ? 1 : 0;
        rPca[i] = i2cPresent(PCA_A) ? 1 : 0;
        rWho[i] = rd8(MPU_A, 0x75);
    }
}

void loop()
{
    static uint32_t k = 0;
    Serial.println();
    Serial.println("######## PROBE v5 : 开机时刻 vs I2C 就绪 ########");
    Serial.println("  t(ms)   0x68  0x40  WHO_AM_I");
    for (int i = 0; i < N; i++) {
        Serial.printf("  %5lu     %d     %d    0x%02X\n",
                      (unsigned long)rT[i], rMpu[i], rPca[i], rWho[i]);
    }
    Serial.printf("[now] 0x68=%d 0x40=%d who=0x%02X   (idx %lu)\n",
                  i2cPresent(MPU_A) ? 1 : 0, i2cPresent(PCA_A) ? 1 : 0,
                  rd8(MPU_A, 0x75), (unsigned long)k++);
    delay(4000);
}

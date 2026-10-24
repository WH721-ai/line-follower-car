/**
 * ============================================================================
 * bipedal_turnctrl —— 双轮差速控制固件 v2.0.0（ESP32-S3 扩展主控）
 * ============================================================================
 * v1.0.0 直接沿用课程第五课的「速度环→俯仰姿态环→力矩」三层结构，但那个结构
 * 依赖商家原厂电机固件的**力矩模式**；而电机主控现在跑的是自编 SimpleFOC
 * **速度模式**固件（R/T/S 协议）。因此 v2.0.0 把控制律改成与"轮速类指令"匹配的
 * 标准差速控制：
 *
 *   轮速外环（编码器）  vTrim = PI(v_ref − v_meas)      —— 慢修正，消稳态偏差
 *   偏航率环（IMU）     wCmd  = ω_ref + kt·(ω_ref − GyroZ)
 *   差速运动学          ωL = (vCmd − wCmd·Lw/2) / Rw
 *                       ωR = (vCmd + wCmd·Lw/2) / Rw
 *   下发                M0 = M0Dir·ωL ,  M1 = M1Dir·ωR
 *
 *   课程原来的第三环（俯仰姿态/平衡环 kp1·Δpitch）对**地面轮式差速车**没有意义
 *   （本体由腿/车架支撑，不需要靠轮子反作用力矩自稳），已移除。
 *   若日后装腿做"平衡车式"控制，电机主控固件已保留力矩模式入口
 *   （S1 侧 `T<u0>,<u1>`），届时把本文件的输出改成力矩即可。
 *
 * 与 v1.0.0 的关键差异：
 *   1. 电机链路：SF_BLDC(libSF_BLDC.a, T/V/A/C/U, 需商家固件)
 *              → MotorLink(2Mbps @ RX18/TX17, R/S 协议, 配自编双电机固件)
 *   2. 控制律：三环（速度→俯仰→力矩） → 差速运动学 + 偏航率环（上述）
 *   3. 单位：v 由"电机 rad/s"改为"车体 m/s"，ω 为车体 rad/s；
 *      新增轮半径 rw / 轮距 lw 两个可直接串口改的物理参数
 *   4. 新增 IMU/舵机**存在性检测**：没接 MPU6050 时不启用偏航率环并告警
 *      （否则 I2C 读失败会返回 0xFFFF，偏航环拿垃圾值 → 上电乱转）
 *   5. 新增直接轮速命令 w<L>,<R>，台架验证时不必经过运动学换算
 *
 * 硬件（与课程一致）：
 *   I2C：SDA=GPIO1  SCL=GPIO2 @400kHz
 *        ├─ PCA9685 (0x40) 4 路舵机：通道 3/4/5/6 = 左前/左后/右前/右后
 *        └─ MPU6050 (0x68)
 *   板间串口：Serial2 @2Mbps，RX=GPIO18 / TX=GPIO17 → 电机主控（ESP32）
 *   v3.1.0 七路灰度（荣洋 RYDZ 数字版，5V 供电；本颗为白线版：白=低、黑=高）：
 *        OUT1..7 → IO14 / IO47 / IO38 / IO40 / IO41 / IO21 / IO48
 *        （StackForce 扩展主控 BUS2，均 INPUT_PULLUP，只读）
 *
 * 串口命令（USB @921600，\n 结尾）：
 *   v<车速 m/s>        目标车体线速度（正=前进）
 *   t<偏航率 rad/s>    目标偏航角速度（正=左转，需先用 g 验证符号）
 *   w<L>,<R>           直接指定左右轮角速度 [rad/s]（绕过运动学，台架调试用）
 *   rw<轮半径 m>       默认 0.015045（实车实测 2026-10-17）
 *   lw<轮距 m>         默认 0.144   （实车实测 2026-10-17）
 *   kt<值>             偏航率环 P（默认 0.6）
 *   kv<P>,<I>          轮速外环 PI（默认 0.4,0.8；输出限幅 ±0.4 m/s）
 *   md<±1>,<±1>        电机方向 M0,M1（默认 -1,+1，2026-10-17 手推/手转实测定案；见下方 M0Dir 注释）
 *   g                  陀螺自检 3 秒：手动把车头向左推，GyroZ 必须为正
 *   c                  重标陀螺零偏（阻塞约 5 秒，车必须静止放平）
 *   pt<值>             俯仰保护阈值 rad，0=关闭（默认 0.6）
 *   d0 / d1            关闭 / 开启 20Hz 状态帧
 *   s                  急停（目标清零）
 *   ?                  帮助
 * ============================================================================
 */

#include <Arduino.h>
#include "Wire.h"
#include "SF_Servo.h"
#include "SF_IMU.h"
#include "pid.h"
#include "bipedal_data.h"
#include "MotorLink.h"
#include <Preferences.h>   // ★v3.9.10 自启速度写 NVS：断电后仍保留（否则每次上电回 0.28，"可调"没意义）
#include <math.h>          // sqrtf：自启增益按 √(v) 归一化

// ---------------- 硬件对象 ----------------
SF_Servo   servos  = SF_Servo(Wire);
SF_IMU     mpu6050 = SF_IMU(Wire);
MotorLink  motors  = MotorLink(Serial2);

// ---------------- 状态数据 ----------------
robotposeparam   robotPose;
robotmotionparam robotMotion;
MotorData        BLDCData;

// ---------------- 硬件存在性 ----------------
bool imuOk   = false;
bool servoOk = false;

// ---------------- 控制参数（均可串口在线修改） ----------------
float Rw        = 0.015045f; // 轮半径 [m]（实车实测：SW 量取 + 实车核对，轮径 30.09 mm）
float Lw        = 0.144f;    // 轮距 [m]（实车实测：两轮接地点中心距 144 mm）
float turnKp    = 0.60f;   // 偏航率环 P
float speedMax  = 1.00f;   // 目标车速限幅 [m/s]
float yawMax    = 4.00f;   // 目标偏航率限幅 [rad/s]
float wheelMax  = 60.0f;   // 轮角速度限幅 [rad/s]
// ★★v3.0.1 差速钳位用的"内轮最低转速"[rad/s]。
//   偏航率环 wCmd = turn + kt·(turn−GyroZ) 在打滑时与实测反号 ⇒ 自我放大：
//   整圈首跑 t=10.67 实测 wCmd 冲到 −6.00 ⇒ 内轮 (0.5−6.0×0.072)/0.015045 = 4.5 rad/s
//   落进静摩擦死区 ⇒ 不转 ⇒ 车真的绕内轮枢轴转 ⇒ GyroZ 更反向 ⇒ wCmd 更负（跑了 17.8 圈）。
//   钳住"内轮速 ≥ wDipMin"就切断这条正反馈；代价是打滑时实际半径比命令值大。
float wDipMin   = 10.0f;   // 内轮速下限 [rad/s]（v3.9.6 9→10：run0616 实测内轮 9.0 rad/s 就粘滑失速，
                           //                        而 10.2 是稳的 ⇒ 下限抬到 10.0 留一点余量）
// ★★v3.8.0 12→9：正式场 R1~R6 是 R=200 mm 半圆，v=0.28 时内轮只有 11.9 rad/s，
//   12 的下限直接把 wCap 钳到 1.381 < 需求 1.40 ⇒ 一进弯就欠舵丢线（run41 实测）。
//   9 给到 1.60 的权限，同时 9 rad/s 仍在粘滑区之上（粘滑实测在 3~5 rad/s）。
float pitchProt = 0.60f;   // 俯仰保护阈值 [rad]，0 = 关闭
// ★★ v2.3.0 直行航向保持（heading hold）
//   动机（2026-10-17 0.5 m 直行实测）：v2.2.0 的 kt 门控在 turn=0 时把陀螺反馈整个关掉
//   ⇒ 直行是**纯开环**，左右轮任何不对称都 1:1 变成航向漂移。
//   实测：起步 1 s 内甩 22.4°（左轮 M0 起步滞后），稳态再缓慢 5°/5 s（左轮慢 5.5%）。
//   架空 A/B 已证明速度环与编码器对称（左右差 1.1%）⇒ 不对称来自地面接触/负载，
//   改 S1 起动策略代价大且只能治标 ⇒ 在 S3 侧加外环：直行时锁存航向，用 yaw 误差闭环。
//   结构：wCmd = kh·(yawHold − yaw) − kd·GyroZ（GyroZ 项只做阻尼，不引入零偏偏置）
//   ★v2.9.4 结构升级：wCmd = kh·eYaw + holdI − kd·GyroZ，holdI = ki·∫eYaw dt。
//     方框 km1 轮相位分解实锤：巡航 eYaw 恒 +1~5°（P-only 稳态下垂，扰动≈轮不对称/
//     线缆拖拽），每边走斜 2~3 cm；减速段下垂继续长到 +10°，每边横漂合计 6~8 cm。
//     P-only 下 eYaw = 所需差速/kh ≈ 0.2 rad/s / 2.0 ≈ 3.4°——只有 I 项能把它收零。
//   直行判定：|turn|<HOLD_TURN_LIM 且 |forward|>HOLD_VEL_MIN 且非 directMode。
float holdKp    = 2.00f;   // 航向保持 P [rad/s 每 rad]（在线 kh<>）
float holdKd    = 0.25f;   // 航向保持 阻尼（在线 kd<>）
float holdKi    = 0.00f;   // ★v2.9.4 航向保持 I [1/s]（在线 ki<>，0=关，默认关保旧行为）
float holdMax   = 0.60f;   // 航向保持输出限幅 [rad/s]，勿超过原地转能力
// ★v2.8.0：滑行段（定距 done 之后到停稳）的航向环限幅。
//   N085 实测：done 后 162 mm 滑行段内 wCmd 恒 0（straight 条件里 forward=0 ⇒ 环被关掉），
//   于是 M0（有相电流环）与 M1（电压模式）自由衰减的差速无人管 → 枢轴 8.9°。
//   这段的差速需求 ≈ (v1−v0)·Rw/Lw = 9 rad/s×0.104 = 0.94 rad/s，比 holdMax=0.6 大。
float holdMaxCoast = 1.10f;// 滑行段专用限幅（在线 kc<>），按 vMeas 渐变到 0
// ★v2.9.0：settle 主动制动增益。done 后 forward 不再钉 0，而是给 -brakeK·vMeas，
//   两轮被主动反向驱动刹车 ⇒ 滑行窗从 0.5 s 缩到 ~0.1 s ⇒ 航向漂移窗口大缩。
//   在线 kb<>，0=关（退回 v2.8.0 自由滑行行为）。sq_kc3 实测 settle 漂 +7.6°/段。
float brakeK     = 0.50f;  // settle 主动制动增益（在线 kb<>）
bool  coastEnabled = false; // ★v2.9.2：J coast 开关（在线 jc0/jc1，默认 0=M0 电压模式对称不需要）

// ---------------- v2.4.0 定距移动：距离反算制动的梯形速度剖面 ----------------
// 为什么不用"定时加速→匀速→定时减速"：时间制的误差会 1:1 累积到落点
// （打滑、静摩擦、路面起伏全变成终点偏差）。这里每拍用【剩余路程】反算可用速度
//     v_brake = sqrt(2·a_dec·s_rem)
// 落点由积分而不是由时间决定：慢了就自动多给速度、提前滑不了就提前收油，
// 因此既能急启急停，又能保住落点精度。
// moveScale 把编码器里程换算成地面里程（实测 R_eff/R_w ≈ 1.019，见 3 组卷尺数据）。
float moveAcc   = 0.30f;    // 起动加速度 [m/s²]（受 S1 斜率封顶）
float moveDec   = 0.30f;    // 制动减速度 [m/s²]
float moveScale = 1.019f;   // 地面里程 / 编码器里程

// ---------------- ★v2.5.0 停车滑行补偿（治"停机枢轴偏转"）----------------
//   现象（d1m_C2，2026-10-18）：1 m 定距落点 105 cm、停机瞬间原地左转 20°。
//   机理（逐帧）：
//     t=3.438 vCmd=+0.237 wL=wR=+15.9 两轮都在前进
//     t=3.501 vCmd=-0.116 wL=wR= -7.7  ← 目标不是降到 0，而是【反向】
//     t=3.542 左轮 -12.9→ 还在跟，右轮 19.1→21.0（反而加速）
//     t=3.584 左轮过零停住(+0.28)，右轮还在 17.3
//   ⇒ 左轮先停、右轮继续推 ⇒ 绕左轮接地点枢轴左转，20° 的几何位移
//     = (Lw/2)·Δθ = 7.2 cm × 0.349 = 2.5 cm，正好是过冲 5 cm 的一半。
//   反向从哪来：moveS 到点 ⇒ forward=0，但 vMeas 还是 0.285
//     ⇒ PID_VEL 误差 -0.285 ⇒ 积分饱和到 -0.116 m/s（限幅 ±limit/3=±0.133）
//     ⇒ vCmd=-0.116 ⇒ wL=wR=-7.7 rad/s。归零指令变成了倒车指令。
//   两条修法：
//     ① 没有前进指令时不跑轮速外环、且不许把指令拉成负（见 loop 里 coasting）。
//     ② 刹车坡度必须用车真能做到的减速度。见下。
//
// ★★v2.5.2 退回"能力约束"，废掉"滑行量同拍反馈"（chk10 实测到的极限环）：
//   v2.5.0/1 的写法是 v_brake = sqrt(2·dec·(rem − v²/(2·decEff)))，把滑行量
//   当成同拍反馈扣进剩余路程。这是个【反阻尼】回路：
//     目标归零 → 车减速 → vMeasF 变小 → 扣掉的滑行量变小 → v_brake 又超过 vmax
//     → 目标回跳到 vmax → 车再加速……
//   chk10 里看得一清二楚：t=0.58 vc=0 → t=0.73 vc=0.175 → t=0.88 vc=0.274
//     → t=0.97 vc=0.029，10 cm 的位移最后走了 12.85 cm。
//   而且它治的病本来就是"坡度不可达"：dec 给 2.2 而车只能做 1.9，目标比车快，
//   在 rem=0 时目标已经归零而车还带着 0.11 m/s ⇒ 那 3.6 cm 是必然的。
//   正解：把坡度本身压到可达值 a_brake = min(moveDec, moveDecEff)。
//   坡度可达 ⇒ 目标降到 0 的那一刻车也正好停住，不需要任何提前量。
//   decEff 是"车真能刹住的减速度"，实测法：R<v> 指令归零后看速度衰减斜率。
float moveDecEff = 1.50f;   // 实测可达减速度 [m/s²]（在线 j<>，默认按 1.9 实测值打折）
// ★★ 2026-10-17 最终定案（手推 + 手转 90° 实测，PUSH_SIGN_TEST_2026-10-17.md）
//   实测三条：
//     ① 直线推动时 S1 回读 (v0,v1) = (+1.82,-1.80) —— 反号且等幅 ⇒ 两路编码器极性相反（镜像）
//     ② 原地左转 90°（车头摆向左侧）时 Δyaw = +1.51 rad，而 GyroZ = -0.27
//        ⇒ yaw 与 GyroZ 反号，且「左转 = gyro[2] 正」，故 GyroZ 必须取 +gyro[2]（见 getMPUValue）
//     ③ 闭环 t=+0.5 实测 GyroZ=+0.51 → 配合 ② 说明当时车是**右转** ⇒ turn>0 曾被整体反相
//   推导：记 s_i = 物理前进 / S1 回读（s1=+1 来自 ②的右轮前滚，s0=-s1 来自 ①的镜像），
//         ρ = S1 回读 / S1 目标（由 ③ 反解得 ρ=+1），e_i = s_i·ρ·M_iDir = 每轮"命令→物理前进"增益。
//         旧值 (1,-1) ⇒ e=(-1,-1) ⇒ v>0 后退、turn>0 右转（两者同时反 = 整车 180° 倒置，
//         所以偏航环"看着是通的"却整体反了，这正是「闭环通过但车往后跑」的成因）。
//         取 (-1,+1) ⇒ e=(+1,+1) ⇒ v>0 前进、turn>0 左转。
//   注意：翻 md 必须同时翻 GyroZ 符号，否则偏航环由负反馈变正反馈（md=-1,+1 那次飞车即此）。
int   M0Dir     = -1;
int   M1Dir     = +1;

// 轮速外环：输出是"车速修正量"[m/s]，限幅 ±0.4 m/s（防外环抢了内环的活）
// ★v2.5.1：limit 0.4 → 0.15。0.4 允许 vTrim 到 ±0.4 ⇒ vCmd 可达 0.7 m/s（vmax 的 2.3 倍）。
//   d1m_D 实测 vCmd 被顶到 0.44（+47%），而轮速其实只差 4.7% ⇒ 限幅收紧，只留打滑补偿。
PIDController PID_VEL{0.4f, 0.8f, 0.0f, 100.0f, 0.15f};

// 直接轮速模式（w 命令）
bool  directMode = false;
float directWL = 0.0f, directWR = 0.0f;

// ---------------- 腿：不控，上电摆固定站姿后不再碰 ----------------
static const uint16_t legPose[4] = {275, 168, 244, 178}; // 左前/左后/右前/右后

static void setLegsFixedPose()
{
    servos.setPWM(LFSERVO_CH, 0, legPose[0]);
    servos.setPWM(LRSERVO_CH, 0, legPose[1]);
    servos.setPWM(RFSERVO_CH, 0, legPose[2]);
    servos.setPWM(RRSERVO_CH, 0, legPose[3]);
}

// ---------------- I2C 从机探测 ----------------
// ★冷启动时序坑（2026-10-16 台架实测）：
//   外扩板与 MCU 同时上电，MPU6050 内部上电复位（POR）未完成时**不会 ACK 地址**。
//   固定时序 `Wire.begin() → delay(50) → i2cPresent()` 在 t≈60ms 探测会拿到
//   endTransmission()==2（地址 NACK），固件因此判定"IMU 未接"（状态帧 imu=0，
//   偏航率环被静默关闭）；同一硬件等到 t≈560ms 再探测就正常 ACK。
//   ⇒ 探测必须"重试"，不能一次性判定。
//   tries×gap 最坏耗时只发生在器件真的缺失时（正常情况第 1 次就返回）。
static bool i2cProbe(uint8_t addr, int tries = 5, uint16_t gapMs = 120)
{
    for (int i = 0; i < tries; i++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) return true;
        if (i + 1 < tries) delay(gapMs);
    }
    return false;
}

static bool i2cPresent(uint8_t addr) { return i2cProbe(addr, 1, 0); }

#ifdef DBG_IMU
// 诊断专用（只在 s3-usbdbg 下编入）：打印 I2C 探测的**原始返回码**
//   0=ACK  2=地址NACK  5=超时
// 这些探测本身会占用 I2C 总线，故只在诊断 env 里调用。
static uint8_t i2cRd(uint8_t addr, uint8_t reg)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xEE;
    if (Wire.requestFrom((int)addr, 1) != 1) return 0xEE;
    return (uint8_t)Wire.read();
}
static void dbgDump(const char *tag)
{
    Wire.beginTransmission(MPU6050_ADDR);
    uint8_t cM = Wire.endTransmission();
    uint8_t w  = i2cRd(MPU6050_ADDR, 0x75);
    Wire.beginTransmission(PCA9685_ADDR);
    uint8_t cP = Wire.endTransmission();
    Serial.printf("[DBG] %-14s t=%5lu  mpu_code=%u who=0x%02X  pca_code=%u  | imuOk=%d servoOk=%d\n",
                  tag, (unsigned long)millis(), cM, w, cP, imuOk ? 1 : 0, servoOk ? 1 : 0);
}
#endif

// ---------------- IMU 读取（轴映射沿用课程定稿） ----------------
// ★单位统一（2026-10-16 台架实测发现并修正）：
//   SF_IMU 的 gyro[] 单位是 **°/s**（raw/65.5，对应 ±500°/s 档），angle[]/angleAcc[] 是 **°**
//   （angleAcc = atan2(...)*360/2π）。而 bipedal_data.h 约定 robotposeparam 全部是
//   **rad / rad/s**，控制律与阈值（t<rad/s>、pt<rad>）也按 rad 设计。
//   老代码把 °/s 直接当 rad/s 用，差 57.2958 倍，后果有二（台架日志均已复现）：
//     ① 偏航率环：wCmd = turn + kt·(turn − GyroZ)，反馈被放大 57.3 倍 → 必振荡。
//        证据：静止时 GyroZ≈±0.2(°/s)，输出 wL/wR≈∓0.3 rad/s，正好 = kt·0.2·(Lw/2)/Rw。
//     ② 俯仰保护：pitchProt=0.6 本意 0.6 rad(34°)，实际按 0.6° 生效
//        → 停车时也反复触发 [PROT] output cut，且恢复门限 0.3° 几乎达不到。
//   故在此一次性转换，保证下游拿到的就是 rad / rad/s。
// ★2026-10-17 陀螺一阶低通（τ=40 ms，fc≈4 Hz）
//   动机：实测静止 GyroZ 噪声 σ≈0.16 rad/s、峰值可达 0.55；kt=0.6 放大后
//   在 wCmd 上叠 ±0.1 rad/s 抖动，而轮子有静摩擦死区 ⇒ 朝零偏方向的噪声脉冲越过死区、
//   反向的被死区吃掉，形成"噪声整流"，表现出来就是车一小步一小步地往一边蹭。
//   40 ms 在 1 Hz 处只有 ≈14° 相位滞后，对已验收的 Kp=0.6 环路无实质影响（环带宽 ~1 Hz）。
//   注意：只滤 GyroZ（控制用）；yaw 仍由 angle[2] 积分给出，不做二次滤波。
static const float GYRO_LPF_TAU = 0.040f;
static float    gyroZf      = 0.0f;
static uint32_t gyroZfUs    = 0;

static void getMPUValue()
{
    mpu6050.update();
    const float D2R = (float)DEG_TO_RAD;     // ° → rad
    robotPose.pitch = -mpu6050.angle[0] * D2R;
    robotPose.roll  =  mpu6050.angle[1] * D2R;
    robotPose.yaw   =  mpu6050.angle[2] * D2R;
    robotPose.GyroX =  mpu6050.gyro[1]  * D2R;
    robotPose.GyroY = -mpu6050.gyro[0]  * D2R;

    const float rawZ = mpu6050.gyro[2] * D2R;  // 该映射下：车头左转 → GyroZ 为正
                                               // ★2026-10-17 手转 90° 实测：原来写成 -gyro[2] 是错的，
                                               //   与 yaw(=+angle[2]) 反号，导致 left/right 整体颠倒。

    // 一阶低通：α = dt/(τ+dt)；dt 用实测差分（loop 约 2 kHz ⇒ dt≈0.5 ms）
    uint32_t nowUs = micros();
    float dt = gyroZfUs ? (float)(nowUs - gyroZfUs) * 1e-6f : 0.0f;
    gyroZfUs = nowUs;
    if (dt <= 0.0f || dt > 0.2f) dt = 0.0005f;   // 首拍或异常间隔兜底
    float alpha = dt / (GYRO_LPF_TAU + dt);
    gyroZf += alpha * (rawZ - gyroZf);

    robotPose.GyroZ = gyroZf;
}

// ---------------- v2.3.0 航向保持：状态与工具函数 ----------------
//   yawHold 只在「进入直行」的那一刻锁存，直行期间不变；一旦出现转向指令（|turn|≥阈值）
//   或停车（|forward| 归零）就释放，下次起步重新锁。
//   这样每次直行都相对「起步瞬间的车头方向」纠偏，不继承上一次的累积误差。
static bool  holdActive = false;
static float yawHold    = 0.0f;
// ★v2.9.4：hold 环 I 项积分器 [rad/s]。latch 新基准 / 退出 hold（转角环、普通指令、
//   停车）时清零；只在实际持有航向期间积分，settle 段冻结（防 windup）。
static float holdI      = 0.0f;
// ★v2.6.0：yawHold 是否"曾是有效值"。与 holdActive 分开 —— `s` 会清 holdActive（退出控制），
//   但 yawHold 的数值仍然有意义，下一段可以用 `x...,1` 沿用它（多段连跑时抵消停机枢轴的朝向误差）。
static bool  yawHoldValid = false;

// ---------------- v2.4.0 定距移动：状态 ----------------
static bool     moveOn    = false;
static float    moveTarget = 0.0f;    // 目标位移 [m]（地面里程）
static float    moveRem    = 0.0f;    // 剩余 [m]
static float    moveVmax   = 0.0f;    // 巡航上限 [m/s]
static float    moveV      = 0.0f;    // 本拍输出的前进指令 [m/s]
static float    moveS      = 0.0f;    // 已走 [m]（地面里程）
static uint32_t moveMicros = 0;
static uint32_t movePrintMs = 0;
static uint32_t moveStartMs = 0;
static float    moveStallMs = 0.0f;   // 末段低速停滞累计（防 0.4 rad/s 以下死不动的死锁）
static float    moveStuckMs = 0.0f;   // ★v2.5.1 卡死累计（指令大而车不动）
// ★v2.5.4 停车提前量：S1 轮速环是慢环（P=0.1/I=1.0，实测起步要 0.2 s 才追上指令），
//   所以 S3 把指令压到 0 时，车轮还在按 τ 之前的指令转 ⇒ 车必然多滑 v·τ。
//   这不是"可以靠反馈消掉的误差"，是一个确定的纯延迟 ⇒ 用纯前馈补偿：
//   在发起移动时按 vmax 一次算定 leadDist，从目标里直接扣掉（之后不再变）。
//   ⇒ 完全开环、无极限环风险（对比 v2.5.0/1 那个 v²/2a 同拍反馈）。
static float    moveLead   = 0.075f;  // S1 轮速环等效延迟 [s]（在线 e<>）
static float    moveTargetEff = 0.0f; // 扣掉 leadDist 后的有效目标 [m]
static bool     moveBraking   = false;// 已进入制动段（指令只许降不许升）
static bool     moveSettleOn  = false;// done 后继续积分到停稳（校准 moveLead 用）
static float    moveReqDist   = 0.0f; // ★v2.7.0 本次 x 的请求距离（原地对齐后才 beginMove）
static bool     moveKeepYaw   = false;// ★v2.7.0 本次 x 是否沿用上一段航向

// ---------------- ★v2.7.0 原地转（gyro 闭环）----------------
//   M085 实证：`keep` 若在【行进中】把 10.5° 转回来，轨迹是一段圆弧：
//   横偏 ≈ L·Δθ/2 = 1.0×0.183/2 = 9.2 cm（实测 8.6 cm）⇒「边走边纠」不可用。
//   正解：起步【前】先原地转到基准朝向（两轮反向 ⇒ 绕轮轴中点自转，平移≈0），再走直线。
//   同一个通道也用作 `t<deg>` 原地转固定角度（1m×1m 的 90° 角）。
// ---------------- ★v3.0.2 松闸（手推标定用）----------------
//   背景：S3 每 50 ms 下发一次 `R0,0`，S1 速度环把「目标 0」执行成抱住不放，
//   DRV8313 零电压时三相下桥全通 = 电磁刹车 ⇒ 手推推不动（10-18 夜间实测：
//   推 18 s 编码器只积 0.16 m、陀螺一动不动）。S1 的 `J` 能 disable 掉，
//   但 50 ms 后就被 R 重新使能 ⇒ 只能让 S3 自己停发 R。
//   `z` 在线切换：仅在【完全静止无任务】时生效，任何运动指令自动解除（不会误松闸跑车）。
static bool     standbyRelease = false; // true = 空闲时保持松闸（J），不心跳 R
static bool     rotOn        = false;   // 原地转进行中
static float    rotTarget    = 0.0f;    // 目标绝对 yaw [rad]
static float    rotRateMax   = 1.0f;    // 角速度上限 [rad/s]
static uint32_t rotStartMs   = 0;
static bool     rotThenMove  = false;   // 转完是否接着跑刚才那条 x
static const float ROT_KP        = 2.50f;   // 角度环 P [1/s]
static const float ROT_KD        = 0.40f;   // 陀螺阻尼
static const float ROT_TOL       = 0.0175f; // 1.0° 到位判据
static const float ROT_GYRO_TOL  = 0.15f;   // rad/s，还要"停得住"才算到位
static const float ROT_MAX_MS    = 6000.0f; // 超时保护

static float wrapPi(float a)
{
    const float PI2 = 6.28318530718f;
    while (a >  3.14159265359f) a -= PI2;
    while (a < -3.14159265359f) a += PI2;
    return a;
}

// ---------------- ★v2.7.0 真正开始一段定距移动 ----------------
//   抽出来是因为 `x...,1` 可能要先原地对齐（rotOn），对齐结束才回来调这里。
//   参数全部取自 move* 全局（moveReqDist / moveVmax / moveAcc / moveDec /
//   moveScale / moveDecEff / moveKeepYaw），所以调用点只负责赋值。
static void beginMove()
{
    const float dd = moveReqDist;
    // 路程不够加速到 vmax 时，峰值被压成三角剖面
    float vTri = sqrtf(2.0f * moveAcc * moveDec * dd / (moveAcc + moveDec));
    const float vPeak = fminf(moveVmax, vTri);
    // 停车提前量（纯前馈，一次算定）：见 v2.5.4 注释
    const float aBrake0 = fminf(moveDec, moveDecEff);
    float leadDist = vPeak * moveLead + 0.5f * aBrake0 * moveLead * moveLead;
    if (leadDist > 0.5f * dd) leadDist = 0.5f * dd;      // 短距离不许扣成负的

    moveTarget    = dd;
    moveTargetEff = dd - leadDist;
    moveRem       = dd;
    moveS         = 0.0f;
    moveV         = 0.0f;
    moveBraking   = false;
    moveSettleOn  = false;
    moveMicros    = micros();
    movePrintMs   = 0;
    moveStartMs   = millis();
    moveStallMs   = 0.0f;
    moveStuckMs   = 0.0f;
    moveOn        = true;
    directMode    = false;
    PID_VEL.reset();

    if (moveKeepYaw) {
        holdActive = imuOk;                  // yawHold 已在原地对齐时确认过，不动
    } else {
        yawHold      = robotPose.yaw;        // 立刻锁存，别等 v 爬到 5 mm/s 才锁
        yawHoldValid = true;
        holdActive   = imuOk;
    }
    Serial.printf("[MOVE] dist=%.3f vmax=%.3f acc=%.2f dec=%.2f scale=%.4f decEff=%.2f lead=%.1fmm keep=%d  (峰值 %.3f)\n",
                  dd, moveVmax, moveAcc, moveDec, moveScale, moveDecEff,
                  leadDist * 1000.0f, moveKeepYaw ? 1 : 0, vPeak);
}

// ---------------- ★v3.0.0 几何链执行器（跑任意 LINE/ARC 链）----------------
//   动机：正式场地 37.02 m / 35 段，其中 24 段是圆弧、还有 3 处不相切的 90° 尖点。
//   旧命令只有「定距直线 x」和「原地转 T」，弧只能靠 v+t 手给却无法按角度自动终止；
//   若由上位机逐段下发，20~100 ms 的串口空档在 0.5 m/s 下就是 1~5 cm 的盲走。
//   ⇒ 整条几何链一次下发到固件，固件自跑，段间无缝（不停车、不重锁航向）。
//
//   三种段（大写，与已有的小写 v/t/x 不冲突）：
//     LINE  L<len>,<v>[,<acc>]       按里程终止（vMeas·scale 积分）
//     ARC   A<deg>,<v>,<R>[,<acc>]   按 Δyaw 终止（陀螺累计），turn = sign(deg)·v/R
//     PIVOT P<deg>[,<rate>]          原地转（复用 T 的角度环），用于不相切尖点
//
//   ★ 关键设计：理想航向 pathIdealYaw 绝对累加（每段结束 += 设计转角）。
//     直线段的 yawHold 钉在 pathIdealYaw（不是实测 yaw）⇒ 单段的角度误差不会
//     被下一段当成新基准吸收，误差不随段数累积。
//     （方框时代 v2.9.3 用 yawHold=实测 yaw，那是为了吸收 settle 段尾漂移；
//       链里段间不停车、没有 settle 漂移，所以理想值更优。）
//
//   ★ 控制律零改动：直线段 turn=0 ⇒ 自动进 hold 环；弧段 turn≠0 ⇒ 自动走偏航率环；
//     PIVOT 段 rotOn ⇒ rateMode 接管。三条通道本来就在主循环里，不需要新增控制器。
// ================= v3.1.0 七路灰度（荣洋 RYDZ 无 MCU 数字版）=================
//   极性（2026-10-19 现场实测）：本颗为【白线版】—— 白 → 低电平，黑 → 高电平。
//        与荣洋"黑线→低电平"的资料相反；`yi` 可在线切换极性，不必重烧。
//   模块为开漏，靠 IO 内部上拉到 3.3V。
//   ⚠ 若模块 OUT↔VCC 之间实测有 ~10k 上拉电阻（=5V 推挽），必须加电平转换，见接线说明。
//   ★本版【只读不控】：仅采样 + 打印，不参与 wCmd，控制律零改动。
//
//   ★★引脚（用户实接，2026-10-19 定案）：
//        OUT1..7 = IO14 / IO47 / IO38 / IO40 / IO41 / IO21 / IO48
//   逐脚可用性（ESP32-S3）：全部为普通 IO，无 strapping、无 PSRAM 占用、无 JTAG 冲突。
//        （IO14✅ IO47✅ IO38✅ IO40✅ IO41✅ IO21✅ IO48✅）
//   已避开的坑：IO45 strapping(VDD_SPI)、IO35/36/37 PSRAM-D4~D7、IO4 RGB 灯、IO9 蜂鸣器、
//        IO1/2 I2C、IO17/18 板间串口、IO19/20 USB、IO42 舵机使能、IO43/44 UART0。
#define GRAY_N 7
static const uint8_t GRAY_PIN[GRAY_N] = {14, 47, 38, 40, 41, 21, 48};  // OUT1..OUT7（车头看 左→右）
static uint8_t  grayBits  = 0;      // bit0=OUT1 … bit6=OUT7，1=压线
static uint8_t  grayCnt   = 0;      // 压线路数
static float    grayE     = 0.0f;   // 横向偏差 [mm]，正=线在车左；全白时=0
static float    grayPitch = 13.4f;  // 相邻灯珠中心距 [mm]（yp 改；实测灯距 13.4，装机高度 9.45 mm）
static float    grayZero  = 0.0f;   // 零点 [mm]（ye 标定，车压中线时做一次）
static bool     grayDbg   = false;  // d2：20 Hz 灰度帧
// ★★极性（2026-10-19 实测）：本模块为【白线版】—— 白 → 低电平，黑 → 高电平
//    grayInv=false ⇒ LOW 视为压线（黑底白线的【正式场地】）
//    grayInv=true  ⇒ HIGH 视为压线（白底黑线的【测试场】）
//    ★v3.7.1 默认改回 false = 正式场地黑底白线。回测试场（白底黑线）改这一行重烧，
//      或者临时用 `yi` 在线切换（只存 RAM，掉电还原）。
//    ★判据：站在【底色】上必须 n=0,e=0；底色就 FULL ⇒ 极性反了，别急着跑。
static bool     grayInv   = false;

// ---- ★v3.2.0 巡线（灰度 → 航向串级）----
//   为什么串级而不是把 Δω 直接叠到 wCmd：直行段跑的是 hold 环，它的 I 项（ki4.0）
//   会把任何恒定 Δω 积分掉。所以灰度只改【目标航向】，P/I/D 全部照旧跟这个目标。
//   e>0（线在车左）⇒ 目标航向左偏 ⇒ 车左转去追线。
static bool     lineOn    = false;  // yf：巡线闭环开关
static float    lineKp    = 1.20f;  // 横向 mm → 角度 rad 的增益（乘在 atan(e/LD) 上）
static float    lineLD    = 0.20f;  // 前瞻距离 [m]（yl 改，默认 200 mm）
static float    lineSign  = 1.0f;   // ★ys±1：传感器装在轮轴前=+1（正反馈则改 -1，见下）
//   ★2026-10-19 实测：传感器若装在**轮轴后方**（或整条装反了 180°），"线偏左就左打"会变成
//     正反馈 —— 左打 ⇒ 尾部传感器看到的线更偏左 ⇒ 继续左打 ⇒ 原地画圈。
//     现场判据：给了左打而 e 一直停在 +33 不动（bits 恒 1100000）= 正反馈 ⇒ 用 ys-1。
static float    lineE     = 0.0f;   // 低通后的横向偏差 [mm]
static bool     lineValid = false;  // 当前 e 是否可信（n∈[1,3]）
static uint32_t lineLostMs = 0;     // 连续丢线时长 [ms]
static float    lineW     = 0.0f;   // 最近一次算出的航向修正 [rad]
// ★★v3.2.4 转角（垂直横线）检测。用户 10-19 给出场地事实：
//   测试场 1 m × 1 m，黑线 ~1 m；**前进中若 OUT3~OUT7 同时压线 ⇒ 那是与行进方向 90° 的横边（转角）**，
//   既不是噪声也不是"宽线饱和"。实测跑5 的 1111111 段就是这个（当时车撞在障碍上）。
//   bits & 0x7C == 0x7C ⇔ i=2..6 全 1 ⇔ OUT3/4/5/6/7 全压。
static bool     lineCorner = false;
// ★★v3.6.0 横边【左右对称】检测 + 方向自适应。为什么必须对称：
//   旧判据只有 `(bits & 0x7C) == 0x7C`（OUT3~OUT7 黑），而右转时的新线在车【右】侧、
//   阵列右半边先压黑 ⇒ `0011111` 命中；左转时新线在车【左】侧 ⇒ `1111100`，
//   bits5/6 是 0，**永远不满足旧判据** ⇒ 左转根本检测不到，只会一路直冲出去。
//   几何依据：阵列跨度 80 mm（6×13.4），转角处新线只覆盖阵列的一半，剩下一半还压在
//   老线/白底上 ⇒ "哪一端先见白，横边就在另一侧"。
//     `0011111` / `0111111`  ⇒ 左端白 ⇒ 新线在右 ⇒ 右转
//     `1111100` / `1111110`  ⇒ 右端白 ⇒ 新线在左 ⇒ 左转
//   `1111111`（七路全黑 = 压出场地/大片深色）两侧都排除，不再被误判成转角（run30 撞车原因）。
static uint8_t  lineCornerCntR = 0;   // 右侧横边连续拍数（5 ms 一拍，≥4 拍=20 ms 才认）
static uint8_t  lineCornerCntL = 0;   // 左侧横边连续拍数
static int8_t   lineCornerDir = 0;    // +1=左转 / -1=右转 / 0=未定（d2 帧第 12 列）
// ★★v3.2.5 转角自动转（`yc<deg>`，0=只停）：命中 90° 横边 ⇒ 原地转到指定角 ⇒ 低速找线 ⇒ 恢复巡线。
//   目的：让小车自己把整圈走完，再用【每段长度 + 每个转角】反推场地形状。
static float    lineCornerDeg = 0.0f;   // 固定转角 [deg]，正=左转、负=右转（yc 改；auto 关时才用）
// ★★v3.6.0 方向自适应：`yr<deg>` 打开，转角量级 = |deg|（默认 90），方向由 lineCornerDir 定。
//   这样同一套参数可以跑"左转链""右转链"甚至"左右混合"的场地，只要求每个弯是直角。
static bool     lineCornerAuto = true;  // 方向自适应（默认开）
static float    lineCornerMag  = 90.0f; // 自适应模式下的转角量级 [deg]
static float    lineCornerUse  = 0.0f;  // 本次转角实际下发的角度（带符号，已定方向）
static bool     lineAfterTurn = false;  // 原地转结束 ⇒ 该恢复巡线
static bool     lineReacq     = false;  // 转角后低速爬行找新线
static uint32_t lineReacqMs   = 0;
static uint8_t  lineReacqHit  = 0;      // 连续有效帧计数（要 3 帧才认，防边缘假捕获）
// ★v3.2.6 横向积分：把 e 积分进 yawHold，使【航向基准】自己转到与线平行。
//   为什么必须它：转角后是"带着角度"捕获新线的（偏 25°），纯 P 的稳态偏置
//   e_eq = LD·tan(θ_err/kp) ≈ 84 mm > ±40 mm 检测跨度 ⇒ 必然再次丢线。
static float    lineKi    = 0.90f;  // yawHold += lineKi·(e/1000)·dt  [rad/s per m]
static uint32_t lineCornerMs  = 0;      // 上次转角时刻（1.2 s 冷却，防同一条横边连触发）
static bool     lineCornerArm = true;   // ★转角"上膛"：必须先看到非横边一次才允许再触发
static float    lineRunV      = 0.30f;  // 段内巡航速度（转角时记住，转完恢复用）
// ══ ★★v3.10.8 出弧自动提档（用户 2026-10-20 需求）══
//   需求："出了六段弧之后把速度提到 0.40"。
//   为什么用【里程 + 前馈已卸】双门，而不是"数第几个弧"或"记时间"：
//     ① 里程是唯一能和赛道几何对上的量：L1 2.30 + L2 0.60 + R1~R6（6×π×0.2=3.77）
//        = 6.67 m。转角原地转时 |vMeas|≈0（两轮反向），不会虚增。
//     ② 单看里程会被打滑骗：弧上内侧轮打滑让编码器虚高约 3%，600 mm 的弧就能吃掉
//        ±20 mm ⇒ 阈值附近可能还在第六个弧上。所以再要求**此刻 lineArcFF 已卸到
//        <0.30**（真的在直道上、不在弧上）。弧上前馈是 1.2~1.4，卸到 0.30 必是出弧之后。
//     ③ 两门同时满足还要【连续 500 ms】才算，避免弧中间的瞬时抖动。
//   提档只改 lineRunV / robotMotion.forward，增益不动 —— 用户要的是"直道快点"，
//   不是"弧上快点"；0.40 走 R200 需要 2.0 rad/s 已顶到 turnMax，弧段绝不能提。
// ★v3.13.2 默认开启提档（用户 10-21 09:59 要求"0.28 起跑、7.36 m 才提档到 0.40"）：
//   门限保持 6700 mm 不动 —— 实测 7.36 m 这个时刻是【6700 门限 + 出弧条件】共同决定的：
//   R7（右 90°，R500，7.170~7.955 m）起点附近 arcff 还没学起来（<0.30），门才放行，
//   所以观测到提档发生在 odo≈7.36 m。把门限直接改成 7360 反而会推到 R7 出弧后（>8 m）。
static float    lineV2      = 0.40f;    // 第二档巡航速度 [m/s]（yv<v> 设；0=关闭）
static float    lineV2Odo   = 6700.0f;  // 提档里程门限 [mm]（yvd<mm> 改）
static float    lineOdoMm   = 0.0f;     // 本次巡线累计里程 [mm]
static bool     lineV2Done  = false;    // 已提档
static uint32_t lineV2Ms    = 0;        // 双门连续满足的起始时刻（0=未满足）

// ══ ★★★v3.10.11 十字识别（治"把十字当转角、在十字原地绕圈"）══
//   用户 2026-10-20 观察：跑完六弧后场上有两个十字（L5×L6 @(1562,1830)、L8×L6 @(1562,1220)），
//   车到那里反复 +90° 原地转，绕不出去，本该一直直行。
//   实测 run g3110（v3.10.8，0.40 m/s）三次误判，全部是"左侧横边⇒左转"：
//     44.6 s  bits=1111110 n=6  yaw=429°   54.3 s 1111000 n=4 yaw=781°   63.7 s 1111000 n=4 yaw=1139°
//     （每次 +90°，三段各 10 s —— 就是"在十字上绕圈"）
//   ★物理原因：十字和直角的灰度签名【完全一样】。线在车底横过来，整条阵列压白；
//     `1111110`(十字) 和 `1111100`(A1 真直角) 在 n、|e|、连通性上无法分开。
//   ⇒ 判据只能换成"之后会怎样"（几何事实，不依赖阈值标定）：
//     ① 十字：继续直行 → 横向那条线只有 ~20 mm 宽，50~100 ms 后阵列恢复 1~3 路（线继续直走）
//     ② 直角：继续直行 → 线已经拐走，阵列变全黑 n=0
//   所以横边命中后【不立刻转】，先"边走边看"最多 XC_WATCH_MS：
//     · 出现 1~3 路有效 ⇒ 判十字，撤销转角，继续循迹（并清上膛，同一条横线不再触发）
//     · n=0 连续 XC_LOST_N 拍 ⇒ 判直角，按原流程转，L_f 减去观察窗口里已经走掉的里程
static bool     lineXcWatch = false;    // 正在"边走边看"验证横边
static uint32_t lineXcMs    = 0;        // 观察窗口起点 [ms]
static float    lineXcDist  = 0.0f;     // 观察窗口内已走里程 [m]
static int8_t   lineXcDir   = 0;        // 候选转角方向（确认后才用）
static uint8_t  lineXcLost  = 0;        // 窗口内连续 n=0 拍数（5 ms 一拍）
static uint8_t  lineXcGood  = 0;        // ★v3.10.13 窗口内连续"线在中心"拍数
// ★★★v3.10.13 实测反例（run _r31012_1，A1 真左直角）：
//   横边命中 n=5 → 仅 70 ms（≈20 mm，连横边都没穿完）就出现 n=3 e=+13.4 ⇒ 被误判成十字 ⇒
//   直行冲出赛道 → 搜线乱转成右转 → 全程崩。成因：①只要求"过 40 ms 且 n 恢复"，单帧残余即可触发；
//   ②没有区分"线回到中心"和"横边边缘残余"。十字的纵向线就在车正下方 ⇒ |e| 必须小且【持续】；
//   直角残余来自横边一角，|e| 常在 13~20 mm 且下一拍就全黑。
//   ⇒ 判十字改为：xt≥XC_MIN_MS 且 n∈[1,3] 且 |e|≤XC_GOOD_E 且【连续 XC_GOOD_N 拍】
#define XC_WATCH_MS   300               // 观察窗口上限 [ms]（0.40 m/s ⇒ 120 mm；正常 60~80 ms 就判完）
#define XC_MIN_MS      80               // 至少看这么久才允许判"十字"（v3.10.12 为 40，太早）
#define XC_GOOD_N       4               // 连续 4 拍（≈20 ms）线在中心 ⇒ 才算十字
#define XC_GOOD_E     7.0f              // 判"线在中心"的 |e| 上限 [mm]（一格 = 6.7 mm）
#define XC_LOST_N       3               // 连续 3 拍（15 ms）n=0 ⇒ 直角
#define XC_LOST_MIN_MS 20               // n=0 计数也要过这么久，防命中瞬间的抖动
// ★★v3.3.0 转角几何正解：灰度条中心到【轮轴中点】的纵向距离 L_f。
//   转角的原地转绕轮轴中点进行，而传感器在车头前 L_f 处 ⇒ 若一到横边就转，
//   转 90° 后传感器必然横移 L_f（约 100 mm），远超 ±40 mm 视场，且与新线平行
//   ⇒ 纯前爬永远不收敛（这正是 v3.2.5 要"斜着咬 25° + 摆头"才勉强接上的原因）。
//   正解：检测到横边后先【继续前进 L_f】让轮轴落到转角点上，再原地转 90°，
//   此时传感器正好落在新线上，横移 = 0，直接续上巡线。
static float    lineFwdMm    = 66.0f;   // L_f [mm]（ylo 改）
// ★★实测标定（10-19 run11）：先设 100 mm，转完读数 e=-33.5 mm ⇒ 走过头 33.5 mm
//   ⇒ 真值 L_f ≈ 100-33.5 = 66.5 mm，取 66。改准后转角接续应落在 e≈0。
static bool     lineAdvancing = false;  // 转角命中后：正在前进 L_f
static float    lineAdvDist   = 0.0f;   // 该阶段已走里程 [m]
static float    lineAdvGoal   = 0.066f; // ★v3.10.11 本次 L_f 目标 [m]（扣掉十字观察窗已走的里程）
static uint32_t lineAdvMs     = 0;
static bool     lineJustTurned = false; // 刚转完：给一段容忍丢线的窗口
static uint32_t lineJustTurnedMs = 0;

// ══ ★★v3.4.0 直接舵量通道（用户 10-19 指出：经典循迹是"跟着模块直接转弯"）══
//   旧架构（lineMode=0）把横向偏差折成【目标航向】再交给航向环：
//       e → lineW(≤11.5°) → yawTgt → wCmd = kh·eYaw − kd·GyroZ
//   中间多了"位置→角度"的一层积分：**横向速度（ė）完全不进环路**，
//   e 只在积分之后才变成转角 ⇒ 起步若与线差 θ，等 e 长到能产生足够 wCmd 时就出视场了。
//   实测铁证（10-19 run12/run13 两次复现）：e 在 0.45 s 内从 −6.7 匀速滑到 −40 丢线，
//   同期 yaw 只动 3°、两轮等速 ⇒ 车直着走、线横向滑出（θ≈13°），wCmd 最大只到 0.32 rad/s。
//   经典循迹（网上通用做法：left=base+PID, right=base−PID）是直接把偏差当【舵量】：
//       turn = Kp·e + Kd·ė        （单位 rad/s，直接进偏航率环，内环还有陀螺反馈做阻尼）
//   ⇒ 位置误差和横向速度同时进环，13° 偏斜一眼就看见（ė≈0.078 m/s ⇒ Kd·ė≈0.6 rad/s）。
//   ★lineMode=1 时：不使用 yawHold / lineKi / hold 环，wCmd 直接被舵量覆盖。
static uint8_t  lineMode    = 1;      // 0=航向串级(旧, ym0) / 1=直接舵量(经典, ym1)
static float    lineKpD     = 24.0f;  // 直接舵量 P  [rad/s 每 m 横偏]
static float    lineKdD     = 9.0f;   // 直接舵量 D  [rad/s 每 (m/s) 横移速度]（★v3.8.6 4→9：ζ 0.22→0.49，run50 存活 7.3 s 最好；13 又会被量化尖峰打死）
// ★★v3.8.2 大偏差增速（非线性 P）：|e| 超过 lineEboost 后，斜率从 Kp 换成 Kp+Kboost。
//   为什么必须要它：R=200 mm 的弧在 v0.28 下要 1.40 rad/s 舵量，纯 Kp24 得靠 e=63 mm 去换，
//   而 7 路灰度视场只有 ±40 mm ⇒ 位置环"还没欠够舵量，线就已经出视野"。
//   run44 实测：入弧 0.55 s 内 e 0→40.2 mm（直线穿越 R0.35 m 弧），steer 追到 1.19 但为时已晚。
//   加上 Kboost 后：w=1.40 只需 e≈21 mm，仍在视野内。
//   ★★v3.8.3 两个关键修正（run45 实测得出）：
//     ① 触发量必须用**原始 grayE**，不能用 lineE。lineE 是 6.7 mm 台阶的低通值，
//        实测滞后 ~0.1 s：t=0.295 时 grayE=20.1 而 lineE 才 10.0 ⇒ 增速要到 grayE=26 mm
//        才真正出力，那时满舵只剩 0.2 s 可用，e 已经在 40 mm 视场边缘。
//     ② 阈值压到 6.8 mm（= 1 个量化台阶，即 6.7 mm 之上）。R=200 的弧从入弯到丢线
//        只有 0.45 s，唯一的早期信号就是"偏了 1~2 档"，等不到更大偏差。
//        直道稳态是 0/±6.7 两档 ⇒ 阈值 6.8 正好落在台阶上沿，不会误触发。
static float    lineKboost = 60.0f;   // 大偏差额外斜率 [rad/s 每 m]（yq 改，0=关）
static float    lineEboost = 0.0068f;  // 增速触发阈值 [m]
// ★★v3.8.5 增速贡献的**上限**。run48 实测：不封顶时 e=13.4 mm 就把舵量砸到 1.60，
//   而 R200 的弧只需要 1.40 ⇒ 每次纠偏都过头，车在 ±33~40 mm 之间越摆越大，5.3 s 后丢线。
//   封在 0.90 后总舵量 = P(≈0.48) + 0.90 + D(≈0.24) − trim(0.12) ≈ 1.50 @ e=20 mm，
//   刚好够 R200；而 e≥30 mm 时 P 自己就把总量顶到上限，仍能全力救回来。
static float    lineBoostMax = 0.90f;
#define LINE_BOOST_FFMAX 0.80f          // ★v3.10.9 |arcff| ≥ 它就不再给 boost（前馈已接管，防止重复补偿）
// ★v3.8.2 全局镜像：实测前进速度（m/s）。lineUpdate() 里要用它判"车到底有没有动"，
//   而 vMeas 是 loop() 后半段的局部量 ⇒ 这里存一份，loop() 每帧赋一次。
static float    gVMeas = 0.0f;
// ★★v3.9.12 舵量限幅 1.60 → 2.00。原因：这个限幅隐含了一个"最小可跑半径" R_min = v/lineTurnMax，
//   它是**与速度强相关**的（增益归一化只能保证 e_ss 与速度无关，管不到限幅）：
//     v=0.28 ⇒ R_min=0.175 m（R342 只需 0.82 rad/s，永远够）
//     v=0.40 ⇒ R_min=0.250 m（够）
//     v=0.60 ⇒ R_min=0.375 m ⇒ **R342 需要 1.754 rad/s > 1.60，直接被砍死在限幅上**
//   0.6 m/s 下 R342 需 1.75、R392 需 1.53（只剩 4% 余量），所以抬到 2.00（R_min=0.30 m）。
//   代价：极端偏航时舵量权限更大 ⇒ 依赖丢线保护兜底；wCap 在高速下为 2(v−0.15)/0.144=6.25 远大于此，不构成二次钳位。
static float    lineTurnMax = 2.00f;  // 舵量限幅 [rad/s]（v3.9.12 1.60→2.00，配合 0.6 m/s 提速）
static float    lineEDot    = 0.0f;   // ė 低通值 [m/s]
static float    lineEPrev   = 0.0f;   // 上一拍 e [m]
static float    lineSteer   = 0.0f;   // 直接舵量输出 [rad/s]（d2 帧第 9 列）
// ★★v3.5.0 弯道稳态滞后（I 项）：纯 P 环在弧段上**必然**要"欠一个固定偏差"才能换出常数舵量。
//   实测 run24 弧段：v=0.287 m/s、gz=−0.618 rad/s ⇒ R=0.46 m，需要舵量 ω=v/R=0.62 rad/s；
//   纯 P 要输出 0.62 就得 e=ω/Kp=0.62/18=34.4 mm，实测 e 恰好恒在 −33.5~−40.2（完美吻合）。
//   这不是故障、也不是打滑，是位置环的固有跟踪滞后 ⇒ 只能把这段常数舵量存进 I，e 才回得到 0。
//   用户判据"黑线常驻 4 号（|e|≤6.7 mm）"在弯道上只有 I 项能做到。
// ★★v3.6.0 I 项重做（治 run25 的过冲）：把"手动 trim"和"学习出来的偏置"拆成两个变量。
//   为什么必须拆：`yb` 写进去的是**标定好的常值**（车/地面/速度一起决定的左漂补偿，
//   -0.116 rad/s），而 I 项学出来的是**随路段变化的量**（弧段要额外 0.2~0.5 rad/s，
//   直道要 0）。共用一个变量的话，I 一学就把标定值冲掉了，跑完弧段进直道还带着弧。
//   旧版 Ki=12 实测过冲链：e −33.5 →（I 猛灌 0.4 rad/s²）→ −6.7 → **+26.8** → 丢线。
//   根因不是"Ki 大"本身，是**输出已经顶格还在积分**（无抗 windup）。
//   ⇒ v3.6.0：条件积分 + 限幅 + 每次转角/重捕把学习量清零（新路段重新学）。
static float    lineKiD     = 3.0f;   // I 增益 [rad/s 每 (m·s)]（★run39 定 3：弯道静差 20 mm 收敛约需 5~8 s；12 会过冲）
static float    lineTrim    = -0.116f; // 手动常值补偿 [rad/s]（yb 写这里，不被 I 覆盖）
//   ★★v3.7.0 固化为 -0.116：run27/run33/run35 稳定段实测的左漂补偿量，把 P 环平衡点搬到 e=0。
//      换地面/换巡线速度必须重标（地面变了这个值就不对，表现为直道常驻偏一侧）。
static float    lineBias    = 0.0f;   // I 学习出的附加偏置 [rad/s]（d2 帧第 11 列）
static float    lineBiasMax = 1.00f;  // 抗饱和（★v3.8.0 0.70→1.00：0.70 只够 R0.48 m，正式场 R0.20 需 1.5）
// ══ ★★v3.8.0 曲率前馈（治"小半径圆弧拐不过去"）══
//   实测 run41（L2→R1，R=200 mm，v=0.28）：入弯 0.5 s 内 e 就从 0 顶到 40.2 mm、wCmd 顶到
//   wCap 1.381，然后丢线。把 R=200 的账算清楚就知道这是**结构性**做不了：
//     ① 需要舵量 ω=v/R=1.40 rad/s；wCap=1.381（wDipMin=12 钳的）、lineTurnMax=1.30 ⇒ 都差一点
//     ② 就算把限幅放开，纯 PD 要输出 1.40 得靠 e：24·e−0.116=1.40 ⇒ e=63 mm > ±40 mm 视场
//     ③ I 项也来不及：Ki=3 在 e=40 mm 时只给 0.12 rad/s²，填 1.4 要 12 s，而 R1 只持续 1.5 s
//   ⇒ 位置环天生无法在 1.5 s 内"欠出"一个 1.4 rad/s 的常值舵量，必须给前馈。
//   物理依据：**稳态圆弧上，"维持该曲率所需的舵量"就等于实际偏航率 GyroZ**。
//   用慢低通把 GyroZ 学成 lineArcFF，位置环就只负责纠偏，不必再欠偏差换舵量。
//   ★为什么用 0.45 s 低通而不是直接赋值：直接相加会形成 steer=PD+GyroZ、GyroZ≈steer 的
//     单位增益正反馈自激；低通把环路增益压到 1 以下，等价于"学曲率"而不是"抄自己"。
static float    lineArcFF    = 0.0f;   // 曲率前馈 [rad/s]（d2 帧新增列）
static float    lineGyroF    = 0.0f;   // ★v3.10.9 偏航率低通（τ=0.25 s），arcff 的学习源
                                       //   真弧 GyroZ 恒同号 ⇒ 低通后 ≈真值；摆荡正负各半 ⇒ ≈0
static float    lineArcFFMax = 1.80f;  // 前馈限幅 [rad/s]（yw 改；0=整条前馈关闭，用于 A/B）
                                       //   ★v3.10.10 1.60→1.80：K 提到 1.15 后 R200@0.28 的
                                       //   目标值 1.15×1.5≈1.72 会被 1.60 削掉，等于白提。
static float    lineArcTauUp = 0.20f;  // 上升时间常数 [s]（入弯要快，否则还没学够就丢线）
static float    lineArcTauDn = 0.18f;  // 衰减时间常数 [s]（★v3.10.7 0.40→0.18：切点必须能在 0.4 s 内跟到反向）
static float    lineArcK     = 0.90f;  // ★★★v3.10.15 回退 1.15→0.90：回到 v3.10.9 基线
                                       //   （用户 10-20 评价"六段弧处理非常好"就是 0.90 时的实测）
                                       //   1.15 本是按执行增益 0.78 反算的补偿，但它打在【学习源】上，
                                       //   会让 arcff 稍微超前于真实所需 ⇒ 与 PD 叠出过冲。先回基线，
                                       //   若六弧 e 稳定停在 +20 mm（=执行增益缺口）再单独上 1.15 验证。
                                       //   ★实测（run g3112 14.4 s）：steer=1.77 时真实偏航率只有
                                       //   1.372 rad/s ⇒ 增益 0.78。arcff 只学到 1.348，缺口
                                       //   0.49 rad/s 必须由 PD 出 ⇒ Kp·e=0.48 ⇒ e=0.48/24=20.0 mm。
                                       //   **这就是弧上死死咬住 +20.1 mm 的真身**（不是"跟不住"，
                                       //   是学习系数没补执行增益造成的固定静差）。
                                       //   取 1.15 ⇒ 环路增益 1.15×0.78=0.90<1（留 10% 防自激），
                                       //   残差 (1/0.78−1.15)×1.37=0.19 ⇒ e_ss=7.8 mm ≈ 一个量化台阶。
#define LINE_ARC_VMAX 0.080f            // ★v3.10.7 允许学习前馈的最大横移率 |ė| [m/s]
                                        //   稳态跟线 ≈0；摆荡时 6.7→20.1 mm/0.05 s = 0.27 ⇒ 可分离

// ══════════════════════════════════════════════════════════════════════════════
// ★★★v3.13.0 四圆区（R16~R19）附加模块 —— 与前面赛段完全解耦
//   c4On==false 时本模块【一个变量都不动】，即等价于 v3.10.16 的原行为。
//   用户 10-20 21:39 定的路线：灰度照常巡线，IMU 认出转过 180° 才干预一次，共三段。
//   本模块只做两件事：
//     ① 每转过 ~180° 把【曲率前馈 lineArcFF】瞬间翻号（v3.13.0 定：翻号取代"强偏 17°"）
//     ② 翻号后把前馈钉住 = ±v/R×1.15，让 PD 只负责横向微调
//   为什么是"翻号"而不是"强行偏一段"（2026-10-20 21:55 量化复盘）：
//     · 强偏 = 开环注入 17° 多余航向，几何并不需要；每前进 0.29 m 就吃掉 40 mm 横向（视场全部）。
//     · 翻号 = 指令侧一步到 0.28/0.300×1.15 = 1.073，乘执行增益 0.85 ⇒ 实测 ω=0.912
//       vs 需要 0.933，残差仅 2.3% ⇒ PD 只要 e≈0.9 mm 就补齐，全程不打满舵。
//     · 舵量天花板：wCap=2(v−12·Rw)/Lw=27.7(v−0.1805)，v=0.28 时只有 1.38 rad/s；
//       若靠 PD 把 ω 从 −0.93 翻到 +0.93，需要指令 1.86/0.85=2.19 > 1.38 ⇒ 必被钳住、
//       车跑宽、线甩出视场。**这就是"看着在线上却拐不过来"的量化真身。**
//       ⇒ 切点只能【先验触发 + 前馈翻号】；等 GyroZ 真的翻号是马后炮：
//         要达到 1.86 的净舵得 e=1.86/24=77 mm，比视场大一倍，物理上不可能。
//   ★为什么不用里程判段（v3.11.x 死跑失败的原因之一）：
//     里程漂移实测 −2~−3.5%（23 m 处约 0.5 m），而 R16 前 0.57 m 是 R15（左 R500，ω=+0.56），
//     错 0.5 m 就把基准角立到 R15 里，第一次触发要晚 100° 以上。
//     ⇒ 入口改用【曲率签名】：全场只有 R16 是 |ω|≥0.55 的右转
//       （R14=R600 ω=−0.47、R15=R500 ω=+0.56、R10=R292 在 14.8 m 且窗外）。
// ══════════════════════════════════════════════════════════════════════════════
#define C4_EN          1        // 总开关（0 = 完全退回 v3.10.16 行为，用于 A/B）
#define C4_R           0.300f   // 单圆半径 [m]（_map_table.txt R16~R19）
#define C4_FFK         1.15f    // 前馈系数：指令侧 = v/C4_R × C4_FFK（≈执行增益 0.85 的倒数）
#define C4_ARM_G       0.55f    // 入口判据 |lineGyroF| ≥ 此值（分得开 R14=R600 ω=−0.47 与 R16 ω=−0.93）
#define C4_ARM_N       2        // 连续帧数（200 Hz ⇒ 10 ms）
#define C4_ARM_ODO_LO  20500.0f // 入口里程窗下界 [mm]（只做粗门控，容忍 0.5 m 漂移）
#define C4_ARM_ODO_HI  24500.0f // 入口里程窗上界 [mm]
#define C4_ARM_LEAD    0.15f    // 检测滞后补偿 [rad]（低通 0.25 s 后才到 −0.55，已走过约 0.15 rad）
#define C4_N           3        // 3 个切点（R16/R17、R17/R18、R18/R19）
#define C4_SEG_RAD     3.05f    // 段内门限 175°：180° 太晚（舵量建立要 0.1 s）
#define C4_SEG1_RAD    3.20f    // 第 1 段 183°（R16 是 189°，比半圆多 9°）
#define C4_END_RAD     0.79f    // ★v3.14.2 第 3 次换向后【走满 45°(≈236 mm)】才交回：
                                //   0.79 rad 既保证换向真的完成（前馈翻过去了、车已在新曲率上），
                                //   又把退出点从"距 A2 只剩 26 mm"提前到约 700 mm，给转角状态机留够时间。
                                //   v3.14.1 曾在这里直接退出（不等这一小段）⇒ 第 3 个切点等于没换向。
#define C4_V           0.28f    // 区内速度 [m/s]：0.40 档实测偏航率 p50 只有 1.00 < R300 所需 1.33
#define C4_EXIT_V      0.25f    // ★v3.14.6 出区速度 [m/s]：A2/A3 只隔 283 mm，必须以低速进转角
#define C4_ODO_BAIL    27700.0f // 兜底撤离里程 [mm]：A2 在 26.762 m，超过即认为已出区
static bool     c4On      = false;      // 四圆区介入中
static bool     c4Done    = false;      // 本趟已走完（防重复武装）
static int      c4N       = 0;          // 已翻号次数 0..C4_N
static int      c4Sign    = -1;         // 当前段曲率符号（−1=右=R16，+1=左=R17）
static float    c4Yaw0    = 0.0f;       // 本段偏航基准（每次翻号重置 ⇒ 误差不跨段累积）
static int      c4ArmCnt  = 0;          // 入口签名连续帧计数
static uint32_t c4HbMs    = 0;          // 心跳
static int      c4ForkPrev = 0;         // 上一帧的分叉方向（0/±1），用于 2 帧确认
static uint32_t c4LastMs   = 0;         // 上次切点触发时刻（冷却）

// ★★★v3.14.0 切点【分叉签名】识别（用户 10-21 10:23 提出，手推数据已证实）
//   几何：R16/R17 是外切圆，切点处两圆相切 ⇒ 两条线在切点附近几乎重合，但会以
//   抛物线张开：δ ≈ s²/(2R)。s=66 mm（前视距离）时 δ=7.3 mm，s=90 mm 时 δ=13.4 mm（一个台阶）。
//   ⇒ 传感器排跨过切点后约 24 mm，最外侧那一路会先看到【另一条弧】的线，
//     而【主线仍在中心】—— 这就是"1 号先扫到线"，方向恰好是下一段弧的弯曲方向。
//
//   ★为什么不能直接判"最外侧亮"（手推 1418 帧统计，`tools/_fork_an.py`）：
//     OUT7 亮 286 帧（20%），其中 282 帧是车自己偏右（`0011111` e=−13.4 / `0001111` e=−20.1
//     / `0111111` 等，全是单簇）；OUT1 亮 18 帧里也有 16 帧是偏左（`1110000` e=+26.8）。
//     直接判"最外侧亮"信噪比 70:1，必然误触发。
//
//   ★真正的签名 = 【中心 OUT4 仍亮】+【最外侧单独成一簇】（与主线之间至少隔一个暗格）
//     = 连通簇数正好 2，且含端点的那一簇不含中心。手推数据里这个签名只命中 6 帧，
//       全部落在 3 个时刻（t≈31.3 / 32.5 / 37.5 / 43.8），无一误报。
//     对照：`1011000`(OUT1 | OUT3,OUT4) 与 `0011001`(OUT3,OUT4 | OUT7) 是分叉；
//           `1111000` / `0011111` 是单簇 = 车偏，不算。
//
//   返回：0=不是分叉  +1=左分叉（下一段左转）  −1=右分叉（下一段右转）
static inline int c4ForkDir(uint8_t b)
{
    if (!(b & 0x08)) return 0;                 // 中心 OUT4(bit3) 必须亮，否则就是整条线平移
    int nCl = 0, cl[7], prev = 0;
    for (int i = 0; i < 7; i++) {
        const int lit = (b >> i) & 1;
        if (lit && !prev) nCl++;
        cl[i] = nCl;
        prev  = lit;
    }
    if (nCl != 2) return 0;                    // 1 簇=偏线 / ≥3 簇=噪声
    if ((b & 0x01) && cl[0] != cl[3]) return +1;   // OUT1 与主线分离 ⇒ 左分叉
    if ((b & 0x40) && cl[6] != cl[3]) return -1;   // OUT7 与主线分离 ⇒ 右分叉
    return 0;
}

// ★★v3.4.1 丢线搜索：线滑出 ±40 mm 视场（n=0）时**不许把舵量归零**——
//   归零 ⇒ wCmd=0 ⇒ 车直着冲出去，再也回不来（run23 实测：n=0 后直冲 20 s）。
//   经典做法：保持最后一次有效打舵方向，最多 500 ms（必须 <600 ms 停车门限），
//   一边前进一边把车头摆回去，线回到视场立刻重新接管。
static float    lineSteerHold = 0.0f; // 最后一次「有效」舵量
static bool     lineSearch    = false;// 正在保持打舵找线
// ══ ★★v3.10.0 边缘丢线记忆搜线（用户 2026-10-19 提出）══
//   用户实测观察（几何链跑 R1~R6 的九段记录）：**每一次冲出弧线，灰度阵列必定先出现
//   "只有最左(1000000) / 最右(0000001) 一路报白"，紧接着就整排全黑(0000000) 丢线**。
//   也就是说：丢线不是突然发生的，它前面一定有一个"线已经压到视场边缘"的预警帧。
//   ⇒ 把那个预警帧的【侧面】记住，丢线后就朝那一侧主动打舵去搜，而不是死板地"保持最后舵量"。
//
//   为什么"保持最后舵量"不够（v3.4.1 的做法）：
//     ① 方向可能不对：丢线那一刻 PD 输出往往因为 lineE 被低通回零而只剩 0.3 的兜底值，
//        方向是"上一帧的纠偏方向"，与"线跑到哪一侧去了"不一定一致；
//     ② 力度不够：0.30 rad/s 要 1 s 才转 17°，而丢线保护 0.6~0.9 s 就停车了 ⇒ 注定搜不回来。
//   为什么"朝边缘侧搜"是对的（符号推导，lineSign=+1）：
//     e>0 = 线在车左（OUT1 侧报白）⇒ 线相对车向左滑走 ⇒ 车相对线偏右 ⇒ 应【左转】追线
//     ⇒ 搜线舵量符号 = lineSign × 边缘方向，与 PD 的 e→w 符号完全一致。
//   ★副带收益：R1 少转 4.8° 的那 4.8°，正是"线先压到 OUT1、然后丢线"的形态 ⇒
//     搜线会自动把这段欠转补上，不需要再去猜 R1 为什么欠转。
static int8_t   lineEdgeSide   = 0;     // +1=线从左边(OUT1)滑出 / -1=右边(OUT7) / 0=未知
static float    lineSearchRate = 1.20f; // 搜线舵量 [rad/s]（ylr 改；必须 ≤ lineTurnMax）
static uint8_t  lineReacqCnt   = 0;     // 搜线恢复：连续有效帧计数（3 帧才认，防边缘假捕获）
#define LINE_SEARCH_MS 6000             // ★★★v3.14.5 搜线窗口 1100→6000 ms（用户 10-21 11:43 要求
                                        //   "丢线就去找，直到找到为止"）。原来 1100 ms 一到 lineSearch=false
                                        //   ⇒ lineDirect 也 false ⇒ 落进"直行锁航向"分支，车直冲出去，
                                        //   直到 1.4 s 停车 —— 中间那 300 ms 根本没在找线。
                                        //   v3.10.2 700→1100 的依据：搜线半径 155 mm 收敛 40 mm 偏移要约 0.9 s。
#define LINE_EDGE_MM   33.0f            // 判定"线已到边缘"的 |e| 阈值 [mm]（=2 个灯距，覆盖 33.5/40.2）

// ══ ★★★v3.14.7 终点停车（用户 10-24 14:0x："加上停车代码，让小车最后停在（起点）区"）══
//   全程里程（`firmware/BLDC_S1/field_track_named.geom` 逐段累加，A 转角不占里程）：
//     ... → L8 18683.7 → R12~R15 → **A5 34719.6** → L12 2300 → **终点 37019.6 mm**
//   L12 与 L1 共线（同一条 y=4800），所以终点**没有任何横线**，灰度看不出"到头了"
//   ⇒ 不主动停车的话，车会顺着线一直冲过 x=200 出赛道，直到丢线保护才停。
//   ★为什么 odo 判据取 36200 而不是 37020：odo 有 −2~−3.5% 负漂移（23 m 处实测约 0.5 m），
//     odo 走到 36200 时真实位置约 37.0 m = 地图终点。**若实测停早了就调大、停晚了就调小**，
//     这一个数就是唯一的停车点旋钮。
//   ★只作用于巡线（lineOn）且只在最后 300 mm 内接管，前面 36.9 m 的行为一个字节没变。
#define LINE_STOP_EN        1
#define LINE_STOP_ODO_MM    36400.0f    // 停车判据里程 [mm]（odo 读数）
                                        // ★10-24 14:2x 用户实测"再往前 20 cm" ⇒ 36200 → 36400
                                        //   odo 有 −2~−3.5% 负漂移，200 mm 真实 ≈ 194 mm odo，差 6 mm 可忽略
                                        //   停早了调大 / 停晚了调小，只动这一个数
#define LINE_STOP_BRAKE_MM  300.0f      // 提前减速距离 [mm]
#define LINE_STOP_LAST_MM   25.0f       // 剩余小于此值 ⇒ 直接停（抵消滑行）
#define LINE_STOP_VSTART    0.26f       // 减速段起始速度参考 [m/s]（L12 实际约 0.25）
#define LINE_STOP_VMIN      0.06f       // 减速段末速 [m/s]

// ══ ★★★v3.10.3 曲率反向检测（治 S 弯切点"冲出视场"）══
//   实测 run 10-20 00:12（L1→A1→L2→R1 尾段，v=0.28，R200 蛇形第一处曲率反转）：
//     t=15.15  e=+6.7  yaw=422.0  wCmd=+1.65
//     t=15.20  e=−6.7  yaw=425.7  wCmd=+1.10   ← 切点：e 在**一帧内**变号（Δe=13.4 mm）
//     t=15.30  e=−20.1 yaw=432.8  wCmd=−1.47
//     t=15.35  e=−33.5 yaw=435.5  wCmd=−1.80   ← 车头比切点多左转了 13.5° 才刹住
//     t=15.60  丢线（线已滑出 ±40 mm 视场）
//   根因不是 PD 慢，是【曲率前馈】慢：lineArcFF 由 GyroZ 学习，下降时间常数 0.40 s
//   ⇒ 切点之后它仍在输出 +1.29 rad/s，PD 那 −0.2 根本压不住 ⇒ 车继续按 R200 左转，
//   0.15 s 就是 13.5°、40 mm —— 刚好等于视场半宽，一帧出界。
//   判据：**e 在单帧内变号，且 |Δe| ≥ 13 mm（=1 个灯距）**。
//     物理依据：0.28 m/s 在 5 ms 内只横移 1.4 mm，能在单帧里做出一整格跳变的
//     只有"线的曲率翻向了"这一件事；直道上的缓慢漂移（+6.7→0→−6.7 要 0.1 s 以上）
//     永远不会满足 ⇒ 不误触发。
//   处置：把前馈【镜像】(FF → −FF)。S 弯两段等半径，镜像就是精确解；
//     不等半径时由 PD + 后续学习在 0.2 s 内纠回。
//   ★镜像后必须【冻结学习 350 ms】：否则 GyroZ 还停在 +1.4，学习会在 0.1 s 内
//     把 FF 拉回 +1.29，等于什么都没做（自相矛盾）。
static int8_t   lineESgnPrev  = 0;      // 上一个非零 e 的符号（过零也算翻转）
static float    lineERawPrev  = 0.0f;   // 上一个非零 e [mm]（算 Δe 用）
static uint32_t lineESgnMs    = 0;      // ★v3.10.6 当前符号已经持续多久 [ms]
static float    lineRevFF     = 0.0f;   // 翻转瞬间的前馈值（镜像源）
static uint32_t lineRevMs     = 0;
static bool     lineRevActive = false;  // 处于翻转窗口（冻结学习）
static bool     lineRevDone   = false;  // 本窗口已镜像过（只镜像一次）
// ★★★v3.10.10 新的曲率反向判据：**e 与 arcff 反号**
//   为什么不用"单帧 Δe 跳变"：R1→R2 的实测（run g3112）里 e 是
//     14.48 +20.1 → 14.73 0 → 14.99 −6.7 → 15.14 −20.1，
//   整整 0.65 s 慢慢滑过去的，**根本没有"一帧跳变"**——Δe 判据在这里永远不会触发。
//   而"e 与 arcff 反号"是物理事实：arcff 还在往左打（+1.35），线却已经跑到右边
//   （e<0）⇒ 前馈的曲率方向已经错了，必须立刻镜像。
//   两个门防止抖动误触发：|e|≥13 mm（2 个量化台阶，单帧抖动只有 6.7）
//                       + 反号持续 ≥120 ms（真反向会一直反，量化抖动过零即回）。
static uint32_t lineOppMs     = 0;      // 「反号」起始时刻 [ms]
static bool     lineOppArmed  = false;  // 反号计时中
// ★★★v3.10.14 镜像【第三次也是最终关闭】。证据链：
//   v3.10.7（g3108/g3109）：无 dwell 时每次过零都翻；加了 dwell 800 + |arcff|≥0.6 后，
//     摆荡本身会把 e 按在一侧 2.0 s ⇒ 判据自指，照翻 ⇒ 摆荡被放大成满舵极限环。
//   v3.10.10 换成「e 与 arcff 反号 + |e|≥13 + 持续 120 ms」重开，本次 run _r31013_1 又误触发：
//     t=15.2 起在 **R1 弧内**，e=−20.1 而 arcff=+1.80（车在弧外侧，本来就该继续左打），
//     判据却理解为"线拐向另一边" ⇒ arcff 由 +1.80 直翻 −1.03（**2.8 rad/s 阶跃**）⇒
//     yaw 增速 94°/s → 21°/s，车被拽直 → 16.2 e=−26.8 → 丢线。
//   ★根本矛盾：【弧上的横向偏差】与【切点曲率反向】在 (e, arcff) 上完全同形。
//     弧上 e 与前馈反号是常态（车偏在弧外侧），不是异常 ⇒ 任何基于该组合的判据都会误触发。
//   ⇒ 判定曲率反向只能靠"之后会怎样"，而切点处 0.18 s 的快速卸力已经把时间裕度给足了。
#define LINE_MIRROR_EN  0               // 0 = 关闭镜像（v3.10.14）
#define LINE_OPP_HOLD 120               // 反号需持续的时长 [ms]
#define LINE_OPP_E    13.0f             // 判反号的 |e| 下限 [mm]
#define LINE_REV_MS   350               // 翻转后冻结学习时长 [ms]（要短于一个 S 弯的 2.2 s）
#define LINE_REV_DE   13.0f             // 判"单帧整格跳变"的 |Δe| 阈值 [mm]
#define LINE_REV_FF_MIN 0.60f           // 只允许在"弧上"镜像：|arcff| 下限 [rad/s]
#define LINE_REV_DWELL  800             // ★★v3.10.6 翻转前 e 必须已在【同一侧】待够这么久 [ms]
                                        //   物理：真正的切点是"线一直在一侧(整个半圆 ~2.2 s)，
                                        //   突然跳到另一侧"。而量化抖动是每 0.2~0.3 s 来回过零一次。
                                        //   用 dwell 把两者分开 —— Δe 分不开（都是 13.4 mm）。
// ★★★v3.10.7 镜像【关闭】。两轮实测证明它判不出"切点"和"摆荡过零"的区别：
//   g3108（无 dwell）每次过零都翻；g3109（dwell 800 + |arcff|≥0.6）在摆荡里
//   e 被推在一侧待 2.0 s 也照样满足门限，arcff 仍然在 +1.17/−1.68 之间逐帧翻
//   （14.767 → 14.817），把摆荡放大成满舵极限环。
//   根因：一旦开始摆，dwell 和 |arcff| 这两个量本身就被摆荡拉长了 —— 自指判据。
//   ⇒ 改用物理上确定的办法：**arcff 的下降时间常数降到 0.18 s**（切点 0.4 s 就跟到
//     反向），不再靠"猜一帧跳变"。少一个判据就少一个自激源。
#define LINE_REV_ENABLE 0

// ---- ★★v3.7.0 无线上电自跑 ----
//   需求：拔掉 USB 也要能跑。拔线以后没人发 yp/ya/yd/yt/yb/ylo/yr，也没人发 yf/v，
//   所以这些必须由固件自己兜住：
//     ① 参数：上面那些 static 默认值已固化成本次验证的工作点（Kp24/Kd11/max1.30/trim−0.116）；
//     ② S1 工作点：S1 的 W/kd/km/kc/kb/jc/ki/e 只存 RAM（★掉电回出厂：slew=5、brakeK=0.50），
//        必须在开机后由 S3 自己补一遍，见 applyS1Workpoint()；
//     ③ 起跑：等"车已就位"（视线有效 + 静止）持续 1.5 s → 再等 3 s（留时间松手）→ 自己开巡线。
//   安全：任何位移都会打断倒计时重新等；跑起来后丢线 0.6 s 自动停车（原有逻辑）；
//         停下后不会自动再起跑（autoSt=2 就锁住），要重跑就按一次复位/重新上电。
static bool     autoRun      = true;   // ★★★v3.13.3 改回默认 ON：用户 10-21 10:04 要求"完全离线跑，
                                       //   上电自动标零 → 等 3 s 发车"，而原来的 yn1 必须接电脑发命令，
                                       //   与"完全离线"直接矛盾。安全阀变成"不持久化"：yn0 只在本
                                       //   次上电内有效，绝不会出现"以前发过 yn0 ⇒ 以后每次上电都不跑"。
static float    autoRunV     = 0.28f;  // 自启巡航速度 [m/s]（★必须 ≥0.19，否则转向权限为 0）
static uint8_t  autoRunSt    = 0;      // 0=等就位 1=倒计时 2=已起跑/已放弃
static uint32_t autoRunOkMs  = 0;      // 就位条件开始成立的时刻
static uint32_t autoRunT0    = 0;      // 倒计时起点
static uint8_t  autoRunTick  = 0;      // 倒计时秒数（打印用）
static float    autoRunGzSum = 0.0f;   // 就位窗口内的 GyroZ 累加（查零偏是否可信）
static uint16_t autoRunGzN   = 0;
static bool     autoLiftSeen = false;  // ★v3.9.11 离线重跑：本趟结束后检测到"被搬动过"
static uint32_t autoLiftMs   = 0;      //            搬动持续计时（要 200 ms 才算）
#define AUTO_ARM_MS  1500              // 就位条件要连续成立这么久
#define AUTO_DELAY_MS 3000             // 倒计时长度（松手时间）
// ★v3.13.1 开机自检时序（用户 10-21 要求："启动之后 IMU 要标零，标零结束后等待 3 s 启动"）
#define BOOT_SETTLE_MS   1500u         // 上电后先稳这么久再标零（板温/USB 供电爬升）
#define BOOT_AFTER_CAL_MS 3000u        // 标零完成 → 再等这么久才允许发车

// ★v3.9.10 自启速度持久化。动机：autoRunV 原来只存 RAM，而自启流程是「上电就生效」，
//   所以每次上电都回到 0.28 —— 那样「可调」只在不断电时有意义，实际没法做速度对比实验。
//   写 NVS 后：ynv0.40 → 断电 → 上电仍是 0.40。只在收到命令时写，不磨损 flash。
static Preferences gPrefs;
static bool        gPrefsOk = false;

static void autoRunLoad()
{
    gPrefsOk = gPrefs.begin("rc", false);
    if (!gPrefsOk) { Serial.println("[AUTO] NVS 不可用，自启速度用默认 0.28；自启开关固定 ON"); }
    float v = 0.28f;
    if (gPrefsOk) {
        v = gPrefs.getFloat("auv", 0.28f);
        if (!(v > 0.05f && v <= 1.20f)) v = 0.28f;
    }
    autoRunV = v;
    // ★★★v3.13.3 完全离线：上电即武装，不再需要任何一条串口命令。
    //   为什么把 v3.9.14 的"默认 false"改回来：用户 10-21 10:04 明确要求
    //   "完全离线跑 / 上电自动标零 / 等 3 s 发车"，而 yn1 必须插电脑发，直接矛盾。
    //   安全阀改成【不持久化】：`aur` 键不再读写，yn0 只在本次上电内有效 ⇒
    //   永远不会出现"以前发过一次 yn0，之后每次上电都不跑"的死状态。
    autoRun = true;
    Serial.printf("[AUTO] NVS 自启速度 = %.2f m/s   自启开关 = ON（上电即武装，无需串口）\n", autoRunV);
}

static void autoRunSave(float v)
{
    autoRunV = v;
    if (gPrefsOk) gPrefs.putFloat("auv", v);
}

static inline bool grayOnLevel(uint8_t pin)
{
    bool low = (digitalRead(pin) == LOW);
    return grayInv ? !low : low;
}

static inline void grayUpdate()
{
    uint8_t b = 0;
    for (uint8_t i = 0; i < GRAY_N; i++)
        if (grayOnLevel(GRAY_PIN[i])) b |= (uint8_t)(1u << i);   // 默认 LOW=压线（白线版）
    grayBits = b;
    // 重心法：e = Σ(3-i)·p / n ，i=0 为最左 ⇒ 线在左时 e>0
    float sum = 0.0f; uint8_t n = 0;
    for (uint8_t i = 0; i < GRAY_N; i++)
        if (b & (uint8_t)(1u << i)) { sum += (float)(3 - (int)i) * grayPitch; n++; }
    grayCnt = n;
    grayE   = n ? (sum / (float)n - grayZero) : 0.0f;
}

// ★v3.2.0 巡线误差：有效性门控 + 一阶低通 + 折算成航向修正量
//   n=0     ⇒ 丢掉线（或全底色）→ 累计丢线时长，>300 ms 后修正量缓慢回零
//   n=1~3   ⇒ 有效（线宽在 1~3 个灯距内），用重心 e
//   n≥4     ⇒ 饱和（十字/宽块/横线），e 不可信 → 保持上一次值，但不算丢线
//   为什么必须低通：13.4 mm 的量化台阶直接求 P 项就是方波，车会"一格一格"地摆。
static inline void lineUpdate(float dt)
{
    // ★★v3.2.3 连通性检查（10-19 实测踩到的致命漏洞）：
    //   重心法对【离散】位型会给出"完美居中"的假读数。例子：1001001（OUT1/4/7 同时压线）
    //   ⇒ Σ(3−i) = 3+0−3 = 0 ⇒ e=0，但物理上根本没有"线在中间"这回事。
    //   实测 t=2 s 就是 1001001 读出 e=-0.0，控制环误判居中，随后车按 holdI 一路画弧。
    //   所以只接受【连续一段】的位型，其余一律当无效。
    bool contig = true;
    if (grayCnt > 0) {
        int first = -1, last = -1;
        for (int i = 0; i < GRAY_N; i++)
            if (grayBits & (uint8_t)(1u << i)) { if (first < 0) first = i; last = i; }
        if (last - first + 1 != (int)grayCnt) contig = false;
    }

    if (grayCnt >= 1 && grayCnt <= 3 && contig) {
        lineE += (grayE - lineE) * (1.0f - expf(-dt / 0.050f));   // τ=50 ms
        lineValid  = true;
        lineLostMs = 0;
        // ★★v3.10.0 边缘预警记忆：线已经压到视场边缘（|e|≥33 mm，即 OUT1/OUT7 单边报白）时，
        //   把侧面记下来。只在"接近边缘"时才写，直道的 0/±6.7 不会污染这个记忆。
        //   用原始 grayE（不滞后）：丢线前只剩一两帧，lineE 的 50 ms 低通值会偏小。
        if (fabsf(grayE) >= LINE_EDGE_MM)
            lineEdgeSide = (grayE > 0.0f) ? +1 : -1;

        // ★★★v3.10.3 曲率反向检测（详见变量声明处）。
        // ★★★v3.10.5/6 两道门，缺一不可：
        //   gate1 |arcff| ≥ 0.60 —— 直道上 FF≈0，镜像 0 无意义。
        //   gate2 翻转前 e 已在同一侧待够 800 ms —— 这是把"真切点"和"量化抖动"
        //         分开的唯一办法。实测（run g3108 六弧全过、都咬在线上）：
        //         半圆上 e 同一侧能待 1~2 s，切点处一次翻转；而直道上摆荡时
        //         e 每 0.2~0.3 s 就过零一次 ⇒ dwell 远小于 800 ⇒ 不触发。
        //         ★没有 gate2 时，arcff 被逐帧镜像（+0.97/−0.97 来回），
        //           前馈永远衰减不掉，直道上就一直左右摆 —— 这正是用户看到的现象。
        {
            int8_t sgnNow = (grayE > 3.0f) ? +1 : ((grayE < -3.0f) ? -1 : 0);
            if (sgnNow != 0) {
                if (lineESgnPrev != 0 && sgnNow != lineESgnPrev) {
                    uint32_t dwell = (uint32_t)(millis() - lineESgnMs);
                    if (LINE_REV_ENABLE &&
                        dwell >= LINE_REV_DWELL &&
                        fabsf(grayE - lineERawPrev) >= LINE_REV_DE &&
                        fabsf(lineArcFF) >= LINE_REV_FF_MIN) {
                        lineRevFF     = lineArcFF;     // 旧曲率的幅度
                        lineRevMs     = millis();
                        lineRevActive = true;
                        lineRevDone   = false;
                    }
                    lineESgnMs = millis();             // 换边 ⇒ 重新计时
                } else if (lineESgnPrev == 0) {
                    lineESgnMs = millis();             // 从"居中"重新进入 ⇒ 计时起点
                }
                lineESgnPrev = sgnNow;
                lineERawPrev = grayE;
            }
        }
    } else {
        lineValid = false;
        if (grayCnt == 0 || !contig) {                 // 真丢线（含离散假位型）
            if (lineOn) lineLostMs += (uint32_t)(dt * 1000.0f + 0.5f);
            if (lineLostMs > 150)                      // 150 ms 宽限后回零，别拿旧值继续打方向
                lineE += (0.0f - lineE) * (1.0f - expf(-dt / 0.200f));
        } else {                                       // n≥4 饱和（十字/大块黑）
            lineLostMs = 0;
            // ★不再"保持上一次值"：v3.2.0 保持 9.5° 的结果是车带着 9.5° 一直画弧冲出场外。
            lineE += (0.0f - lineE) * (1.0f - expf(-dt / 0.300f));
        }
    }
    float off = lineKp * atanf((lineE * 0.001f) / (lineLD > 0.02f ? lineLD : 0.02f));
    lineW = lineSign * _constrain(off, -0.20f, 0.20f);            // ±11.5°

    // ★★v3.4.0 直接舵量：turn = Kp·e + Kd·ė（rad/s）
    //   ė 由 5 ms 采样的 e 差商再重低通（13.4 mm 量化 ⇒ 差商尖峰 ±1.3 m/s，必须滤）。
    //   ★为什么 D 是主角：P 只能对"已经偏了多少"反应；D 对"正在以多快速度横滑"反应。
    //     起步 13° 偏斜时 e 才 6.7 mm（P 只给 0.09 rad/s），但 ė≈0.078 m/s
    //     （D 直接给 0.62 rad/s）⇒ 这才是能在出视场前把车掰回来的那一项。
    float eM = lineE * 0.001f;
    float dRaw = (eM - lineEPrev) / (dt > 1e-4f ? dt : 0.005f);
    lineEPrev = eM;
    lineEDot += (dRaw - lineEDot) * (1.0f - expf(-dt / 0.120f));   // τ=120 ms
    lineSteer = lineSign * (lineKpD * eM + lineKdD * lineEDot);      // 纯 PD（不含 trim / I / 前馈）
    // ★★v3.8.2/3 大偏差增速：|e| 超过阈值后额外加 (Kboost × 超出量)。
    //   ★用原始 grayE（不滞后），不用滤波后的 lineE —— run45 实测 lineE 慢 0.1 s，
    //     在 R200 这种"0.45 s 就丢线"的工况下，0.1 s 滞后等于直接失败。
    {
        float eRaw = (lineValid ? grayE : lineE) * 0.001f;
        float ae = fabsf(eRaw);
        // ★★v3.10.9 前馈已经建立时不再给 boost —— 两者是【重复补偿】。
        //   实测 g3110 六弧：e 被稳稳压在 +20.1 mm 不动，steer 却已顶到 1.799。
        //   算式：arcff 学到 1.45（R200@0.28 正好需要 1.4），
        //        boost 又按 (20.1−6.8)/1000×60 = 0.79 加进来 ⇒ 合计 2.24 > turnMax 2.00
        //        ⇒ 满舵过转 ⇒ 线往左滑 ⇒ e 反而稳在 20 mm 下不来（正反馈）。
        //   boost 的定位是"入弯/出视场边缘时的救援"，此刻 arcff 还小（<0.8），
        //   等 arcff 学到 0.8 以上就说明前馈已经接管，boost 必须让位，否则两者叠加必过转。
        if (ae > lineEboost && lineKboost > 0.0f) {
            // ★★★v3.10.10 让位改成【线性淡出】，不能再硬切：
            //   实测 run g3112 13.93 s：arcff 0.748→0.902 跨过 0.80 那一帧，
            //   steer 从 1.922 一帧掉到 1.122 —— 0.80 rad/s 的阶跃，
            //   相当于给车身一个瞬时扰动，是后面那串摆荡的起爆点。
            //   现在按 |arcff| 在 0.80~1.20 之间线性收，交接连续。
            float wgt = 1.0f - (fabsf(lineArcFF) - LINE_BOOST_FFMAX) / 0.40f;
            wgt = _constrain(wgt, 0.0f, 1.0f);
            float extra = (ae - lineEboost) * lineKboost * wgt;
            extra = _constrain(extra, 0.0f, lineBoostMax);   // ★v3.8.5 封顶，防过冲
            if (extra > 0.0f)
                lineSteer += lineSign * (eRaw > 0.0f ? extra : -extra);
        }
    }

    // ★★v3.8.0 曲率前馈：稳态圆弧上"维持曲率所需的舵量" = 实际偏航率 ⇒ 把 GyroZ 慢学进 lineArcFF。
    //   学习门限：线要落在视野中间（|e|<27 mm = n≤3 的正常档），否则说明正在纠偏，
    //   这时学到的是"纠偏动作"而不是"路段曲率" ⇒ 只缓慢卸掉，不学。
    //   ★两个工程细节：
    //     ① 学习系数 0.90（不是 1.0）：留 10% 让位置环继续工作。若是 1.0，则
    //        steer = PD + GyroZ 且 GyroZ ≈ steer ⇒ 环路增益正好 1，会让 e 缓慢漂走。
    //     ② 上升快、下降慢（0.20 / 0.40 s）：入弯必须在 0.4 s 内学够，否则一开始就得靠
    //        e 去换 1.4 rad/s（要 63 mm，早就出视场了）；出弯则慢慢卸，避免舵量突抽。
    // ★v3.10.9 偏航率低通（喂给 arcff 学习用）。无条件更新，转角/丢线时也跟着走。
    lineGyroF += (robotPose.GyroZ - lineGyroF) * (dt / (0.25f + dt));
    // ★★★v3.10.3 翻转窗口到点关闭（详见变量声明处）
    if (lineRevActive && (uint32_t)(millis() - lineRevMs) > LINE_REV_MS)
        lineRevActive = false;
    if (!lineOn) lineRevActive = false;             // 关巡线 ⇒ 清窗口，别把镜像带到下一趟

    // ★★★v3.10.10 曲率反向检测：e 与 arcff 反号且持续 ⇒ 线已经拐向另一边，前馈方向错了
    {
        bool opp = LINE_MIRROR_EN &&
                   (fabsf(lineArcFF) >= LINE_REV_FF_MIN) &&
                   (fabsf(grayE) >= LINE_OPP_E) &&
                   ((grayE * lineArcFF) < 0.0f) &&
                   lineOn && lineValid && !lineCorner && !lineAdvancing &&
                   !lineReacq && !lineSearch && !lineRevActive;
        if (opp) {
            if (!lineOppArmed) { lineOppArmed = true; lineOppMs = millis(); }
            else if ((uint32_t)(millis() - lineOppMs) >= LINE_OPP_HOLD) {
                lineRevFF     = lineArcFF;      // 旧曲率（镜像后取反）
                lineRevMs     = millis();
                lineRevActive = true;
                lineRevDone   = false;
                lineOppArmed  = false;
                Serial.printf("[LINE] ★曲率反向 e=%.1f arcff=%.2f -> 镜像\n", grayE, lineArcFF);
            }
        } else {
            lineOppArmed = false;
        }
    }

    if (lineOn && lineValid && !lineCorner && !lineAdvancing && !lineReacq &&
        !lineSearch && !lineAfterTurn && lineArcFFMax > 0.0f) {
        if (lineRevActive) {
            // ★曲率已反：把前馈镜像一次，然后冻结 350 ms（不许 GyroZ 把旧的学回来）
            if (!lineRevDone) { lineArcFF = -lineRevFF; lineRevDone = true; }
        } else if (fabsf(lineE) < 27.0f) {
            // ★★★v3.10.9 学习源改成【低通后的偏航率】lineGyroF（τ=0.25 s），不再用 ė 当门。
            //   为什么撤掉 v3.10.7 的 |ė|<0.08 门：
            //     R200 弧上 e 在 6.7↔20.1 mm 之间跳（量化台阶 13.4 mm），
            //     帧间隔 0.05 s ⇒ ė=0.27 > 0.08 ⇒ **整个 R1 的学习都被禁掉**。
            //     实测 g3110：arcff 从 −0.42 涨到 1.45 用了 1.9 s，而 R200 半圆只有 2.24 s
            //     ⇒ 一直是"学不够 → 舵量不足 → 偏到 20 mm → ė 更大 → 更学不到"的死锁。
            //   为什么低通能代替它：真弧上 GyroZ 恒同号（+1.4 持续 2.2 s），
            //     0.25 s 低通后仍是 +1.4；而摆荡时 GyroZ 正负各半，低通后 ≈0
            //     ⇒ 学出来的 tgt≈0，arcff 自然衰减，不会把摆荡学成曲率。
            //   这一步从"限制什么时候能学"改成"让输入本身只含真曲率"，物理上更干净。
            float tgt = lineArcK * lineGyroF;
            float tau = (fabsf(tgt) > fabsf(lineArcFF)) ? lineArcTauUp : lineArcTauDn;
            lineArcFF += (tgt - lineArcFF) * (dt / (tau + dt));
        } else {
            // ★★v3.8.6 冻结，不再泄放。run48 实测：arcFF 在 e≈0 时学到 1.048
            //   （正好等于 R200 弧需要的 1.4 × 学习系数 0.75），但 e 一大就按 1.5 s
            //   时间常数泄到 0.32，于是下一段又只能靠"把 e 撑到 33~40 mm"去换舵量 ⇒
            //   摆幅越来越大，5~7 s 后丢线。学到的东西不该因为"正在纠偏"就丢掉。
            //   真出弯/丢线/转角时走下面的 0.30 s 快泄，那里才是该卸的地方。
        }
    } else {                                        // 转角/丢线/未巡线 ⇒ 快速卸掉，别把弯道曲率带进直道
        lineArcFF += (0.0f - lineArcFF) * (dt / (0.30f + dt));
        // ★v3.10.9 同时把低通拉回瞬时值：原地转 90° 的 GyroZ 能到 1.5~2.0 并持续 3 s，
        //   若让 0.25 s 低通慢慢衰减，转完恢复跟线那一刻 lineGyroF 还残留 1.x
        //   ⇒ 一帧就学出 arcff≈1.2，车在直道上凭空画弧。非跟线状态直接跟随瞬时值最安全。
        lineGyroF = robotPose.GyroZ;
    }
    lineArcFF = _constrain(lineArcFF, -lineArcFFMax, lineArcFFMax);

    // ══ ★★★v3.13.0 四圆区：曲率前馈【钉住/翻号】（唯一插入点，c4On==false 时跳过）══
    //   为什么不写在 loop 的四圆块里：lineSteer 在 lineUpdate 内部就由 lineArcFF 合成了，
    //   在外面改 arcff 赶不上这一帧（v3.10.20 踩过：钉值永远晚一帧、等于没钉）。
    //   翻号后 arcff 立刻变成反向 ±1.073，PD 只剩横向微调的活 ⇒ 切点没有"前馈空缺期"。
    if (c4On) {
        const float vC4 = fmaxf(fabsf(robotMotion.forward), 0.10f);
        lineArcFF = (float)c4Sign * (vC4 / C4_R) * C4_FFK;
    }

    // ★★v3.6.0 I 项 + 抗 windup。学习条件与限幅：
    //   ① 只在"正常跟线"时学：转角状态机 / 丢线搜索期间一律冻结
    //      （否则会把"转 90° 用的舵量"学成稳态偏置，转完进直道带着一个常值弧）
    //   ② ★条件积分（climatic anti-windup）：输出已经顶在同一方向的上限时不再灌
    //      —— 这是 run25 过冲的直接病根：饱和期间 I 照灌 0.4 rad/s²，等 e 回零时
    //         bias 已经攒到 0.4+，于是冲过 0 到 +26.8 再丢线。
    //   ③ 硬限幅 ±lineBiasMax（0.70 已覆盖 R0.46 m 弧段所需 0.62）
    float eI  = lineSign * eM;                     // 输出空间的"该往哪边加"
    float uPD = lineSteer + lineTrim;              // 不含 I / 前馈的输出
    float uRaw = uPD + lineBias + lineArcFF;
    //   ④ ★v3.8.1 车没动就不积分：run42 实测轮子完全不转（12V 未上）时，
    //      e 恒为 −6.7 ⇒ I 一路灌到满舵 −1.00，一旦恢复供电会直接窜出去。
    //      判据用轮速而不是指令（指令在卡住时照样有值）。
    const bool carMoving = (fabsf(gVMeas) > 0.05f);
    if (lineOn && lineValid && !lineAdvancing && !lineReacq && !lineSearch && carMoving && lineKiD > 0.0f) {
        float du = lineKiD * eI * (dt > 1e-4f ? dt : 0.005f);
        bool pushHi = (uRaw >  lineTurnMax) && (du > 0.0f);
        bool pushLo = (uRaw < -lineTurnMax) && (du < 0.0f);
        if (!pushHi && !pushLo) lineBias += du;    // ★条件积分
    }
    lineBias  = _constrain(lineBias, -lineBiasMax, lineBiasMax);
    lineSteer = _constrain(uPD + lineBias + lineArcFF, -lineTurnMax, lineTurnMax);

    // ★v3.4.1 丢线搜索窗口：有效时记下舵量；n=0 且未超时则保持该方向继续找线
    // ★v3.10.0 恢复时要【连续 3 帧】有效才清掉边缘记忆：线在视场边缘"擦一下"（1~2 帧）
    //   就清记忆的话，下一次真丢线时就没有方向可搜了 ⇒ 必须让记忆活到车真的跟上为止。
    if (lineValid) {
        lineSteerHold = lineSteer;
        lineSearch    = false;
        // ★★★v3.10.3 只有"线回到视场中间"才算重新捕获成功。
        //   原来只要 lineValid 就 +1，可车被一路推到边缘时那些帧（0000011, e=−33.5）
        //   仍然有效 ⇒ 3 帧后把刚记下的 lineEdgeSide 清成 0 ⇒ 真丢线时无方向可搜，
        //   退回 lineSteerHold=−1.36（R_s=206 mm，与 R200 几乎同心，永远靠不近）。
        //   实测日志 "丢线→搜线 side=+0" 就是这个 bug。
        if (fabsf(grayE) < LINE_EDGE_MM) {
            if (lineReacqCnt < 200) lineReacqCnt++;
            if (lineReacqCnt >= 3)  lineEdgeSide = 0;      // 稳定居中 ⇒ 边缘记忆作废
        } else {
            lineReacqCnt = 0;
            lineEdgeSide = (grayE > 0.0f) ? +1 : -1;       // 还在边缘 ⇒ 每帧刷新方向
        }
    } else {
        lineReacqCnt = 0;
        // ★★★v3.10.11 十字观察窗内【禁止搜线】：真直角穿过去后 n=0，若立刻进搜线，
        //   车会在观察窗里先打舵转向（还会污染 lineEdgeSide），等确认时已经斜着走了。
        bool nowSearch = (lineOn && grayCnt == 0 && !lineXcWatch && lineLostMs < LINE_SEARCH_MS);
        if (nowSearch && !lineSearch)          // 只在进入那一刻打一行，便于事后对日志
            Serial.printf("[LINE] 丢线→搜线 side=%+d rate=%.2f rad/s\n",
                          (int)lineEdgeSide, lineSearchRate);
        lineSearch = nowSearch;
    }

    // ★★v3.6.0 转角检测（20 ms 去抖，左右对称 + 方向判定）
    //   edgeR：OUT3~OUT7 黑 且 左端 OUT1/OUT2 至少一路白 ⇒ 新线在右 ⇒ 右转
    //   edgeL：OUT1~OUT5 黑 且 右端 OUT6/OUT7 至少一路白 ⇒ 新线在左 ⇒ 左转
    //   `1111111` 两端都被排除（全黑 = 压出场地/大片深色，run30 曾据此连误判两次撞车）
    //   转完一次后必须先"看到非横边"才重新上膛，否则车停在横边上会连转两次。
    {
        uint8_t b = grayBits;
        // ★★v3.10.4 横边判据从"5 路"放宽到"4 路"。
        //   实测 run 00:18（L1 末端→A1）：车在直道末端右偏 6.7 mm，转角横边只压到
        //   OUT1~OUT4 ⇒ 位型 1111000；旧判据要 (b&0x1F)==0x1F（OUT1~OUT5 全白），
        //   于是一次都没触发，车冲过转角、丢线，随后搜线摆头时看到 0011111，
        //   被误判"右侧横边⇒右转"，A1 实际转了 −92°（应 +90°），后面全废。
        //   为什么放宽安全：正常巡线只会出现 1~3 路【连续】白（n≥4 已判为饱和/无效），
        //   所以"连续 4 路白"必然是横边，不存在误触发。
        bool edgeR = ((b & 0x78u) == 0x78u) && ((b & 0x03u) != 0x03u);   // OUT4~OUT7 白
        bool edgeL = ((b & 0x0Fu) == 0x0Fu) && ((b & 0x60u) != 0x60u);   // OUT1~OUT4 白
        if (edgeR) { if (lineCornerCntR < 8) lineCornerCntR++; } else lineCornerCntR = 0;
        if (edgeL) { if (lineCornerCntL < 8) lineCornerCntL++; } else lineCornerCntL = 0;
        if (!edgeR && !edgeL) lineCornerArm = true;
        lineCorner = lineCornerArm && (lineCornerCntR >= 4 || lineCornerCntL >= 4);
        if (lineCorner) lineCornerDir = (lineCornerCntR >= lineCornerCntL) ? -1 : +1;
    }
}

#define PATH_MAX_SEG 64
#define PS_LINE      0
#define PS_ARC       1
#define PS_PIVOT     2

struct PathSeg {
    uint8_t kind;
    float   a;      // LINE: 长度 [m] / ARC,PIVOT: 转角 [rad]（带符号，正=左转）
    float   v;      // 线速度 [m/s]
    float   r;      // ARC: 半径 [m] / PIVOT: 角速度上限 [rad/s]
    float   acc;    // 加减速 [m/s²]，0 = 用 moveAcc
};

static PathSeg  pathQ[PATH_MAX_SEG];
static int      pathN        = 0;
static bool     pathArm      = false;  // G 之后：装配模式（只入队，不执行）
static bool     pathOn       = false;  // 链执行中
static int      pathI        = 0;
static float    pathSegS     = 0.0f;   // 直线段已走 [m]
static float    pathYawAcc   = 0.0f;   // 弧段累计转角 [rad]
static float    pathPrevYaw  = 0.0f;   // 弧段上一拍 yaw（wrapPi 差分用）
static float    pathV        = 0.0f;   // 链内速度指令（斜坡）
static float    pathIdealYaw = 0.0f;   // 理想累加航向（绝对基准）
static uint32_t pathSegMs    = 0;
static uint32_t pathMicros   = 0;
static bool     rotThenChain = false;  // 原地转完成后接链的下一段

// ★★★v3.9.0 几何链 ARC 段的【偏航率直接闭环】
//   10-19 run54（R200 六连弧，几何链首跑）实测：
//     指令 turn=1.40 rad/s，GyroZ 只到 1.15~1.24 ⇒ 执行增益仅 0.77~0.84；
//     实际走出的半径 = v/ω = 230 mm（设计 200），六段全偏 +22~+36 mm（重复性极好）。
//     ★转角仍准到 0.1°，因为 ARC 段是按 Δyaw 终止的（时间换角度）；
//       但【位置】全错 —— 每段纵向多 60 mm、横向多 30 mm，六个半圆后车明显出线。
//   机理：轮速差 → 偏航率 这个通道在大差速时被内轮拖滑吃掉 18%（历史证据：2.8% 轮速差
//     只换 1.89° 偏航，理论应 11.4° ⇒ 快的那只轮在打滑）。
//     主律 wCmd=turn+kt(turn-GyroZ) 是纯 P，对"执行增益 k"只能把误差压到 1/k，补不到 1。
//   修法：ARC 段不再走 kt 环，改成 wt + I + Kd·eW 直接给 wCmd：
//     - I 项把稳态误差积掉（wCmd 自己涨到需要的 ~1.71 rad/s）
//     - 初值 pathArcIff·wt 按实测打滑率预置，进弧第一拍就基本对，不用等 0.4 s 积分
//     - Kd 提供阻尼（≈ 原 kt 的作用），防止段切换时 ±1.4 的阶跃把轮子打滑
static float    pathArcI      = 0.0f;   // ARC 偏航率闭环的积分量（含打滑补偿）
static uint32_t pathArcUs     = 0;      // ARC 闭环上一拍时间戳
static float    pathArcKi     = 2.0f;   // ARC 积分增益 [1/s²]
static float    pathArcKd     = 0.6f;   // ARC 比例增益（原 kt）
static float    pathArcIff    = 0.06f;  // 打滑前馈：执行增益不足 ⇒ 多给 iff·ω
static float    pathArcILim   = 0.60f;  // I 项限幅（run0617 定：执行增益只有 ~0.8，要靠 I 补到 1.88 才能真转到 1.5）

// ★★★v3.9.2 几何链的【横向反馈】（几何链 + 灰度 = 前馈 + 反馈）
//   10-19 run59 实测：车摆上 L2 起点时灰度就 e=+20~+40 mm，纯开环把这点偏差原样
//   保持了 8 m，第 6 个半圆直接撞上东西 —— 开环几何链【天生不能纠横向】。
//   但纯巡线又追不上 R200（反馈要 e≥20 mm 才能凑出 1.2 rad/s，必然过冲）。
//   正解是两者相加：几何链给已知弧的【前馈】ω=v/R，灰度只用来纠那几厘米的横向偏差。
//   前馈占 1.2 rad/s，灰度修正最多 ±0.35 ⇒ 不会像纯巡线那样被量化台阶带跑。
static float    pathLatK      = 3.0f;   // 横向→偏航率 [rad/s per m]（v3.9.2 备用增益，v3.9.3 已改用 lineSteer）
static float    pathLatMax    = 0.90f;  // 【直道】横向修正限幅 [rad/s]（v3.9.6 0.55→0.90：直道权限够）
// ★★★v3.9.6 【弧段】横向修正限幅——这是 run0616 六弧崩掉的直接肇因
//   R200@v0.24 只要 ω=1.2，内轮就 10.2 rad/s，已经贴着 M0 失速点（实测 9.0 rad/s 就卡住）。
//   弧上再把 corr 放到 ±0.55 ⇒ wCmd 1.75 被 wDipMin 钳到 1.45 ⇒ 内轮 9.0 ⇒ 粘滑失速
//   ⇒ 实测 ω 只有 0.52（要求 1.2）⇒ R 变 455 mm ⇒ 车直接跑出赛道。
//   所以弧上只允许 ±0.18 的微调：几何前馈负责转，灰度只修几毫米的横向。
static float    pathArcLatMax = 0.18f;  // 【弧段】横向修正限幅 [rad/s]

// ★★★v3.9.2 堵转/碰撞保护
//   run59 撞上后整整 13 s：yaw 纹丝不动（-1229.2°），vCmd 还在给 0.24，轮子在原地打滑
//   ⇒ vMeas 一点都不小（0.19~0.29），靠"速度为零"判不出来。必须判【进度】：
//   直线段看里程 pathSegS、弧段看已转角度 pathYawAcc，1.2 s 没有增量 ⇒ 判定卡死并停车。
static float    pathPrevProg  = 0.0f;
static uint32_t pathProgMs    = 0;

static const char *pathKindName(uint8_t k)
{
    return (k == PS_LINE) ? "LINE" : ((k == PS_ARC) ? "ARC" : "PIVOT");
}

static void pathStartSeg(int i);

static void pathStop()
{
    pathOn       = false;
    pathArm      = false;
    rotThenChain = false;
    robotMotion.forward = 0.0f;
    robotMotion.turn    = 0.0f;
    PID_VEL.reset();
}

// 当前段完成：理想航向累加设计转角 → 起下一段（或整链结束）
static void pathFinishSeg()
{
    if (pathI < 0 || pathI >= pathN) { pathStop(); return; }
    const PathSeg sg = pathQ[pathI];                 // 值拷贝：pathI 下面会变
    if (sg.kind != PS_LINE) pathIdealYaw = wrapPi(pathIdealYaw + sg.a);
    Serial.printf("[PATH] seg %d/%d %s done  idealYaw=%.1f°  (yaw实=%.1f°)\n",
                  pathI + 1, pathN, pathKindName(sg.kind),
                  pathIdealYaw * 57.2958f, robotPose.yaw * 57.2958f);
    pathI++;
    if (pathI >= pathN) {
        Serial.println("[PATH] ALL DONE");
        pathStop();
        return;
    }
    pathStartSeg(pathI);
}

static void pathStartSeg(int i)
{
    if (i < 0 || i >= pathN) { pathStop(); return; }
    pathI       = i;
    PathSeg &sg = pathQ[i];
    pathSegS    = 0.0f;
    pathYawAcc  = 0.0f;
    pathPrevYaw = robotPose.yaw;
    pathSegMs   = millis();
    pathMicros  = 0;
    pathPrevProg = 0.0f;              // ★v3.9.2 卡死检测的进度基准
    pathProgMs   = millis();
    directMode  = false;
    moveOn      = false;
    moveSettleOn= false;

    if (sg.kind == PS_LINE) {
        robotMotion.turn = 0.0f;
        // 航向基准钉在理想航向（不是实测 yaw）⇒ 单段角度误差不累积
        yawHold      = pathIdealYaw;
        yawHoldValid = true;
        holdActive   = imuOk;
        holdI        = 0.0f;
        pathV        = robotMotion.forward;
        Serial.printf("[PATH] seg %d/%d LINE  len=%.3f v=%.3f  yawHold=%.1f°\n",
                      i + 1, pathN, sg.a, sg.v, pathIdealYaw * 57.2958f);
    } else if (sg.kind == PS_ARC) {
        float w = sg.v / fmaxf(sg.r, 0.05f);
        if (sg.a < 0.0f) w = -w;
        robotMotion.turn = _constrain(w, -yawMax, yawMax);
        pathV = robotMotion.forward;
        pathArcI  = pathArcIff * w;      // ★v3.9.0 打滑前馈初值（符号跟转向）
        pathArcUs = 0;                   // 首拍不积分，避免跨段 dt 污染
        Serial.printf("[PATH] seg %d/%d ARC   deg=%+.1f v=%.3f R=%.3f ⇒ w=%+.2f rad/s  I0=%+.2f\n",
                      i + 1, pathN, sg.a * 57.2958f, sg.v, sg.r, robotMotion.turn, pathArcI);
    } else {
        rotTarget    = wrapPi(pathIdealYaw + sg.a);
        rotRateMax   = _constrain((sg.r > 0.01f) ? sg.r : 1.0f, 0.10f, 2.50f);
        rotStartMs   = millis();
        rotThenMove  = false;
        rotThenChain = true;
        rotOn        = true;
        robotMotion.forward = 0.0f;
        robotMotion.turn    = 0.0f;
        pathV = 0.0f;
        PID_VEL.reset();
        Serial.printf("[PATH] seg %d/%d PIVOT deg=%+.1f rate=%.2f  target=%.1f°\n",
                      i + 1, pathN, sg.a * 57.2958f, rotRateMax,
                      rotTarget * 57.2958f);
    }
}

// ---------------- 串口命令 ----------------
static bool     debugFrame     = false;
static uint32_t lastCmdMs      = 0;
static uint32_t gyroPrintUntil = 0;
static uint32_t lastLinkWarnMs = 0;

// ---- loop 频率计（命令 h 开关，默认关；用于判定"S3 每拍要多久"）----
// 无 delay 自由跑时，loop 周期直接决定 IMU 采样率与偏航环的相位滞后。
static bool     hzOn     = false;
static uint32_t hzWinUs  = 0;
static uint32_t hzCnt    = 0;
static uint32_t hzLastUs = 0;
static uint32_t hzMaxUs  = 0;
static uint32_t hzMinUs  = 0xFFFFFFFFu;

static void ackParams()
{
    Serial.printf("[OK] v=%.3f t=%.3f rw=%.4f lw=%.3f kt=%.3f velP=%.2f velI=%.2f dir=%d,%d direct=%d\n",
                  robotMotion.forward, robotMotion.turn,
                  Rw, Lw, turnKp, PID_VEL.P, PID_VEL.I, M0Dir, M1Dir, directMode ? 1 : 0);
}

// ★★v3.7.0 上电补发 S1 工作点。
//   为什么必须补：S1 的 W(爬升率)/kd/km/kc/kb/jc/ki/e 全部只存 RAM，掉电回出厂
//   （slew=5 rad/s²、brakeK=0.50、holdKi=0 …）。带电脑跑时这些由 _line_run.py 每次下发，
//   拔线自己跑就没人发了 —— 症状是"起步像卡死"（0.28 m/s 要 3.7 s 才爬上去）+ 停车段乱刹。
//   与 _line_run.py 的 WP 列表保持一致；改一边必须改另一边。
static void applyS1Workpoint()
{
    static const char *wp[] = {
        "W140",                                      // 爬升率（S1 默认 5；140 = run35 实测工作点，勿降）
        // ★v3.9.13 把 S1 速度环输出限幅 7.0→9.0 V（硬件钳位 10 V）。
        //   动机：0.6 m/s 时外轮在 R500 上要 45.6 rad/s ⇒ uq≈0.144×45.6=6.57 V，
        //   加带载 IR 约 0.7 V ⇒ 7.3 V > 7 V 限幅 ⇒ 外轮被削、差速建不起来 ⇒ 车转不够冲出线。
        //   这解释了"直线能跑（39.9 rad/s，余量 8%）、一进弧就冲（外轮超限）"。
        //   只抬限幅、不动 P/I/D（P=0.1 是 v3.14.1 实测值，P=0.2 会 bang-bang 自激，绝不能动）。
        //   低速段 uq 远小于 9 V，限幅根本不起作用 ⇒ 对 0.4 及以下无任何影响。
        "K0.1,1.0,0,9.0",
        "kd0.6", "km1.0", "kc1.1", "kb0", "jc0",
        "ki4.0", "e0.20"
    };
    for (uint8_t i = 0; i < sizeof(wp) / sizeof(wp[0]); i++) {
        motors.sendRaw(wp[i]);
        delay(15);
    }
    Serial.println("[BOOT] S1 工作点已补发（W140 + kd/km/kc/kb/jc/ki/e）");
    Serial.printf("[BOOT] ★v3.14.7-STOP = v3.14.6-FULLPASS + 终点停车（odo %.3f m / %.0f mm，前 %.0f mm 降速到 %.2f）\n",
                  LINE_STOP_ODO_MM * 0.001f, LINE_STOP_ODO_MM, LINE_STOP_BRAKE_MM, LINE_STOP_VMIN);
}

static void printHelp()
{
    Serial.println("[BIPEDAL] cmds:");
    Serial.println("  v<m/s> t<rad/s> w<L>,<R> rw<m> lw<m> kt<> kv<P>,<I> md<+-1>,<+-1>");
    Serial.println("  kh<P> kd<D> km<max>   (直行航向保持增益/阻尼/限幅；kh0 关)");
    Serial.println("  kc<max>              滑行段（done 后到停稳）航向权限 [rad/s]，默认 1.10；0=关");
    Serial.println("  kb<gain>             settle 主动制动增益（forward=-kb·vMeas），默认 0.50；0=关");
    Serial.println("  jc<0|1>              J coast 开关（0=R近0对称[M0 voltage], 1=J自由衰减），默认 0 ★v2.9.2");
    Serial.println("  x<dist>,<vmax>[,<acc>[,<dec>[,<scale>[,<decEff>[,<keepYaw>]]]]]  定距移动（x 单独=中止）");
    Serial.println("      keepYaw=1 ⇒ 沿用上一段航向（偏差 >1.2° 时先【原地转正】再走直线；默认 0=重新锁存）");
    Serial.println("  T<deg>[,<rate>]      原地转相对角度（正=左转/逆时针，默认 1.0 rad/s），转完更新航向基准");
    Serial.println("  ★v3.0.0 几何链：G 装配 → L/A/P 入队 → Q 启动 / X 中止（整链下发，段间无缝不停车）");
    Serial.println("      L<len>,<v>[,<acc>]        直线段（按里程终止）");
    Serial.println("      A<deg>,<v>,<R>[,<acc>]    弧段（按 Δyaw 终止；turn=±v/R，R=场地弧半径）");
    Serial.println("      P<deg>[,<rate>]           原地转（不相切尖点，如场地 3 处 90°）");
    Serial.println("  j<decEff>            刹车坡度上限 = 车真能做到的减速度 [m/s²]（默认 1.50）");
    Serial.println("  e<sec>               停车提前量用的轮速环延迟 [s]（默认 0.075）");
    Serial.println("  @<line>              原样转发给 S1（如 @W30 改斜率）");
    Serial.println("  z                    空闲松闸开关 ★v3.0.2：手推/手转标定用（DRV 高阻，轮子自由）");
    Serial.println("  ★v3.2.0 七路灰度 + 巡线： y=自检  yp<mm>=灯距  ye=零点  yi=极性  d2=20Hz 帧");
    Serial.println("                        yf=巡线开关  yk<kg>=横向增益  yl<LDmm>=前瞻（配 v<speed> 用）");
    Serial.println("  ★v3.3.0 转角： yc<deg>=固定方向转角  ylo<mm>=车头前伸 L_f（转角前先走它，0=一到就转）"
                   "  yj<val>=横向积分");
    Serial.println("  ★v3.6.0 转角方向自适应： yr<deg>=开自适应+设量级(默认90；yr0=关)"
                   "  横边在右(0011111)⇒右转 / 在左(1111100)⇒左转");
    Serial.println("  ★v3.4.0 直接舵量： ym<0|1>=0 航向串级/1 经典(默认)  ya<Kp>=舵量P  yd<Kd>=舵量D  yt<max>=舵量限幅");
    Serial.println("  ★v3.6.0 弯道 I 项： yz<ki>=I 增益(默认3)  yb<trim>=手动常值补偿");
    Serial.println("  ★v3.8.3 大偏差增速： yq<gain>=超出 6.8 mm(一档)后的额外斜率(默认150，0=关)");
    Serial.println("           用**原始** e 触发（滤波后的 lineE 慢 0.1 s，R200 来不及）；e=20 mm 就顶满舵");
    Serial.println("  ★v3.8.0 曲率前馈： yw<max>=前馈限幅(默认1.60，0=关)  把弧段所需常值舵量学出来，");
    Serial.println("           位置环就不必靠 63 mm 的大偏差去换 R0.20 m 的舵量（这是 R1~R6 拐不过去的根因）");
    Serial.println("  ★v3.7.0 无线上电自跑： yn=状态  yn1=开  yn0=关  ynv<v>=自启速度(默认0.28)");
    Serial.println("           上电 → 等 IMU 标零 → 看到线且车静止 1.5 s → 倒计时 3 s → 自己开巡线；");
    Serial.println("           停下后不会自己再跑（要重跑按复位/重新上电）。");
    Serial.println("  g c h i pt<rad> d0 d1 s ?");
}

static void handleCommand(String cmd)
{
    cmd.trim();
    if (cmd.length() == 0) return;
    lastCmdMs = millis();
    const char *s = cmd.c_str();
    float a, b;
    int d0, d1;

    switch (s[0]) {
    case 'v':
        if (sscanf(s + 1, "%f", &a) == 1) {
            if (pathOn) { pathStop(); Serial.println("[PATH] 被手动 v 终止"); }
            robotMotion.forward = _constrain(a, -speedMax, speedMax);
            directMode = false;
            ackParams();
        }
        break;
    case 't':
        if (sscanf(s + 1, "%f", &a) == 1) {
            if (pathOn) { pathStop(); Serial.println("[PATH] 被手动 t 终止"); }
            robotMotion.turn = _constrain(a, -yawMax, yawMax);
            directMode = false;
            ackParams();
        }
        break;
    case 'w':
        if (sscanf(s + 1, "%f,%f", &a, &b) == 2) {
            directWL = _constrain(a, -wheelMax, wheelMax);
            directWR = _constrain(b, -wheelMax, wheelMax);
            directMode = true;
            ackParams();
        }
        break;
    case 'r':
        if (cmd.length() >= 3 && s[1] == 'w' && sscanf(s + 2, "%f", &a) == 1 && a > 0.001f) {
            Rw = a; ackParams();
        }
        break;
    case 'l':
        if (cmd.length() >= 3 && s[1] == 'w' && sscanf(s + 2, "%f", &a) == 1 && a > 0.001f) {
            Lw = a; ackParams();
        }
        break;
    case 'k':
        if (cmd.length() < 3) break;
        if (s[1] == 't' && sscanf(s + 2, "%f", &a) == 1) {
            turnKp = a; ackParams();
        } else if (s[1] == 'v' && sscanf(s + 2, "%f,%f", &a, &b) == 2) {
            PID_VEL.P = a; PID_VEL.I = b; ackParams();
        } else if (s[1] == 'h' && sscanf(s + 2, "%f", &a) == 1) {
            holdKp = (a < 0.0f) ? 0.0f : a;        // 0 = 关掉航向保持（退化为纯开环直行）
            Serial.printf("[OK] holdKp=%.2f  kd=%.2f  max=%.2f\n", holdKp, holdKd, holdMax);
        } else if (s[1] == 'd' && sscanf(s + 2, "%f", &a) == 1) {
            holdKd = (a < 0.0f) ? 0.0f : a;
            Serial.printf("[OK] holdKp=%.2f  kd=%.2f  max=%.2f\n", holdKp, holdKd, holdMax);
        } else if (s[1] == 'm' && sscanf(s + 2, "%f", &a) == 1) {
            holdMax = _constrain(a, 0.05f, yawMax);
            Serial.printf("[OK] holdKp=%.2f  kd=%.2f  max=%.2f\n", holdKp, holdKd, holdMax);
        } else if (s[1] == 'c' && sscanf(s + 2, "%f", &a) == 1) {
            // ★v2.8.0：滑行段航向环限幅（0 = 滑行段不做航向修正，退回 v2.7.0 行为）
            holdMaxCoast = _constrain(a, 0.0f, yawMax);
            Serial.printf("[OK] holdMaxCoast=%.2f rad/s  (滑行段航向权限, 0=关)\n", holdMaxCoast);
        } else if (s[1] == 'i' && sscanf(s + 2, "%f", &a) == 1) {
            // ★v2.9.4 航向保持 I 增益（0=关）。治 P-only 稳态下垂：方框实测巡航 eYaw
            //   恒 +1~5°（所需差速 0.2~0.35 rad/s / kh=2 ⇒ eYaw 下垂 3~10°），
            //   每边走斜 6~8 cm。ki=4 时 ~0.7 s 即可顶起 0.2 rad/s 的纠偏量。
            holdKi = (a < 0.0f) ? 0.0f : a;
            Serial.printf("[OK] holdKp=%.2f ki=%.2f kd=%.2f max=%.2f\n",
                          holdKp, holdKi, holdKd, holdMax);
        } else if (s[1] == 'b' && sscanf(s + 2, "%f", &a) == 1) {
            // ★v2.9.0：settle 主动制动增益（0 = 关，退回 v2.8.0 自由滑行）
            brakeK = _constrain(a, 0.0f, 2.0f);
            Serial.printf("[OK] brakeK=%.2f  (settle 主动制动, 0=关)\n", brakeK);
        } else if (s[1] == 'w' && sscanf(s + 2, "%f", &a) == 1) {
            // ★v3.9.9 轮速上限在线设置：kw<rad/s>
            //   动机：台架实测 60 rad/s 时 iq 仅 0.24 A，说明 61 rad/s 是 wheelMax 这个
            //   软件钳位、不是物理极限。要探真正的天花板必须能改它，否则每试一级要重烧。
            wheelMax = _constrain(a, 5.0f, 200.0f);
            Serial.printf("[OK] wheelMax=%.1f rad/s  (%.3f m/s)\n", wheelMax, wheelMax * Rw);
        }
        break;
    case 'm':
        if (cmd.length() >= 4 && s[1] == 'd' &&
            sscanf(s + 2, "%d,%d", &d0, &d1) == 2 &&
            (d0 == 1 || d0 == -1) && (d1 == 1 || d1 == -1)) {
            M0Dir = d0; M1Dir = d1; ackParams();
        }
        break;
    case 'j':   // ★v2.5.0 设定停车滑行补偿用的实测减速能力
        if (cmd.length() >= 3 && s[1] == 'c') {
            // ★v2.9.4 修复：jc0/jc1 原来写在 case 'k' 的 'c' 分支里，永远不可达
            //   （"jc" 落在本 case，sscanf("c..","%f") 失败即 break）⇒ 在线开关一直
            //   是静默无效。coastEnabled 默认 false 恰是想要值，历史测试未受影响。
            coastEnabled = (s[2] == '1');
            Serial.printf("[OK] coastEnabled=%d  (settle J coast, 0=R近0对称, 1=J自由衰减)\n",
                          coastEnabled ? 1 : 0);
        } else if (sscanf(s + 1, "%f", &a) == 1) {
            moveDecEff = _constrain(a, 0.10f, 5.00f);
            Serial.printf("[OK] moveDecEff=%.3f m/s²  (刹车坡度上限 = min(dec, 本值))\n", moveDecEff);
        }
        break;
    case 'e':   // ★v2.5.4 S1 轮速环等效延迟 [s]（停车提前量 = vmax·e）
        if (sscanf(s + 1, "%f", &a) == 1) {
            moveLead = _constrain(a, 0.0f, 0.40f);
            Serial.printf("[OK] moveLead=%.3f s  (提前量 = v·τ + aτ²/2)\n", moveLead);
        }
        break;
    case 'x': {
        // ★v3.0.0：x 与几何链互斥 —— 下发 x 时先把链停掉（避免两套状态机抢 forward/turn）
        if (pathOn || pathArm) {
            pathStop();
            pathN = 0; pathI = 0;
            Serial.println("[PATH] 被 x 命令终止");
        }
        //   单独一条 `x` = 中止。航向由 hold 环负责，这里只管纵向。
        //   v2.5.0 加第 6 参 decEff（停车滑行补偿用的实测减速能力）。
        //   ★v2.6.0 加第 7 参 keepYaw（0=重新锁存 / 1=沿用上一段航向）。
        //     分段跑（如 1m×1m 方框）时，每段停稳会被停机枢轴带偏 3~11°；
        //     若下一段重新锁存，就把这个偏角当成了新基准（1 m 直线会偏 5~19 cm）。
        //     置 1 则保留上一段的 yawHold，让 hold 环在行进中把朝向拉回原基准。
        float d, v, ac, dc, sc, de, kp;
        int n = sscanf(s + 1, "%f,%f,%f,%f,%f,%f,%f", &d, &v, &ac, &dc, &sc, &de, &kp);
        if (n < 2) {
            moveOn = false;
            moveSettleOn = false;
            rotOn  = false;
            rotThenMove = false;
            robotMotion.forward = 0.0f;
            robotMotion.turn    = 0.0f;
            PID_VEL.reset();
            Serial.println("[MOVE] abort");
            break;
        }
        float dd = _constrain(d, 0.010f, 20.0f);
        moveVmax = _constrain(v, 0.005f, speedMax);
        if (n >= 3) moveAcc   = _constrain(ac, 0.02f, 5.00f);
        if (n >= 4) moveDec   = _constrain(dc, 0.02f, 5.00f);
        if (n >= 5) moveScale = _constrain(sc, 0.80f, 1.25f);
        if (n >= 6) moveDecEff = _constrain(de, 0.10f, 5.00f);
        // ★v2.6.0 第 7 参：沿用上一段航向
        bool  keepYaw = false;
        float dYaw0   = 0.0f;
        if (n >= 7 && kp > 0.5f) {
            if (!yawHoldValid || !imuOk) {
                Serial.println("[MOVE] keep 被拒（无历史航向 / IMU 未就绪）⇒ 重新锁存");
            } else {
                dYaw0 = wrapPi(yawHold - robotPose.yaw);
                if (fabsf(dYaw0) > 1.0472f) {                 // >60°：车被挪动过，不能沿用
                    Serial.printf("[MOVE] keep 被拒（Δ=%.1f° > 60°，车被挪动过）⇒ 重新锁存\n",
                                  dYaw0 * 57.2958f);
                } else {
                    keepYaw = true;
                    Serial.printf("[MOVE] keep yawHold=%.1f°  cur=%.1f°  Δ=%+.1f°\n",
                                  yawHold * 57.2958f, robotPose.yaw * 57.2958f,
                                  dYaw0 * 57.2958f);
                }
            }
        }

        moveReqDist  = dd;
        moveKeepYaw  = keepYaw;
        moveOn       = false;
        moveSettleOn = false;
        directMode   = false;

        // ★v2.7.0 keep 的正确用法：起步【前】原地把航向对到 yawHold，再走直线。
        //   M085 实证：边走边纠会走成弧线，横偏 ≈ L·Δθ/2（10.5° → 8.6 cm）。
        if (keepYaw && fabsf(dYaw0) > 1.2f * 0.0174533f) {
            rotOn       = true;
            rotTarget   = yawHold;
            rotRateMax  = 0.9f;
            rotStartMs  = millis();
            rotThenMove = true;
            robotMotion.forward = 0.0f;
            robotMotion.turn    = 0.0f;
            PID_VEL.reset();
            Serial.printf("[MOVE] 先原地对齐 %+.1f° → 再走 %.3f m\n", dYaw0 * 57.2958f, dd);
            break;
        }
        beginMove();
        break;
    }
    case 'T': {
        // ★v2.7.0 原地转相对角度：T<deg>[,<rate>]（deg 带符号，正 = 左转/逆时针）
        //   陀螺闭环；转完把 yawHold 更新为当前朝向 ⇒ 后面可以接 `x...,1`。
        //   ★注意：小写 `t` 已被"设偏航率"占用（robotMotion.turn），所以用大写 T。
        float deg = 0.0f, r = 1.0f;
        int n = sscanf(s + 1, "%f,%f", &deg, &r);
        if (n < 1) { Serial.println("[ERR] T<deg>[,<rate>]"); break; }
        if (!imuOk) { Serial.println("[ERR] IMU 未就绪，无法原地转"); break; }
        // ★v2.9.3：转角基准从"当前 yaw"改为"本段锁定的 yawHold"（有效时）。
        //   根因：settle 段尾部漂移 +7° 发生在 T90 之前，旧版 target=yaw+90 把漂移
        //   焊死进下一段航向 ⇒ 方框 4 段累计横向 ~53 cm（眼测）。
        //   以 yawHold 为基准 ⇒ 漂移被转角本身纠掉（转角精度 ±1°）。
        //   无 yawHold（首次/被 s 清除）时退回旧行为（从当前 yaw 起算）。
        float tBase = yawHoldValid ? yawHold : robotPose.yaw;
        if (yawHoldValid) {
            float dHold = wrapPi(robotPose.yaw - yawHold) * 57.2958f;
            if (fabsf(dHold) > 0.5f)
                Serial.printf("[TURN] 吸收段尾漂移 %+.1f°（yawHold 基准）\n", dHold);
        }
        rotTarget   = wrapPi(tBase + deg * 0.0174533f);
        rotRateMax  = _constrain(r, 0.10f, 2.50f);
        rotStartMs  = millis();
        rotThenMove = false;
        rotOn       = true;
        moveOn      = false;
        moveSettleOn= false;
        directMode  = false;
        robotMotion.forward = 0.0f;
        robotMotion.turn    = 0.0f;
        PID_VEL.reset();
        Serial.printf("[TURN] target=%.1f° (Δ=%+.1f°) rate=%.2f rad/s\n",
                      rotTarget * 57.2958f, deg, rotRateMax);
        break;
    }
    // ---------------- ★v3.0.0 几何链：G 装配 → L/A/P 入队 → Q 启动 / X 中止 ----------------
    case 'G':
        pathStop();
        pathN = 0; pathI = 0; pathArm = true;
        Serial.printf("[PATH] armed (queue cleared, max %d segs)\n", PATH_MAX_SEG);
        break;
    case 'L': {   // L<len>,<v>[,<acc>]   直线段
        float len, v, ac;
        const int n = sscanf(s + 1, "%f,%f,%f", &len, &v, &ac);
        if (n < 2) { Serial.println("[ERR] L<len>,<v>[,<acc>]"); break; }
        if (!pathArm) { Serial.println("[ERR] 先发 G 进入装配模式"); break; }
        if (pathN >= PATH_MAX_SEG) { Serial.println("[ERR] 队列已满"); break; }
        PathSeg &q = pathQ[pathN++];
        q.kind = PS_LINE;
        q.a    = _constrain(len, 0.005f, 100.0f);
        q.v    = _constrain(v, 0.005f, speedMax);
        q.r    = 0.0f;
        q.acc  = (n >= 3) ? _constrain(ac, 0.02f, 3.00f) : 0.0f;
        Serial.printf("[PATH] +%02d LINE  len=%.3f v=%.3f%s\n",
                      pathN, q.a, q.v, (q.acc > 0.0f) ? " (带acc)" : "");
        break;
    }
    case 'A': {   // A<deg>,<v>,<R>[,<acc>]   弧段（按 Δyaw 终止）
        float deg, v, r, ac;
        const int n = sscanf(s + 1, "%f,%f,%f,%f", &deg, &v, &r, &ac);
        if (n < 3) { Serial.println("[ERR] A<deg>,<v>,<R>[,<acc>]"); break; }
        if (!pathArm) { Serial.println("[ERR] 先发 G 进入装配模式"); break; }
        if (pathN >= PATH_MAX_SEG) { Serial.println("[ERR] 队列已满"); break; }
        PathSeg &q = pathQ[pathN++];
        q.kind = PS_ARC;
        q.a    = _constrain(deg, -3600.0f, 3600.0f) * 0.0174533f;
        q.v    = _constrain(v, 0.005f, speedMax);
        q.r    = _constrain(r, 0.05f, 100.0f);
        q.acc  = (n >= 4) ? _constrain(ac, 0.02f, 3.00f) : 0.0f;
        Serial.printf("[PATH] +%02d ARC   deg=%+.1f v=%.3f R=%.3f ⇒ w=%+.2f%s\n",
                      pathN, q.a * 57.2958f, q.v, q.r,
                      (q.a < 0.0f ? -1.0f : 1.0f) * q.v / q.r, " rad/s");
        break;
    }
    case 'P': {   // P<deg>[,<rate>]   原地转（尖点）
        float deg, rt = 1.0f;
        const int n = sscanf(s + 1, "%f,%f", &deg, &rt);
        if (n < 1) { Serial.println("[ERR] P<deg>[,<rate>]"); break; }
        if (!pathArm) { Serial.println("[ERR] 先发 G 进入装配模式"); break; }
        if (pathN >= PATH_MAX_SEG) { Serial.println("[ERR] 队列已满"); break; }
        PathSeg &q = pathQ[pathN++];
        q.kind = PS_PIVOT;
        q.a    = _constrain(deg, -360.0f, 360.0f) * 0.0174533f;
        q.v    = 0.0f;
        q.r    = _constrain(rt, 0.10f, 2.50f);
        q.acc  = 0.0f;
        Serial.printf("[PATH] +%02d PIVOT deg=%+.1f rate=%.2f\n",
                      pathN, q.a * 57.2958f, q.r);
        break;
    }
    case 'Q':   // 启动整链
        if (pathN == 0) { Serial.println("[ERR] 队列为空（先 G 再入队）"); break; }
        if (!imuOk)     { Serial.println("[ERR] IMU 未就绪 ⇒ ARC/PIVOT 无法闭环"); break; }
        pathArm      = false;
        pathOn       = true;
        pathI        = 0;
        pathIdealYaw = wrapPi(robotPose.yaw);
        pathV        = 0.0f;
        pathSegMs    = millis();
        pathMicros   = 0;
        directMode   = false;
        // ★★v3.9.4 清掉巡线的学习量：几何链自带 ω=v/R 前馈，若再叠加巡线的
        //   lineArcFF（实测残留 1.206 rad/s）+ lineBias（-0.25），ARC 段的
        //   wCmd 会变成 1.2+1.2=2.4 ⇒ 直接过转冲出赛道。几何链与巡线的前馈只能有一份。
        lineArcFF     = 0.0f;
        lineBias      = 0.0f;
        lineSteerHold = 0.0f;
        lineE         = 0.0f;
        moveOn       = false;  moveSettleOn = false;
        rotOn        = false;  rotThenMove  = false;
        Serial.printf("[PATH] START %d segs  yaw0=%.1f°\n", pathN, pathIdealYaw * 57.2958f);
        pathStartSeg(0);
        break;
    case 'X':   // 中止整链
        Serial.printf("[PATH] abort @ seg %d/%d\n", pathI + 1, pathN);
        pathStop();
        break;
    case 'I': {  // ★v3.9.1 几何链 ARC 偏航率闭环增益：I<iff>,<ki>[,<kd>[,<latK>[,<arcLatMax>]]]
        float f1, f2, f3, f4, f5;
        const int n = sscanf(s + 1, "%f,%f,%f,%f,%f", &f1, &f2, &f3, &f4, &f5);
        if (n < 2) {
            Serial.printf("[ARC] iff=%.3f  ki=%.2f  kd=%.2f  latK=%.2f(直道±%.2f/弧±%.2f)  Ilim=%.2f"
                          "  (用法 I<iff>,<ki>[,<kd>[,<latK>[,<arcLatMax>]]])\n",
                          pathArcIff, pathArcKi, pathArcKd, pathLatK, pathLatMax,
                          pathArcLatMax, pathArcILim);
            break;
        }
        pathArcIff = _constrain(f1, 0.0f, 1.0f);
        pathArcKi  = _constrain(f2, 0.0f, 20.0f);
        if (n >= 3) pathArcKd     = _constrain(f3, 0.0f, 4.0f);
        if (n >= 4) pathLatK      = _constrain(f4, 0.0f, 20.0f);
        if (n >= 5) pathArcLatMax = _constrain(f5, 0.0f, 1.0f);
        Serial.printf("[ARC] iff=%.3f  ki=%.2f  kd=%.2f  latK=%.2f  arcLatMax=%.2f（只存 RAM）\n",
                      pathArcIff, pathArcKi, pathArcKd, pathLatK, pathArcLatMax);
        break;
    }
    case '@':
        // ★v2.4.0：把一行原样转给 S1，例如 @W30 改斜率、@K0.1,1.0,0,7 改速度环
        if (cmd.length() > 1) {
            motors.sendRaw(s + 1);
            Serial.printf("[TX>] %s\n", s + 1);
        }
        break;
    case 'p':
        if (cmd.length() >= 3 && s[1] == 't' && sscanf(s + 2, "%f", &a) == 1) {
            pitchProt = (a < 0.0f) ? 0.0f : a;
            Serial.printf("[OK] pitchProt=%.3f rad (%s)\n", pitchProt, pitchProt == 0.0f ? "OFF" : "on");
        }
        break;
    case 'g':
        if (!imuOk) { Serial.println("[ERR] IMU 未接，无法自检"); break; }
        gyroPrintUntil = millis() + 3000;
        Serial.println("[CHK] push nose LEFT, GyroZ must be POSITIVE");
        break;
    case 'c':
        if (!imuOk) { Serial.println("[ERR] IMU 未接"); break; }
        Serial.println("[CAL] keep robot STILL ~5s ...");
        mpu6050.ensureGyroRange();   // ★先确认量程，否则零偏会按错的换算系数记下来
        mpu6050.calGyroOffsets();
        gyroZf = 0.0f;      // ★重置低通状态：标定后静止真值应≈0，从 0 起重收敛（40 ms）
        gyroZfUs = 0;       //   dt 也重新起算，避免把标定那 ~3 s 当成一个 dt
        Serial.printf("[CAL] done  (GYRO_CONFIG=0x%02X  sens=%.1f)\n",
                      mpu6050.gyroCfgReg, mpu6050.sensLSB);
        break;
    case 'i': {   // IMU 寄存器体检：一眼看出量程是否掉档
        if (!imuOk) { Serial.println("[ERR] IMU 未接"); break; }
        int rg = mpu6050.ensureGyroRange();
        Serial.printf("[IMU] WHO_AM_I=0x%02X  PWR_MGMT_1=0x%02X  CONFIG=0x%02X  SMPLRT_DIV=%u\n",
                      mpu6050.peek(MPU6050_WHO_AM_I), mpu6050.peek(MPU6050_PWR_MGMT_1),
                      mpu6050.peek(MPU6050_CONFIG), mpu6050.peek(MPU6050_SMPLRT_DIV));
        Serial.printf("[IMU] GYRO_CONFIG=0x%02X (range bits 0x%02X)  ACCEL_CONFIG=0x%02X\n",
                      mpu6050.peek(MPU6050_GYRO_CONFIG), mpu6050.gyroCfgReg,
                      mpu6050.peek(MPU6050_ACCEL_CONFIG));
        Serial.printf("[IMU] sens=%.1f LSB/(dps)  state=%s  cfgLostEver=%d\n",
                      mpu6050.sensLSB,
                      rg == 0 ? "OK" : (rg == 1 ? "RESTORED" : "NOACK"),
                      mpu6050.cfgLost ? 1 : 0);
        break;
    }
    case 'h':
        hzOn     = !hzOn;
        hzWinUs  = micros();
        hzCnt    = 0;
        hzLastUs = 0;
        hzMaxUs  = 0;
        hzMinUs  = 0xFFFFFFFFu;
        Serial.printf("[HZ] loop-rate meter %s (1s window)\n", hzOn ? "ON" : "OFF");
        break;
    case 'd':
        if (cmd.length() > 1 && s[1] == '2') {          // ★v3.1.0 d2 = 灰度 20 Hz 帧
            grayDbg = !grayDbg;
            Serial.printf("[DBG] gray frame %s\n", grayDbg ? "ON" : "OFF");
        } else {
            debugFrame = (cmd.length() > 1 && s[1] == '1');
            Serial.printf("[DBG] frame %s\n", debugFrame ? "ON" : "OFF");
        }
        break;
    case 's':
        robotMotion.forward = 0.0f;
        robotMotion.turn    = 0.0f;
        directWL = directWR = 0.0f;
        directMode = false;          // 停止同时退出直通轮速模式（否则 w 之后 s 会卡在直通）
        PID_VEL.reset();             // ★必须清积分：否则残留 vTrim 会在下一次启动时把车带跑
        holdActive = false;          // ★v2.3.0：清航向锁存，下次起步重新取当前车头方向
        moveOn     = false;          // ★v2.4.0：停止同时终止定距移动
        moveSettleOn = false;
        rotOn      = false;          // ★v2.7.0：停止同时终止原地转
        rotThenMove= false;
        pathStop();                  // ★v3.0.0：停止同时终止几何链
        if (lineOn) { lineOn = false; Serial.println("[LINE] follow OFF（随 s 一起关）"); }  // ★v3.2.1
        lineAdvancing  = false;      // ★v3.3.0 清转角几何状态机
        lineJustTurned = false;
        lineReacq      = false;
        lineReacq = false; lineAfterTurn = false;   // ★v3.2.5 清转角状态机
        lineXcWatch = false; lineXcLost = 0; lineXcGood = 0;   // ★v3.10.13 清十字观察窗
        Serial.println("[STOP]");
        break;
    case 'y':   // ★v3.1.0 七路灰度（只读）
        if (cmd.length() > 1 && s[1] == 'p') {            // yp<pitch_mm>
            if (sscanf(s + 2, "%f", &a) == 1 && a > 1.0f && a < 60.0f) {
                grayPitch = a;
                grayUpdate();
                Serial.printf("[GRAY] pitch = %.2f mm\n", grayPitch);
            } else {
                Serial.println("[ERR] yp<pitch_mm>  (1~60)");
            }
        } else if (cmd.length() > 1 && s[1] == 'e') {      // ye：当前重心设为零点
            grayZero = 0.0f;
            grayUpdate();
            grayZero = grayE;
            Serial.printf("[GRAY] zero = %.2f mm（车压中线时此值应≈0，标定后 e 以此为0）\n", grayZero);
        } else if (cmd.length() > 1 && s[1] == 'i') {      // yi：极性切换（白线版 ↔ 黑线版）
            grayInv = !grayInv;
            grayUpdate();
            Serial.printf("[GRAY] polarity = %s（%s 视为压线）\n",
                          grayInv ? "BLACK-LINE" : "WHITE-LINE",
                          grayInv ? "HIGH" : "LOW");
        } else if (cmd.length() > 1 && s[1] == 'f') {      // yf：巡线闭环开关
            lineOn = !lineOn;
            if (lineOn) { holdActive = false; lineLostMs = 0; lineBias = 0.0f; lineArcFF = 0.0f; lineSteer = 0.0f; }   // 重新锁车头 + 清 I/前馈
            Serial.printf("[LINE] follow %s  kp=%.3f  LD=%.0f mm  mode=%s  (需 v>0 才会动)\n",
                          lineOn ? "ON" : "OFF", lineKp, lineLD * 1000.0f,
                          lineMode ? "直接舵量" : "航向串级");
        } else if (cmd.length() > 1 && s[1] == 'k') {      // yk<kg>：横向→航向增益
            if (sscanf(s + 2, "%f", &a) == 1 && a > 0.0f && a < 5.0f) {
                lineKp = a;
                Serial.printf("[LINE] kp = %.3f（13.4 mm 偏差 ⇒ %.1f° 目标航向修正）\n",
                              lineKp, 57.2958f * lineKp * atanf(0.0134f / lineLD));
            } else {
                Serial.println("[ERR] yk<kg>  (0~5)");
            }
        } else if (cmd.length() > 2 && s[1] == 'l' && s[2] == 'o') {  // ylo<mm>：L_f 前伸量
            if (sscanf(s + 3, "%f", &a) == 1 && a >= 0.0f && a <= 300.0f) {
                lineFwdMm = a;
                Serial.printf("[LINE] L_f = %.0f mm（灰度条中心→轮轴中点；转角前先走它）\n",
                              lineFwdMm);
            } else {
                Serial.println("[ERR] ylo<mm>  (0~300)");
            }
        } else if (cmd.length() > 2 && s[1] == 'l' && s[2] == 'r') {  // ★v3.10.0 ylr<rate>：丢线搜线舵量
            if (sscanf(s + 3, "%f", &a) == 1 && a >= 0.0f && a <= 2.00f) {
                lineSearchRate = a;
                Serial.printf("[LINE] 搜线舵量 = %.2f rad/s（%.0f°/s，700 ms 可转 %.0f°）%s\n",
                              lineSearchRate, 57.2958f * lineSearchRate,
                              57.2958f * lineSearchRate * 0.7f,
                              (a <= 0.0f) ? "  [已关闭：退回保持最后舵量]" : "");
            } else {
                Serial.println("[ERR] ylr<rate>  (0~2.00, 0=关)");
            }
        } else if (cmd.length() > 1 && s[1] == 'l') {      // yl<LD_mm>：前瞻距离
            if (sscanf(s + 2, "%f", &a) == 1 && a > 20.0f && a < 1000.0f) {
                lineLD = a * 0.001f;
                Serial.printf("[LINE] LD = %.0f mm\n", lineLD * 1000.0f);
            } else {
                Serial.println("[ERR] yl<LD_mm>  (20~1000)");
            }
        } else if (cmd.length() > 1 && s[1] == 's') {      // ys±1：横向修正符号（装反/装后时用）
            if (sscanf(s + 2, "%f", &a) == 1) {
                lineSign = (a < 0.0f) ? -1.0f : 1.0f;
                Serial.printf("[LINE] sign = %+.0f（%s）\n", lineSign,
                              lineSign > 0 ? "传感器在轮轴前" : "★反相：传感器在轮轴后/装反");
            } else {
                Serial.println("[ERR] ys<±1>");
            }
        } else if (cmd.length() > 1 && s[1] == 'c') {      // yc<deg>：固定方向转角（0=只停）
            if (sscanf(s + 2, "%f", &a) == 1 && fabsf(a) <= 180.0f) {
                lineCornerDeg  = a;
                lineCornerAuto = false;                   // ★显式指定方向 ⇒ 关掉自适应
                if (fabsf(a) < 1.0f)
                    Serial.println("[LINE] corner turn = 0°（只停车；方向自适应已关）");
                else
                    Serial.printf("[LINE] corner turn = %+.0f°（固定%s；方向自适应已关，要用写 yr<deg>）\n",
                                  lineCornerDeg, a > 0.0f ? "左转" : "右转");
            } else {
                Serial.println("[ERR] yc<deg>  (-180~180, 0=只停)");
            }
        } else if (cmd.length() > 1 && s[1] == 'r') {      // ★v3.6.0 yr<deg>：方向自适应 + 转角量级
            if (sscanf(s + 2, "%f", &a) == 1 && fabsf(a) <= 180.0f) {
                if (fabsf(a) < 1.0f) {
                    lineCornerAuto = false;
                    Serial.println("[LINE] corner 方向自适应 OFF（改用 yc<deg> 的固定角度）");
                } else {
                    lineCornerAuto = true;
                    lineCornerMag  = fabsf(a);
                    Serial.printf("[LINE] corner 方向自适应 ON，量级 %.0f°\n"
                                  "       判据：右侧黑(0011111/0111111)⇒右转  左侧黑(1111100/1111110)⇒左转\n",
                                  lineCornerMag);
                }
            } else {
                Serial.println("[ERR] yr<deg>  (0=关自适应, 1~180=量级)");
            }
        } else if (cmd.length() > 1 && s[1] == 'm') {      // ym<0|1>：巡线控制结构
            if (sscanf(s + 2, "%f", &a) == 1 && (a == 0.0f || a == 1.0f)) {
                lineMode = (uint8_t)a;
                Serial.printf("[LINE] mode = %s\n", lineMode
                              ? "1 直接舵量 turn=Kp·e+Kd·ė（经典循迹）"
                              : "0 航向串级（旧：e→目标航向→hold 环）");
            } else {
                Serial.println("[ERR] ym<0|1>");
            }
        } else if (cmd.length() > 1 && s[1] == 'a') {      // ya<kp>：直接舵量 P
            if (sscanf(s + 2, "%f", &a) == 1 && a >= 0.0f && a <= 100.0f) {
                lineKpD = a;
                Serial.printf("[LINE] Kp(steer) = %.2f rad/s per m（e=13.4 mm ⇒ %.2f rad/s = %.1f°/s）\n",
                              lineKpD, lineKpD * 0.0134f, 57.2958f * lineKpD * 0.0134f);
            } else {
                Serial.println("[ERR] ya<kp>  (0~100)");
            }
        } else if (cmd.length() > 1 && s[1] == 'd') {      // yd<kd>：直接舵量 D
            if (sscanf(s + 2, "%f", &a) == 1 && a >= 0.0f && a <= 100.0f) {
                lineKdD = a;
                Serial.printf("[LINE] Kd(steer) = %.2f rad/s per (m/s)（ė=0.078 m/s ⇒ %.2f rad/s）\n",
                              lineKdD, lineKdD * 0.078f);
            } else {
                Serial.println("[ERR] yd<kd>  (0~100)");
            }
        } else if (cmd.length() > 1 && s[1] == 't') {      // yt<max>：舵量限幅
            if (sscanf(s + 2, "%f", &a) == 1 && a > 0.05f && a <= 2.00f) {
                lineTurnMax = a;
                Serial.printf("[LINE] steer max = %.2f rad/s（%.0f°/s，R=%.3f m @v0.28）\n",
                              lineTurnMax, 57.2958f * lineTurnMax, 0.28f / lineTurnMax);
            } else {
                Serial.println("[ERR] yt<max>  (0.05~2.00)");
            }
        } else if (cmd.length() > 1 && s[1] == 'q') {      // ★v3.8.2 yq<gain>：大偏差增速斜率（0=关）
            if (sscanf(s + 2, "%f", &a) == 1 && a >= 0.0f && a <= 500.0f) {
                lineKboost = a;
                Serial.printf("[LINE] 大偏差增速 Kboost = %.1f rad/s per m（阈值 %.1f mm，封顶 %.2f）\n",
                              lineKboost, lineEboost * 1000.0f, lineBoostMax);
            } else {
                Serial.println("[ERR] yq<gain>  (0~500, 0=关)");
            }
        } else if (cmd.length() > 2 && s[1] == 'w') {      // ★v3.8.0 yw<max>：曲率前馈限幅（0=关）
            if (sscanf(s + 2, "%f", &a) == 1 && a >= 0.0f && a <= 2.00f) {
                lineArcFFMax = a;
                if (a <= 0.0f) lineArcFF = 0.0f;
                Serial.printf("[LINE] 曲率前馈 max = %.2f rad/s%s（学习 k=%.2f，τ %.2f/%.2f s）\n",
                              lineArcFFMax, a <= 0.0f ? "  [已关闭]" : "", lineArcK, lineArcTauUp, lineArcTauDn);
            } else {
                Serial.println("[ERR] yw<max>  (0~2.00, 0=关)");
            }
        } else if (cmd.length() > 1 && s[1] == 'j') {      // yj<val>：横向→航向基准的积分增益
            if (sscanf(s + 2, "%f", &a) == 1 && a >= 0.0f && a <= 5.0f) {
                lineKi = a;
                Serial.printf("[LINE] ki(basis) = %.2f  (e=10 mm ⇒ %.2f°/s)\n",
                              lineKi, lineKi * 0.010f * 57.2958f);
            } else {
                Serial.println("[ERR] yj<val>  (0~5)");
            }
        } else if (cmd.length() > 1 && s[1] == 'z') {      // yz<ki>：弯道稳态 I 增益
            if (sscanf(s + 2, "%f", &a) == 1 && a >= 0.0f && a <= 60.0f) {
                lineKiD = a;
                Serial.printf("[LINE] I(steer) = %.2f rad/s per (m·s)  (e=34 mm ⇒ %.2f rad/s/s)\n",
                              lineKiD, lineKiD * 0.034f);
            } else {
                Serial.println("[ERR] yz<ki>  (0~60)");
            }
        } else if (cmd.length() > 1 && s[1] == 'b') {      // yb<trim>：手动常值补偿（不被 I 覆盖）
            if (sscanf(s + 2, "%f", &a) == 1 && a >= -1.0f && a <= 1.0f) {
                lineTrim = a;
                lineBias = 0.0f;                          // 手标一次 ⇒ 学习量清零，从干净状态起跑
                Serial.printf("[LINE] trim = %+.3f rad/s（I 学习量已清零）\n", lineTrim);
            } else {
                Serial.println("[ERR] yb<trim>  (-1.0~1.0)");
            }
        } else if (cmd.length() > 2 && s[1] == 'v') {      // ★v3.10.8 yv<v> / yvd<mm>：出弧自动提档
            if (cmd.length() > 3 && s[2] == 'd') {
                if (sscanf(s + 3, "%f", &a) == 1 && a >= 0.0f && a <= 30000.0f) {
                    lineV2Odo = a;
                    Serial.printf("[LINE] 提档里程门限 = %.0f mm\n", lineV2Odo);
                } else {
                    Serial.println("[ERR] yvd<mm>  (0~30000)");
                }
            } else {
                if (sscanf(s + 2, "%f", &a) == 1 && a >= 0.0f && a <= 1.20f) {
                    lineV2     = a;
                    lineV2Done = false;
                    lineV2Ms   = 0;
                    Serial.printf("[LINE] 第二档速度 = %.2f m/s%s  里程门限 %.0f mm\n",
                                  lineV2, (lineV2 <= 0.0f) ? "  [已关闭]" : "", lineV2Odo);
                } else {
                    Serial.println("[ERR] yv<v>  (0~1.20，0=关)");
                }
            }
        } else if (cmd.length() > 1 && s[1] == 'n') {      // ★v3.7.0 yn<0|1> / ynv<v>：无线上电自跑
            if (cmd.length() > 2 && s[2] == 'v') {        // ★v3.9.10 上限 0.60→1.20，且写 NVS
                if (sscanf(s + 3, "%f", &a) == 1 && a > 0.05f && a <= 1.20f) {
                    autoRunSave(a);
                    // 起跑时增益按速度归一化（Kp∝v），所以稳态横偏 e_ss=(v/R)/Kp=0.28/(R·24)
                    // 与速度无关 ⇒ "能稳跑多大半径"是个常数，不受这次改速度影响。
                    const float kpEff = 24.0f * autoRunV / 0.28f;
                    const float rComf = 0.28f / (24.0f * 0.035f);   // e_ss ≤ 35 mm
                    const float rEdge = 0.28f / (24.0f * 0.040f);   // e_ss = 40 mm（视场边缘）
                    Serial.printf("[AUTO] 自启速度 = %.2f m/s（已存 NVS，断电保留）\n", autoRunV);
                    Serial.printf("[AUTO]   起跑增益 Kp=%.1f Kd=%.1f（按 v/0.28 归一化）\n",
                                  kpEff, 9.0f * sqrtf(autoRunV / 0.28f));
                    Serial.printf("[AUTO]   纯灰度可稳跑 R ≥ %.2f m；R < %.2f m 会出视场（与速度无关）\n",
                                  rComf, rEdge);
                    Serial.printf("[AUTO]   R=%.2f m 时 内轮 %.1f rad/s（门限 10） 外轮 %.1f rad/s（台架上限 93）\n",
                                  rComf,
                                  (autoRunV * (1.0f - 0.072f / rComf)) / 0.015045f,
                                  (autoRunV * (1.0f + 0.072f / rComf)) / 0.015045f);
                } else {
                    Serial.println("[ERR] ynv<v>  (0.05~1.20)");
                }
            } else if (cmd.length() > 2 && s[2] == 'a') {  // ★v3.9.10 yna：重新武装，免断电
                autoRunSt    = 0;
                autoRunOkMs  = 0;
                autoRunT0    = 0;
                autoRunTick  = 0;
                autoRunGzSum = 0.0f;
                autoRunGzN   = 0;
                Serial.printf("[AUTO] 已重新武装（v=%.2f），摆好车即自动起跑\n", autoRunV);
            } else if (cmd.length() > 2 && s[2] == 'r') {  // ★v3.9.10 ynr：速度恢复默认并清 NVS
                autoRunSave(0.28f);
                if (gPrefsOk) gPrefs.remove("auv");
                autoRunSt = 0; autoRunOkMs = 0;
                Serial.println("[AUTO] 已恢复默认 0.28 m/s，NVS 记录已清除");
            } else if (cmd.length() > 1 && cmd[1] == 'n' && cmd.length() >= 3 &&
                       (s[2] == '0' || s[2] == '1')) {
                autoRun = (s[2] == '1');               // ★v3.13.3 只改 RAM：yn0 仅供本次上电临时关，
                                                       //   不写 NVS ⇒ 下次上电仍是自动武装（完全离线）
                if (!autoRun) { autoRunSt = 2; }           // 关掉时同时撤销待起跑
                Serial.printf("[AUTO] 上电自跑 %s（本次上电有效，断电后恢复 ON；st=%d）\n",
                              autoRun ? "ON" : "OFF", (int)autoRunSt);
            } else {
                Serial.printf("[AUTO] 上电自跑 %s  v=%.2f  st=%d\n"
                              "        yn1 开 / yn0 关 / ynv<0.05~1.20> 改速度(存NVS) / yna 重新武装 / ynr 恢复默认\n",
                              autoRun ? "ON" : "OFF", autoRunV, (int)autoRunSt);
            }
        } else {                                          // y：单次自检
            grayUpdate();
            Serial.printf("[GRAY] pin:");
            for (uint8_t i = 0; i < GRAY_N; i++) Serial.printf(" %d", GRAY_PIN[i]);
            Serial.println();
            Serial.printf("[GRAY] lvl:");
            for (uint8_t i = 0; i < GRAY_N; i++) Serial.printf(" %d", digitalRead(GRAY_PIN[i]));
            Serial.println();
            Serial.printf("[GRAY] bits=%d%d%d%d%d%d%d  n=%d  e=%.1f mm  pitch=%.2f  zero=%.2f  pol=%s  %s\n",
                          grayBits&1,(grayBits>>1)&1,(grayBits>>2)&1,(grayBits>>3)&1,
                          (grayBits>>4)&1,(grayBits>>5)&1,(grayBits>>6)&1, grayCnt, grayE,
                          grayPitch, grayZero, grayInv ? "BLK" : "WHT",
                          grayCnt == 0 ? "NO-LINE(无压线)" :
                          (lineCorner ? (lineCornerDir > 0 ? "CORNER(左横边⇒左转)" : "CORNER(右横边⇒右转)") :
                           (grayCnt == GRAY_N ? "FULL(全压)" : "OK")));
            Serial.printf("[LINE] follow=%s  mode=%s  kp=%.3f  LD=%.0f mm  sign=%+.0f  eF=%.1f mm  valid=%d  lost=%lu ms\n",
                          lineOn ? "ON" : "OFF", lineMode ? "直接舵量" : "航向串级",
                          lineKp, lineLD * 1000.0f, lineSign, lineE,
                          (int)lineValid, (unsigned long)lineLostMs);
            Serial.printf("[LINE] corner: %s  量级 %.0f°  %s  L_f=%.0f mm\n",
                          lineCornerAuto ? "AUTO(按横边出现在哪侧)" : "FIXED",
                          lineCornerAuto ? lineCornerMag : fabsf(lineCornerDeg),
                          lineCornerAuto ? "左/右自动判" :
                          (lineCornerDeg < -0.5f ? "右转" : (lineCornerDeg > 0.5f ? "左转" : "只停")),
                          lineFwdMm);
            // ★v3.13.2 把速度策略也打出来，跑之前一眼能核对
            Serial.printf("[LINE] 速度: 起跑 %.2f m/s  第二档 %.2f m/s（门限 %.0f mm，当前 %s）\n",
                          autoRunV, lineV2, lineV2Odo,
                          lineV2Done ? "已提档" : (lineV2 > 0.0f ? "待提档" : "关闭"));
            if (lineMode) {
                Serial.printf("[LINE] steer: Kp=%.2f Kd=%.2f Ki=%.2f max=%.2f  ė=%+.3f m/s  trim=%+.3f I=%+.3f ⇒ turn=%+.3f rad/s (%.1f°/s)\n",
                              lineKpD, lineKdD, lineKiD, lineTurnMax, lineEDot, lineTrim, lineBias,
                              lineSteer, lineSteer * 57.2958f);
            } else {
                Serial.printf("[LINE] 航向串级: kp=%.3f ki=%.2f  修正=%+.1f°\n", lineKp, lineKi, lineW * 57.2958f);
            }
        }
        break;
    case 'z':   // ★v3.0.2 松闸：空闲时 disable 电机（DRV 高阻）⇒ 手推/手转无阻力
        standbyRelease = !standbyRelease;
        Serial.printf("[REL] standby release %s（空闲时松闸%s）\n",
                      standbyRelease ? "ON" : "OFF", standbyRelease ? "，任何运动指令自动解除" : "");
        break;
    case '?':
        printHelp();
        break;
    default:
        Serial.printf("[ERR] unknown: %s\n", s);
        break;
    }
}

static void readSerial()
{
    static String buf;
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n' || inChar == '\r') {
            if (buf.length() > 0) { handleCommand(buf); buf = ""; }
        } else {
            buf += inChar;
            // ★v2.5.3 32 → 96：x<dist>,<vmax>,<acc>,<dec>,<scale>,<decEff> 全长 37 字符，
            //   旧的 32 上限正好把带 decEff 的 6 参命令整条丢掉（现象是车不动，串口只回
            //   [ERR] unknown: 00 —— 即截断后的残尾）。@转发给 S1 的长命令也靠这个。
            if (buf.length() > 96) buf = "";
        }
    }
}

// ---------------- 主流程 ----------------
void setup()
{
    Serial.begin(921600);
    // ★v3.9.11 离线跑（拔掉 USB，只吃 12 V）关键一行：
    //   本板 Serial = HWCDC（USB-Serial/JTAG）。USB 一旦拔掉就没有主机消费 TX FIFO，
    //   按默认 tx 超时写串口会阻塞主循环 —— 表现是"拔线后车不跑/跑到一半停住"。
    //   给它 20 ms 上限：没人收就直接丢，绝不拖住控制环。
    Serial.setTxTimeoutMs(20);
#ifdef DBG_IMU
    {   // 台架专用：等主机发来一个字节再往下走，保证完整开机日志能被抓到
        //   （USB-CDC 若无人读取，前面的打印会丢）
        uint32_t t0 = millis();
        while (!Serial.available() && millis() - t0 < 8000) delay(10);
        while (Serial.available()) Serial.read();
    }
#endif

    Wire.begin(1, 2, 400000UL);   // SDA=GPIO1  SCL=GPIO2
    delay(300);                   // 给外扩板留上电稳定时间（见 i2cProbe 注释）

#ifdef DBG_IMU
    dbgDump("pre-probe");         // 老代码就是在这个时刻探测 → 这里应能看到 mpu_code=2
#endif

    imuOk   = i2cProbe(MPU6050_ADDR);   // 内部含延时重试，见函数注释
    // ★v3.9.8 二轮兜底：10-19 run0618 出现过上电后 imu=0（状态帧第 14 列为 0），
    //   同一块板子重新上电就好 ⇒ 更像"总线/芯片还没站稳"而不是芯片坏。
    //   首轮 5×120 ms 失败后，整条 I2C 复位一次再拉长到 10×150 ms。
    if (!imuOk) {
        Serial.println("[WARN] MPU6050 首轮探测失败 → 复位 I2C，1 s 后重试 10 次");
        Wire.end();
        delay(1000);
        Wire.begin(1, 2, 400000UL);
        delay(100);
        imuOk = i2cProbe(MPU6050_ADDR, 10, 150);
        Serial.printf("[IMU] 二轮探测 %s\n", imuOk ? "成功" : "仍失败（本次仅开环/巡线，无陀螺阻尼）");
    }
    servoOk = i2cProbe(PCA9685_ADDR);
#ifdef DBG_IMU
    dbgDump("detect-done");
#endif

    if (servoOk) {
        servos.init();
        setLegsFixedPose();        // 固定站姿，此后不再碰腿
        Serial.println("[OK] PCA9685 在线，腿已摆固定站姿");
    } else {
        Serial.println("[WARN] PCA9685 未检测到 → 跳过舵机/腿姿");
    }

    if (imuOk) {
        Serial.println("[..] MPU6050 在线，开始陀螺零偏标定，车必须静止 ~5s");
        mpu6050.init();            // 内部含 ensureGyroRange() + calGyroOffsets()
        int rg = mpu6050.ensureGyroRange();
        Serial.printf("[IMUCFG] GYRO_CONFIG[4:3]=0x%02X  sens=%.1f LSB/(dps)  %s\n",
                      mpu6050.gyroCfgReg, mpu6050.sensLSB,
                      rg == 0 ? "±500 OK" : (rg == 1 ? "!! 复位过，已纠正" : "!! I2C 无应答"));
        Serial.println("[OK] MPU6050 就绪");
    } else {
        Serial.println("[WARN] MPU6050 未检测到 → 偏航率环不生效，仅开环差速");
    }

    motors.init();                 // Serial2 @2Mbps, RX18/TX17；上电先发 S 停机
    // ★★v3.7.0 拔线自跑的前提：S1 的爬升率和工作点只存 RAM，必须每次上电由 S3 补发
    delay(400);                    // 等 S1 自己的 FOC/编码器初始化跑完再发，免得被它的 setup 吞掉
    applyS1Workpoint();

    // v3.1.0：7 路灰度输入（模块开漏，内部上拉到 3.3V）
    for (uint8_t i = 0; i < GRAY_N; i++) pinMode(GRAY_PIN[i], INPUT_PULLUP);
    delay(2);
    grayUpdate();
#ifdef DBG_IMU
    dbgDump("after-all-init");
#endif

    lastCmdMs = millis();
    Serial.println("\n[BIPEDAL] v3.14.6 ready.");  // v3.14.6: ★★★修「出四圆区过 A2/A3 冲太快直接冲出去」。①真因：v3.14.1 的"已知转角跳过观察窗"分支从不设 `lineRunV`（观察窗分支才设），而转角执行完用 `forward = lineRunV` 恢复速度 ⇒ 恢复的是 7.36 m 提档留下的 **0.40**！现在 known 分支补上 `lineRunV = robotMotion.forward`。②出区速度 0.28 → 0.25（`C4_EXIT_V`），A2→A3 只隔 283 mm，低速进转角。  // v3.14.5: ★★★丢线后长搜线：LINE_SEARCH_MS 1100→6000、停车门限→6000。  // v3.14.2: ★★★修「四圆区最后一个切点没换向」——换向必须跑满 45° 才交回。  // v3.14.1: 转角表驱动 + 跳过观察窗。  // v3.14.0: ★★★四圆区切点用【分叉签名】判据（实车已验证成功）。  // v3.13.4: 补回观察窗超时撤销 + 转角里程先验。  // v3.13.3: 完全离线版。  // v3.13.2: 起跑 0.28、6700 mm 提档 0.40。  // v3.13.0: 基线 v3.10.16 + 四圆区独立模块。
    autoRunLoad();                                  // ★必须在打印 [AUTO] 之前，否则打的是默认值
    Serial.printf("[GRAY] %d ch on GPIO", GRAY_N);
    for (uint8_t i = 0; i < GRAY_N; i++) Serial.printf(" %d", GRAY_PIN[i]);
    Serial.printf("  init bits=%d%d%d%d%d%d%d n=%d\n",
                  grayBits&1,(grayBits>>1)&1,(grayBits>>2)&1,(grayBits>>3)&1,
                  (grayBits>>4)&1,(grayBits>>5)&1,(grayBits>>6)&1, grayCnt);
    Serial.printf("[GRAY] PSRAM=%u B  flash=%u MB  core=%d MHz  (PSRAM=0 ⇒ IO35/36/37 安全)\n",
                  (unsigned)ESP.getPsramSize(), (unsigned)(ESP.getFlashChipSize() >> 20),
                  (int)getCpuFrequencyMhz());
    Serial.printf("[BIPEDAL] move: acc=%.2f dec=%.2f scale=%.4f m/s²\n", moveAcc, moveDec, moveScale);
    printHelp();
    ackParams();
    Serial.printf("[BIPEDAL] hold: kh=%.2f ki=%.2f kd=%.2f max=%.2f (kh0 = 关, ki0 = 无I)\n",
                  holdKp, holdKi, holdKd, holdMax);
    Serial.println("[BIPEDAL] 首次上电建议顺序： g 验证陀螺符号 → w10,-10 看两轮反向 → v0.2 试走");
    Serial.printf("[AUTO] 无线上电自跑 %s  v=%.2f（yn0 关 / yn1 开 / ynv<0.05~1.20> 改速度并存NVS / yna 重新武装 / ynr 恢复0.28）\n",
                  autoRun ? "ON" : "OFF", autoRunV);
}

void loop()
{
    static uint32_t lastDbgMs  = 0;
    static uint32_t lastGyroMs = 0;
    static bool     fallen     = false;

    readSerial();

    BLDCData = motors.getBLDCData();   // 顺便泵收发
    if (imuOk) getMPUValue();

    // ---- v3.1.0：7 路灰度采样（5 ms 节流）+ d2 调试帧（20 Hz）----
    {
        static uint32_t lastGrayMs = 0, lastGrayDbgMs = 0;
        uint32_t nowMs = millis();
        if (nowMs - lastGrayMs >= 5) {
            lastGrayMs = nowMs;
            grayUpdate();
            lineUpdate(0.005f);
            // ══ ★★v3.10.8 出弧自动提档（见变量声明处的推导）══
            {
                static bool linePrevOn = false;
                if (lineOn && !linePrevOn) {          // 巡线刚打开 ⇒ 里程/标志清零
                    lineOdoMm  = 0.0f;
                    lineV2Done = false;
                    lineV2Ms   = 0;
                }
                linePrevOn = lineOn;
                if (lineOn) lineOdoMm += fabsf(gVMeas) * 0.005f * 1000.0f;

                // ══ ★★★v3.13.0 四圆区状态机（独立附加块：c4On==false 时只走一个 else 复位）══
                //   段推进只看【偏航角】，里程只做入口粗门控与兜底撤离。
                if (C4_EN) {
                    if (!lineOn) {
                        c4On = false; c4Done = false; c4N = 0; c4ArmCnt = 0;
                    } else if (!c4Done) {
                        // ① 武装：里程窗内第一次出现"够紧的右转" ⇒ R16 入口
                        if (!c4On) {
                            const bool sig = (lineOdoMm >= C4_ARM_ODO_LO) &&
                                             (lineOdoMm <= C4_ARM_ODO_HI) &&
                                             (lineGyroF <= -C4_ARM_G);
                            if (sig) {
                                if (++c4ArmCnt >= C4_ARM_N) {
                                    c4On   = true;
                                    c4N    = 0;
                                    c4Sign = -1;
                                    c4Yaw0 = robotPose.yaw - (float)c4Sign * C4_ARM_LEAD;   // 检测晚 0.2 s ⇒ 基准补回真起点
                                    lineBias = 0.0f;
                                    c4HbMs  = nowMs;
                                    Serial.printf("[C4] ☆四圆区介入 @%.2f m  g=%+.2f ⇒ 基准 yaw=%.1f°  "
                                                  "前馈钉 %+.2f（灰度继续管横向）\n",
                                                  lineOdoMm * 0.001f, lineGyroF,
                                                  c4Yaw0 * 57.2958f,
                                                  (float)c4Sign * (C4_V / C4_R) * C4_FFK);
                                }
                            } else {
                                c4ArmCnt = 0;
                            }
                        }
                        // ② 段推进 / 切点翻号
                        if (c4On) {
                            const float dYaw = robotPose.yaw - c4Yaw0;          // 本段净转角（带符号）
                            if (c4N < C4_N) {
                                // ── ① 主判据：【分叉签名】——两条弧在切点同时进视场 ──
                                //   比等偏航角转满 175° 更早（前视 66 mm 就是提前量），
                                //   而且方向是"看出来的"（哪一侧张开），不靠翻转推断。
                                const int fk = c4ForkDir(grayBits);
                                bool fireFork = false;
                                if (fk != 0 && fabsf(dYaw) >= 1.00f &&          // 本段至少转过 57°，
                                    (uint32_t)(nowMs - c4LastMs) >= 800u) {    // 且距上次切点 ≥800 ms
                                    // 分叉方向必须与当前段曲率相反，否则是同侧误触
                                    if (((fk > 0) && (c4Sign < 0)) || ((fk < 0) && (c4Sign > 0))) {
                                        if (c4ForkPrev == fk) fireFork = true; // 连续 2 帧确认
                                    }
                                }
                                c4ForkPrev = fk;
                                // ── ② 兜底：偏航角 175°（分叉没采到时才用）──
                                const float trig = (c4N == 0) ? C4_SEG1_RAD : C4_SEG_RAD;
                                const bool fireYaw = (fabsf(dYaw) >= trig);
                                if (fireFork || fireYaw) {
                                    c4N++;
                                    c4Sign = fireFork ? ((fk > 0) ? +1 : -1) : -c4Sign;
                                    c4Yaw0 = robotPose.yaw;
                                    c4LastMs   = nowMs;
                                    c4ForkPrev = 0;
                                    lineBias = 0.0f;                        // 旧方向的 I 项不许带过来
                                    Serial.printf("[C4] ◆切点 #%d @%.2f m  %s  本段 %+.0f°  ⇒ 曲率翻%s  "
                                                  "前馈 %+.2f（g=%+.2f n=%d e=%.1f）\n",
                                                  c4N, lineOdoMm * 0.001f,
                                                  fireFork ? "【分叉】" : "【角度兜底】",
                                                  dYaw * 57.2958f,
                                                  (c4Sign < 0) ? "右" : "左",
                                                  (float)c4Sign * (fmaxf(fabsf(robotMotion.forward), 0.10f) / C4_R) * C4_FFK,
                                                  robotPose.GyroZ, (int)grayCnt, grayE);
                                    // ★★★v3.14.2 注意：这里【绝对不能】立刻退出！
                                    //   v3.14.1 就在这里 `c4On=false`，结果第 3 个切点（R18→R19）
                                    //   【没有换向】：c4Sign 刚翻成 +1，可 lineArcFF 的钉住是在
                                    //   lineUpdate 里靠 `if (c4On)` 执行的，这一帧之后 c4On 就没了
                                    //   ⇒ 前馈永远停在 R18 的 −1.07（右转），车顺着旧曲率一路右转出去。
                                    //   实测现象就是用户说的"最后一个切点没有换向"。
                                    //   ⇒ 换向必须【跑完一小段】才算数，交给下面 C4_END_RAD 那支处理。
                                }
                            } else if (fabsf(dYaw) >= C4_END_RAD) {
                                // ③ 收工：第 3 次换向后已走满 C4_END_RAD（45°≈236 mm）
                                //   ⇒ 车确实在新方向上稳定了，这才交回纯灰度。
                                c4On   = false;
                                c4Done = true;
                                lineCornerMs   = 0;      // ★清横边冷却：A2 立刻可识别
                                lineCornerArm  = true;   // ★允许重新上膛
                                lineCornerCntR = 0;
                                lineCornerCntL = 0;
                                lineXcWatch    = false;
                                lineXcDist     = 0.0f;
                                // ★★★v3.14.6 出区速度 0.28 → 0.25（用户 10-21 12:44 实测：
                                //   "转两个转角那里冲太快，直接冲出去了"）。出区点距 A2 只有约 700 mm，
                                //   而 A2/A3 之间只有 283 mm ⇒ 这段必须以低速进转角，否则动量带着车
                                //   越过转角点、原地转时轮轴已经偏出去 100 mm 以上，转完必出赛道。
                                robotMotion.forward = C4_EXIT_V;
                                Serial.printf("[C4] ★★ 四圆区走完 @%.2f m（%d 个切点，第 3 次换向后"
                                              "已走 %.0f°）⇒ 交回纯灰度，速度 %.2f m/s，距 A2 还有约 %.0f mm\n",
                                              lineOdoMm * 0.001f, c4N, dYaw * 57.2958f,
                                              C4_EXIT_V, (26762.0f - lineOdoMm));
                            }
                            if (c4On && lineOdoMm > C4_ODO_BAIL) {   // 兜底撤离
                                c4On = false; c4Done = true;
                                Serial.printf("[C4] !! 兜底撤离 @%.2f m（里程超 %.2f）\n",
                                              lineOdoMm * 0.001f, C4_ODO_BAIL * 0.001f);
                            }
                            if (c4On) {
                                // 区内速度钉 0.28：0.40 档实测偏航率 p50 只有 1.00，
                                // 而 R300 要 1.33 ⇒ 0.40 在这个区结构上走不了
                                robotMotion.forward = C4_V;
                                if ((uint32_t)(nowMs - c4HbMs) >= 500u) {
                                    c4HbMs = nowMs;
                                    Serial.printf("[C4] · %d/%d 本段 %+.0f°/%.0f°  sign=%d  arcff=%+.2f  "
                                                  "g=%+.2f  wCmd=%+.2f  n=%d e=%.1f  odo=%.2f\n",
                                                  c4N, C4_N, dYaw * 57.2958f,
                                                  ((c4N == 0) ? C4_SEG1_RAD : C4_SEG_RAD) * 57.2958f,
                                                  c4Sign, lineArcFF, robotPose.GyroZ, lineSteer,
                                                  (int)grayCnt, grayE, lineOdoMm * 0.001f);
                                }
                            }
                        }
                    }
                }

                if (lineOn && lineV2 > 0.0f && !lineV2Done) {
                    bool gate = (lineOdoMm >= lineV2Odo) &&
                                (fabsf(lineArcFF) < 0.30f) &&
                                !lineCorner && !lineAdvancing && !lineReacq && !lineJustTurned;
                    if (gate) {
                        if (lineV2Ms == 0) lineV2Ms = nowMs;
                        if (nowMs - lineV2Ms >= 500) {
                            float vFrom = lineRunV;
                            lineV2Done  = true;
                            lineRunV    = lineV2;
                            robotMotion.forward = lineV2;
                            Serial.printf("[LINE] ★出弧提档 %.2f -> %.2f m/s"
                                          "（里程 %.2f m / 门限 %.2f m，arcff=%.2f）\n",
                                          vFrom, lineV2,
                                          lineOdoMm * 0.001f, lineV2Odo * 0.001f, lineArcFF);
                        }
                    } else {
                        lineV2Ms = 0;
                    }
                }
            }
            // 丢线超时保护：巡线中连续 0.6 s 完全看不到线 ⇒ 停车并关掉巡线
            //   ★v3.2.4 由 1.5 s 缩到 0.6 s：1 m 测试线跑完还剩 1.5 s 就是 0.45 m，
            //     足够把车送出场地。0.6 s @0.30 m/s ≈ 0.18 m，刚好在线尾刹住。
            //   ★★v3.10.0 600→900 ms：搜线窗口本身放宽到 700 ms（要留出 1.2 rad/s 转 30° 的时间），
            //     停车门限必须在它之后，否则搜线还没转完就被停掉 ⇒ 保护形同虚设。
            //   ★★v3.10.2 900→1400 ms：搜线半径改成 155 mm（比 R200 更紧）后，收敛 40 mm 偏移
            //     需要约 0.9 s 的弧长 ⇒ 窗口 1100 ms、停车 1400 ms 才够用。
            //     安全性：搜线期间车在【转向】不是直冲，1.4 s @0.28 m/s 的实际位移 < 0.4 m。
            //     安全性：搜线期间车是在【转向】不是直冲，0.9 s 的实际前进位移 < 0.27 m。
            if (lineOn && !lineReacq && !lineAdvancing && !lineJustTurned &&
                lineLostMs > (c4On ? 2500u : 6000u)) {      // ★★★v3.14.5 停车门限 1400/1600→6000 ms
                                                            //   与 LINE_SEARCH_MS 同步：6 s 内一直主动找线，
                                                            //   6 s 还找不到才停（兜底防盲跑撞墙）。
                lineOn = false;
                robotMotion.forward = 0.0f;
                robotMotion.turn    = 0.0f;
                PID_VEL.reset();
                Serial.println("[LINE] !! 丢线 >6 s -> 停车（巡线已关闭）");
            }
            // ★v3.2.4 转角保护：1 m×1 m 场地，遇到 90° 横边就停，别撞墙
            // ★v3.2.5 `yc<deg>` 非 0 时改为：停车 → 原地转 → 低速找线 → 接着跟
            // ★★★v3.10.11 第一段：横边命中 ⇒ 只"上观察窗"，不立刻转（十字/直角此时无法区分）
            // ★★★v3.10.15 十字识别【整段关闭】，回到 v3.10.9 的"横边命中即转 90°"。
            //   理由（10-20 复盘）：十字识别治的是"L6 上两个十字被误转"，但它给 A1 真直角开了口子
            //   （v3.10.12 崩在这），而 A1 是所有六弧的必经入口 ⇒ 先保证主干能跑，再谈十字。
            //   ★★v3.10.16 重新打开（唯一改动）：v3.10.15 实测六弧全通（2.30 m A1 转 88.3° 落点 e=0.0；
            //   4.4~7.4 m 六弧✅；7.36 m 自动提档 0.40；10 m 直道稳跑），唯一崩点就是"横边命中即转"
            //   在 17.07 m 的十字上自拐出赛道 ⇒ 缺的正是这段十字识别。而 0.40 m/s 下 20 mm 横边
            //   只占一帧灰度(50 ms)，第一个十字漏采、第二个采到，纯属采样相位 ⇒ 不是控制问题。
            //   v3.10.13 已实测：本判据在 A1 上正确判"直角确认 n=0 观察 90 ms -> 转 88.8°"。
            // ══ ★★★v3.14.1 已知转角表（里程先验 + 固定方向）══
            //   用户 10-21 10:42 实测：四圆区稳稳跑过去了，但末尾 A2 的 90° 转不过去。
            //   把 v3.10.18 的里程先验升级成【表驱动】：
            //     MM  = 地图累计里程（_map_table.txt）⇒ 授权"这里允许转"
            //     DEG = 该处应转的角度（0 = 不指定，仍用灰度自适应方向）
            //   ★方向不再靠灰度判边：A2/A3 的横线是【垂直压在圆弧末端】上的（不是相切），
            //     7 路会同时全亮 ⇒ lineCornerDir 在那种情况下没有意义，甚至可能判反。
            //   全图：A1 2300(+90，自适应) / A2 26762(−90 右) / A3 27045(−90 右)
            //        / A4 27885(−44.5，自适应) / A5 34720(+90 左)
            static const float LINE_CORNER_MM[5]  = { 2300.0f, 26762.0f, 27045.0f, 27885.0f, 34720.0f };
            static const float LINE_CORNER_DEG[5] = { 0.0f,   -90.0f,   -90.0f,   0.0f,   +90.0f };
            #define LINE_CORNER_WIN 1300.0f
            int cornerIdx = -1;
            for (int ci = 0; ci < 5; ci++) {
                if (fabsf(lineOdoMm - LINE_CORNER_MM[ci]) <= LINE_CORNER_WIN) { cornerIdx = ci; break; }
            }
            const bool nearCorner = (cornerIdx >= 0);
            bool       knownHit   = false;      // ★v3.14.1 已知转角命中（doCornerTurn 在其后声明）
#define LINE_XC_EN 1

            // ★v3.14.1 已知转角【压到横线立刻转】，不再走 300 ms 观察窗。
            //   观察窗的唯一用途是区分"十字 vs 直角"，而里程表已经把两者分开了；
            //   多等 300 ms 只会让车带着弧上前馈（≈0.84 rad/s）继续往弧内侧偏 ——
            //   而 A2 要往【外】右转 90°，方向正好相反，这就是"过不去"的直接原因。
            if (LINE_XC_EN && lineOn && lineCorner && nearCorner && !lineAdvancing && !lineReacq &&
                !lineXcWatch && !c4On && (uint32_t)(nowMs - lineCornerMs) > 450u) {
                lineCornerArm  = false;
                lineCornerCntR = 0;
                lineCornerCntL = 0;
                lineXcDist     = 0.0f;
                knownHit       = true;
                // ★★★v3.14.6 关键补漏：known 分支必须像观察窗分支一样记住当前速度！
                //   转角执行完会用 `robotMotion.forward = lineRunV` 恢复速度，而这个分支
                //   （v3.14.1 加的"跳过观察窗"）从来没设过 lineRunV ⇒ 恢复的是 7.36 m 提档时
                //   留下的 **0.40**。这就是"出四圆圈后过 A2/A3 冲太快直接冲出去"的真因，
                //   比我原先以为的 0.28 太快严重得多。
                lineRunV       = robotMotion.forward;
                Serial.printf("[LINE] ★已知转角 #%d @%.2f m（表内 %.2f m）⇒ 压到横线直接转 %+.0f°  "
                              "v=%.2f  bits=%d%d%d%d%d%d%d n=%d\n",
                              cornerIdx + 1, lineOdoMm * 0.001f, LINE_CORNER_MM[cornerIdx] * 0.001f,
                              LINE_CORNER_DEG[cornerIdx], lineRunV,
                              grayBits&1,(grayBits>>1)&1,(grayBits>>2)&1,(grayBits>>3)&1,
                              (grayBits>>4)&1,(grayBits>>5)&1,(grayBits>>6)&1, (int)grayCnt);
            }
            if (LINE_XC_EN && lineOn && lineCorner && !lineAdvancing && !lineReacq && !lineXcWatch &&
                !c4On &&                       // ★v3.13.0 四圆区内没有直角，禁止十字/直角状态机抢方向
                !nearCorner &&                 // ★v3.14.1 已知转角已在上面直接转掉，这里只管"未知横边"
                (nowMs - lineCornerMs) > 1200) {
                lineXcWatch  = true;
                lineXcMs     = nowMs;
                lineXcDist   = 0.0f;
                lineXcDir    = lineCornerDir;
                lineXcLost   = 0;
                lineXcGood   = 0;
                lineRunV     = robotMotion.forward;
                lineCornerArm  = false;           // 同一条横线不许连触发
                lineCornerCntR = 0;
                lineCornerCntL = 0;
                Serial.printf("[LINE] 横边命中 %s  bits=%d%d%d%d%d%d%d  n=%d  yaw=%.1f° -> 先直行验证\n",
                              lineCornerDir > 0 ? "(线在左)" : "(线在右)",
                              grayBits&1,(grayBits>>1)&1,(grayBits>>2)&1,(grayBits>>3)&1,
                              (grayBits>>4)&1,(grayBits>>5)&1,(grayBits>>6)&1, (int)grayCnt,
                              robotPose.yaw * 57.2958f);
            }

            // ★★★v3.10.11 第二段：观察窗 —— 线继续直走 ⇒ 十字；变全黑 ⇒ 直角
            bool doCornerTurn = false;
            if (knownHit) doCornerTurn = true;   // ★v3.14.1 已知转角（表内）在上面已经判掉
            if (LINE_XC_EN && lineXcWatch) {
                if (!lineOn) {
                    lineXcWatch = false;                      // 巡线已关 ⇒ 放弃本次判定
                } else {
                    uint32_t xt = nowMs - lineXcMs;
                    lineXcDist += fabsf(gVMeas) * 0.005f;
                    // ★v3.10.13 "线在中心"必须持续，不能只看单帧
                    bool xcCenterOK = (xt >= XC_MIN_MS) && (grayCnt >= 1) && (grayCnt <= 3)
                                   && lineValid && (fabsf(grayE) <= XC_GOOD_E);
                    if (xcCenterOK) { if (lineXcGood < 200) lineXcGood++; }
                    else            { lineXcGood = 0; }

                    if (lineXcGood >= XC_GOOD_N) {
                        // ① 横向白线只有 ~20 mm 宽，穿过去后【纵向线回到车正下方】⇒ 这是十字
                        lineXcWatch  = false;
                        lineXcGood   = 0;
                        lineXcLost   = 0;
                        lineCornerMs = nowMs;                 // 借冷却，别在同一处反复判
                        robotMotion.turn = lineSteer;         // 立刻恢复循迹（不等下一拍）
                        Serial.printf("[LINE] ★十字（线持续在中心 n=%d e=%.1f mm，观察 %.0f ms / %.0f mm）-> 不转，继续循迹\n",
                                      (int)grayCnt, grayE, (double)xt, (double)(lineXcDist * 1000.0f));
                    } else {
                        if (grayCnt == 0 && xt >= XC_LOST_MIN_MS) {
                            if (lineXcLost < 200) lineXcLost++;
                        } else {
                            lineXcLost = 0;
                        }
                        if (lineXcLost >= XC_LOST_N) {
                            // ① n=0 连续 3 拍 ⇒ 线已经拐走 ⇒ 真直角
                            lineXcWatch   = false;
                            lineXcGood    = 0;
                            doCornerTurn  = true;
                            lineCornerDir = lineXcDir;        // 用命中那一刻的方向
                            Serial.printf("[LINE] 直角确认（n=%d 观察 %.0f ms / %.0f mm）-> 转\n",
                                          (int)grayCnt, (double)xt, (double)(lineXcDist * 1000.0f));
                        } else if (xt > XC_WATCH_MS) {
                            // ★★★v3.10.17（v3.13.4 补回）观察窗超时且【线一直在】⇒ 误报警，撤销转角。
                            //   依据（10-20 实测）："4 路连续白"分不出"横线压上来"和"普通线扫过去"——
                            //   硬打舵时一条普通线会逐帧点亮相邻通道：
                            //     1100000(n=2)→0100000→0110001→0110011→0110110→0111100→1111000（7 帧=350 ms 渐进生长）
                            //   而真横边是【一帧突变】0000110(n=2)→1111111(n=7)；真事件 86/95/95 ms 就判完。
                            //   代价不对称：误判"不转"只少转一个弯；误判"转"直接拐出赛道 ⇒ 默认撤销。
                            lineXcWatch   = false;
                            lineXcGood    = 0;
                            lineXcLost    = 0;
                            lineCornerMs  = nowMs;            // 借冷却，别在下一帧又触发同一条线
                            robotMotion.turn = lineSteer;     // 立刻恢复循迹（不等下一拍）
                            Serial.printf("[LINE] 观察窗超时：线一直在（n=%d e=%.1f）⇒ 误报警，撤销转角，继续循迹\n",
                                          (int)grayCnt, grayE);
                        }
                    }
                }
            } else if (!LINE_XC_EN && lineOn && lineCorner && !lineAdvancing && !lineReacq &&
                       (nowMs - lineCornerMs) > 1200) {
                // ★v3.10.15 = v3.10.9 行为：横边命中【直接转】，无观察窗 ⇒ L_f 全额 66 mm
                lineXcDist     = 0.0f;
                lineCornerArm  = false;
                lineCornerCntR = 0;
                lineCornerCntL = 0;
                doCornerTurn   = true;
                Serial.printf("[LINE] 横边命中 %s  bits=%d%d%d%d%d%d%d  n=%d  yaw=%.1f° -> 直接转（十字识别关闭）\n",
                              lineCornerDir > 0 ? "(线在左)" : "(线在右)",
                              grayBits&1,(grayBits>>1)&1,(grayBits>>2)&1,(grayBits>>3)&1,
                              (grayBits>>4)&1,(grayBits>>5)&1,(grayBits>>6)&1, (int)grayCnt,
                              robotPose.yaw * 57.2958f);
            }

            // 里程先验【拒绝】逻辑（v3.10.18 原义）：不在表内的转角一律当误报撤销。
            //   v3.14.1 起表已提到上方（cornerIdx / nearCorner），这里只保留拒绝动作。
            if (doCornerTurn && lineOn && !nearCorner) {
                Serial.printf("[LINE] 拒绝转角：里程 %.0f mm 不在任何已知转角 ±%.0f mm 内"
                              " ⇒ 当作误报警（十字），继续直行\n",
                              (double)lineOdoMm, (double)LINE_CORNER_WIN);
                doCornerTurn     = false;
                lineCornerMs     = nowMs;      // 借冷却，别在同一处反复判
                lineXcDist       = 0.0f;
                robotMotion.turn = lineSteer;  // 立刻恢复循迹
            }
            if (doCornerTurn && lineOn) {
                lineCornerMs = nowMs;
                robotMotion.turn = 0.0f;
                // ★★v3.14.1 方向优先级：里程表指定 > 灰度自适应 > yc 固定值
                //   表里 DEG≠0 的（A2/A3 右 90°、A5 左 90°）一律按表走，不再看灰度判边——
                //   那几条横线是垂直压在弧上的，7 路同时全亮，判边结果不可信。
                if (cornerIdx >= 0 && fabsf(LINE_CORNER_DEG[cornerIdx]) > 1.0f) {
                    lineCornerUse = LINE_CORNER_DEG[cornerIdx];
                } else {
                    lineCornerUse = lineCornerAuto
                                  ? ((lineCornerDir >= 0) ? +lineCornerMag : -lineCornerMag)
                                  : lineCornerDeg;
                }
                lineBias  = 0.0f;                 // ★新路段重新学 I，别把上一段的弧带进直道
                lineArcFF = 0.0f;                 // ★v3.8.0 前馈同样清零：转角后是全新方向的直线
                // ★★★v3.10.11 观察窗里已经直行了一段，L_f 必须扣掉，否则轮轴会停过转角点
                float lfRemain = lineFwdMm - lineXcDist * 1000.0f;
                if (lfRemain < 0.0f) lfRemain = 0.0f;
                lineAdvGoal = lfRemain * 0.001f;
                Serial.printf("[LINE] >> CORNER  %s  转 %+.0f°  L_f %.0f->%.0f mm\n",
                              lineCornerDir > 0 ? "左侧横边⇒左转" :
                              (lineCornerDir < 0 ? "右侧横边⇒右转" : "方向未定"),
                              lineCornerUse, lineFwdMm, lfRemain);
                if (fabsf(lineCornerUse) > 1.0f && lfRemain > 5.0f) {
                    // ★★v3.3.0 几何正解第一步：先前进 L_f，让轮轴落到转角点上
                    lineAdvancing = true;
                    lineAdvDist   = 0.0f;
                    lineAdvMs     = nowMs;
                    robotMotion.forward = 0.15f;
                    Serial.printf("[LINE] 先前进 L_f=%.0f mm 到转角点，再原地转 %+.0f°%s\n",
                                  lfRemain, lineCornerUse,
                                  lineCornerAuto ? "（自适应方向）" : "");
                } else if (fabsf(lineCornerUse) > 1.0f) {
                    lineOn = false;
                    robotMotion.forward = 0.0f;
                    PID_VEL.reset();
                    rotRateMax  = 1.0f;
                    rotStartMs  = millis();
                    rotTarget   = wrapPi(robotPose.yaw + lineCornerUse * 0.0174533f);
                    rotOn       = true;
                    lineAfterTurn = true;
                    Serial.printf("[LINE] corner turn %+.0f° (L_f=0)\n", lineCornerUse);
                } else {
                    lineOn = false;
                    robotMotion.forward = 0.0f;
                    PID_VEL.reset();
                    Serial.println("[LINE] corner auto-turn OFF (yc0) -> 停车");
                }
            }
            // ★★v3.3.0 转角几何正解第二步：L_f 走完 ⇒ 轮轴已在转角点 ⇒ 原地转 90°
            if (lineAdvancing) {
                lineAdvDist += fabsf(robotMotion.forward) * 0.005f;
                if (lineAdvDist >= lineAdvGoal || (nowMs - lineAdvMs) > 3000) {
                    lineAdvancing = false;
                    robotMotion.forward = 0.0f;
                    robotMotion.turn    = 0.0f;
                    PID_VEL.reset();
                    rotRateMax  = 1.0f;
                    rotStartMs  = millis();
                    rotTarget   = wrapPi(robotPose.yaw + lineCornerUse * 0.0174533f);
                    rotOn       = true;
                    lineAfterTurn = true;
                    Serial.printf("[LINE] L_f 到位 %.0f mm -> 原地转 %+.0f°\n",
                                  lineAdvDist * 1000.0f, lineCornerUse);
                } else {
                    robotMotion.forward = 0.15f;
                    robotMotion.turn    = 0.0f;
                }
            }
            // ★v3.3.0 刚转完的容忍窗口：0.6 s 内允许看不到线（L_f 有 ±15 mm 误差）
            if (lineJustTurned) {
                if (grayCnt >= 1 && grayCnt <= 3 && lineValid) {
                    lineJustTurned = false;
                    lineE      = grayE;
                    holdActive = false;
                    robotMotion.turn    = 0.0f;
                    robotMotion.forward = lineRunV;
                    Serial.printf("[LINE] 转后续上 e=%.1f mm -> v=%.2f\n", grayE, lineRunV);
                } else if (nowMs - lineJustTurnedMs > 600) {
                    lineJustTurned = false;
                    lineReacq      = true;
                    lineReacqMs    = nowMs;
                    lineReacqHit   = 0;
                    Serial.println("[LINE] 转后 0.6 s 未见线 -> 进入搜索");
                }
            }
            // ★v3.2.5 转角后找新线。为什么不能只"直着爬"：
            //   原地转是绕轮轴中点，而灰度条在车头 d≈100 mm 处 ⇒ 转完 90° 后传感器
            //   正好横移 d，落在新线外侧（>±40 mm 检测跨度）且与新线平行，直爬永远不会收敛。
            //   所以改成"低速爬 + 慢摆头扫"，用横向扫掠把新线扫进视场。
            if (lineReacq) {
                if (grayCnt >= 1 && grayCnt <= 3 && lineValid) {
                    if (++lineReacqHit >= 3) {                 // 连续 3 帧(150 ms) 才认，防边缘假捕获
                        lineReacq  = false;
                        lineReacqHit = 0;
                        lineE      = grayE;
                        holdActive = false;               // 重新锁当前车头（此后由 lineKi 慢慢拉正）
                        robotMotion.turn    = 0.0f;
                        robotMotion.forward = (fabsf(grayE) > 20.0f) ? 0.10f : lineRunV;
                        Serial.printf("[LINE] 咬住新线 e=%.1f mm -> v=%.2f\n",
                                      grayE, robotMotion.forward);
                    } else {
                        robotMotion.forward = 0.10f;
                    }
                } else {
                    lineReacqHit = 0;
                    if (nowMs - lineReacqMs > 5000) {
                        lineReacq = false;
                        lineOn    = false;
                        robotMotion.forward = 0.0f;
                        robotMotion.turn    = 0.0f;
                        PID_VEL.reset();
                        Serial.println("[LINE] !! 搜索 5 s 未咬住线 -> 停车");
                    } else if (nowMs - lineReacqMs > 300) {   // 前 0.3 s 可能还压在横边上，先离开再判
                        // ★v3.3.0 兜底搜索：L_f 若偏大/偏小，就朝【线所在的一侧】恒定偏置斜切，
                        //   2.5 s 后反向回摆一次（覆盖 L_f 估计误差两侧）。
                        float ph   = (nowMs - lineReacqMs) * 0.001f;
                        float bias = (lineCornerUse < 0.0f) ? +0.30f : -0.30f;  // 右转后线在车左
                        if (ph > 2.5f && ph < 3.8f) bias = -bias;
                        robotMotion.forward = 0.10f;
                        robotMotion.turn    = bias;
                    } else {
                        robotMotion.forward = 0.10f;
                        robotMotion.turn    = 0.0f;
                    }
                }
            }
            if (grayDbg && nowMs - lastGrayDbgMs >= 50) {
                lastGrayDbgMs = nowMs;
                char bb[GRAY_N + 1];
                for (uint8_t i = 0; i < GRAY_N; i++)
                    bb[i] = (grayBits & (uint8_t)(1u << i)) ? '1' : '0';
                bb[GRAY_N] = '\0';
                Serial.printf("L,%s,%.1f,%.1f,%.3f,%d,%d,%d,%.3f,%.3f,%.3f,%d,%.3f\n", bb, grayE, lineE, lineW,
                              (int)grayCnt, (int)lineValid, (int)lineCorner, lineSteer,
                              lineTrim + lineBias, lineBias, (int)lineCornerDir, lineArcFF);
            }
            // ---- ★★v3.7.0 无线上电自跑：就位检测 → 倒计时 → 自己开巡线 ----
            //   为什么要有"就位"这一步而不是上电就跑：上电瞬间人还扶着车/线还没对准，
            //   直接起跑就是一次撞车。判据只有两条：①能看到线（1~3 路，排除横边全黑）
            //   ②车基本不动。两条连续成立 1.5 s 再给 3 s 松手时间。
            // ══ ★★★v3.13.1 开机自检：稳 1.5 s → 陀螺标零 → 再等 3 s 才允许发车 ══
            //   为什么开机还要再标一次（setup 里 mpu6050.init() 已经标过）：
            //     init 那一刀是在上电瞬间做的，此时板温没稳、供电还在爬升，实测零偏不可信
            //     （静止 GyroZ 残留 0.3~1°/s ⇒ 一圈 180° 的弧跑下来偏到 0.3~1 m 级）。
            //     真正可用的判据是【上电稳定之后、发车之前】这一刀。
            //   时序：loop 首拍 +1.5 s → 标零（阻塞约 4 s，车必须静止）→ +3 s → 允许起跑。
            //   这 3 s 是用户 10-21 明确要求的（"标零结束后等待 3 s 启动"）。
            {
                static uint8_t  bootSt      = 0;
                static bool     bootStarted = false;
                static uint32_t bootT0      = 0;
                if (!bootStarted) { bootStarted = true; bootT0 = millis(); }
                const uint32_t nbm = millis();
                if (bootSt == 0) {
                    if ((uint32_t)(nbm - bootT0) >= BOOT_SETTLE_MS) {
                        if (imuOk) {
                            Serial.println("[CAL] ★开机自动标零：车必须静止 ~5 s ...");
                            mpu6050.ensureGyroRange();
                            mpu6050.calGyroOffsets();
                            gyroZf   = 0.0f;       // 与 `c` 命令同款：低通/dt 状态一起重置
                            gyroZfUs = 0;
                            Serial.printf("[CAL] 开机标零完成 (GYRO_CONFIG=0x%02X sens=%.1f)\n",
                                          mpu6050.gyroCfgReg, mpu6050.sensLSB);
                        } else {
                            Serial.println("[CAL] IMU 未接，跳过开机标零");
                        }
                        // ★v3.13.3 标零期间 loop 被阻塞约 4 s，S1 侧没人喂数据；
                        //   工作点只存 RAM，这里补发一次最保险（纯参数，不会让车动）。
                        applyS1Workpoint();
                        bootSt = 1;
                        bootT0 = millis();
                        if (autoRun) {
                            // 直接进倒计时：3 s 后发车。就位条件（在线上且两轮静止）一旦不满足，
                            // 下面 st==1 的分支会自己撤销回 st=0 ⇒ 不会带着人手发车。
                            autoRunSt   = 1;
                            autoRunT0   = bootT0;
                            autoRunTick = (uint8_t)(BOOT_AFTER_CAL_MS / 1000);
                            Serial.printf("[CAL] 标零结束 → 等 %u s 起跑\n", (unsigned)autoRunTick);
                        } else {
                            Serial.println("[CAL] 标零结束（自启 OFF：发 yf 手动巡线）");
                        }
                    }
                } else if (bootSt == 1) {
                    if ((uint32_t)(nbm - bootT0) >= BOOT_AFTER_CAL_MS) {
                        bootSt = 2;
                        Serial.println("[CAL] 开机自检完成");
                    }
                }
            }
            if (autoRun && autoRunSt < 2) {
                bool onLine = (grayCnt >= 1 && grayCnt <= 3) && !lineCorner && lineValid;
                // ★"静止"用 S1 回读的两轴轮速判（vMeas 是 loop 后半段才算出来的局部量，
                //   这里拿不到）。阈值 2 rad/s ≈ 0.03 m/s，比 20 Hz 帧的单帧跳动大一点。
                bool still  = (fabsf(BLDCData.vel[0]) < 2.0f && fabsf(BLDCData.vel[1]) < 2.0f);
                if (autoRunSt == 0) {
                    if (onLine && still) {
                        autoRunGzSum += robotPose.GyroZ;   // 就位期间顺手统计零偏质量
                        autoRunGzN++;
                        if (autoRunOkMs == 0) {
                            autoRunOkMs = nowMs;
                            Serial.printf("[AUTO] 车已就位 n=%d e=%.1f mm -> 确认 %.1f s\n",
                                          (int)grayCnt, grayE, AUTO_ARM_MS * 0.001f);
                        } else if (nowMs - autoRunOkMs >= AUTO_ARM_MS) {
                            autoRunSt   = 1;
                            autoRunT0   = nowMs;
                            autoRunTick = (uint8_t)(AUTO_DELAY_MS / 1000);
                            float gzm = (autoRunGzN ? autoRunGzSum / (float)autoRunGzN : 0.0f) * 57.2958f;
                            Serial.printf("[AUTO] 松手，%u s 后起跑  静止 GyroZ 均值 %+.2f °/s%s\n",
                                          (unsigned)autoRunTick, gzm,
                                          (fabsf(gzm) > 0.30f) ? "  !! 零偏可疑，建议重新上电并保持静止"
                                                               : "");
                        }
                    } else {
                        autoRunOkMs  = 0;               // 条件一断就重来，防止"半路条件"累计
                        autoRunGzSum = 0.0f;
                        autoRunGzN   = 0;
                    }
                } else {                               // 倒计时中
                    if (!onLine || !still) {           // 被拿起来/推动了 ⇒ 撤销
                        autoRunSt   = 0;
                        autoRunOkMs = 0;
                        autoRunTick = 0;
                        autoRunGzSum = 0.0f;
                        autoRunGzN   = 0;
                        Serial.println("[AUTO] 就位条件被破坏 -> 撤销起跑，重新等待");
                    } else if (nowMs - autoRunT0 >= AUTO_DELAY_MS) {
                        applyS1Workpoint();            // 兜底再补一次 S1 工作点（万一开机那次丢了）
                        lineBias      = 0.0f;          // 与 yf 同样的起跑状态
                        lineLostMs    = 0;
                        holdActive    = false;
                        // ★v3.9.10 速度归一化增益：Kp 与 v 成正比、Kd 与 √v 成正比。
                        //   依据 ë = v(Kp·e + Kd·ė) ⇒ ωn=√(v·Kp)、ζ=√v·Kd/(2√Kp)。
                        //   Kp∝v ⇒ 稳态横偏 e_ss=(v/R)/Kp 与速度无关（R500 恒约 23 mm，不会随提速冲出视场）；
                        //   Kd∝√v ⇒ 阻尼比 ζ 随速度略升，抵消 50 ms 低通带来的相位滞后。
                        //   0.28 m/s 时正好回到标定值 Kp24 / Kd9，不破坏既有工作点。
                        {
                            float s = autoRunV / 0.28f;
                            lineKpD = _constrain(24.0f * s, 8.0f, 90.0f);
                            lineKdD = _constrain(9.0f * sqrtf(s), 3.0f, 30.0f);
                        }
                        robotMotion.turn    = 0.0f;
                        robotMotion.forward = autoRunV;
                        directMode    = false;
                        lineOn        = true;
                        autoRunSt     = 2;             // 锁住：停下后不会自己再跑
                        Serial.printf("[AUTO] 起跑 v=%.2f  Kp=%.1f Kd=%.1f max=%.2f trim=%+.3f（按速度归一化）\n",
                                      autoRunV, lineKpD, lineKdD, lineTurnMax, lineTrim);
                    } else {
                        uint8_t left = (uint8_t)((AUTO_DELAY_MS - (nowMs - autoRunT0)) / 1000 + 1);
                        if (left != autoRunTick) {
                            autoRunTick = left;
                            Serial.printf("[AUTO] %u\n", (unsigned)left);
                        }
                    }
                }
            }

            // ---- ★★v3.9.11 离线重跑：搬起来再放回线上 ⇒ 自动重新武装 ----
            //   为什么需要：拔掉 USB 后没人能发 `yna`，跑一趟就锁死在 autoRunSt=2。
            //   判据（三条全满足，"被搬动"要持续 200 ms 防误触）：
            //     ① 本趟已结束：st==2 且巡线已关（跑的过程中 lineOn 恒 true，绝不误触发）
            //     ② 车被搬动过：陀螺转过 >0.6 rad/s，或某轮被手拨转 >4 rad/s
            //     ③ 现在放回线上且静止：n=1~3、非横边、两轮 <2 rad/s
            //   之后回到 st=0，走原来的"就位 1.5 s → 3 s 倒数 → 起跑"流程。
            if (autoRun && autoRunSt == 2 && !lineOn) {
                bool moved = (fabsf(robotPose.GyroZ) > 0.6f) ||
                             (fabsf(BLDCData.vel[0]) > 4.0f) ||
                             (fabsf(BLDCData.vel[1]) > 4.0f);
                if (moved) {
                    if (autoLiftMs == 0) autoLiftMs = nowMs;
                    if (nowMs - autoLiftMs >= 200) autoLiftSeen = true;
                } else {
                    autoLiftMs = 0;
                }
                bool placed = (grayCnt >= 1 && grayCnt <= 3) && !lineCorner && lineValid &&
                              fabsf(BLDCData.vel[0]) < 2.0f && fabsf(BLDCData.vel[1]) < 2.0f;
                if (autoLiftSeen && placed) {
                    autoLiftSeen = false;
                    autoLiftMs   = 0;
                    autoRunSt    = 0;
                    autoRunOkMs  = 0;
                    autoRunT0    = 0;
                    autoRunTick  = 0;
                    autoRunGzSum = 0.0f;
                    autoRunGzN   = 0;
                    Serial.printf("[AUTO] 搬动后重新就位 -> 自动重新武装（v=%.2f）\n", autoRunV);
                }
            }
        }
    }

    // ---- IMU 量程抽查（每 2 s）：芯片被静默复位后自动纠正换算 + 重写 ----
    static uint32_t lastCfgMs = 0;
    if (imuOk && millis() - lastCfgMs >= 2000) {
        lastCfgMs = millis();
        int rg = mpu6050.ensureGyroRange();
        if (rg == 1) {
            Serial.printf("[IMUCFG] !! GYRO_CONFIG lost -> now 0x%02X, sens=%.1f\n",
                          mpu6050.gyroCfgReg, mpu6050.sensLSB);
        } else if (rg == 2) {
            Serial.println("[IMUCFG] !! I2C no-ack");
        }
    }

    // ---- ★v3.2.1 巡线期间指令常驻 ----
    //   `v0.30` 只是"给一次目标"，500 ms 后失控看门狗会把 forward 指数衰减到 0
    //   （2026-10-19 第一趟实测：车冲了约 0.5 s 就停住，vMeas 全程 0.000）。
    //   巡线的语义是"一直跟着线走，直到 s 停"，所以只要 lineOn 就压住看门狗。
    if (lineOn) lastCmdMs = millis();

    // ---- 失控保护：500ms 无指令 → 目标指数衰减归零 ----
    if (millis() - lastCmdMs > 500) {
        robotMotion.forward *= 0.98f;
        robotMotion.turn    *= 0.98f;
        if (fabsf(robotMotion.forward) < 0.01f) robotMotion.forward = 0.0f;
        if (fabsf(robotMotion.turn)    < 0.01f) robotMotion.turn    = 0.0f;
    }

    // ---- 俯仰保护（摔倒）→ 输出截断，回正自动恢复 ----
    if (imuOk && pitchProt > 0.0f) {
        if (fabsf(robotPose.pitch) > pitchProt) {
            if (!fallen) { fallen = true; Serial.println("[PROT] pitch over limit, output cut"); }
        } else if (fallen && fabsf(robotPose.pitch) < pitchProt * 0.5f) {
            fallen = false;
            Serial.println("[PROT] recovered");
        }
    } else {
        fallen = false;
    }

    // ---- 轮速 / 偏航率 / 差速运动学 ----
    // 编码器反馈（乘方向系数归一到"前进为正"）
    // 只对"实际存在的轴"求平均：某一路未接入（状态帧回 'X'）时不把它的 0 混进来，
    // 否则单电机台架测试时 vMeas 恒为真值的一半，速度外环会把轮子开到 2 倍速。
    int   present = 0;
    float vSum    = 0.0f;
    for (int i = 0; i < 2; i++) {
        if (BLDCData.state[i] == 'X') continue;
        vSum += ((i == 0) ? M0Dir : M1Dir) * BLDCData.vel[i];
        present++;
    }
    float vMeas = (present ? vSum / (float)present : 0.0f) * Rw;

    // ★板间链路断时，BLDCData 里是"冻结的旧读数"，绝不能当测量值：
    //   否则 PID_VEL 会对着一个固定误差一直积分，饱和到 ±limit/3 = ±0.133 m/s
    //   （实测 link=0 持续 40 s ⇒ vCmd=-0.135 ⇒ 两轮目标 -8.98 rad/s，
    //    链路一恢复车就倒冲 13 cm/s）。2026-10-17 闭环首测前发现。
    const bool linkOk = motors.online();
    if (!linkOk) vMeas = 0.0f;
    gVMeas = vMeas;                  // ★v3.8.2 供 lineUpdate() 判"车是否真的在动"

    // ★★v3.7.0 拔线自跑兜底：S1 的 FOC/编码器初始化比 S3 慢，开机那一次 sendRaw 很可能
    //   被它的 setup 吞掉；S1 若自己重启，工作点也会退回默认值（W5/kd0.25/km0.6）。
    //   所以链路"断开→恢复"的上升沿再补发一次，保证真正开跑时参数一定是对的。
    {
        static bool linkPrev = false;
        if (linkOk && !linkPrev) applyS1Workpoint();
        linkPrev = linkOk;
    }

    // ★v2.5.1：vMeas 低通（τ=0.12 s）——只给外环/PID 用，里程积分仍用原始值。
    //   依据 d1m_D：20 Hz 状态帧单帧可跳 ±30%（巡航 sd 0.106 m/s），PID 的 I 项
    //   对整流后的噪声有偏置 ⇒ 2.5 s 内把 vTrim 顶到 +0.14 ⇒ vCmd=0.44（比 vmax 高 47%）。
    //   轮速本身没打滑（0.286 vs 0.300 只差 4.7%），这个偏置纯粹是噪声喂出来的。
    static float    vMeasLp = 0.0f;
    static uint32_t vLpUs   = 0;
    {
        uint32_t nU = micros();
        float dl = (vLpUs == 0) ? 0.0f : (nU - vLpUs) * 1e-6f;
        vLpUs = nU;
        if (dl < 0.0f || dl > 0.2f) dl = 0.0f;      // 首拍/卡顿不更新
        const float tau = 0.12f, a = dl / (tau + dl);
        vMeasLp += (vMeas - vMeasLp) * ((dl > 0.0f) ? a : 0.0f);
    }
    const float vMeasF = vMeasLp;                   // 供 PID / 滑行补偿使用

    // ---- ★v3.0.0 几何链：段内推进 + 段切换 ----
    //   放在 vCmd/wCmd 取值【之前】：这里设的 forward/turn 当拍即生效。
    //   PIVOT 段由 rotOn 通道接管（rotOn=true ⇒ 下面 rotOn 块置 rateMode，hold 环不覆盖）。
    //   直线段 turn=0 ⇒ 自动进 hold 环；弧段 turn≠0 ⇒ 自动走偏航率环。控制律零改动。
    if (pathOn && !rotOn) {
        uint32_t nU = micros();
        float dtP = (pathMicros == 0) ? 0.0f : (nU - pathMicros) * 1e-6f;
        pathMicros = nU;
        if (dtP < 0.0f || dtP > 0.2f) dtP = 0.0f;
        PathSeg &sg = pathQ[pathI];
        const float accS = (sg.acc > 0.01f) ? sg.acc : moveAcc;

        if (sg.kind == PS_LINE) {
            pathSegS += fabsf(vMeas) * moveScale * dtP;
            if (pathV < sg.v) pathV = fminf(sg.v, pathV + accS * dtP);
            else              pathV = sg.v;
            robotMotion.forward = pathV;
            // ★v3.9.3 直线段也吃灰度修正（run60 的 L4 漂了 58 mm）。
            //   有有效 e 就按巡线 PD 给小幅舵量；真丢线（n=0）退回纯 hold 环，
            //   免得 lineSteerHold 把车带成一条弧。
            float lcorr = 0.0f;
            if (!lineCorner) {
                if (lineValid)          lcorr = lineSteer;
                else if (grayCnt > 0)   lcorr = lineSteerHold;
            }
            robotMotion.turn = _constrain(lcorr, -pathLatMax, pathLatMax);
            if (pathSegS >= sg.a) pathFinishSeg();
        } else {                                   // ARC
            const float dY = wrapPi(robotPose.yaw - pathPrevYaw);
            pathPrevYaw = robotPose.yaw;
            pathYawAcc  += dY;
            if (pathV < sg.v) pathV = fminf(sg.v, pathV + accS * dtP);
            else              pathV = sg.v;
            float w = sg.v / fmaxf(sg.r, 0.05f);
            if (sg.a < 0.0f) w = -w;
            robotMotion.forward = pathV;
            robotMotion.turn    = _constrain(w, -yawMax, yawMax);
            if (fabsf(pathYawAcc) >= fabsf(sg.a)) pathFinishSeg();
        }
        if (pathOn) {
            lastCmdMs = millis();                  // 压住 500 ms 无指令衰减看门狗
            // ★v3.9.2 堵转/碰撞：轮子可能在空转（vMeas 不小），只能看"进度有没有涨"
            //   ★v3.9.3 必须加 driving 条件：vCmd=0 时"没进度"是应该的，
            //   run61 就是停车状态下被判卡死，把整条链中止了。
            const float prog = (pathQ[pathI].kind == PS_LINE) ? pathSegS : fabsf(pathYawAcc);
            if (fabsf(robotMotion.forward) < 0.05f) {
                pathPrevProg = prog;
                pathProgMs   = millis();
            } else if (fabsf(prog - pathPrevProg) > 0.02f) {
                pathPrevProg = prog;
                pathProgMs   = millis();
            } else if (millis() - pathProgMs > 1200UL) {
                Serial.printf("[PATH] ★卡死中止（%s 1.2 s 无进度：里程 %.3f m / 转角 %.1f°）\n",
                              pathKindName(pathQ[pathI].kind), pathSegS, pathYawAcc * 57.2958f);
                pathStop();
            }
            if (pathOn && millis() - pathSegMs > 30000UL) {
                Serial.printf("[PATH] ★段超时（%s）中止\n",
                              pathKindName(pathQ[pathI].kind));
                pathStop();
            }
        }
    }

    float vCmd = robotMotion.forward;
    float wCmd = robotMotion.turn;

    // ---- ★v2.7.0 原地转（gyro 闭环）----
    //   vCmd 钉 0、只给 wCmd ⇒ 两轮反向、绕轮轴中点自转，平移≈0。
    //   入口：`t<deg>`；或 `x...,1` 发现航向偏差 >1.2° 的原地对齐阶段（rotThenMove）。
    bool rateMode = false;             // 本拍 wCmd 由"原地转"通道接管（hold 环不得覆盖）
    if (rotOn) {
        const float eYawR = wrapPi(rotTarget - robotPose.yaw);
        const bool  inTol = (fabsf(eYawR) < ROT_TOL) && (fabsf(robotPose.GyroZ) < ROT_GYRO_TOL);
        const bool  toLong = (float)(millis() - rotStartMs) > ROT_MAX_MS;
        if (inTol || toLong) {
            rotOn     = false;
            rateMode  = false;
            yawHold      = robotPose.yaw;        // 转完成为新基准（t 之后接 x keep 就靠这个）
            yawHoldValid = true;
            holdActive   = imuOk;
            Serial.printf("[TURN] done  yaw=%.1f°  err=%+.2f°  t=%.2f s%s\n",
                          robotPose.yaw * 57.2958f, eYawR * 57.2958f,
                          (millis() - rotStartMs) * 0.001f,
                          toLong ? "  (超时中止)" : "");
            if (rotThenMove) { rotThenMove = false; beginMove(); }
            else if (lineAfterTurn) {                // ★v3.3.0 转角转完 ⇒ 传感器应已落在新线上
                lineAfterTurn = false;
                lineOn        = true;
                lineE         = 0.0f;
                lineLostMs    = 0;
                lineReacq     = false;
                lineJustTurned = true;
                lineJustTurnedMs = millis();
                robotMotion.forward = 0.15f;         // 先慢，等确认压线再恢复巡航
                robotMotion.turn    = 0.0f;
                holdActive    = false;
                Serial.println("[LINE] corner turn done -> 续跑（等新线 0.6 s）");
            }
            else if (rotThenChain) {                 // ★v3.0.0 链里的 PIVOT 段转完 ⇒ 接下一段
                rotThenChain = false;
                pathFinishSeg();
            }
        } else {
            vCmd     = 0.0f;
            wCmd     = _constrain(ROT_KP * eYawR - ROT_KD * robotPose.GyroZ,
                                  -rotRateMax, rotRateMax);
            rateMode = true;
            lastCmdMs = millis();              // 压住 500 ms 无指令衰减看门狗
        }
    }

    // ---- ★v2.4.0 定距移动：距离反算速度（距离闭环，不是时间闭环）----
    //   每拍按剩余路程给制动上限 v_brake=sqrt(2·a_dec·s_rem)，再叠加起动加速度限幅。
    //   落点由里程积分决定 ⇒ 打滑/静摩擦只改变过程，不累积到终点。
    //   航向交给 v2.3.0 的 hold 环（turn=0 且 forward>0 自动进入）。
    if (moveOn) {
        uint32_t nowU = micros();
        float dt = (nowU - moveMicros) * 1e-6f;
        moveMicros = nowU;
        if (dt < 0.0f || dt > 0.2f) dt = 0.0f;      // 首拍 / 卡顿：本拍不积分

        moveS  += vMeas * moveScale * dt;
        moveRem = moveTarget - moveS;
        const float remEff = moveTargetEff - moveS;   // ★v2.5.4 已扣停车提前量

        // ★v2.5.2 刹车坡度 = min(请求减速度, 实测可达减速度)。
        //   v2.5.1 的 sSlide 同拍反馈已删除（它把 rem 越扣越多，形成"减速→松油门→
        //   再加速"的极限环，chk10 多走 2.9 cm）。
        // ★v2.5.4 用 remEff（扣掉前馈提前量）；并加单向约束：
        //   制动段指令只许降不许升，从根上掐死任何回弹。
        const float aBrake = fminf(moveDec, moveDecEff);
        float vBrake = sqrtf(2.0f * aBrake * fmaxf(0.0f, remEff));
        float vDes   = fminf(moveVmax, vBrake);
        if (moveBraking && vDes > moveV) vDes = moveV;                  // 制动段不许回弹
        if (vDes > moveV + moveAcc * dt) vDes = moveV + moveAcc * dt;   // 起动加速度限幅
        if (vDes < 0.0f) vDes = 0.0f;
        if (!moveBraking && vDes < moveV - 1e-4f) moveBraking = true;   // 首次下降即判入制动段
        moveV = vDes;

        robotMotion.forward = moveV;
        robotMotion.turn    = 0.0f;
        lastCmdMs = millis();                        // 压住 500 ms 无指令衰减看门狗

        // 末段停滞检测：v<0.015 且轮子实测不动 ⇒ 已到 0.4 rad/s 以下走不动的区
        if (moveV < 0.015f && fabsf(vMeas) < 0.004f) moveStallMs += dt * 1000.0f;
        else                                         moveStallMs = 0.0f;

        // ★v2.5.1 卡死保护（d1m_D 教训）：车被线缠住时两轮都停，但 moveS 不涨、
        //   rem 不减小 ⇒ vDes 一直给全速，控制器就对着一个卡死的车猛推，
        //   解脱瞬间车以 0.44 m/s 冲出去。判据反过来写：**指令够大而车不动**。
        //   （vDes 掉 0 的滑行段 moveV 很小，不会被误判，所以要与上面那条并存。）
        if (moveV > 0.05f && fabsf(vMeasF) < 0.02f) moveStuckMs += dt * 1000.0f;
        else                                       moveStuckMs = 0.0f;

        if (remEff <= 0.0015f || moveStallMs > 800.0f || moveStuckMs > 500.0f
            || (millis() - moveStartMs) > 30000UL) {
            moveOn = false;
            moveSettleOn = true;          // ★v2.5.4 继续积分到停稳，用来校准 moveLead
            robotMotion.forward = 0.0f;
            robotMotion.turn    = 0.0f;
            PID_VEL.reset();
            Serial.printf("[MOVE] done  s=%.4f rem=%.4f vstop=%.3f t=%.2f s%s%s%s\n",
                          moveS, moveRem, vMeas, (millis() - moveStartMs) * 0.001f,
                          moveStallMs > 800.0f ? "  (末段停滞收尾)" : "",
                          moveStuckMs > 500.0f ? "  (★卡死中止：指令有、车不动)" : "",
                          (millis() - moveStartMs) > 30000UL ? "  (超时中止!)" : "");
        } else if (millis() - movePrintMs >= 100) {
            movePrintMs = millis();
            Serial.printf("[MV] s=%.4f rem=%.4f v=%.4f\n", moveS, moveRem, moveV);
        }
    }

    // ★v2.5.4 停稳统计：done 之后继续按编码器积分，直到真的不动，
    //   打印"含滑行的实际落点" —— 这是校准 moveLead 的唯一依据（省掉卷尺）。
    if (!moveOn && moveSettleOn) {
        uint32_t nowU = micros();
        float dt = (nowU - moveMicros) * 1e-6f;
        moveMicros = nowU;
        if (dt < 0.0f || dt > 0.2f) dt = 0.0f;
        moveS += vMeas * moveScale * dt;
        // ★v2.9.1 settle 段改用 coast（J 命令两轴自由衰减）。
        //   v2.9.0 的 brakeK 主动制动实测无效（M0 跟指令快、M1 滞后 → 仍不对称）。
        //   现 forward 不再参与 settle（kb 默认 0）；coast 由主循环 setCoast(true) 发 J 实现。
        //   brakeK 保留在固件里（0=关）但不启用。
        robotMotion.forward = -brakeK * vMeas;
        if (fabsf(vMeas) < 0.010f || (millis() - moveStartMs) > 8000UL) {
            moveSettleOn = false;
            robotMotion.forward = 0.0f;      // 停稳后归零，避免残留制动指令
            Serial.printf("[MOVE] settle s=%.4f  (目标 %.4f, 偏差 %+.1f mm, done 后滑行 %.1f mm)\n",
                          moveS, moveTarget, (moveS - moveTarget) * 1000.0f,
                          (moveS - moveTargetEff) * 1000.0f);
        }
    }
    float wL, wR;

    // ★★2026-10-17 修"一上电就朝左微转"（台架实测定案）
    //   病因：控制律 wCmd = turn + kt·(turn − GyroZ)，turn=0 时并不等于 0，而是 −kt·ε。
    //   零偏 ε 系统性为负（反复实测 −0.02 ~ −0.05 rad/s），于是两轮一直收到 ∓0.15 rad/s 的
    //   自转指令 → 车持续左转。判决性对照：kt=0 时 wCmd 与 wL/wR 严格归零、iq0 从 0.05→0.00 A，
    //   说明零偏本身不产生动作，是 kt 项把它翻译成了指令。
    //   修法一（门控）：|turn| 很小时陀螺反馈不参与 —— 静止时本来不需要纠偏。
    //                   用 0.01~0.04 线性渐变，避免在阈值处跳变。
    //   修法二（静止硬断）：见下方 idle。
    static const float TURN_GATE_LO   = 0.01f;   // rad/s，以下 gate=0
    static const float TURN_GATE_HI   = 0.04f;   // rad/s，以上 gate=1
    static const float IDLE_VEL_LIM   = 0.005f;  // m/s，forward 与 vMeas 的门限
    static const float IDLE_TURN_LIM  = 0.02f;   // rad/s，turn 的门限
    static const float HOLD_TURN_LIM  = 0.02f;   // rad/s，|turn| 小于此视为直行
    static const float HOLD_VEL_MIN   = 0.005f;  // m/s，有前进指令才保持航向
    static bool idlePrev = false;

    if (!fallen) {
        float ktEff = imuOk ? turnKp : 0.0f;
        float gate  = _constrain((fabsf(robotMotion.turn) - TURN_GATE_LO)
                                 / (TURN_GATE_HI - TURN_GATE_LO), 0.0f, 1.0f);

        // ★v2.3.0：直行 ⇒ 航向保持；有转向指令 ⇒ 走原偏航率环（已验收，不改）。
        //   ★v2.5.1 加 `|| moveOn`：定距过程中 vDes 会短暂归 0（滑行补偿生效那一拍），
        //   若因此判成"不直行"就会释放航向锁存，下一拍再锁到**当前已经偏掉的朝向**上
        //   （d1m_D 实测多出一次 `[HOLD] lock yaw=-2.195 rad`）。
        // ★v2.8.0：`|| moveSettleOn` —— 滑行段也要保持航向。
        //   done 那一拍 forward 归 0，若只看 forward 就会判成"不直行"⇒ wCmd 钉 0 ⇒
        //   162 mm 的滑行段零权限，两轮自由衰减差 9 rad/s ⇒ 枢轴 8.9°（N085）。
        //   这就是"kd 从 0.25 加到 0.6 完全无效"的真因：不是增益不够，是环被关掉了。
        // ★★v3.4.0 直接舵量通道是否接管本拍：巡线中 + e 有效 + 不在转角状态机里。
        //   接管时 hold 环整条旁路（不进 straight），wCmd 最后被 lineSteer 覆盖。
        bool lineDirect = lineOn && (lineMode == 1)
                       && !lineAdvancing && !lineReacq
                       && (lineValid || lineSearch);

        bool straight = imuOk && !directMode && !lineDirect && !c4On
                     && fabsf(robotMotion.turn) < HOLD_TURN_LIM
                     && (fabsf(robotMotion.forward) > HOLD_VEL_MIN || moveOn || moveSettleOn);

        if (rateMode) {
            // ★v2.7.0 原地转：wCmd 已在上面由角度环给出，这里不许 hold 环/偏航率环覆盖。
            holdActive = false;
            holdI      = 0.0f;         // ★v2.9.4 转角环接管：I 清零
        } else if (straight) {
            // ★v2.8.0 滑行段不许重锁：此刻车头已经偏了几度，重锁会把偏角当成新基准
            if (!holdActive && !moveSettleOn) {       // 进入直行：锁存当前车头方向
                yawHold      = robotPose.yaw;
                yawHoldValid = true;
                holdI        = 0.0f;                 // ★v2.9.4 新基准：I 积分器清零
                Serial.printf("[HOLD] lock yaw=%.3f rad (%.1f deg)\n",
                              yawHold, yawHold * 57.2958f);
            }
            holdActive = true;
            // ★v3.2.0 巡线串级：灰度只改【目标航向】，hold 的 P/I/D 照旧跟这个目标。
            //   （直接往 wCmd 叠加 Δω 会被 I 项 ki4.0 当常值扰动积分掉，等于没接。）
            //   e>0 线在左 ⇒ lineW>0 ⇒ 目标航向更左 ⇒ 车左转追线。
            float yawTgt = yawHold;
            // ★v2.9.4 I 项积分：只在非 settle 段积分（settle 段权限按 vMeas 塌缩，
            //   eYaw 会冲到 +10°，积分进去就是 windup）；抗饱和 |holdI|≤0.4。
            static uint32_t holdMicros = 0;
            float dtH = (micros() - holdMicros) * 1e-6f;
            holdMicros = micros();
            if (dtH < 0.0f || dtH > 0.2f) dtH = 0.0f;   // 首拍/卡顿不积分
            if (lineOn && lineValid && !lineAdvancing && lineMode == 0) {
                // ★v3.2.6 横向积分进【航向基准】：yawHold 自己转到与线平行。
                //   只靠 P（lineW）的话，捕获时若车头与线差 θ，稳态横偏 e_eq=LD·tan(θ/kp)，
                //   θ=25° 就 84 mm，早就出视场了 ⇒ 必须让基准角收敛掉这个 θ。
                yawHold = wrapPi(yawHold + lineKi * (lineE * 0.001f) * dtH);
                yawTgt  = wrapPi(yawHold + lineW);
            }
            // 航向误差 = 目标 − 当前（正=车头偏左，需右转 ⇒ wCmd 取负）
            float eYaw = wrapPi(yawTgt - robotPose.yaw);
            // ★★v3.2.3 巡线期间【禁止】航向 I 积分：
            //   ki4.0 + |holdI|≤0.4 相当于自带 23°/s 的常值转弯权限。灰度只能改目标航向
            //   （最多 ±11.5°）⇒ 稳态时 holdI 会把这点权限吃光并顶在 ±0.4，
            //   车就按一个固定方向的弧一直转，灰度再也掰不回来。
            //   10-19 实测：t=3~6 s e 恒 +33.5、yaw 单向 +22°，随后 1111111 冲出场外，就是这个机理。
            //   巡线本身有绝对横向反馈，不需要靠 I 补下垂；退出巡线后 holdI 自然重新积分。
            if (lineOn) {
                holdI = 0.0f;
            } else if (holdKi > 0.0f && !moveSettleOn) {
                holdI += holdKi * eYaw * dtH;
                holdI  = _constrain(holdI, -0.40f, 0.40f);
            }
            // ★v2.8.0 滑行段用更大的限幅，但按当前车速渐变：
            //   轮子快停时两轮差速需求本身就趋零，满限幅硬给会反过来把车拧一下。
            float hm = holdMax;
            if (moveSettleOn) {
                float g = _constrain(vMeas / 0.15f, 0.0f, 1.0f);
                hm = holdMaxCoast * g;
            }
            wCmd = _constrain(holdKp * eYaw + holdI - holdKd * robotPose.GyroZ, -hm, hm);
        } else {
            holdActive = false;
            holdI      = 0.0f;         // ★v2.9.4 退出 hold：I 清零（防旧边的下垂量带进下一段）
            wCmd = robotMotion.turn + gate * ktEff * (robotMotion.turn - robotPose.GyroZ);
            // ★v3.2.0 巡线：弧/差速段没有航向 I 项，可以直接把横向修正叠到偏航率上
            if (lineOn && lineValid && lineMode == 0) wCmd += lineW;
            // ★★v3.4.0 直接舵量接管：turn 直接就是偏航率参考，内环照旧用陀螺做阻尼。
            //   ★必须重算门控：上面那个 gate 是按 robotMotion.turn(=0) 算的，恒 0 ⇒ 陀螺阻尼会失效。
            if (lineDirect) {
                // ★v3.4.1 丢线窗口里用"最后一次有效舵量"（带 0.30 rad/s 下限）继续找线
                // ★★v3.10.0 优先用【边缘记忆】主动搜线（用户方案）：线从哪边滑出去就往哪边转，
                //   直到重新看见线。没有边缘记忆时才退回"保持最后舵量"。
                //   ★为什么搜线舵量要写死而不是按 |e| 比例：此刻已经 n=0，没有任何横向信息，
                //     唯一可用的就是"方向"；力度必须够，否则在 700 ms 窗口里转不回来。
                float st = lineSteer;
                // ★只在这个「真丢线且仍在搜线窗口内」（lineSearch：n=0 且 <700 ms）时才接管。
                //   ★不能用 !lineValid 当条件：n≥4 的饱和帧（压到横向胶带/十字）也会让 lineValid
                //     为假，那种情况不该打满 1.2 rad/s，否则压一次横边就被甩出去。
                if (lineSearch) {
                    if (lineEdgeSide != 0 && lineSearchRate > 0.0f) {
                        // ★★v3.10.1 搜线舵量必须【随速度】放大，不能写死：
                        //   搜线时车沿一条半径 R_s = v/ω_s 的圆弧走。
                        //   ★★v3.10.2 关键修正（run 10-20 00:12 实测）：R_s 必须【比赛道半径更紧】，
                        //   而不是"差不多大"。两段同心弧的横向间距是恒定的：
                        //   若 R_s > R_line，车只会平行地跑在线外侧，永远靠不近（实测 R_s=v/1.2=230
                        //   对 R200，搜线 0.9 s、方向完全正确、仍以 40 mm 外沿冲出）。
                        //   收敛速率 ≈ (R_line − R_s)(1−cosθ)：R_s=160 时每 90° 收 40 mm，
                        //   约 0.25 m / 0.9 s 就能咬回；R_s=140 时 70° 就够。
                        //   取 0.155 m（R200−45），再被 lineTurnMax / wCap 兜住上限。
                        //   v=0.28 ⇒ 1.83 rad/s（wCap 1.84，刚好用满）；v=0.40 ⇒ 2.00（被 lineTurnMax 钳）。
                        float sr = lineSearchRate;
                        float vNow = fabsf(robotMotion.forward);
                        if (vNow > 0.05f) {
                            float need = vNow / 0.155f;
                            if (need > sr) sr = need;
                        }
                        sr = _constrain(sr, 0.0f, lineTurnMax);
                        st = lineSign * (float)lineEdgeSide * sr;
                    } else {
                        st = lineSteerHold;
                        if (fabsf(st) > 0.02f && fabsf(st) < 0.30f)
                            st = (st > 0.0f) ? 0.30f : -0.30f;
                    }
                }
                float gs = _constrain((fabsf(st) - TURN_GATE_LO)
                                      / (TURN_GATE_HI - TURN_GATE_LO), 0.0f, 1.0f);
                wCmd = st + gs * ktEff * (st - robotPose.GyroZ);
            }
            // ★★★v3.13.0 四圆区：线暂时看不见（n=0）时【按钉住的几何曲率继续走】，
            //   不许落到下面的 wCmd=0（turn 环在 turn=0 时输出恒 0 ⇒ 车会直着冲出去）。
            //   这不是死跑：一旦重新看到线（lineDirect=true），上面那段灰度 PD 立刻接管。
            if (c4On && !lineDirect) wCmd = lineArcFF;
            // ★★v3.9.0 几何链 ARC：偏航率【直接闭环】，绕过 kt 环的打滑欠跟踪。
            //   为什么不能用 kt：kt 环要求 GyroZ→turn，而"轮速差→偏航率"的实际增益只有
            //   0.82 ⇒ 纯 P 只能补到 82%（run54 实测 1.18/1.40），半径 230 mm。
            //   wt + I + Kd·eW 里 I 负责把稳态误差积掉，Kd 只做阻尼。
            //   wCmd 会涨到 ~1.71 rad/s，仍受下面 wDipMin 钳位保护（v0.28 时上限 2.01）。
            if (pathOn && pathI >= 0 && pathI < pathN && pathQ[pathI].kind == PS_ARC) {
                const PathSeg &asg = pathQ[pathI];
                float wt = asg.v / fmaxf(asg.r, 0.05f);
                if (asg.a < 0.0f) wt = -wt;
                // ★v3.9.3 灰度【主导】横向修正：直接复用巡线那套 PD 舵量（Kp24/Kd9），
                //   不再用 pathLatK·e 那种人为小增益（v3.9.2 的 3.0 太弱：e=33 mm 只给
                //   0.10 rad/s，占前馈的 8%，8 m 跑下来只压到 60 mm 偏差）。
                //   e=6.7 mm ⇒ 0.16 rad/s（前馈 1.2 的 13%），e=20 mm ⇒ 0.48。
                //   丢线（n=0，弧上传感器跑出 ±40 mm 视场）时保持最后一次有效舵量，
                //   否则弧会突然失去修正、靠前馈硬走 —— run60 的偏差就是这么攒出来的。
                float corr = 0.0f;
                if (!lineCorner) {
                    // ★v3.9.6 真丢线（n=0）时【不许】继续用 lineSteerHold：
                    //   丢线那一刻的舵量往往是打满的（它在拼命找线），把它保持进弧里
                    //   等于让几何前馈 1.2 再叠 0.55 ⇒ 内轮进失速区（run0616 的 455 mm 就是这么来的）。
                    //   丢线就纯按几何走，等灰度重新看到线再修。
                    if (lineValid && grayCnt >= 1 && grayCnt <= 3) { corr = lineSteer; }
                }
                corr = _constrain(corr, -pathArcLatMax, pathArcLatMax);
                const uint32_t nu = micros();
                float dtA = (pathArcUs == 0) ? 0.0f : (nu - pathArcUs) * 1e-6f;
                pathArcUs = nu;
                if (dtA < 0.0f || dtA > 0.2f) dtA = 0.0f;
                const float eW = wt + corr - robotPose.GyroZ;
                if (dtA > 0.0f) {
                    pathArcI += pathArcKi * eW * dtA;
                    pathArcI  = _constrain(pathArcI, -pathArcILim, pathArcILim);
                }
                wCmd = wt + corr + pathArcI + pathArcKd * eW;
            }
            // ★v3.0.1 差速权限钳位：保证较慢那只轮的角速度 ≥ wDipMin。
            //   由 wL = (v − w·Lw/2)/Rw ≥ wDipMin 解出 |w| ≤ 2(v − wDipMin·Rw)/Lw。
            //   ★只在有前进速度时生效：纯原地转走 rateMode 通道（vCmd=0），不会到这里；
            //     若 forward≈0 还钳，wCap→0 会把角度环直接掐死。
            //   ★v3.10.12 用【指令与实际中较大的那个】算钳位：wCap 的物理含义是
            //     "内轮 ≥ wDipMin"，真正决定它的是**实际**前进速度 vMeas，不是指令。
            //     实测 10-20 01:14：指令 0.28 但实际被轮速外环顶到 0.30~0.36，
            //     wCap 仍按 0.28 算成 1.799 ⇒ 偏航率固定 1.39 ⇒ R=0.30/1.39=216 mm，
            //     比 R200 大 16 mm，整段弧一路向外漂，e 从 +20 涨到 +33.5 就丢了。
            const float vFwd = fmaxf(fabsf(robotMotion.forward), fabsf(gVMeas));
            if (vFwd > 0.03f) {
                float wCap = 2.0f * (vFwd - wDipMin * Rw) / Lw;
                wCap = _constrain(wCap, 0.0f, yawMax);
                wCmd = _constrain(wCmd, -wCap, wCap);
            }
            // ★★v3.8.4 速率环不许把舵量放大超过 lineTurnMax。
            //   run46 实测：steer=1.60 时 wCmd 被 kt 放大到 2.008 ⇒ 实际偏航冲到 1.9 rad/s
            //   （R200 的弧只需要 1.4），车转头过头 ⇒ e 在 ±20 mm 之间越摆越大，1.8 s 后丢线。
            //   kt 放大在舵量 0.3 时只多给 0.18（无害），到 1.6 却多给 0.96（必然过冲）
            //   ⇒ 这是"幅度相关"的失稳。钳住后偏航靠电机自身斜率（≈14 rad/s²）上升，
            //   到 1.6 只要 0.11 s，一点不慢，但不会冲过目标。
            if (lineDirect)
                wCmd = _constrain(wCmd, -lineTurnMax, lineTurnMax);
        }

        // 修法二：指令与实测同时近似为零 ⇒ 车本该静止，直接把输出钉死为 0。
        //   即使门控漏过一点残余指令、或速度环积分残留，也不让它变成真实轮速。
        //   （directMode 直通时不管，那是台架手动给轮速用的。）
        bool idle = (!directMode) && !rateMode
                 && fabsf(robotMotion.forward) < IDLE_VEL_LIM
                 && fabsf(robotMotion.turn)    < IDLE_TURN_LIM
                 && fabsf(vMeas)               < IDLE_VEL_LIM;

        if (idle) {
            if (!idlePrev) { PID_VEL.reset(); idlePrev = true; }  // 只清一次积分
            vCmd = 0.0f;
            wCmd = 0.0f;
            wL = wR = 0.0f;
        } else {
            idlePrev = false;
            // ★v2.5.0：没有前进指令时不跑轮速外环，且指令不许变负。
            //   停车后 forward=0 而 vMeas 还是 +0.285 ⇒ 误差 -0.285
            //   ⇒ PID 把 vTrim 积到 -0.116 m/s ⇒ 两轮目标 -7.7 rad/s（倒车）。
            //   左轮（M0，有相电流环）0.08 s 就跟着过零，右轮（M1，只有电压模式）
            //   还在 +19 rad/s ⇒ 绕左轮接地点枢轴左转 20°，落点多走 5 cm。
            //   静止/滑行时本来就不需要速度闭环：目标为 0，跟踪 0 更没有意义。
            static bool coastPrev = false;
            // ★v2.5.2 定距期间不跑轮速外环。moveS 已经是【距离闭环】，轮速外环唯一
            //   还能做的事是补编码器尺度，而 scale 参数已经把它做掉了；留着它反倒在
            //   起步瞬态把 I 项积起来（chk10 实测 vCmd=0.297 = vmax 的 148%，到稳态
            //   2.5 s 都退不掉，直接毁掉速度剖面）。⇒ 定距期间 vTrim 恒 0。
            // ★v2.9.0：settle 段 PID 也不跑（forward=-brakeK·vMeas 是开环制动指令，
            //   PID 会把误差 forward-vMeas 当成跟踪目标，反向积分饱和重现 v2.5.0 枢轴）。
            const bool  coasting  = moveOn || moveSettleOn || (fabsf(robotMotion.forward) < 0.010f);
            float vTrim = 0.0f;
            if (linkOk && !coasting) {
                vTrim    = PID_VEL(robotMotion.forward - vMeasF);  // 轮速外环（用低通值）
                coastPrev = false;
            } else if (!coastPrev) {
                PID_VEL.reset();       // 只在进入滑行那一拍清一次，之后不再调用
                coastPrev = true;
            }
            // ★v2.5.1 总限幅：vTrim 再大也不许把指令推出 speedMax（否则 vCmd 可到 0.7 m/s）
            // ★★v3.10.12 速度外环【不许把前进速度放大超过 6%】。
            //   实测 10-20 01:14 run（_r31011_3）：转角原地转时 forward=0 ⇒ 外环 PID 复位，
            //   恢复行驶瞬间误差 0→0.28 让 P+I 一起冲，vCmd 冲到 0.397、实际 0.36 m/s（+29%）。
            //   0.28 时 R200 刚好跑得动（1.39 rad/s），到 0.36 就只剩 R259 ⇒ 弧上向外甩、
            //   e 从 +20 涨到 +33.5、打满舵、极限环、丢线。弧上速度只要多 10% 余量就归零，
            //   所以必须在【指令】侧就掐死，不能指望前馈去追。
            const float fwdAbs = fabsf(robotMotion.forward);
            float vCmdHi = speedMax;
            if (fwdAbs > 0.05f && robotMotion.forward > 0.0f)
                vCmdHi = fminf(speedMax, fwdAbs * 1.06f);
            vCmd = _constrain(robotMotion.forward + vTrim, -speedMax, vCmdHi);

            // ══ ★★★v3.14.7 终点停车（宏见 785 行附近）══
            //   ★位置铁律：必须写在 `vCmd = robotMotion.forward + vTrim` **之后**，
            //     因为这一行每帧都从 robotMotion.forward 重算 vCmd，写在这之前等于没写。
            //   ★同时必须清 robotMotion.forward/turn：否则下一帧 3424 行又把 vCmd 算回来，
            //     而且 3439 行 `coasting` 判据（|forward|<0.010）也不会成立 ⇒ 速度环继续推。
            if (LINE_STOP_EN) {
                static bool finLatch = false;
                if (lineOdoMm < LINE_STOP_ODO_MM - 2000.0f) finLatch = false;   // 新一趟复位
                if (lineOn && lineOdoMm >= LINE_STOP_ODO_MM - LINE_STOP_BRAKE_MM) {
                    const float rem = LINE_STOP_ODO_MM - lineOdoMm;             // 剩余 [mm]
                    if (rem <= LINE_STOP_LAST_MM) {
                        lineOn              = false;
                        robotMotion.forward = 0.0f;
                        robotMotion.turn    = 0.0f;
                        lineRunV            = 0.0f;
                        vCmd                = 0.0f;
                        wCmd                = 0.0f;
                        PID_VEL.reset();
                        // autoRunSt 起跑时已是 2（锁死），这里再写一次纯属双保险：
                        //   车停在起点区时灰度仍压在线上，不锁会满足"就位"条件再次起跑。
                        autoRunSt = 2;
                        finLatch  = false;
                        Serial.printf("[FIN] ★★ 到达终点 @%.2f m ⇒ 停车（判据 %.2f m）\n",
                                      lineOdoMm * 0.001f, LINE_STOP_ODO_MM * 0.001f);
                    } else {
                        // 线性降速：rem 从 BRAKE_MM 走到 LAST_MM 时，速度从 VSTART 降到 VMIN
                        const float k = (rem - LINE_STOP_LAST_MM) /
                                        (LINE_STOP_BRAKE_MM - LINE_STOP_LAST_MM);
                        const float vLim = LINE_STOP_VMIN +
                                           (LINE_STOP_VSTART - LINE_STOP_VMIN) * _constrain(k, 0.0f, 1.0f);
                        if (vCmd > vLim) vCmd = vLim;
                        if (!finLatch) {
                            finLatch = true;
                            Serial.printf("[FIN] 终点减速 @%.2f m，剩 %.0f mm ⇒ 限速 %.2f m/s\n",
                                          lineOdoMm * 0.001f, rem, vLim);
                        }
                    }
                }
            }

            if (directMode) {
                wL = directWL;
                wR = directWR;
            } else {
                // 差速运动学：ωL=(v−ω·Lw/2)/Rw , ωR=(v+ω·Lw/2)/Rw
                wL = (vCmd - wCmd * Lw * 0.5f) / Rw;
                wR = (vCmd + wCmd * Lw * 0.5f) / Rw;
                wL = _constrain(wL, -wheelMax, wheelMax);
                wR = _constrain(wR, -wheelMax, wheelMax);
            }
        }
    } else {
        wL = wR = 0.0f;
        vCmd = 0.0f;
        wCmd = 0.0f;
        idlePrev = false;
        holdActive = false;
    }

    // ★v2.9.2：M0 切 voltage 后两轴对称，settle 段不需要 J coast（回 R 近 0 对称滑行）
    //   v2.9.1 的 J coast 实测治不了（机械摩擦不对称替代电流环不对称），已弃用
    //   coastEnabled 在线 jc0/jc1 切换（默认 0=禁用，M0 电压模式对称）
    // ★v3.0.2：standbyRelease（z）——空闲松闸。判据用「无任何任务」而不用外部命令，
    //   这样任何一条运动指令都会自动退出松闸（fail-safe），不会出现"松着闸就跑车"。
    const bool idleNow = !pathOn && !moveOn && !moveSettleOn && !rotOn && !directMode
                         && fabsf(robotMotion.forward) < 1e-6f && fabsf(robotMotion.turn) < 1e-6f;
    if ((moveSettleOn && coastEnabled) || (standbyRelease && idleNow)) {
        motors.setCoast(true);   // 发 J 一次 + 抑制 R 心跳（S1 → MOT.disable() → DRV 高阻）
    } else {
        motors.setCoast(false);  // 恢复 R
        motors.setTargets(M0Dir * wL, M1Dir * wR);
    }

    // ---- 板间链路掉线告警（每秒最多一次）----
    if (!motors.online() && millis() - lastLinkWarnMs > 1000) {
        lastLinkWarnMs = millis();
        Serial.println("[WARN] 电机主控无回帧：检查 S1 固件是否已烧 / 板间串口 / 供电");
    }

    // ---- 陀螺符号自检（非阻塞，10Hz） ----
    if (imuOk && millis() < gyroPrintUntil && millis() - lastGyroMs >= 100) {
        lastGyroMs = millis();
        Serial.printf("GYRO(rad/s),%.4f,%.4f,%.4f  (nose LEFT -> GyroZ > 0)\n",
                      robotPose.GyroX, robotPose.GyroY, robotPose.GyroZ);
    }

    // ---- 20Hz 状态帧 ----
    if (debugFrame && millis() - lastDbgMs >= 50) {
        lastDbgMs = millis();
        // pitch,roll,yaw,GyroZ, v0,v1, vMeas,vCmd,wCmd, wL,wR, link, imu, S1状态(M0M1), iq0
        Serial.printf("S,%.3f,%.3f,%.3f,%.3f, %.2f,%.2f, %.3f,%.3f,%.3f, %.2f,%.2f, %d,%d, %c%c,%.3f\n",
                      robotPose.pitch, robotPose.roll, robotPose.yaw, robotPose.GyroZ,
                      BLDCData.vel[0], BLDCData.vel[1],
                      vMeas, vCmd, wCmd,
                      wL, wR,
                      motors.online() ? 1 : 0, imuOk ? 1 : 0,
                      BLDCData.state[0], BLDCData.state[1], BLDCData.iq[0]);
    }

    // ---- loop 频率统计（1 s 窗口；命令 h 开关）----
    // avg = 1e6/winUs*hzCnt 是真实自由跑频率；min/max 是单拍耗时抖动，
    // 对偏航环的意义比均值更大（一次长拍 = 一次相位滞后）。
    if (hzOn) {
        uint32_t nowUs = micros();
        if (hzLastUs) {
            uint32_t d = nowUs - hzLastUs;
            if (d > hzMaxUs) hzMaxUs = d;
            if (d < hzMinUs) hzMinUs = d;
        }
        hzLastUs = nowUs;
        hzCnt++;
        if (nowUs - hzWinUs >= 1000000u) {
            uint32_t winUs = nowUs - hzWinUs;
            float hz = (float)hzCnt * 1000000.0f / (float)winUs;
            Serial.printf("[HZ] %.1f Hz  period avg=%.0f us  min=%lu  max=%lu  n=%lu\n",
                          hz, 1000000.0f / hz,
                          (unsigned long)hzMinUs, (unsigned long)hzMaxUs, (unsigned long)hzCnt);
            hzWinUs = nowUs;
            hzCnt   = 0;
            hzMaxUs = 0;
            hzMinUs = 0xFFFFFFFFu;
        }
    }
}

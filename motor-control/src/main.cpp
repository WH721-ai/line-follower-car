/**
 * ============================================================================
 * BLDC_S1 —— 电机主控固件 v3.15.0（双无刷电机 FOC / M1 与 M0 软件对齐）
 * ============================================================================
 * 由 v2.8.0 单电机固件扩展而来（v2.8.0 已备份为
 *   BACKUP_main_v2.8.0_single_motor.cpp.txt，保留 C3 周期内速度调制等拍翼专用功能）。
 *
 * 【v3.15.0 变更（2026-10-16，第二颗电机到货 → M1 软件对齐）】
 *   目标：把 M1 在【软件层面】补齐到与 M0 同等能力（可标定、可复现），
 *         为"先让 M1 跑得和 M0 一样好，再调两轴同时运动"做准备。
 *         ★本版不动 M0 的任何现场标定结论，也不改控制算法/增益。
 *   ① 标定命令加轴参数：C[v],<ax> / P[v],<ax> / A[v],<ax>（缺省轴 0，兼容旧用法）
 *   ② NVS 增 m1dir / m1zea —— M1 也能开机直接复用标定，不再每次赌运动式对齐
 *   ③ setup() 把 M0 的四级保险（复用NVS → initFOC → 静态兜底 → 存盘）抽成
 *      initAxisFOC(i) 两轴共用；M1 首次上电也能自动兜底，不会失败一次就永久 NOSENSOR
 *   ④ F / B / W 加轴参数（F[sec],<ax> / B[n],<ax>）—— M1 也能做速度反馈体检与飞行记录
 *   ⑤ Y<v>,<ax> 成功即写 NVS（原来只解除 NOSENSOR 不存盘 → 下次开机又退回未对齐）
 *   ⑥ 开环电压上限【按轴】生效（useOpenloopVoltOne/useRunVoltOne）：原 setVoltLim()
 *      会把两轴一起压到 2V —— 标定 M1 时会顺带把正在跑的 M0 卸力
 *   ⑦ E 命令两轴对称打印（原来 M1 只打一个 a 字段）
 *   ⑧ N<n>,<ax> 可只设单轴极对数（缺省仍两轴同设，兼容旧用法）
 *   ⑨ P（测极对数）改为【只写被测轴】。原实现两轴一起写 → 测 M1 会把 M0 的 pp 也改掉、
 *      导致 M0 已标好的 zea 立即失效。现在两轴互不干扰。
 *   ⑩ L 开环测试改为【两轴一律允许】。原来 M1 处于 NOSENSOR 时 L0,2 被拒 —— 而首次接上
 *      M1 时对齐往往失败(方向未知)，"开环验相序"这唯一首验手段就被堵死。开环不碰编码器，
 *      本就不该被 NOSENSOR 挡。配套把 NOSENSOR 从"粘性标记"改为"由 dir 是否未知推导"，
 *      使 stopAll() 不会把标记抹掉。
 *   【硬件限制，软件补不了】相电流采样/过流保护只有 M0（ADC 35/33/32 只接一组分流）
 *     → M1 无 iq 判据、无 OC_FAULT，只有 DRV8313 自带 2.5A 限流 + 固件堵转检测。
 *
 * 【v3.14.1 变更（2026-10-15 台架实测）】速度环比例增益 P 0.2 → 0.1
 *   现象：v3.14.0 烧入后闭环全面劣化——2 rad/s 都 STALL，10/20/40 全「有电流却不转」
 *         （iq 1.4~1.6A、uq 顶满 7V、cnt14 只微微抖动后冻结）。
 *   排查链（逐层排除，结论都不是）：
 *     · 机械卡死？——开环 O3/O6 完全正常（2.27s 转 1.02 圈），排除；
 *     · 补偿表坏？——Xoff 关表后重测，结果逐项几乎一致，排除；
 *     · 编码器/功率级？——同上开环正常，排除。
 *   真因两条（叠加）：
 *     ① 电角度基准漂移：C（开环旋转法）的回归截距里混进了恒定【负载角】，有系统偏差，
 *        实测 C 两次给出 1.5467 / 2.4712（相差 82°电角度）。改用 A（静态锁定法，
 *        负载角=0、无偏）后 zea=1.0383，iq 立刻从 1.4A 降到 0.18A。
 *        ⇒ **标 zea 一律用 A（或 Y/initFOC），不要用 C。**
 *     ② 速度环 P 增益过大：本板速度估计自带高频噪声（见 main 内注释），P=0.2 时
 *        35 rad/s 误差即打满 7V ⇒ bang-bang 极限环。P=0.1 后翻转帧 10→0、iq ↓4.5×。
 *   结果：2/5/10/20/30/40 rad/s 全程 TRACK，零翻转，iq ≤ 0.107A。
 *
 * 【v3.14.0 变更（保守口径，2026-10-14 台架实测驱动）】
 *   实证：40 rad/s 软启动时固件报 iq≈1.65A，而台面电源（12.02V/CV 模式）只取 0.49A
 *   ⇒ 固件功率=1.5·uq·iq≈13.6W > 输入 5.9W，物理不可能 ⇒ **iq 采样虚高约 2×**，
 *   40 rad/s 的「硬启动 OC_FAULT」是采样伪报警。本版按 1.8×（保守，非满 2×）校正：
 *     ① 电流采样增益 50→90（=50×1.8）：读数略偏高 → 过流早跳，偏安全；
 *     ② OC 判据 dq.q → iq_filtered（滤波值，滤开关尖峰）+ OC_CONFIRM_COUNT 5→10；
 *     ③ PID_velocity.limit 6→7V（保守解锁高速，仍远低于 10V 硬件钳位）；
 *     ④ TARGET_SLEW_RATE 10→5 rad/s²（减速一半，压冷启动冲击电流）；
 *     ⑤ H1 隔离时 fail-safe 清零残留目标（原 H1 只"不听新命令"，电机仍按上位机最后一条
 *        R 继续转 —— 实测 H1 后 3s 仍净转 −3.2 rad）；
 *     ⑥ 记录仪 vraw/vfilt 由 int16 加宽到 int32（>32.767 rad/s 原会溢出为负）。
 *
 * 【本版目标】给「双驱差速小车」提供两台电机的独立速度控制：
 *   - 扩展主控（ESP32-S3）通过板间串口 @2Mbps 下发左右轮目标角速度
 *   - 本板（电机主控 ESP32）各自跑 SimpleFOC 速度闭环（电压模式）
 *   - 同时保留力矩模式入口，日后若要做"平衡车式"控制可不再重刷
 *
 * 【相对 v2.8.0 修复的 4 个缺陷】（双电机后必然暴露，单电机时被掩盖）
 *   ① LEDC 通道写死为 0/1/2 → 两个驱动实例互相抢占同一批通道，
 *      第二个电机 PWM 完全不可控。现按实例分配通道基址（M0=0..2，M1=3..5）。
 *   ② MagneticSensorMT6701SPI::getSensorAngle() 里的 last_valid / initialized
 *      是「函数内 static」→ 两个编码器实例共享同一份状态，
 *      一个轴的跳变会污染另一个轴。现改为成员变量。
 *   ③ 每个编码器实例各自 SPIClass(VSPI) 并各自 begin() → 同一外设重复初始化。
 *      两个 MT6701 共用 SCK/MISO（仅 CS 不同），现改为全局限一次、共享一个 SPIClass。
 *   ④ ledcAttachPin() 早于 ledcSetup() → 触发
 *      "E (120) ledc: LEDC is not initialized"。现严格先 setup 后 attach。
 *
 * 【硬件】（Pins_Specify.h）
 *   M0: 三相 PWM = 4/2/13      编码器 MT6701 CS = 18
 *   M1: 三相 PWM = 12/14/27    编码器 MT6701 CS = 22
 *   编码器共用: CLK=23, DO=19, MOSI=0（SPI 1MHz，只读）
 *   使能 ENABLE = 25  →【两路共用一根】故改由固件集中控制，不交给 SimpleFOC 的
 *                        per-driver enable/disable（否则一路停机把另一路也断电）
 *   相电流采样: 35/33/32 → 仅 M0 有（1 组 3 相分流）。M1 无采样，
 *              过流依赖 DRV8313 自带保护
 *
 * 【串口协议】
 *   USB  Serial  @115200（调试日志 + 同样能吃下面所有命令）
 *   板间 Serial2 @2Mbps  （与扩展主控通信，引脚用 ESP32 UART2 默认 16/17）
 *
 *   入（\n 结尾，大小写不敏感）：
 *     R<v0>,<v1>   速度模式：设 M0/M1 目标角速度 [rad/s]
 *     R<v>         两个电机都设为 v（单参数，兼容旧上位机）
 *     R            两个电机都设为 3.0 rad/s
 *     T<u0>,<u1>   力矩模式：设 M0/M1 目标 Uq [V]（|u|<=10）
 *     S            立即停机（两路目标清零 + 断使能）
 *     K<p>,<i>     设两路速度环 P/I
 *     Q            立即回一帧状态
 *     ?            打印帮助
 *
 *   【v3.1.0/v3.2.0 新增：闭环电角度诊断/标定 + 板间口隔离（排查"开环能转、闭环却不转"）】
 *     背景：开环(velocity_openloop)只用时间算换相角，不碰 zero_electric_angle；
 *           闭环用 elecAngle = norm(dir*pole_pairs*mechAngle - zero_electric_angle)。
 *           两者唯一差别就是 zero_electric_angle（及 dir）。库里 alignSensor 只把
 *           电压矢量钉在 270° 保持 700ms 就采样，转子若未就位则 zea 系统性偏差。
 *     H[0/1]       板间串口隔离：H1 后只认 USB 命令（单板标定前必开——S3 会 20Hz 灌 R）
 *     P[v][,<ax>]  直接测量真实极对数：开环施加已知电角速度，编码器实测机械角速度反算
 *                  pp_true = (pole_pairs_assumed × v) / ω_mech。★闭环不转时先跑这个
 *                  ★只写被测那一轴（另一轴的 pp/zea 不受影响）
 *     C[v][,<ax>]  开环对准法标定 zero_electric_angle（默认 2 rad/s；轴缺省 0）
 *                  利用"开环时转子确实同步跟随磁场"反解 zea，并给出离散度 maxDev / 一致性 r
 *                  ★v3.14.1 更正：C 的回归【截距里含恒定负载角】⇒ 有系统偏差。实测同一硬件
 *                    两次 C 给出 1.5467 / 2.4712（相差 82°电角度）。C 只适合当"一致性/离散度"
 *                    诊断用，**定标一律用 A**（静态锁定，负载角=0、无偏）。
 *     Z            打印两轴 zero_electric_angle / sensor_direction
 *     Z<a>,<b>     手工设两轴 zero_electric_angle [rad]（配合上位机脚本扫一圈找最优）
 *     D<a>,<b>     手工设 sensor_direction（+1/-1）
 *     A[v][,<ax>]  手工重新对齐：270° 保持 1.5s 后再算 zea（默认 3V；轴缺省 0）
 *     N<n>[,<ax>]  强制设极对数（默认 7）；缺省两轴同设，给轴号则只设该轴。
 *                  改后该轴 zea 失效，须重跑 A 重标
 *
 *   【v3.6.0 关键发现：开环电压可调 + 原始编码器读数】
 *     读源码得两条事实，直接解释了前面一连串怪象：
 *       ① velocityOpenloop() 里 Uq = voltage_limit —— 开环直接拿电压上限当施加电压。
 *          默认 10V 在静止/低速会灌入大电流 → 过流保护立刻把开环打回闭环。
 *          实测：L 开环只撑了约 0.25s 就掉出（这也是 C 标定"转子没跟稳"的真因）。
 *       ② 开环时 velocityOpenloop() 会执行 shaft_angle=_normalizeAngle(shaft_angle+target*Ts)，
 *          即【开环下 A/V 帧反映的是指令积分器，不是编码器】——用 A 判编码器是错的。
 *     V<v>         设电压上限 [0.5~10]（开环 L/C/P 前先降到 2~3）
 *     E            打印原始传感器读数（a / mech / v）与 shaft_angle，绕过开环覆写
 *     F[sec][,<ax>] 速度反馈体检：按环频统计反馈量的均值/标准差/翻转次数 + Δ计数直方图
 *     O<v>         开环电压上限 [0.5~10]（默认 2；与闭环的 V 分离，进 L/C/P 自动采用）
 *     B[n][,<ax>] / W   飞行记录仪：按环频存 n 拍 (t,原始计数,v原始,v滤波,iq,uq) 到 RAM，停机后倒出
 *
 *   【v3.9.0 根因修复：编码器偏心补偿（X 命令）】
 *     实测确诊：MT6701 的磁铁/芯片偏心安装 → 编码器角带「每机械转一次」的正弦误差，
 *     机械角振幅 ±19.5°（=±136° 电角度）。开环换相角=时间积分、不读编码器，故开环正常；
 *     闭环 elecAngle=norm(dir*pp*θ_enc−zea) 直接吃进该误差 → 每转里两次把换相矢量推偏过
 *     ±90° → 力矩周期过零/反向 → 电机激荡。
 *     对策：用开环磁场角当「精确时钟」测出误差
 *       err = normalize(dir*pp*θ_enc − pp*θ_shaft)，按 θ_enc 分 256 箱取均值，取负作补偿表，
 *       在 getSensorAngle() 里加回去（表存 NVS，上电自动载入，initFOC 前生效）。
 *     X            标定 M0：开环慢转 8s 自动建表 → 存 NVS → 立即启用
 *     X0 / X1      标定对应轴
 *     X?           打印两轴补偿状态（on / 峰值角度）
 *     Xoff         清除两轴补偿表并关闭
 *     ★标定完自动收尾；之后请重跑 C 重标 zea（此时 zea 已在补偿后的角度上算）再跑闭环
 *
 *   【v3.10.0 关键修复：编码器读数"永久锁死" + 速度环可用增益】
 *     实测确诊（飞行记录仪 _rec_g21b.csv 分段分析）：闭环"有电流却不转"的真凶不是 zea、
 *     也不是增益，而是 getSensorAngle() 的异常读数丢弃逻辑把读数【永久锁死】：
 *       一次 SPI 受扰读出 Δcnt=8192（=恰好 π，低 13 位不变、只翻最高位）的假跳变，
 *       旧逻辑 `if(diff>π && !crossing) return _last_valid;` 且【不更新】_last_valid，
 *       若该假跳变持续出现 → 读数永远停在旧值 → v≡0 → 速度环把 uq 顶到限幅灌 1.4A → 不转。
 *     修复：加 _rej 连续异常计数，最多连丢 2 拍，第 3 拍无条件接受，保证绝不死锁。
 *     配套：init() 之后设速度环默认值（init 会把 PID_velocity.limit 重置为 voltage_limit）：
 *       P0.2 / I1.0 / D0 / limit6.0 / LPF_velocity.Tf=0.003（5ms 默认滞后过大，空载易自激）
 *     K<p>,<i>[,<d>[,<lim>]]  在线改速度环 PID（含输出限幅）
 *     G<tf>                   在线改速度反馈低通时间常数 Tf [s]（0.0002~0.05）
 *
 *   【v3.11.0 修复：对齐失败会把该轴"永久锁死"（真陷阱）】
 *     症状：开机 initFOC() 失败（转子上电不动 → alignSensor 的 moved<0.062rad）→ dir 保持
 *       UNKNOWN(0)、zea 保持 NOT_SET(-12345) → 固件贴 AX_NOSENSOR，且【库内已 disable() 电机】。
 *       后果：R/T 被固件拒绝；L 开环虽被允许，但 move()/loopFOC() 见 enabled==0 直接 return
 *       → 零 PWM、零电流、零位移 → 极易误判成"硬件没电"。且无任何命令能解锁。
 *     修复：
 *       ① 开机 M0 对齐失败 → 自动提高对齐电压(6V)+enable()+重跑一次 initFOC；
 *       ② L 开环前强制 MOT[i]->enable()（开环必须一定有力，才能把固件/硬件问题分开）；
 *       ③ 新增 Y[v],<轴> = 运行时重对齐+解除 NOSENSOR（唯一的复活手段）。
 *
 *   【v3.12.0 修复：三个独立缺陷（闭环调试的三个真障碍）】
 *     ① 「只发 R 闭环不转，先发 L/Y 就转」——SimpleFOC 的 loopFOC()/move() 在 !enabled 时
 *        直接 return（BLDCMotor.cpp:345/416），而主循环从不 enable()。L/Y 恰好 enable() 过，
 *        所以表现为"有时能转"。修复：setVel/setTorque 在 freshStart 时 MOT[i]->enable()。
 *     ② 「开机对齐时好时坏」——失败只发生在 alignSensor 的运动式方向判定
 *        （moved<0.062rad → 返回 0 → dir=UNKNOWN → NOSENSOR 锁死）。库里若 dir 已知就跳过运动
 *        判定、改用静止矢量反算 zea，完全可复现。修复：NVS 复用上次标定 + 失败时静态兜底。
 *     ③ A 命令在 dir 未知时会把 zea 算成 0（假对齐）。修复：A 先补已知 dir 再算并存盘。
 *
 *   出（每 50ms 一帧，IDLE 时 200ms 心跳）：
 *     A<a0>,V<v0>,A1<a1>,V1<v1>,S<c0><c1>,I<i0>
 *       a0/v0 = M0 编码器绝对角[rad] / 实测角速度[rad/s]
 *       a1/v1 = M1 同上
 *       c0/c1 = 每轴状态字符 I=IDLE R=RUN S=STALL O=OC
 *       i0    = M0 的 Iq 电流[A]（M1 无采样，恒 0）
 * ============================================================================
 */

#include <SimpleFOC.h>
#include <Preferences.h>
#include "Pins_Specify.h"

// ===========================================================================
// 1) 3PWM 驱动（修缺陷 ①④）
// ===========================================================================
class MyDriver3PWM : public BLDCDriver {
    int pinA, pinB, pinC;
    int chA, chB, chC;
public:
    // chBase: 本实例独占的 3 个 LEDC 通道基址（M0 传 0 → 0/1/2；M1 传 3 → 3/4/5）
    // 注意：使能脚不在驱动里控制（两点原因：双路共用一根 GPIO25；且 SimpleFOC 会在
    //       initFOC 内部反复 enable/disable，交给它会把另一路电机一起断电）
    MyDriver3PWM(int a, int b, int c, int chBase)
        : pinA(a), pinB(b), pinC(c), chA(chBase), chB(chBase + 1), chC(chBase + 2) {}

    int init() override {
        // 缺陷④：必须先 ledcSetup 再 ledcAttachPin
        ledcSetup(chA, 25000, 10); ledcAttachPin(pinA, chA);
        ledcSetup(chB, 25000, 10); ledcAttachPin(pinB, chB);
        ledcSetup(chC, 25000, 10); ledcAttachPin(pinC, chC);
        ledcWrite(chA, 0); ledcWrite(chB, 0); ledcWrite(chC, 0);
        initialized = true;
        return 1;
    }
    void enable()  override { /* 由固件集中控制 GPIO25 */ }
    void disable() override { ledcWrite(chA, 0); ledcWrite(chB, 0); ledcWrite(chC, 0); }
    void setPwm(float Ua, float Ub, float Uc) override {
        auto w = [&](int c, float u) {
            ledcWrite(c, constrain(u / voltage_power_supply * 1023.0f, 0.0f, 1023.0f));
        };
        w(chA, Ua); w(chB, Ub); w(chC, Uc);
    }
    void setPhaseState(PhaseState, PhaseState, PhaseState) override {}
};

// ===========================================================================
// 2) MT6701 SPI 编码器（修缺陷 ②③）
// ===========================================================================
// 两个 MT6701 共用 SCK=23 / MISO=19 / MOSI=0，只有 CS 不同 ⇒ SPI 外设全局只初始化一次
static SPIClass &mtSpi() {
    static SPIClass spi(VSPI);
    static bool inited = false;
    if (!inited) {
        inited = true;
        spi.begin(MT6701_CLK, MT6701_DO, 0, -1); // SCK=23, MISO=19, MOSI=0, SS=-1
        spi.setFrequency(1000000);               // 1MHz，抗干扰
        spi.setDataMode(SPI_MODE0);
    }
    return spi;
}

#define ENC_COMP_N 256   // 偏心补偿表箱数（每箱 2π/256 ≈ 1.406° 机械）

class MagneticSensorMT6701SPI : public Sensor {
    int   cs;
    float _last_valid = 0.0f;  // 缺陷②：原为函数内 static，两实例共享 → 改为成员
    bool  _valid_init = false;
    uint8_t _rej = 0;          // ★v3.10.0 连续异常读数计数（防止"丢弃"逻辑永久锁死）
public:
    // ★v3.9.0 编码器偏心补偿表：comp[i] = 该机械角区间要加的角度补偿 [rad]
    float   comp[ENC_COMP_N];
    uint8_t compOn     = 0;
    float   compAmpDeg = 0.0f;   // 补偿峰值（机械角, deg），仅用于打印/诊断

    MagneticSensorMT6701SPI(int _cs) : cs(_cs) {
        for (int i = 0; i < ENC_COMP_N; i++) comp[i] = 0.0f;
    }

    void init() {
        pinMode(cs, OUTPUT);
        digitalWrite(cs, HIGH);
        mtSpi();               // 缺陷③：SPI 只初始化一次
        delay(10);
        getSensorAngle();
    }

    // 按原始编码器机械角查表加补偿，并回绕回 [0,2π)
    float applyComp(float a) {
        int i = (int)(a * (ENC_COMP_N / _2PI));
        if (i < 0) i = 0; else if (i >= ENC_COMP_N) i = ENC_COMP_N - 1;
        float r = a + comp[i];
        while (r < 0.0f)  r += _2PI;
        while (r >= _2PI) r -= _2PI;
        return r;
    }

    float getSensorAngle() override {
        SPIClass &spi = mtSpi();
        noInterrupts();
        digitalWrite(cs, LOW);
        spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        uint16_t raw = spi.transfer16(0x0000);
        digitalWrite(cs, HIGH);
        spi.endTransaction();
        interrupts();

        raw = (raw >> 2) & 0x3FFF;                 // MT6701 14bit
        float a = raw * (_2PI / 16384.0f);
        if (compOn) a = applyComp(a);              // ★v3.9.0 偏心补偿（在有效性判据之前做，保证下游一致）

        if (!_valid_init) { _valid_init = true; _last_valid = a; return a; }

        float diff = fabs(a - _last_valid);
        if (diff > _PI) {
            // 正常跨 0/2π 边界 vs SPI 受扰读出的假跳变
            bool near_boundary = (a < 0.5f && _last_valid > _2PI - 0.5f) ||
                                 (_last_valid < 0.5f && a > _2PI - 0.5f);
            // ★v3.10.0 关键修复：原实现 `return _last_valid;` 且【不更新】_last_valid，
            //   一旦异常读数持续（实测：一次差正好 π 的假跳变，低13位不变、只翻最高位），
            //   读数就【永久锁死】→ v 恒 0 → 闭环把电压顶到 10V 灌 1.4A → "有电流却不转"。
            //   现在最多连续丢 2 拍，第 3 拍一律接受，保证绝不死锁。
            if (!near_boundary) {
                _rej++;
                if (_rej < 3) return _last_valid;   // 丢弃本次异常读数（不更新 _last_valid）
            }
        }
        _rej = 0;
        _last_valid = a;
        return a;
    }
};

// ===========================================================================
// 3) 硬件对象
// ===========================================================================
MagneticSensorMT6701SPI sensor0(MT6701_CS0);   // M0 → CS=18
MagneticSensorMT6701SPI sensor1(MT6701_CS1);   // M1 → CS=22
MyDriver3PWM driver0(M0PWMA, M0PWMB, M0PWMC, 0); // LEDC 通道 0/1/2
MyDriver3PWM driver1(M1PWMA, M1PWMB, M1PWMC, 3); // LEDC 通道 3/4/5
BLDCMotor   motor0(7);                          // 7 极对
BLDCMotor   motor1(7);

BLDCMotor              *MOT[2] = { &motor0, &motor1 };
MyDriver3PWM           *DRV[2] = { &driver0, &driver1 };
MagneticSensorMT6701SPI *SEN[2] = { &sensor0, &sensor1 };

// 两路共用的使能脚（GPIO25）
static inline void setEnable(bool en) { digitalWrite(ENABLE, en ? HIGH : LOW); }

// ★v3.12.0：本台架 M0 编码器方向在【每一次成功对齐】中恒为 -1（多次实测）。
//   仅用作"运动式对齐失败时"的兜底方向；一旦 A/Y 成功会把真实值写进 NVS 覆盖它。
// ★v3.15.0：M1 同型号装配（同电机 + 同 MT6701），暂沿用同一缺省 -1。若 M1 方向实际相反，
//   用 D1,1（或 D-1,1）再跑 A3,1 修正，之后 NVS 会记住真实值。
#define AX_DIR_DEFAULT(i)  ((i) == 0 ? -1 : -1)

// ★v3.15.0：NVS 键名两轴对称（原来只有 m0dir/m0zea → M1 无法持久化标定）
static inline const char *nvsDirKey(int i) { return (i == 0) ? "m0dir" : "m1dir"; }
static inline const char *nvsZeaKey(int i) { return (i == 0) ? "m0zea" : "m1zea"; }

// ===========================================================================
// 4) 电流采样（仅 M0；M1 无分流采样）
// ===========================================================================
// ★v3.14.0：放大倍数校正。原按 50 V/V 算，实测（电源 0.49A@40rad/s 而固件报 1.65A）虚高约 2×；
//   取保守值 1.8×（宁可读数略高、过流早跳，也不放过真过流）→ 50×1.8 = 90。
//   疑似真因：板上 INA240 实际增益 100 V/V（非 50），或低边采样未与 PWM 同步。
#define CS_GAIN_CORRECTED    90.0f  // = 50 × 1.8（保守校正，非满 2×）
#define OC_CURRENT_LIMIT_A   2.0f   // 校正后 ≈ 真值 1.8A 触发（电机 3A / 驱动 2.5A 之内，留裕量）
#define OC_CONFIRM_COUNT     10     // 连续 N×10ms 超限才触发（100ms 窗口，滤开关尖峰）
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, CS_GAIN_CORRECTED, 35, 33, 32);
float iq_filtered = 0.0f;

// ===========================================================================
// 5) 每轴状态
// ===========================================================================
enum AxisState : uint8_t { AX_IDLE = 0, AX_RUN = 1, AX_STALL = 3, AX_OC = 4, AX_NOSENSOR = 5 };

struct Axis {
    float    target    = 0.0f;  // 经斜率限制后的目标
    float    targetCmd = 0.0f;  // 指令目标
    float    velocity  = 0.0f;  // 实测角速度
    uint8_t  mode      = 1;     // 1=速度模式  4=力矩模式（与商家 SF_Motor 常量一致）
    AxisState state    = AX_IDLE;
    uint32_t runStart  = 0;
    float    lastAngle = 0.0f, stallAccum = 0.0f;
    uint32_t lastStallUpdate = 0, lastStallCheck = 0;
};

static Axis ax[2];

#define TARGET_SLEW_RATE  5.0f   // ★v3.14.0：10→5，斜率减半，降低高速冷启动的加速电流冲击（保护机构）

// ★v3.16.0：斜率改为运行期可调（`W<rad/s²>`），默认仍 5。
//   动机：0.0752 m/s²（=5×Rw）意味着 0→0.3 m/s 要 4 s，完全做不了"急启急停"。
//   台架上先扫出既不丢步、又不触发过流的斜率，再决定是否改默认值。
//   注意：斜率只是【目标】的限速；实际轮速还受速度环、静摩擦和 DRV 限流的约束。
static float slewRate = TARGET_SLEW_RATE;

// ---- 开环速度测试模式（L 命令）----
// 绕过编码器/FOC 电角度，直接用固定电压矢量开环换相旋转。用途：把故障域一刀切开——
//   开环能平稳转 = 功率级/相序/极对数没问题，病在编码器或电角度；
//   开环也不转/只嗡嗡 = 相序、极对数或功率级的问题。
// 进入后忽略 R/T 指令（避免上位机干扰），到时自动退出（安全）。
static uint8_t  testMode  = 0;
static uint32_t testUntil = 0;
#define TEST_MODE_MS  12000

// ---- 板间串口隔离（H 命令）----
// 扩展主控(S3) 运行时以 ~20Hz 持续向本板下发 R<vL>,<vR>（实测：USB 一个字节不发，
// 仍有 ~19Hz 的 [RX] R 回显）。单板台架标定（如电角度扫描）时这些命令会立刻盖掉
// 本机设定的目标速度 → 必须先 H1 把板间口静音，否则标定结果全是假的。
static uint8_t  hostLock = 0;   // 1 = 只接受 USB(Serial) 命令，忽略板间 Serial2

// ---- 电压上限分离（V = 闭环 / O = 开环）----
// ★坑：MOT.voltage_limit 有两个用途 —— 开环 velocityOpenloop() 直接拿它当 Uq，
// 同时它也是闭环 setPhaseVoltage 的输出钳位。所以「为开环防过流而 V2」会顺带把
// 闭环输出也压到 2V，之后跑闭环测试就会莫名其妙没力气。现在两者分离：
// 进开环（L/C/P）自动降到 O 值；退出（stopAll / 标定收尾 / 测试超时）自动恢复 V 值。
static float runVLimit = 10.0f;   // 闭环电压上限（V 命令）
static float olVLimit  =  2.0f;   // 开环电压上限（O 命令）
static inline void setVoltLim(float vl) { for (int i = 0; i < 2; i++) MOT[i]->voltage_limit = vl; }
static inline void useOpenloopVolt() { setVoltLim(olVLimit); }
static inline void useRunVolt()      { setVoltLim(runVLimit); }
// ★v3.15.0：按轴版本。单轴标定（C/P/X）必须只动自己那一轴的 voltage_limit ——
//   原来的 setVoltLim() 会把两轴一起压到开环 2V，标定 M1 时会把正在闭环跑的 M0 一并卸力。
static inline void useOpenloopVoltOne(int i) { MOT[i]->voltage_limit = olVLimit; }
static inline void useRunVoltOne(int i)      { MOT[i]->voltage_limit = runVLimit; }

// ---- 开环对准法标定 zero_electric_angle（C 命令）----
// 为什么不用库内 alignSensor()：它只在 270° 处保持 700ms 就认定转子"就位"，遇到齿槽/摩擦
// 会停在错误的档位 → zea 系统性偏差（实测同一台电机多次开机竟得到 6.27 / 3.89 / 2.66 三种值，
// 而 zea 本应是磁铁+编码器安装决定的固定量）。
//
// 本法依据一个已被实测证实的事实：**开环时转子确实同步跟随磁场**（`L4,0` → V0=+4.00 平稳转）。
// 于是可以反解转子电角度：
//   同步时  转子电角度 φ ≡ 施加的电角度 = normalize(pole_pairs × shaft_angle_openloop)
//   而读数  elecAngle()  = normalize(sensor_direction × pole_pairs × mechAngle − zea)
//   令两者相等 ⇒ zea = normalize(sensor_direction×pole_pairs×mechAngle − normalize(pole_pairs×shaft_angle))
// 连续采样 N 次：真同步则每次算出的 zea 完全一致（离散度≈0）；离散度大 = 没跟住，结果不可信。
// 也就是说这个标定法**自带可信度判据**。
static uint8_t  calState = 0;   // 0=空闲 1=锁定磁场 2=采样zea 3=收尾 4=测极对数
static uint8_t  calMode  = 0;   // 0=只标 zea(C)  1=测极对数(P)
static uint8_t  calAxis  = 0;   // ★v3.15.0 本标定作用于哪一轴（0=M0 / 1=M1）
static uint32_t calT0    = 0;
static float    calV     = 2.0f;
static float    calCos   = 0.0f, calSin = 0.0f;
static int      calN     = 0;
static float    calPrev  = 0.0f, calMaxDev = 0.0f;
static float    calA0    = 0.0f;  // 测极对数用的起始机械角

// ---- 速度反馈体检（F 命令）----
// 目的：量化「速度反馈在这个工作点上到底有多脏」。做法：开环匀速驱动（真实转速恒定），
// 按环频(≈1kHz)统计反馈量的均值/标准差 —— 标准差就是噪声；同时统计原始角每周期跨越的
// 编码器计数（Δ计数），既能看量化台阶，也能反推真实环路周期 Ts。
// 理论：14bit ⇒ 1 计数 = 2π/16384 = 0.000383 rad；Ts≈1ms ⇒ 1 计数/周期 = 0.38 rad/s。
// 注意：开环时 MOT->shaft_velocity 被库覆写成指令值，只有 SEN->getVelocity() 才是实测。
static uint8_t  fbOn = 0, fbIsOpen = 0, fbAxis = 0;   // ★v3.15.0 fbAxis = 体检的是哪一轴
static uint32_t fbUntil = 0, fbN = 0, fbFN = 0, fbFlips = 0, fbZero = 0;
static double   fbSum = 0.0, fbSum2 = 0.0, fbFSum = 0.0, fbFSum2 = 0.0;
static float    fbMin = 0.0f, fbMax = 0.0f;
static int8_t   fbLastSign = 0;
static float    fbAngPrev = 0.0f;
static int32_t  fbCntSum = 0;
static uint16_t fbHist[8] = {0};

// ---- 飞行记录仪（B / W 命令）----
// 用途：F 命令只给统计量，看到"1kHz 级速度波动 45%"却分不清是
//   ① 真机械脉动（齿槽 stick-slip）  ② 环路周期抖动  ③ 编码器读数噪声
// 所以按环频把每拍的原始数据存进 RAM，停机后慢慢倒出来，在 PC 上做谱/相关分析。
#define REC_MAX 2000
struct Rec { uint32_t t; int32_t vraw; int32_t vfilt; uint16_t cnt; int16_t iq; int16_t uq; };
static Rec      recBuf[REC_MAX];
static uint8_t  recOn = 0;
static uint8_t  recAxis = 0;    // ★v3.15.0 记录仪采的是哪一轴
static uint16_t recN = 0, recWant = 0;

// ---- 编码器偏心补偿标定（X 命令，v3.9.0）----
// 补偿表本体存在各 MagneticSensorMT6701SPI 实例里（comp[]/compOn）；这里只放标定状态机与累加器。
static Preferences encPrefs;
static uint8_t   encCalAxis  = 0;    // 正在标定的轴
static uint8_t   encCalState = 0;    // 0=空闲 1=锁定磁场 2=采样 3=收尾
static uint32_t  encCalT0    = 0;
static float     encCalV     = 2.0f; // 开环标定速度 [rad/s]（机械）
static float     encCalSec   = 8.0f; // 采样时长 [s]
static double    encBinSum[ENC_COMP_N];
static double    encBinSS[ENC_COMP_N];   // 每箱 err² 累加 → 算箱内离散度，判断标定时转子是否真锁相
static uint16_t  encBinCnt[ENC_COMP_N];
static float     encMechPrev  = 0.0f;    // 上一拍机械角（用于累计真实行程）
static double    encTravel    = 0.0;     // 采样期间累计机械角行程 [rad]
static double    encSumAll   = 0.0;
static uint32_t  encTotN     = 0;

static char stateChar(AxisState s) {
    switch (s) {
        case AX_RUN:      return 'R';
        case AX_STALL:    return 'S';
        case AX_OC:       return 'O';
        case AX_NOSENSOR: return 'X';
        default:          return 'I';
    }
}

static void resetStall(int i) {
    ax[i].lastAngle      = MOT[i]->shaft_angle;   // 用缓存角，别再每 50ms 多打一次 SPI（会扰动刚在 move() 里算好的速度）
    ax[i].stallAccum     = 0.0f;
    ax[i].lastStallCheck = millis();
    ax[i].lastStallUpdate = millis();
}

// 启动/调速一个轴（速度模式）
static void setVel(int i, float v) {
    if (testMode) return;                     // 开环测试模式期间不接受常规速度指令
    if (ax[i].state == AX_NOSENSOR) return;   // 该轴 FOC 未对齐（无编码器/电机）→ 拒绝启动
    bool freshStart = (ax[i].state != AX_RUN);
    bool spinUp     = (fabsf(ax[i].targetCmd) < 0.1f && fabsf(v) >= 0.1f);
    bool reversal   = (v * ax[i].targetCmd < 0.0f);

    ax[i].targetCmd = v;
    ax[i].state     = AX_RUN;
    if (ax[i].mode != 1) { ax[i].mode = 1; MOT[i]->controller = MotionControlType::velocity; }

    // ★v3.12.0：闭环必须 enabled==1 才会出力（loopFOC/move 都在 !enabled 时 return）。
    if (freshStart) { ax[i].runStart = millis(); resetStall(i); MOT[i]->enable(); }
    else if (spinUp || reversal) { resetStall(i); }  // 同向连续调速不重置（否则堵转保护失效）
}

// 启动/调力矩一个轴（力矩模式，目标单位 V）
static void setTorque(int i, float u) {
    if (testMode) return;                     // 开环测试模式期间不接受常规力矩指令
    if (ax[i].state == AX_NOSENSOR) return;   // 同上：未对齐的轴不给力矩
    bool freshStart = (ax[i].state != AX_RUN);
    ax[i].targetCmd = constrain(u, -10.0f, 10.0f);
    ax[i].target    = ax[i].targetCmd;
    ax[i].state     = AX_RUN;
    if (ax[i].mode != 4) { ax[i].mode = 4; MOT[i]->controller = MotionControlType::torque; }
    if (freshStart) { ax[i].runStart = millis(); resetStall(i); MOT[i]->enable(); }  // ★v3.12.0
}

static void stopAll() {
    for (int i = 0; i < 2; i++) {
        // ★v3.15.0：NOSENSOR 改为「由 dir 是否未知推导」而不是粘性标记。
        //   原因：L 开环测试会把 state 置成 AX_RUN（开环不需要编码器），若用粘性标记，
        //   一次 L 就会把该轴的 NOSENSOR 永久抹掉 → 之后 R 闭环会在未标定的轴上乱来。
        ax[i].state     = ((int)MOT[i]->sensor_direction == 0) ? AX_NOSENSOR : AX_IDLE;
        ax[i].targetCmd = 0.0f;
        ax[i].target    = 0.0f;   // 停机不走斜率
    }
    if (testMode) testMode = 0;            // 停机即退出开环测试
    calState = 0;                          // 停机即中止标定
    for (int i = 0; i < 2; i++)
        if (MOT[i]->controller != MotionControlType::velocity)
            MOT[i]->controller = MotionControlType::velocity;  // 恢复闭环速度模式
    useRunVolt();                 // 退出开环：电压上限恢复为闭环值（V 命令）
    setEnable(false);             // 两路共用使能 → 一起断
    Serial.println("[RX] stop all");
}

// 开环对准法标定：非阻塞状态机，由 loop() 在 calState!=0 时逐步推进
// ★v3.15.0：作用于 calAxis（0=M0 / 1=M1），两轴共用同一套流程。
static void calStep(uint32_t now) {
    int i = calAxis;
    if (calState == 1) {                   // 阶段1：开环慢转，让转子锁定磁场
        MOT[i]->target = calV;             // velocity_openloop 把 target 当速度用
        if (now - calT0 > 1500) {
            if (calMode == 1) {            // P 命令：转去测极对数
                calState = 4; calT0 = now; calA0 = SEN[i]->getAngle();
                Serial.printf("[PP] M%d 已锁定磁场，开始测机械角速度…\n", i);
            } else {                       // C 命令：转去测 zea
                calState = 2; calT0 = now;
                calCos = calSin = 0.0f; calN = 0; calPrev = 0.0f; calMaxDev = 0.0f;
                Serial.printf("[CAL] M%d 转子已锁定磁场，开始采样…\n", i);
            }
        }
        return;
    }
    if (calState == 4) {                   // 阶段4：实测机械角速度 → 反算极对数
        // 开环时施加的电角速度 = pole_pairs × target（固件自己定的，不需要知道真值）；
        // 转子同步跟随 ⇒ 机械角速度 = 电角速度 / pp_true ⇒ pp_true = 电角速度 / 机械角速度。
        // 这是唯一不依赖任何极对数假设的直接测量。
        if (now - calT0 > 1000) {
            float a1 = SEN[i]->getAngle();
            float dt = (float)(now - calT0) * 0.001f;
            float wm = (a1 - calA0) / dt;                  // 机械角速度 rad/s（传感器原始量）
            float we = (float)MOT[i]->pole_pairs * calV;   // 施加的电角速度 rad/s
            float pp = (fabsf(wm) > 0.05f) ? we / fabsf(wm) : 0.0f;
            Serial.printf("[PP] M%d 电角速度=%.2f  实测机械=%.3f rad/s  ->  极对数 ≈ %.3f\n",
                          i, we, wm, pp);
            if (pp > 1.0f && pp < 60.0f) {
                int n = (int)lroundf(pp);
                // ★v3.15.0：只写被测轴。原实现两轴同写 → 测 M1 会把 M0 的 pp 一起改掉，
                //   使 M0 已标好的 zea 立即失效（两轴互相干扰）。
                MOT[i]->pole_pairs = n;
                Serial.printf("[PP] M%d 已设 pole_pairs=%d（原假设 7）→ 请对该轴重跑 A 重标 zea\n",
                              i, n);
                if (i == 0) Serial.println("[PP] (另一轴 M1 的 pole_pairs 未被改动；需要时用 N<n>,1)");
                else        Serial.println("[PP] (另一轴 M0 的 pole_pairs 未被改动；需要时用 N<n>,0)");
            } else {
                Serial.println("[PP] [!!] 结果不合理（转子没跟住/速度太低），未改 pole_pairs");
            }
            MOT[i]->controller = MotionControlType::velocity;
            MOT[i]->target     = 0.0f;
            ax[i].targetCmd = ax[i].target = 0.0f;
            ax[i].state = AX_IDLE;
            useRunVoltOne(i);              // 退出开环测 pp → 只恢复该轴的闭环电压上限
            setEnable(false);
            calState = 0;
        }
        return;
    }
    if (calState == 2) {                   // 阶段2：采样 zea 候选值
        float mech    = SEN[i]->getMechanicalAngle();
        float applied = _normalizeAngle((float)MOT[i]->pole_pairs * MOT[i]->shaft_angle);
        float z = _normalizeAngle((float)MOT[i]->sensor_direction * MOT[i]->pole_pairs * mech - applied);
        if (calN > 0) {
            float d = fabsf(z - calPrev);
            if (d > _PI) d = _2PI - d;
            if (d > calMaxDev) calMaxDev = d;
        }
        calPrev = z;
        calCos += cosf(z); calSin += sinf(z); calN++;
        if (now - calT0 > 1000) calState = 3;
        return;
    }
    if (calState == 3) {                   // 阶段3：圆均值 + 可信度判据 + 落盘
        float mean = _normalizeAngle(atan2f(calSin, calCos));
        float r    = (calN > 0) ? sqrtf(calCos * calCos + calSin * calSin) / (float)calN : 0.0f;
        Serial.printf("[CAL] M%d N=%d  zea=%.4f rad (%.1f elec°)  maxDev=%.4f rad  r=%.4f\n",
                      i, calN, mean, mean * 180.0f / _PI, calMaxDev, r);
        if (calMaxDev < 0.15f && r > 0.99f)
            Serial.println("[CAL] [OK]  转子稳定同步 → 该 zea 可信");
        else
            Serial.println("[CAL] [!!]  离散度偏大 → 转子没跟稳，结果存疑（可用更小的 v 重试）");
        MOT[i]->zero_electric_angle = mean;
        MOT[i]->controller = MotionControlType::velocity;
        MOT[i]->target     = 0.0f;
        ax[i].targetCmd = ax[i].target = 0.0f;
        ax[i].state = AX_IDLE;
        useRunVoltOne(i);              // 退出开环标定 → 只恢复该轴的闭环电压上限
        setEnable(false);
        calState = 0;
        return;
    }
}

// ===========================================================================
// 5a-2) 编码器偏心补偿标定（X 命令，v3.9.0）
// ===========================================================================
// 原理：开环匀速时磁场角 θ_field = pp·shaft_angle 是只随时间走的「精确时钟」；
//   同步的转子满足 θ_field ≡ pp·θ_true（差一个恒定滞后，会被减均值抹掉）。于是
//     err(i) = normalize(dir·pp·θ_enc − pp·θ_shaft) 在箱 i 上的均值  =  pp·ε(i) (+常数)
//   补偿表  comp[i] = −(err_mean(i) − err_global_mean) / pp   [机械 rad]
//   ★减全局均值：把「转子跟随磁场的固定滞后」这个直流分量去掉，交给 C 标定去吸收。
//   ★采样期间必须关补偿（compOn=0），否则测到的是残留误差而非原始误差。
// 标定完自动收尾（停机、恢复闭环电压上限、存 NVS、启用）。非阻塞，由 loop() 推进。

// 把某轴的补偿表写入 NVS（key 形如 t0/t1、开关 o0/o1）
static void encCompSave(int ax_) {
    char k[4], ko[4];
    snprintf(k,  sizeof(k),  "t%d", ax_);
    snprintf(ko, sizeof(ko), "o%d", ax_);
    encPrefs.putBytes(k, SEN[ax_]->comp, sizeof(float) * ENC_COMP_N);
    encPrefs.putUChar(ko, SEN[ax_]->compOn);
}

// 从 NVS 载入补偿表（无表则保持关闭）
static void encCompLoad(int ax_) {
    char k[4], ko[4];
    snprintf(k,  sizeof(k),  "t%d", ax_);
    snprintf(ko, sizeof(ko), "o%d", ax_);
    size_t need = sizeof(float) * ENC_COMP_N;
    if (encPrefs.getBytesLength(k) == need) {
        encPrefs.getBytes(k, SEN[ax_]->comp, need);
        SEN[ax_]->compOn = encPrefs.getUChar(ko, 1);
        float mn = 1e9f, mx = -1e9f;
        for (int i = 0; i < ENC_COMP_N; i++) {
            if (SEN[ax_]->comp[i] < mn) mn = SEN[ax_]->comp[i];
            if (SEN[ax_]->comp[i] > mx) mx = SEN[ax_]->comp[i];
        }
        SEN[ax_]->compAmpDeg = (mx - mn) * 0.5f * 180.0f / _PI;
        Serial.printf("[ENCC] M%d 载入补偿表 on=%d 峰值=%.1f°机械\n",
                      ax_, SEN[ax_]->compOn, SEN[ax_]->compAmpDeg);
    } else {
        SEN[ax_]->compOn = 0;
        Serial.printf("[ENCC] M%d 无补偿表（未标定）\n", ax_);
    }
}

#define ENC_ALIGN_MS 1200        // 先把磁场钉住让转子吸合的时间 [ms]
#define ENC_RAMP_MS  1500.0f     // 0 → encCalV 线性升速时间 [ms]

static void encCalStep(uint32_t now) {
    int i = encCalAxis;
    if (encCalState == 1) {                    // 阶段1：磁场钉住不动，让转子吸合到磁场（关键：静磁场对准）
        MOT[i]->target = 0.0f;                 // velocityOpenloop(0)：磁场角不推进，Uq 仍施加 → 转子对准
        if (now - encCalT0 > ENC_ALIGN_MS) { encCalState = 5; encCalT0 = now; }
        return;
    }
    if (encCalState == 5) {                    // 阶段1b：从 0 线性升速到 encCalV（避免起步丢步）
        float f = (float)(now - encCalT0) / ENC_RAMP_MS;
        if (f > 1.0f) f = 1.0f;
        MOT[i]->target = encCalV * f;
        if (f >= 1.0f) {
            for (int k = 0; k < ENC_COMP_N; k++) { encBinSum[k] = 0.0; encBinSS[k] = 0.0; encBinCnt[k] = 0; }
            encSumAll = 0.0; encTotN = 0; encTravel = 0.0;
            encMechPrev = SEN[i]->getMechanicalAngle();
            SEN[i]->compOn = 0;                // ★采样期间关补偿，测的是原始误差
            encCalState = 2; encCalT0 = now;
            Serial.printf("[ENCC] M%d 已吸合→升速到 %.2f rad/s，开始采样 %.0fs…\n", i, encCalV, encCalSec);
        }
        return;
    }
    if (encCalState == 2) {                    // 阶段2：按环频把 err 分箱累加
        float mech    = SEN[i]->getMechanicalAngle();          // 0..2π（原始，compOn=0）
        float dm      = mech - encMechPrev;                    // 累计真实行程（判转子有没有真转）
        if (dm >  _PI) dm -= _2PI;
        if (dm < -_PI) dm += _2PI;
        encTravel += fabs(dm);
        encMechPrev = mech;
        float applied = _normalizeAngle((float)MOT[i]->pole_pairs * MOT[i]->shaft_angle);
        float err     = _normalizeAngle(
            (float)MOT[i]->sensor_direction * MOT[i]->pole_pairs * mech - applied);
        int   b = (int)(mech * (ENC_COMP_N / _2PI));
        if (b < 0) b = 0; else if (b >= ENC_COMP_N) b = ENC_COMP_N - 1;
        encBinSum[b] += err; encBinSS[b] += (double)err * err; encBinCnt[b]++;
        encSumAll += err; encTotN++;
        if (now - encCalT0 > (uint32_t)(encCalSec * 1000.0f)) encCalState = 3;
        return;
    }
    if (encCalState == 3) {                    // 阶段3：算表 + 填洞 + 平滑 + 质量判定 + 按需存 NVS
        int covered = 0; uint16_t minCnt = 0xFFFF;
        for (int k = 0; k < ENC_COMP_N; k++)
            if (encBinCnt[k] > 0) { covered++; if (encBinCnt[k] < minCnt) minCnt = encBinCnt[k]; }
        float syncSdDeg = 0.0f;
        bool enough = (encTotN >= 200 && covered > 0);
        if (enough) {
            double gmean = encSumAll / (double)encTotN;
            // 1) 逐箱均值偏差 → 补偿（电角度 → 机械角）
            //   ★符号：err = normalize(dir·pp·θ_enc − applied) 是「已按 dir 映射」的偏差，
            //     而补偿是直接加在原始 θ_enc 上的，所以必须再乘回 dir：
            //        comp = −dir·(err−mean)/pp
            //     dir=−1 时若不乘回，符号翻转 → 误差翻倍（实测 C 的 r 从 0.92 掉到 0.10）。
            for (int k = 0; k < ENC_COMP_N; k++) {
                SEN[i]->comp[k] = (encBinCnt[k] > 0)
                    ? (float)(-(double)MOT[i]->sensor_direction
                              * (encBinSum[k] / (double)encBinCnt[k] - gmean)
                              / (double)MOT[i]->pole_pairs)
                    : 0.0f;
            }
            // 2) 空箱借最近非空箱的值（保证表连续、无跳变）
            int f = 0; while (encBinCnt[f] == 0) f++;
            float last = 0.0f;
            for (int j = 0; j < ENC_COMP_N; j++) {
                int idx = (f + j) % ENC_COMP_N;
                if (encBinCnt[idx] > 0) last = SEN[i]->comp[idx];
                SEN[i]->comp[idx] = last;
            }
            // 3) 5 点环形平滑（误差是光滑的；平滑压掉采样噪声）
            float tmp[ENC_COMP_N];
            for (int k = 0; k < ENC_COMP_N; k++) {
                float s = 0.0f;
                for (int d = -2; d <= 2; d++) s += SEN[i]->comp[(k + d + ENC_COMP_N) % ENC_COMP_N];
                tmp[k] = s / 5.0f;
            }
            for (int k = 0; k < ENC_COMP_N; k++) SEN[i]->comp[k] = tmp[k];

            float mn = 1e9f, mx = -1e9f;
            for (int k = 0; k < ENC_COMP_N; k++) {
                if (SEN[i]->comp[k] < mn) mn = SEN[i]->comp[k];
                if (SEN[i]->comp[k] > mx) mx = SEN[i]->comp[k];
            }
            SEN[i]->compAmpDeg = (mx - mn) * 0.5f * 180.0f / _PI;

            // 4) 箱内离散度 → 标定时转子是否真锁相（锁相则同箱内 err 几乎不变）
            double ss2 = 0.0; uint32_t nn = 0;
            for (int k = 0; k < ENC_COMP_N; k++) {
                if (encBinCnt[k] > 1) {
                    double m2 = encBinSum[k] / encBinCnt[k];
                    double v2 = encBinSS[k] / encBinCnt[k] - m2 * m2;
                    if (v2 > 0.0) { ss2 += v2 * encBinCnt[k]; nn += encBinCnt[k]; }
                }
            }
            syncSdDeg = (nn > 0) ? (float)(sqrt(ss2 / (double)nn) * 180.0 / _PI) : 0.0f;
        }

        Serial.printf("[ENCC] M%d 采样 N=%lu 机械行程=%.2f 圈 覆盖 %d/%d 箱 (最小 %u 次/箱)\n",
                      i, (unsigned long)encTotN, encTravel / _2PI, covered, ENC_COMP_N,
                      (unsigned)(minCnt == 0xFFFF ? 0 : minCnt));
        if (enough)
            Serial.printf("[ENCC] 补偿峰值=%.1f°机械 (≈%.1f°电角度)  标定同步残差=%.1f°电角度\n",
                          SEN[i]->compAmpDeg, SEN[i]->compAmpDeg * (float)MOT[i]->pole_pairs, syncSdDeg);

        bool good = enough
                 && covered >= (int)(ENC_COMP_N * 0.6f)
                 && syncSdDeg <= 15.0f
                 && SEN[i]->compAmpDeg >= 1.0f;
        if (good) {
            SEN[i]->compOn = 1;
            encCompSave(i);
            Serial.println("[ENCC] 质量合格 → 已启用并存 NVS");
            // ★v3.14.1：原提示"下一步跑 C 重标 zea"是错的。C 是开环旋转法，回归截距里
            //   混有恒定负载角 ⇒ 系统性偏差（实测同一硬件 C 两次给出相差 82°电角度的值）。
            //   一律改用 A（静态锁定法，负载角=0）或 Y（库内 initFOC 同法）。
            Serial.println("[ENCC] done ok=1  ★下一步：跑 A 重标 zea（静态锁定、无负载角偏置；勿用 C）");
        } else {
            SEN[i]->compOn = 0;
            if (!enough || encTravel / _2PI < 0.5)
                Serial.println("[ENCC] [!!] 机械行程<0.5圈或采样太少 → 标定时转子根本没转（电压不足/被卡住）");
            else if (covered < (int)(ENC_COMP_N * 0.6f))
                Serial.println("[ENCC] [!!] 覆盖率<60% → 转子没跟着磁场走（丢步）");
            else if (syncSdDeg > 15.0f)
                Serial.println("[ENCC] [!!] 同步残差大 → 标定时转子没锁稳（速度/电压不匹配）");
            else
                Serial.println("[ENCC] [!!] 补偿峰值<1° → 该轴几乎无偏心，无需补偿");
            Serial.println("[ENCC] 质量不合格 → 未保存、未启用（本次结果作废）");
            Serial.println("[ENCC] done ok=0");
        }
        MOT[i]->controller = MotionControlType::velocity;
        MOT[i]->target     = 0.0f;
        ax[i].targetCmd = ax[i].target = 0.0f;
        ax[i].state = AX_IDLE;
        useRunVoltOne(i);              // ★v3.15.0 退出开环 → 只恢复【本轴】的闭环电压上限
        setEnable(false);
        encCalState = 0;
        return;
    }
}

// ===========================================================================
// 5b) 速度反馈体检（F 命令）：每个环路周期统计一次
// ★v3.15.0：作用于 fbAxis（0=M0 / 1=M1）—— M1 的增益该不该沿用 M0 的 P=0.1，靠它实测。
// ===========================================================================
static void fbStep() {
    int i = fbAxis;
    float vRaw = SEN[i]->getVelocity();   // 本周期原始差分（开环/闭环都有效）
    if (!fbIsOpen) {                      // 闭环时 shaft_velocity 才是真实反馈（过 LPF）
        float vf = MOT[i]->shaft_velocity;
        fbFSum  += (double)vf;
        fbFSum2 += (double)vf * (double)vf;
        fbFN++;
    }
    if (fbN == 0) { fbMin = vRaw; fbMax = vRaw; }
    else { if (vRaw < fbMin) fbMin = vRaw; if (vRaw > fbMax) fbMax = vRaw; }
    fbSum  += (double)vRaw;
    fbSum2 += (double)vRaw * (double)vRaw;
    if (fabsf(vRaw) < 1e-6f) fbZero++;
    int8_t s = (vRaw > 0.0001f) ? 1 : ((vRaw < -0.0001f) ? -1 : 0);
    if (fbLastSign != 0 && s != 0 && s != fbLastSign) fbFlips++;
    if (s != 0) fbLastSign = s;

    float ang = SEN[i]->getMechanicalAngle();  // 用 loopFOC 已更新的缓存角，不再多打 SPI
    if (fbN > 0) {
        float d = ang - fbAngPrev;
        if (d >  _PI) d -= _2PI;
        if (d < -_PI) d += _2PI;
        int32_t c = (int32_t)lroundf(d * (16384.0f / _2PI));
        fbCntSum += c;
        int32_t ac = (c < 0) ? -c : c;
        if (ac > 7) ac = 7;
        fbHist[ac]++;
    }
    fbAngPrev = ang;
    fbN++;
}

static float fbPct(double s, double s2, uint32_t n) {
    if (n < 2) return 0.0f;
    double m = s / (double)n;
    double v = s2 / (double)n - m * m;
    if (v < 0.0) v = 0.0;
    return (fabsf((float)m) > 1e-6f) ? (float)sqrt(v) / fabsf((float)m) * 100.0f : 0.0f;
}

static void fbReport() {
    if (fbN == 0) { Serial.println("[FB] [!!] 没采到样（电机没在转？）"); return; }
    double rmean = fbSum / (double)fbN;
    double rvar  = fbSum2 / (double)fbN - rmean * rmean;
    if (rvar < 0.0) rvar = 0.0;
    float  rsd   = (float)sqrt(rvar);
    float  cpl   = (float)fbCntSum / (float)fbN;                       // 每周期平均计数
    // Δ角度/平均角速度 = 真实环路周期。★单位：下面是 ms（原版误标成 us，差 1000 倍）
    float  tsMs  = (fabsf((float)rmean) > 0.05f)
                   ? (1000.0f * (cpl * (_2PI / 16384.0f)) / (float)rmean) : 0.0f;
    float  oneC  = (tsMs > 0.001f) ? ((_2PI / 16384.0f) * 1000.0f / tsMs) : 0.0f;

    Serial.printf("[FB] M%d N=%lu  模式=%s", (int)fbAxis, (unsigned long)fbN, fbIsOpen ? "开环" : "闭环");
    Serial.println();
    Serial.printf("[FB] 原始差分 均值=%.4f  标准差=%.4f  (噪声占比 %.1f%%)  min=%.4f max=%.4f",
                  (float)rmean, rsd, fbPct(fbSum, fbSum2, fbN), fbMin, fbMax);
    Serial.println();
    if (fbFN > 0) {
        double fmean = fbFSum / (double)fbFN;
        Serial.printf("[FB] LPF后(环内实采) 均值=%.4f  (噪声占比 %.1f%%)",
                      (float)fmean, fbPct(fbFSum, fbFSum2, fbFN));
        Serial.println();
    } else {
        Serial.println("[FB] LPF后统计：开环模式下不可用（库会把 shaft_velocity 覆写成指令）");
    }
    Serial.printf("[FB] 零值占比=%.1f%%  符号翻转=%lu (%.1f%%)",
                  100.0f * (float)fbZero / (float)fbN, (unsigned long)fbFlips,
                  100.0f * (float)fbFlips / (float)fbN);
    Serial.println();
    Serial.printf("[FB] Δ计数/周期=%.2f  等效 Ts=%.0f us  1 计数=%.3f rad/s",
                  cpl, tsMs * 1000.0f, oneC);
    Serial.println();
    Serial.printf("[FB] |Δ|直方图 0..7+ : %u %u %u %u %u %u %u %u",
                  fbHist[0], fbHist[1], fbHist[2], fbHist[3],
                  fbHist[4], fbHist[5], fbHist[6], fbHist[7]);
    Serial.println();
    float noise = fbPct(fbSum, fbSum2, fbN);
    if (noise > 15.0f)
        Serial.println("[FB] [!!] 反馈噪声 >15% → 低速段速度环可用增益区间被压死，这就是抖动主因");
    else
        Serial.println("[FB] [OK] 反馈噪声 <15% → 问题更可能在电角度或环路相位滞后");
}

// ===========================================================================
// 6) 状态帧
// ===========================================================================
static void sendStatusFrame(bool alsoLocal = false) {
    char buf[144];
    // 角度用 motor.shaft_angle（累计多圈，适合做里程计）而非 sensor 的 0~2π 回绕值
    // 速度取 ax[i].velocity：未对齐的轴被强制为 0，避免把悬空编码器的噪声喂给上位机
    snprintf(buf, sizeof(buf),
             "A%.3f,V%.3f,A1%.3f,V1%.3f,S%c%c,I%.3f",
             MOT[0]->shaft_angle, ax[0].velocity,
             MOT[1]->shaft_angle, ax[1].velocity,
             stateChar(ax[0].state), stateChar(ax[1].state),
             iq_filtered);
    Serial2.println(buf);
    if (alsoLocal) { Serial.print("[S1] frame "); Serial.println(buf); }
}

// ===========================================================================
// 7) 命令解析
// ===========================================================================
static void printHelp() {
    Serial.println("[S1] cmds: R<v0>,<v1> | R<v> | R | T<u0>,<u1> | L<v0>,<v1> | S | J | K<p>,<i> | W<slew> | Q | ?");
    Serial.println("[S1]   ★轴参数 <ax>: 0=M0 / 1=M1；凡带 [<ax>] 的命令，缺省一律 0（兼容旧用法）");
    Serial.println("[S1]   L = 开环速度测试(绕过编码器, 12s 自动退出), 例: L2,0 / L0,2");
    Serial.println("[S1]   H[0/1] = 板间串口隔离 (H1 只收 USB 命令; 单板台架标定前必开)");
    Serial.println("[S1]   P[v][,<ax>] = 直接测量该轴真实极对数并写入 (默认 2 rad/s; 只写该轴) ★先跑这个");
    Serial.println("[S1]   C[v][,<ax>] = 开环对准法标 zea (默认2rad/s) [!]截距含负载角=>有系统偏差,勿用于定标");
    Serial.println("[S1]   A[v][,<ax>] = 手工重新对齐 (默认3V,保持1.5s后重算zea) ★推荐:静态锁定,负载角=0无偏");
    Serial.println("[S1]   Z      = 打印两轴 zero_electric_angle / sensor_direction");
    Serial.println("[S1]   Z<a>,<b> = 手工设两轴 zero_electric_angle [rad] (闭环不转时的电角度微调; ★存 NVS)");
    Serial.println("[S1]   D<a>,<b> = 手工设两轴 sensor_direction (+1/-1) (★存 NVS)");
    Serial.println("[S1]   N<n>[,<ax>] = 强制设极对数 (默认 7; 缺省两轴同设; 改后该轴 zea 失效需重跑 A)");
    Serial.println("[S1]   Y[v],<ax> = 重对齐+解除 NOSENSOR 锁 (默认6V/轴0; 失败自动静态兜底; 成功存 NVS)");
    Serial.println("[S1]   V<v>   = 设闭环电压上限 [0.5~10] (默认 10; 按轴生效, 不动正在标定的轴)");
    Serial.println("[S1]   O<v>   = 开环电压上限 [0.5~10] (默认 2; 与闭环上限 V 分离, 进 L/C/P 自动采用)");
    Serial.println("[S1]   E      = 打印两轴原始编码器读数 (开环下 A 字段是指令积分器, 用 E 才准)");
    Serial.println("[S1]   F[sec][,<ax>] = 速度反馈体检 (先 O2 + L2,0 匀速转起来再发; 打印噪声占比+Δ计数直方图)");
    Serial.println("[S1]   B[n][,<ax>] = 飞行记录仪: 按环频记录 n 拍 (默认 1000, 上限 2000)");
    Serial.println("[S1]   W      = 倒出记录 (CSV: D,t_us,cnt14,vraw,vfilt,iq,uq)");
    Serial.println("[S1]   W<r>   = 设速度目标斜率 [rad/s²] 1~300, 默认5 (=0.0752 m/s² @Rw) ★v3.16.0");
    Serial.println("[S1]   X      = 标定 M0 编码器偏心补偿 (开环慢转 8s 自动建表+存NVS) ★闭环激荡必先跑");
    Serial.println("[S1]   X0/X1  = 标定对应轴;  X? = 打印补偿状态;  Xoff = 清除两轴补偿");
    Serial.println("[S1]            ★X 完成后必须重跑 A 重标 zea（勿用 C：其截距含负载角偏置），再跑闭环");
    Serial.println("[S1]   S      = 停机（两轴置 IDLE + 断使能）");
    Serial.println("[S1]   J      = coast 自由衰减（两轴置 IDLE + 断 PWM；settle 用，治 M0/M1 制动不对称）★v3.16.1");
    Serial.println("[S1]   [M1 无相电流采样 ⇒ 无 iq 读数/无 OC_FAULT, 靠 DRV8313 自带 2.5A 限流 + 堵转检测]");
}

static void parseCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;
    char c = (char)toupper(cmd.charAt(0));
    String arg = cmd.substring(1);
    arg.trim();

    float a, b;

    if (c == 'R') {
        int n = sscanf(arg.c_str(), "%f,%f", &a, &b);
        if (n == 2)      { setVel(0, a); setVel(1, b); }
        else if (n == 1) { setVel(0, a); setVel(1, a); }
        else             { setVel(0, 3.0f); setVel(1, 3.0f); }
        Serial.printf("[RX] R v0=%.2f v1=%.2f\n", ax[0].targetCmd, ax[1].targetCmd);
    }
    else if (c == 'T') {
        int n = sscanf(arg.c_str(), "%f,%f", &a, &b);
        if (n == 2)      { setTorque(0, a); setTorque(1, b); }
        else if (n == 1) { setTorque(0, a); setTorque(1, a); }
        Serial.printf("[RX] T u0=%.2f u1=%.2f\n", ax[0].targetCmd, ax[1].targetCmd);
    }
    else if (c == 'L') {
        // 开环速度测试：直接开环换相，绕过编码器与 FOC 电角度
        int n = sscanf(arg.c_str(), "%f,%f", &a, &b);
        float tv0 = (n >= 1) ? a : 2.0f;
        float tv1 = (n >= 2) ? b : 0.0f;
        for (int i = 0; i < 2; i++) {
            MOT[i]->controller = MotionControlType::velocity_openloop;
            // ★v3.11.0：开机 initFOC() 失败时库会 disable() → loopFOC()/move() 直接 return，
            //   于是"开环也推不动"（电流≈0），会把"固件问题"误判成"硬件没电"。这里强制 enable()。
            MOT[i]->enable();
            ax[i].mode      = 1;
            ax[i].runStart  = millis();
        }
        ax[0].targetCmd = ax[0].target = tv0;
        ax[1].targetCmd = ax[1].target = tv1;
        // ★v3.15.0：两轴一律允许开环。开环只用时间算换相角、不碰编码器/zea，
        //   所以「还没标定 / 对齐失败(NOSENSOR)」不该挡住它 —— 这正是首次接上 M1 时
        //   验证相序的唯一手段（原来 M1 若是 NOSENSOR，L0,2 会被拒绝，把首验路堵死）。
        //   NOSENSOR 标记不会因此丢失：stopAll() 现在按 dir 是否未知重新推导它。
        ax[0].state = AX_RUN;
        ax[1].state = AX_RUN;
        useOpenloopVolt();                     // 开环用 O 值（默认 2V），避免 10V 直接过流
        testMode  = 1;
        testUntil = millis() + TEST_MODE_MS;
        setEnable(true);
        Serial.print("[TEST] openloop v0="); Serial.print(tv0, 2);
        Serial.print(" v1="); Serial.print(tv1, 2);
        Serial.print("  "); Serial.print(TEST_MODE_MS); Serial.println(" ms 后自动退出");
    }
    else if (c == 'S') {
        stopAll();
    }
    else if (c == 'J') {
        // ★v3.16.1 coast：两轴自由衰减（settle 专用，治 M0/M1 制动不对称）
        //   根因：settle 段 vCmd≈0 → M0(电流环)猛刹、M1(电压模式)惯性滑 → 右轮快 → 左偏 +7.6°/段。
        //   J 把两轴都置 IDLE → 主循环 setEnable(false) → DRV8313 断电 → 两轴纯机械摩擦自由衰减（对称）。
        //   下一个 R 命令会 freshStart + enable 自动恢复闭环。
        for (int i = 0; i < 2; i++) {
            ax[i].targetCmd = 0.0f;
            ax[i].target    = 0.0f;
            ax[i].state     = ((int)MOT[i]->sensor_direction == 0) ? AX_NOSENSOR : AX_IDLE;
            MOT[i]->disable();
        }
        Serial.println("[RX] J: coast (free-wheel both motors)");
    }
    else if (c == 'K') {
        // ★v3.10.0：扩展为 K<p>,<i>[,<d>[,<lim>]]
        //   注：库在 init() 里把 PID_velocity.limit 重置为 voltage_limit，且闭环输出【不受】
        //   voltage_limit 钳位（只有设了 phase_resistance 才钳）→ 想限幅必须直接设 limit。
        float p, iv, dv, lv;
        int n = sscanf(arg.c_str(), "%f,%f,%f,%f", &p, &iv, &dv, &lv);
        if (n >= 2) {
            for (int i = 0; i < 2; i++) {
                MOT[i]->PID_velocity.P = p;
                MOT[i]->PID_velocity.I = iv;
                if (n >= 3) MOT[i]->PID_velocity.D = dv;
                if (n >= 4) MOT[i]->PID_velocity.limit = lv;
            }
            Serial.printf("[RX] K P=%.3f I=%.3f D=%.3f lim=%.2f\n",
                          p, iv, MOT[0]->PID_velocity.D, MOT[0]->PID_velocity.limit);
        }
    }
    else if (c == 'G') {
        // ★v3.10.0：速度反馈低通时间常数（默认 5ms 滞后太大，易自激）
        float tf = (sscanf(arg.c_str(), "%f", &a) == 1) ? a : 0.003f;
        tf = constrain(tf, 0.0002f, 0.05f);
        for (int i = 0; i < 2; i++) MOT[i]->LPF_velocity.Tf = tf;
        Serial.printf("[RX] G LPF_velocity.Tf=%.5f s\n", tf);
    }
    else if (c == 'C') {
        // 开环对准法标定 zero_electric_angle（见上方 calStep 的长注释）
        // ★v3.15.0：C[v][,<ax>]，轴缺省 0（兼容旧用法）
        if (testMode)  { Serial.println("[RX] C ignored (test mode)"); return; }
        if (calState)  { Serial.println("[RX] C busy"); return; }
        float v = 2.0f; int k = 0;
        int nn = sscanf(arg.c_str(), "%f,%d", &v, &k);
        if (nn < 1) v = 2.0f;
        if (nn < 2) k = 0;
        if (k < 0) k = 0; else if (k > 1) k = 1;
        calV    = constrain(v, 0.5f, 8.0f);
        calAxis = (uint8_t)k;
        ax[k].targetCmd = ax[k].target = 0.0f;
        ax[k].state = AX_IDLE;
        MOT[k]->controller = MotionControlType::velocity_openloop;
        MOT[k]->target = 0.0f;
        useOpenloopVoltOne(k);         // ★v3.15.0 只压本轴的电压上限，不动另一轴
        setEnable(true);
        calMode = 0; calState = 1; calT0 = millis();
        Serial.printf("[CAL] 开环对准标定 M%d: v=%.2f rad/s（约 3s）\n", k, calV);
    }
    else if (c == 'P') {
        // 直接测量真实极对数：开环施加已知电角速度，用编码器实测机械角速度反算
        // ★v3.15.0：P[v][,<ax>]，轴缺省 0（兼容旧用法）
        if (testMode)  { Serial.println("[RX] P ignored (test mode)"); return; }
        if (calState)  { Serial.println("[RX] P busy"); return; }
        float v = 2.0f; int k = 0;
        int nn = sscanf(arg.c_str(), "%f,%d", &v, &k);
        if (nn < 1) v = 2.0f;
        if (nn < 2) k = 0;
        if (k < 0) k = 0; else if (k > 1) k = 1;
        calV    = constrain(v, 0.5f, 8.0f);
        calAxis = (uint8_t)k;
        ax[k].targetCmd = ax[k].target = 0.0f;
        ax[k].state = AX_IDLE;
        MOT[k]->controller = MotionControlType::velocity_openloop;
        MOT[k]->target = 0.0f;
        useOpenloopVoltOne(k);         // ★v3.15.0 只压本轴
        setEnable(true);
        calMode = 1; calState = 1; calT0 = millis();
        Serial.printf("[PP] 测量 M%d 极对数: 开环 v=%.2f（电角速度 %.1f rad/s），约 2.5s\n",
                      k, calV, calV * (float)MOT[k]->pole_pairs);
    }
    else if (c == 'N') {
        // 直接设定极对数：用于二分确认真实极对数（P 实测≈7.95，7/8 未定案）。
        // 改 pp 会让该轴 zea 失效 → 之后必须重跑 A 重标。
        // ★v3.15.0：N<n> 两轴同设（兼容旧用法）；N<n>,<ax> 只设该轴。
        int n = 7, k = -1;
        int nn = sscanf(arg.c_str(), "%f,%d", &a, &k);
        if (nn >= 1) n = (int)a;
        if (nn < 2)  k = -1;
        if (k > 1)   k = 1;          // k==-1 表示"两轴"
        if (n < 1)  n = 1;
        if (n > 60) n = 60;
        if (k < 0) { MOT[0]->pole_pairs = n; MOT[1]->pole_pairs = n; }
        else       { MOT[k]->pole_pairs = n; }
        Serial.printf("[RX] N pole_pairs=%d (%s)（该轴 zea 已失效，请重跑 A 重标）\n",
                      n, (k < 0) ? "两轴" : (k == 0 ? "仅M0" : "仅M1"));
    }
    else if (c == 'H') {
        // 板间串口隔离：H1 后只认 USB 命令。单板标定前必须先开，否则 S3 的 20Hz R 流会捣乱。
        int n = sscanf(arg.c_str(), "%f", &a);
        if (n == 1) hostLock = (a >= 0.5f) ? 1 : 0;
        // ★v3.14.0 fail-safe：H1 隔离上位机的【同时】清零残留目标。
        //   实测 H1 只"不听新命令"，电机仍按上位机最后一条 R 继续转（3s 净转 −3.2 rad）。
        if (hostLock) {
            for (int i = 0; i < 2; i++) { ax[i].targetCmd = 0.0f; ax[i].target = 0.0f; }
        }
        Serial.printf("[RX] H hostLock=%d (1=只收 USB 命令, 忽略板间 Serial2; 已清零残留目标)\n", hostLock);
    }
    else if (c == 'Z') {
        // 闭环电角度微调：zea 是「开环能转、闭环有电流却不转」时唯一需要怀疑的量
        int n = sscanf(arg.c_str(), "%f,%f", &a, &b);
        if (n >= 1) {
            MOT[0]->zero_electric_angle = _normalizeAngle(a);
            if (n >= 2) MOT[1]->zero_electric_angle = _normalizeAngle(b);
            // ★v3.13.0：手工设定的 zea 直接存 NVS。否则"扫到的可用 zea"一重启就丢，
            //   开机会退回旧值（本台架实测：扫到 2.16 可用，但 NVS 里是 5.2182 就会又不转）。
            // ★v3.15.0：M1 同样存盘（原来只存 m0*）
            encPrefs.putFloat(nvsZeaKey(0), MOT[0]->zero_electric_angle);
            if ((int)MOT[0]->sensor_direction != 0)
                encPrefs.putInt(nvsDirKey(0), (int)MOT[0]->sensor_direction);
            if (n >= 2) {
                encPrefs.putFloat(nvsZeaKey(1), MOT[1]->zero_electric_angle);
                if ((int)MOT[1]->sensor_direction != 0)
                    encPrefs.putInt(nvsDirKey(1), (int)MOT[1]->sensor_direction);
            }
            Serial.printf("[RX] Z zea0=%.4f zea1=%.4f (已存 NVS)\n",
                          MOT[0]->zero_electric_angle, MOT[1]->zero_electric_angle);
        } else {
            Serial.printf("[S1] Z M0 zea=%.4f dir=%d | M1 zea=%.4f dir=%d\n",
                          MOT[0]->zero_electric_angle, (int)MOT[0]->sensor_direction,
                          MOT[1]->zero_electric_angle, (int)MOT[1]->sensor_direction);
        }
    }
    else if (c == 'D') {
        // 手工翻转编码器方向：用于排除对齐时方向判反（症状同样是闭环不转）
        // ★v3.15.0：设完即存 NVS（原来不存 → 下次开机又回到未知/旧值）
        int n = sscanf(arg.c_str(), "%f,%f", &a, &b);
        if (n >= 1) {
            MOT[0]->sensor_direction = (Direction)((a >= 0.0f) ? 1 : -1);
            if (n >= 2) MOT[1]->sensor_direction = (Direction)((b >= 0.0f) ? 1 : -1);
            for (int k = 0; k < n && k < 2; k++)
                encPrefs.putInt(nvsDirKey(k), (int)MOT[k]->sensor_direction);
            Serial.printf("[RX] D dir0=%d dir1=%d (已存 NVS)\n",
                          (int)MOT[0]->sensor_direction, (int)MOT[1]->sensor_direction);
        }
    }
    else if (c == 'A') {
        // 手工重新对齐：把电压矢量钉在 270° 并保持 1.5s（库里只保持 700ms），让转子
        // 充分就位后再读传感器角反算 zea。用于排除「自动对齐时转子没到位 → zea 系统性偏差」。
        // 与库内 alignSensor 完全同法，只是把稳定时间拉长。
        // ★v3.15.0：A[v][,<ax>]，缺省 3V/轴0（兼容旧用法）
        if (testMode) { Serial.println("[RX] A ignored (test mode)"); return; }
        float v = 3.0f; int k = 0;
        int nn = sscanf(arg.c_str(), "%f,%d", &v, &k);
        if (nn < 1) v = 3.0f;
        if (nn < 2) k = 0;
        v = constrain(v, 0.5f, 10.0f);
        if (k < 0) k = 0; else if (k > 1) k = 1;
        ax[k].targetCmd = ax[k].target = 0.0f;
        ax[k].state = AX_IDLE;
        MOT[k]->target = 0.0f;
        // ★v3.12.0：A = "确定性对齐"。若 dir 未知（开机运动式判定失败的残留）必须先补一个已知
        //   方向，否则下面的 dir*pp*mech 恒为 0 → zea 被算成 0（假对齐，比对齐失败更坏）。
        if ((int)MOT[k]->sensor_direction == 0) {
            int d = encPrefs.getInt(nvsDirKey(k), AX_DIR_DEFAULT(k));
            if (d == 0) d = AX_DIR_DEFAULT(k);
            MOT[k]->sensor_direction = (Direction)d;
            Serial.printf("[RX] A 注意: M%d dir 原为未知 -> 采用 %d（若方向反了用 D%d,%d 再 A）\n",
                          k, (int)MOT[k]->sensor_direction, -(int)MOT[k]->sensor_direction, k);
        }
        if (ax[k].mode != 1) { ax[k].mode = 1; MOT[k]->controller = MotionControlType::velocity; }
        setEnable(true);
        MOT[k]->enable();                 // ★v3.12.0：A 之后可直接 R 闭环（enabled 必须为 1）
        uint32_t t0 = millis();
        while (millis() - t0 < 1500) { MOT[k]->setPhaseVoltage(v, 0, _3PI_2); delay(2); }
        MOT[k]->setPhaseVoltage(0, 0, 0);
        SEN[k]->update();
        float zea = _normalizeAngle(
            _electricalAngle((float)MOT[k]->sensor_direction * SEN[k]->getMechanicalAngle(),
                             MOT[k]->pole_pairs));
        MOT[k]->zero_electric_angle = zea;
        encPrefs.putInt(nvsDirKey(k), (int)MOT[k]->sensor_direction);  // ★v3.12.0 存盘供下次开机复用
        encPrefs.putFloat(nvsZeaKey(k), zea);
        setEnable(false);
        Serial.printf("[RX] A M%d volt=%.2f -> zea=%.4f (dir=%d) 已存 NVS\n",
                      k, v, zea, (int)MOT[k]->sensor_direction);
    }
    else if (c == 'Y') {
        // ★v3.11.0 运行时"复活+重对齐"。开机 initFOC 失败 → 该轴被永久锁死（NOSENSOR 标记 +
        //   库内 disable()），此后 R/T/L 全部无效，且没有任何命令能解锁 —— 这是个真陷阱。
        //   Y 清掉 dir/zea、enable()、用指定电压重跑 initFOC；成功则解除 NOSENSOR。
        //   用法：Y 或 Y<v> 或 Y<v>,<轴>   例：Y / Y6 / Y6,1
        // ★v3.15.0：① 失败后再走一次"已知 dir + 纯静态 zea"兜底（与 setup 同款，可复现）
        //           ② 成功即写 NVS（原来只解锁不存盘 → 下次开机又退回未对齐）
        if (testMode)  { Serial.println("[RX] Y ignored (test mode)"); return; }
        if (calState || encCalState) { Serial.println("[RX] Y busy"); return; }
        float v = 6.0f; int k = 0;
        int nn = sscanf(arg.c_str(), "%f,%d", &v, &k);
        if (nn < 1) v = 6.0f;
        if (nn < 2) k = 0;
        v = constrain(v, 1.0f, 10.0f);
        if (k < 0) k = 0; else if (k > 1) k = 1;
        MOT[k]->sensor_direction     = Direction::UNKNOWN;
        MOT[k]->zero_electric_angle  = NOT_SET;
        MOT[k]->voltage_sensor_align = v;
        ax[k].targetCmd = ax[k].target = 0.0f;
        if (ax[k].state != AX_NOSENSOR) ax[k].state = AX_IDLE;
        setEnable(true);
        MOT[k]->enable();
        int ok = MOT[k]->initFOC();
        if (ok && (int)MOT[k]->sensor_direction != 0) {
            ax[k].state = AX_IDLE;                        // 解除 NOSENSOR 锁
            Serial.printf("[RX] Y M%d OK zea=%.4f dir=%d (NOSENSOR 已解除)\n",
                          k, MOT[k]->zero_electric_angle, (int)MOT[k]->sensor_direction);
        } else {
            // ★v3.15.0：与 setup() 同款静态兜底 —— 运动式方向判定（moved<0.062rad）正是
            //   "开机对齐时好时坏"的唯一原因；换成已知 dir + 纯静止矢量反算 zea 即可复现。
            Serial.printf("[RX] Y M%d 运动式对齐失败 -> 静态兜底(已知 dir=%d, %.1fV)\n",
                          k, AX_DIR_DEFAULT(k), v);
            MOT[k]->sensor_direction     = (Direction)AX_DIR_DEFAULT(k);
            MOT[k]->zero_electric_angle  = NOT_SET;
            MOT[k]->voltage_sensor_align = v;
            MOT[k]->enable();
            int ok2 = MOT[k]->initFOC();
            if (ok2 && (int)MOT[k]->sensor_direction != 0 &&
                MOT[k]->zero_electric_angle != (float)NOT_SET) {
                ax[k].state = AX_IDLE;
                Serial.printf("[RX] Y M%d 静态兜底 OK zea=%.4f dir=%d\n",
                              k, MOT[k]->zero_electric_angle, (int)MOT[k]->sensor_direction);
                Serial.printf("[RX]    若方向反了: D%d,%d 再 A%.0f,%d 重标\n",
                              -(int)MOT[k]->sensor_direction, k, v, k);
            } else {
                ax[k].state = AX_NOSENSOR;
                MOT[k]->disable();
                Serial.printf("[RX] Y M%d 仍失败 zea=%.4f dir=%d -> 查 12V/相线/编码器\n",
                              k, MOT[k]->zero_electric_angle, (int)MOT[k]->sensor_direction);
            }
        }
        // ★v3.15.0：成功即写 NVS，供下次开机 initAxisFOC 直接复用
        if ((int)MOT[k]->sensor_direction != 0 && MOT[k]->zero_electric_angle != (float)NOT_SET) {
            encPrefs.putInt(nvsDirKey(k), (int)MOT[k]->sensor_direction);
            encPrefs.putFloat(nvsZeaKey(k), MOT[k]->zero_electric_angle);
            Serial.printf("[RX] Y M%d 标定已存 NVS (dir=%d zea=%.4f)\n",
                          k, (int)MOT[k]->sensor_direction, MOT[k]->zero_electric_angle);
        }
    }
    else if (c == 'V') {
        // 闭环电压上限。★注意 voltage_limit 同时被开环当 Uq 用、也是闭环输出钳位，
        // 所以「为开环防过流而 V2」会顺带把闭环也压到 2V → 闭环测试会莫名没力气。
        // 现已分离：V 设闭环上限，O 设开环上限（进 L/C/P 自动用 O，退出自动回 V）。
        float v = (sscanf(arg.c_str(), "%f", &a) == 1) ? a : 3.0f;
        v = constrain(v, 0.5f, 10.0f);
        runVLimit = v;
        // ★v3.15.0：按轴生效 —— 正在开环标定的那一轴保留它的开环上限，其余轴才吃新值
        if (!testMode) {
            for (int k = 0; k < 2; k++) {
                bool busy = (calState && calAxis == k) || (encCalState && encCalAxis == k);
                if (!busy) MOT[k]->voltage_limit = v;
            }
        }
        Serial.printf("[RX] V run_voltage_limit=%.2f V", v); Serial.println("（开环上限请用 O）");
    }
    else if (c == 'E') {
        // 直接打印【原始传感器读数】。★开环时 FOC 会把 shaft_angle 覆写成指令积分器
        // （velocityOpenloop 里 shaft_angle=_normalizeAngle(shaft_angle+target*Ts)），
        // 所以开环下用 A 字段看编码器是错的；E 读的是 sensor 自身的角，不受覆写影响。
        // ★v3.15.0：两轴对称打印（原来 M1 只打一个 a 字段，跟 M0 无法对比）
        Serial.printf("[S1] enc0 a=%.4f mech=%.4f v=%.4f | enc1 a=%.4f mech=%.4f v=%.4f\n",
                      SEN[0]->getAngle(), SEN[0]->getMechanicalAngle(), SEN[0]->getVelocity(),
                      SEN[1]->getAngle(), SEN[1]->getMechanicalAngle(), SEN[1]->getVelocity());
        Serial.printf("[S1] shaft0=%.4f shaft1=%.4f\n", MOT[0]->shaft_angle, MOT[1]->shaft_angle);
    }
    else if (c == 'F') {
        // 速度反馈体检：先 O2 + L2,0 把电机开环匀速转起来，再发 F，读统计（见 fbReport）
        // ★v3.15.0：F[sec][,<ax>]，轴缺省 0。M1 的增益该不该沿用 M0 的 P=0.1，靠它实测。
        float sec = 5.0f; int k = 0;
        int nn = sscanf(arg.c_str(), "%f,%d", &sec, &k);
        if (nn < 1) sec = 5.0f;
        if (nn < 2) k = 0;
        if (k < 0) k = 0; else if (k > 1) k = 1;
        sec = constrain(sec, 1.0f, 20.0f);
        if (fbOn) { Serial.println("[RX] F busy"); return; }
        fbAxis   = (uint8_t)k;
        fbIsOpen = (MOT[k]->controller == MotionControlType::velocity_openloop) ? 1 : 0;
        fbOn = 1; fbN = 0; fbFN = 0; fbFlips = 0; fbZero = 0; fbCntSum = 0;
        fbSum = fbSum2 = fbFSum = fbFSum2 = 0.0;
        fbMin = fbMax = 0.0f; fbLastSign = 0;
        for (int q = 0; q < 8; q++) fbHist[q] = 0;
        fbAngPrev = SEN[k]->getMechanicalAngle();
        fbUntil = millis() + (uint32_t)(sec * 1000.0f);
        Serial.printf("[FB] 开始体检 M%d %.1fs（保持当前驱动不变）", k, sec); Serial.println();
    }
    else if (c == 'O') {
        // 开环电压上限（与闭环上限 V 分离，见 runVLimit/olVLimit 注释）
        float v = (sscanf(arg.c_str(), "%f", &a) == 1) ? a : 2.0f;
        olVLimit = constrain(v, 0.5f, 10.0f);
        // ★v3.15.0：按轴生效。原来 setVoltLim() 两轴一起改 —— 标 M1 时会把正在跑的 M0 卸力。
        if (testMode)      setVoltLim(olVLimit);      // 开环测试模式本身就是两轴一起开环
        else {
            if (calState)    MOT[calAxis]->voltage_limit    = olVLimit;
            if (encCalState) MOT[encCalAxis]->voltage_limit = olVLimit;
        }
        Serial.printf("[RX] O openloop_voltage_limit=%.2f V", olVLimit); Serial.println();
    }
    else if (c == 'B') {
        // 飞行记录仪：按环频记录 recWant 拍（默认 1000，上限 2000）；停机后用 W 倒出
        // ★v3.15.0：B[n][,<ax>]，轴缺省 0
        int n = 1000, k = 0;
        int nn = sscanf(arg.c_str(), "%f,%d", &a, &k);
        if (nn >= 1) n = (int)a;
        if (nn < 2)  k = 0;
        if (k < 0) k = 0; else if (k > 1) k = 1;
        if (n < 10) n = 10;
        if (n > REC_MAX) n = REC_MAX;
        recAxis = (uint8_t)k;
        recN = 0; recWant = (uint16_t)n; recOn = 1;
        Serial.printf("[REC] 开始记录 M%d %d 拍（%.0f ms）", k, n, n * 1.14f);
        Serial.println();
    }
    else if (c == 'W') {
        // ★v3.16.0：W 有两个用途，靠"有无数值参数"区分，保证向后兼容。
        //   W<斜率>  ⇒ 运行期设速度目标斜率 [rad/s²]（两轴共用，只存 RAM）
        //              1~300；上限远超人能跟上的量级，实际由过流/堵转保护兜底
        //   W（无参）⇒ 原「飞行记录仪倒出」CSV（保持 v3.15.0 行为不变）
        if (arg.length() > 0 && sscanf(arg.c_str(), "%f", &a) == 1) {
            slewRate = constrain(a, 1.0f, 300.0f);
            Serial.printf("[RX] W slew=%.1f rad/s² (=%.4f m/s² @Rw)\n", slewRate, slewRate * 0.015045f);
            return;
        }
        // 倒出记录：CSV 行 D,<us>,<cnt14>,<vraw_mrad>,<vfilt_mrad>,<iq_mA>,<uq_mV>
        recOn = 0;
        Serial.printf("[REC] M%d N=%u", (int)recAxis, recN); Serial.println();
        Serial.println("[REC] D,t_us,cnt14,vraw_mradps,vfilt_mradps,iq_mA,uq_mV");
        for (uint16_t k = 0; k < recN; k++) {
            Serial.printf("D,%lu,%u,%d,%d,%d,%d",
                          (unsigned long)recBuf[k].t, recBuf[k].cnt, recBuf[k].vraw,
                          recBuf[k].vfilt, recBuf[k].iq, recBuf[k].uq);
            Serial.println();
        }
        Serial.println("[REC] END");
    }
    else if (c == 'X') {
        // 编码器偏心补偿（v3.9.0）：X=标定M0  X0/X1=选轴  X?=状态  Xoff=清除
        if (arg == "?") {
            for (int k = 0; k < 2; k++)
                Serial.printf("[ENCC] M%d compOn=%d 峰值=%.1f°机械\n", k, SEN[k]->compOn, SEN[k]->compAmpDeg);
            return;
        }
        if (arg.equalsIgnoreCase("off") || arg.equalsIgnoreCase("clear")) {
            for (int k = 0; k < 2; k++) {
                SEN[k]->compOn = 0;
                for (int q = 0; q < ENC_COMP_N; q++) SEN[k]->comp[q] = 0.0f;
                SEN[k]->compAmpDeg = 0.0f;
                encCompSave(k);
            }
            Serial.println("[ENCC] 已清除两轴补偿表并关闭");
            return;
        }
        if (testMode)  { Serial.println("[RX] X ignored (test mode)"); return; }
        if (calState || encCalState) { Serial.println("[RX] X busy"); return; }
        int axis = 0;
        if (arg.length() > 0 && sscanf(arg.c_str(), "%f", &a) == 1) axis = (a >= 0.5f) ? 1 : 0;
        encCalAxis = (uint8_t)axis;
        encCalV    = 2.0f;
        encCalSec  = 8.0f;
        ax[axis].targetCmd = ax[axis].target = 0.0f;
        ax[axis].state = AX_IDLE;
        MOT[axis]->controller = MotionControlType::velocity_openloop;
        MOT[axis]->target = 0.0f;
        useOpenloopVoltOne(axis);      // ★v3.15.0 只压本轴（原来两轴一起压到 2V）
        setEnable(true);
        encCalState = 1; encCalT0 = millis();
        Serial.printf("[ENCC] 开始标定 M%d 偏心补偿: 开环 v=%.2f rad/s, 采样 %.0fs（约 1.5s 预热）\n",
                      axis, encCalV, encCalSec);
        Serial.println("[ENCC] 标定期间请勿下发其它命令；完成后自动存 NVS 并启用");
    }
    else if (c == 'Q') {
        sendStatusFrame(true);    // 同时在本机 USB 口回显，便于单板调试
    }
    else if (c == '?') {
        printHelp();
    }
    else {
        Serial.printf("[S1] unknown cmd: %s\n", cmd.c_str());
        printHelp();
    }
}

// ===========================================================================
// 7b) ★v3.15.0 每轴 FOC 上电初始化（M0/M1 共用同一套"四级保险"）
// ===========================================================================
// 优先级：① NVS 里上次成功的 dir/zea（直接复用，跳过运动式对齐 → 开机 100% 确定）
//         ② 无记录则正常 alignSensor
//         ③ 仍失败则"已知 dir + 纯静态 zea"兜底（与转子是否转动无关，可复现）
//         ④ 成功后持久化，供下次开机直接复用
// 注意：库里 sensor_direction 一旦已知，alignSensor 就【不做】moved 判定，只做静止 270°
//   矢量 → 读角反算 zea，与转子是否转动无关，故可复现。这也是"开机对齐时好时坏"的解药。
// 前置条件：SEN[i]/DRV[i]/MOT[i] 已 init+link，且 GPIO25 已置高。
static void initAxisFOC(int i) {
    // ① 复用 NVS
    int   sdir = encPrefs.getInt(nvsDirKey(i), 0);
    float szea = encPrefs.getFloat(nvsZeaKey(i), (float)NOT_SET);
    if (sdir != 0 && szea != (float)NOT_SET) {
        MOT[i]->sensor_direction    = (Direction)sdir;
        MOT[i]->zero_electric_angle = szea;
        Serial.printf("[S1] M%d 复用 NVS 标定 dir=%d zea=%.4f（跳过运动式对齐）\n", i, sdir, szea);
    } else {
        Serial.printf("[S1] M%d 无 NVS 标定 -> 正常对齐\n", i);
    }
    MOT[i]->initFOC();
    // ② 失败 → 静态兜底（已知 dir + 纯静态 zea）
    if ((int)MOT[i]->sensor_direction == 0) {
        int d = encPrefs.getInt(nvsDirKey(i), AX_DIR_DEFAULT(i));
        if (d == 0) d = AX_DIR_DEFAULT(i);
        Serial.printf("[S1] M%d 对齐失败 -> 静态兜底(已知 dir=%d, 6V)\n", i, d);
        MOT[i]->sensor_direction     = (Direction)d;
        MOT[i]->zero_electric_angle  = NOT_SET;
        MOT[i]->voltage_sensor_align = 6.0f;
        MOT[i]->enable();
        MOT[i]->initFOC();
        if ((int)MOT[i]->sensor_direction == 0)
            Serial.printf("[S1] M%d 静态兜底也失败 -> 试 D1,%d 或 D-1,%d 后再 A3,%d / Y6,%d\n",
                          i, i, i, i, i);
    }
    // ③ 持久化
    if ((int)MOT[i]->sensor_direction != 0 && MOT[i]->zero_electric_angle != (float)NOT_SET) {
        encPrefs.putInt(nvsDirKey(i), (int)MOT[i]->sensor_direction);
        encPrefs.putFloat(nvsZeaKey(i), MOT[i]->zero_electric_angle);
    }
    Serial.printf("[S1] M%d ZeroElecAngle=%.4f  dir=%d\n",
                  i, MOT[i]->zero_electric_angle, (int)MOT[i]->sensor_direction);
}

// ===========================================================================
// 8) setup
// ===========================================================================
void setup() {
    Serial.begin(115200);
    Serial2.begin(2000000);       // 板间串口（ESP32 UART2 默认引脚），与 v2.8.0 保持一致
    delay(100);
    Serial.println("[S1] Dual FOC Velocity v3.16.2 (M0:4/2/13+CS18, M1:12/14/27+CS22, EN=25 shared)");  // v3.16.2: M0 切 voltage 治 settle 左漂
    encPrefs.begin("enccomp", false);   // ★v3.9.0 打开 NVS 命名空间，稍后逐轴载入偏心补偿表

    // 使能脚：先拉高，让 initFOC 的通电对齐能做；setup 结束再按状态决定
    pinMode(ENABLE, OUTPUT);
    digitalWrite(ENABLE, HIGH);

    for (int i = 0; i < 2; i++) {
        SEN[i]->init();
        encCompLoad(i);                 // ★v3.9.0 上电载入偏心补偿表（须在 initFOC 前，让 zea 在补偿后角度上算）
        MOT[i]->linkSensor(SEN[i]);
        DRV[i]->voltage_power_supply = 12;
        DRV[i]->voltage_limit        = 10;
        DRV[i]->init();
        MOT[i]->linkDriver(DRV[i]);
        MOT[i]->controller     = MotionControlType::velocity;
        MOT[i]->pole_pairs     = 7;
        MOT[i]->voltage_limit  = 10;
        MOT[i]->voltage_sensor_align = 3;
        MOT[i]->init();
        // ★v3.10.0 速度环参数：必须放在 init() 之后（init() 会把 PID_velocity.limit 重置为 voltage_limit）。
        // 来历（2026-10-14 台架实测，空载低惯量电机）：
        //   P0.1/I0.02（v2.8.0 带翼负载值）→ 输出仅 ~0.2V，推不动
        //   P1.0/I0.5                     → 自激：uq 打到 ±10V，v 峰值 115 rad/s，净转≈0
        //   P0.2/I1.0                     → uq 仅 ±0.7V，v 平滑收敛到目标（v3.10~v3.13 默认）
        //   I≥1.5                         → 积分饱和 → 堵转灌电流
        // ★v3.14.1（2026-10-15 台架实测）：P 0.2 → 0.1
        //   原因：本板速度估计自带高频噪声——Sensor::getVelocity() 的分子(角度增量)取 angle_prev、
        //   分母(Ts)取 angle_prev_ts-vel_angle_prev_ts，两者取自不同时刻；实测稳态下 vraw/vfilt
        //   在 ±(15~65) rad/s 逐帧乱跳，而 cnt14 单调推进证明转子其实是平滑的。
        //   P=0.2 时 35 rad/s 的误差就能把输出打满 7V ⇒ bang-bang 极限环：uq 每 1.14ms 在 ±5V
        //   翻转、v 峰值 52.8、翻转帧 10/25、iq 0.28A。
        //   P=0.1 后（同一 zea/同一硬件）：翻转 0/25、v 峰值 12.1、iq 降到 0.06A（↓4.5×），
        //   2~40 rad/s 全程 TRACK 且零翻转。⇒ 增益降一半，别去动阈值。
        MOT[i]->PID_velocity.P = 0.1f;
        MOT[i]->PID_velocity.I = 1.0f;
        MOT[i]->PID_velocity.D = 0.0f;
        MOT[i]->PID_velocity.limit = 7.0f;   // ★v3.14.0：6→7V（保守解锁高速；仍远低于 10V 硬件钳位，防自激）
        MOT[i]->LPF_velocity.Tf = 0.003f;    // 5ms 默认滞后过大
    }

    // 电流采样（仅 M0）：极性按商家 13 例程（A/B 取负），跳过内部通电对齐
    current_sense.gain_a *= -1.0f;
    current_sense.gain_b *= -1.0f;
    current_sense.linkDriver(&driver0);
    current_sense.skip_align = true;
    current_sense.init();
    // ★v3.16.2：M0 不 link current_sense → velocity 模式自动退回 voltage（和 M1 对称）
    //   治直线段 settle 左漂根因：M0 电流环跟 target 猛刹、M1 电压模式惯性滑 → 不对称 → 左转
    //   切 voltage 后两轴都惯性滑行（对称），settle 漂 ~0°。
    //   代价：M0 失去电流环 OC_FAULT（DRV8313 自带 2.5A 限流 + 堵转检测仍在）。
    //   current_sense.init() 保留（万一要切回电流环只需取消下行注释）。
    // motor0.linkCurrentSense(&current_sense);

    // 逐个对齐（initFOC 会通电让转子就位；此时翼/轮须有转动空间）
    // ★v3.12.0：把"运动式方向判定"从关键路径上摘掉（它是开机对齐时好时坏的唯一原因）。
    // ★v3.15.0：两轴共用同一套四级保险 → M1 也具备"复用NVS / 静态兜底 / 存盘"能力。
    initAxisFOC(0);
    initAxisFOC(1);

    // 对齐失败的轴（Direction::UNKNOWN = 0）标记为 NOSENSOR：拒绝启动、上报速度 0。
    // 典型场景：该路编码器未接（CS 悬空 → SPI 噪声会让速度反馈变垃圾），或没装电机。
    for (int i = 0; i < 2; i++) {
        if ((int)MOT[i]->sensor_direction == 0) {
            ax[i].state = AX_NOSENSOR;
            Serial.print("[WARN] M"); Serial.print(i);
            Serial.println(" 未对齐(无编码器/电机) → 该轴已禁用，状态帧上报速度为 0");
        }
    }

    // 上电默认断使能（与 v2.8.0 一致：防止待机时闭环自激啸叫）
    setEnable(false);

    printHelp();
    Serial.println("[S1] Ready.");
}

// ===========================================================================
// 9) loop
// ===========================================================================
void loop() {
    uint32_t now = millis();

    // ---- 收命令 ----
    // H1 后忽略板间 Serial2（S3 会 ~20Hz 持续下发 R，会盖掉本机台架测试的目标）
    if (!hostLock) {
        while (Serial2.available()) parseCommand(Serial2.readStringUntil('\n'));
    }
    while (Serial.available())  parseCommand(Serial.readStringUntil('\n'));

    // ---- 共用的使能脚：任一路在跑、或正在标定，就使能 ----
    setEnable(calState != 0 || encCalState != 0 || ax[0].state == AX_RUN || ax[1].state == AX_RUN);

    // ---- 开环测试模式超时退出 ----
    if (testMode && millis() > testUntil) {
        Serial.println("[TEST] timeout -> 退出开环，恢复闭环");
        stopAll();
    }

    // ---- 开环对准法标定状态机（C 命令）----
    if (calState) calStep(now);
    // ---- 编码器偏心补偿标定状态机（X 命令）----
    if (encCalState) encCalStep(now);

    // ---- 逐轴更新 ----
    for (int i = 0; i < 2; i++) {
        // 标定期间由 calStep 接管（已切到 velocity_openloop）
        // ★v3.15.0：从写死 i==0 改为按 calAxis，使 M1 也能被 C/P 标定接管
        if (i == calAxis && calState) {
            MOT[i]->loopFOC();
            MOT[i]->move();
            ax[i].velocity = MOT[i]->shaft_velocity;
            continue;
        }
        // 偏心补偿标定期间：对应轴由 encCalStep 接管（已切到 velocity_openloop）
        if (i == encCalAxis && encCalState) {
            MOT[i]->loopFOC();
            MOT[i]->move();
            ax[i].velocity = MOT[i]->shaft_velocity;
            continue;
        }
        // FOC 未对齐的轴（无编码器/电机）：不跑 FOC、不打 PWM、速度上报 0
        if (ax[i].state == AX_NOSENSOR) {
            ax[i].velocity = 0.0f;
            MOT[i]->target = 0.0f;
            DRV[i]->setPwm(0, 0, 0);
            continue;
        }
        ax[i].velocity = MOT[i]->shaft_velocity;

        if (ax[i].state == AX_RUN) {
            if (ax[i].mode == 1 && !testMode) {
                // 速度模式：斜率限制（每 ms 一步）
                static uint32_t lastSlew[2] = {0, 0};
                if (now != lastSlew[i]) {
                    lastSlew[i] = now;
                    float dv = ax[i].targetCmd - ax[i].target;
                    float maxDv = slewRate * 0.001f;
                    if (dv >  maxDv) dv =  maxDv;
                    else if (dv < -maxDv) dv = -maxDv;
                    ax[i].target += dv;
                }
            } else {
                ax[i].target = ax[i].targetCmd;  // 力矩模式不走斜率
            }
            MOT[i]->target = ax[i].target;
        } else {
            MOT[i]->target = 0.0f;
        }

        MOT[i]->loopFOC();
        MOT[i]->move();
        if (i == fbAxis && fbOn) fbStep();     // 速度反馈体检（F 命令，★v3.15.0 按轴）
        if (i == recAxis && recOn) {           // 飞行记录仪（B 命令，★v3.15.0 按轴）
            if (recN < recWant && recN < REC_MAX) {
                Rec &r = recBuf[recN];
                r.t     = _micros();
                r.cnt   = (uint16_t)(((int32_t)lroundf(SEN[recAxis]->getMechanicalAngle()
                            * (16384.0f / _2PI))) & 0x3FFF);
                r.vraw  = (int32_t)constrain(SEN[recAxis]->getVelocity() * 1000.0f, -100000.0f, 100000.0f);
                r.vfilt = (int32_t)constrain(MOT[recAxis]->shaft_velocity * 1000.0f, -100000.0f, 100000.0f);
                // iq 只有 M0 有采样；记录 M1 时该列无意义，强制 0 以免误读
                r.iq    = (recAxis == 0)
                          ? (int16_t)constrain(iq_filtered * 1000.0f, -32000.0f, 32000.0f) : 0;
                r.uq    = (int16_t)constrain(MOT[recAxis]->voltage.q * 1000.0f, -32000.0f, 32000.0f);
                recN++;
                if (recN >= recWant) recOn = 0;
            } else recOn = 0;
        }

        // 堵转检测：仅速度模式（力矩模式下目标是电压，堵转判据不成立）
        // 每 50ms 累积角度（处理 wrap），每 1000ms 判一次
        if (ax[i].mode == 1 && !testMode && ax[i].state == AX_RUN && now - ax[i].lastStallUpdate > 50) {
            float ang = MOT[i]->shaft_angle;   // ★改用 loopFOC 已更新的缓存角，不再额外打 SPI
            float da  = ang - ax[i].lastAngle;
            if (da >  _PI) da -= _2PI;
            if (da < -_PI) da += _2PI;
            ax[i].stallAccum += da;
            ax[i].lastAngle = ang;
            ax[i].lastStallUpdate = now;
        }
        if (ax[i].mode == 1 && !testMode && ax[i].state == AX_RUN &&
            (now - ax[i].runStart > 3000) &&
            (now - ax[i].lastStallCheck > 1000)) {
            // 目标 >2rad/s 才判堵转（避免 r0 零速保持被误判）
            if (fabsf(ax[i].target) > 2.0f && fabsf(ax[i].velocity) < 20.0f &&
                fabsf(ax[i].stallAccum) < 1.5f) {
                ax[i].state     = AX_STALL;
                ax[i].targetCmd = 0.0f;
                ax[i].target    = 0.0f;
                Serial.printf("[S1] STALL M%d\n", i);
            }
            ax[i].stallAccum      = 0.0f;
            ax[i].lastStallCheck  = now;
        }
    }

    // ---- 电流监测 + 过流保护（仅 M0 有采样）----
    static uint32_t lastCurrentRead = 0;
    static int ocCount = 0;
    if (now - lastCurrentRead >= 10) {
        lastCurrentRead = now;
        DQCurrent_s dq = current_sense.getFOCCurrents(motor0.electrical_angle);
        iq_filtered = 0.8f * iq_filtered + 0.2f * dq.q;
        // ★v3.14.0：判据由瞬时 dq.q 改为滤波值 iq_filtered，避免开关尖峰直接触发过流
        if (ax[0].state == AX_RUN && fabsf(iq_filtered) > OC_CURRENT_LIMIT_A) {
            if (++ocCount >= OC_CONFIRM_COUNT) {
                ax[0].state     = AX_OC;
                ax[0].targetCmd = 0.0f;
                ax[0].target    = 0.0f;
                Serial.println("[S1] OC_FAULT M0");
            }
        } else {
            ocCount = 0;
        }
    }

    // ---- 速度反馈体检收尾（F 命令）----
    if (fbOn && millis() > fbUntil) { fbOn = 0; fbReport(); }

    // ---- 状态回传：RUNNING 20Hz；IDLE 变化时 + 5Hz 心跳 ----
    static uint32_t lastTx = 0;
    static AxisState lastReported[2] = { AX_IDLE, AX_IDLE };
    bool running = (ax[0].state == AX_RUN || ax[1].state == AX_RUN || calState != 0 || encCalState != 0);
    bool changed = (ax[0].state != lastReported[0] || ax[1].state != lastReported[1]);
    if ((running && now - lastTx >= 50) || (!running && (changed || now - lastTx >= 200))) {
        lastTx = now;
        lastReported[0] = ax[0].state;
        lastReported[1] = ax[1].state;
        sendStatusFrame();
    }

    delay(1);
}

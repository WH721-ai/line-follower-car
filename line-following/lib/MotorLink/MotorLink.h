/**
 * MotorLink —— 扩展主控(ESP32-S3) → 电机主控(ESP32) 的电机指令链路
 * ============================================================================
 * 为什么不用课程的 SF_BLDC(libSF_BLDC.a)：
 *   · .a 里是商家原厂的 T/V/A/C/U 协议（`%c%f,%f,%f,%f,%f%c`），
 *     需要电机主控跑商家原厂固件；
 *   · 商家原厂 main.cpp 在本机已不存在（BLDC_S1 非 git 仓库、无备份、
 *     Gitee 官方例程仓库只有 LICENSE），恢复等于照头文件盲猜；
 *   · 而电机主控上现在跑的是自编 SimpleFOC 固件，协议是 R/S/M/P/H。
 * 所以这里直接说「自编双电机固件 v3.0.0」的 R/T/S 协议，两端都由我们掌握。
 *
 * 板间串口参数（不是猜的）：
 *   把课程 libSF_BLDC.a 反汇编 SF_BLDC::init() 得到
 *     0x17: movi.n a14, 17      ← txPin
 *     0x19: movi.n a13, 18      ← rxPin
 *     0x1e: callx8 → HardwareSerial::begin(baud, config, 18, 17, ...)
 *   即 S3 侧 RX=GPIO18 / TX=GPIO17；与已台架验证过的 BLDC_S3 固件一致。
 *   波特率 2Mbps（与电机主控 Serial2.begin(2000000) 对应）。
 * ============================================================================
 */
#ifndef _MOTORLINK_h
#define _MOTORLINK_h

#include <Arduino.h>

#define MOTORLINK_BAUD     2000000
#define MOTORLINK_RX_PIN   18      // S3 收 ← S1 发
#define MOTORLINK_TX_PIN   17      // S3 发 → S1 收

/** 一帧状态解析结果 */
struct MotorData {
    float angle[2];   // 电机轴累计角 [rad]（0=M0 / 1=M1）
    float vel[2];     // 电机轴角速度 [rad/s]
    float iq[2];      // Iq 电流 [A]（M1 无采样，恒 0）
    char  state[2];   // 'I' 空闲 / 'R' 运行 / 'S' 堵转 / 'O' 过流 / '?' 未知
};

class MotorLink {
public:
    explicit MotorLink(HardwareSerial &serial);

    void init();
    void setTargets(float m0, float m1);     // 目标：速度模式=[rad/s]；力矩模式=[V]
    MotorData getBLDCData();                 // 泵收发 + 返回最新缓存
    void setModes(uint8_t m0, uint8_t m1) { _mode0 = m0; _mode1 = m1; } // 兼容原 SF_BLDC 接口
    // ★v2.9.1：coast 模式（settle 段专用）。true = 发 J 让两轴自由衰减 + 抑制 R 心跳；
    //   false = 恢复正常 R 下发。治 M0(电流环)刹得比 M1(电压模式)狠 → settle 左偏。
    void setCoast(bool c);

    bool     online()   const { return _online; }
    uint32_t lastRxMs() const { return _lastRx; }

    // ★v2.4.0：把任意一行原样转给 S1（用于 S1 独有的 W/K/F/B 等命令）。
    //   注意会与 pumpTx 的 R 目标共用串口，只在非运动时用。
    //   转发后 1.5 s 内把 S1 的文本回复转印到 USB，否则看不到命令是否被接受。
    void     sendRaw(const char *line) { _s->println(line); _lastTx = millis(); _echoUntil = millis() + 1500; }

private:
    HardwareSerial *_s;
    String   _rxBuf;

    float    _t0 = 0.0f, _t1 = 0.0f;         // 本次目标
    float    _sent0 = 1e9f, _sent1 = 1e9f;   // 上次已下发目标
    uint32_t _lastTx = 0, _lastRx = 0;
    uint32_t _echoUntil = 0;                 // ★v2.4.0 转发命令后的回显窗口
    bool     _online = false;
    bool     _coasting = false;               // ★v2.9.1 coast 模式：抑制 R 下发
    uint8_t  _mode0 = 1, _mode1 = 1;         // 1=速度 4=力矩

    MotorData _data;

    void pumpRx();
    void pumpTx();
};

#endif // _MOTORLINK_h

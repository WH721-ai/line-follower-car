#include "MotorLink.h"

MotorLink::MotorLink(HardwareSerial &serial) : _s(&serial) {
    for (int i = 0; i < 2; i++) {
        _data.angle[i] = 0.0f;
        _data.vel[i]   = 0.0f;
        _data.iq[i]    = 0.0f;
        _data.state[i] = '?';
    }
    _rxBuf.reserve(80);
}

void MotorLink::init() {
    _s->begin(MOTORLINK_BAUD, SERIAL_8N1, MOTORLINK_RX_PIN, MOTORLINK_TX_PIN);
    delay(20);
    // 上电先让电机主控停机：避免它保持上一次的目标（例如上一次实验留下的转速）
    _s->println("S");
    _lastTx = millis();
}

void MotorLink::setTargets(float m0, float m1) {
    _t0 = m0;
    _t1 = m1;
}

// ★v2.9.1：coast 模式（settle 段专用）
//   进入时发一次 J（S1 把两轴置 IDLE → DRV8313 断电 → 纯机械摩擦自由衰减，对称）。
//   pumpTx 在 coasting 期间不发 R（避免 M0 电流环猛刹、M1 电压模式惯性滑的不对称）。
//   退出时只清标志位——下一次 pumpTx 会自动恢复 R 心跳。
void MotorLink::setCoast(bool c) {
    if (c && !_coasting) {
        _s->println("J");
        _lastTx = millis();
    }
    _coasting = c;
}

// 目标下发：变化超过 0.05 rad/s 立刻发；否则 50Hz 心跳式刷新
void MotorLink::pumpTx() {
    if (_coasting) return;  // ★v2.9.1 coast 期间不发 R（两轴自由衰减，无目标）
    uint32_t now = millis();
    bool changed = (fabsf(_t0 - _sent0) > 0.05f) || (fabsf(_t1 - _sent1) > 0.05f);
    if (!changed && now - _lastTx < 50) return;
    if (now - _lastTx < 4) return;          // 最小间隔，2Mbps 下富余

    char buf[48];
    snprintf(buf, sizeof(buf), "R%.3f,%.3f", _t0, _t1);
    _s->println(buf);
    _sent0 = _t0;
    _sent1 = _t1;
    _lastTx = now;
}

// 收状态帧
//   新（双电机固件 v3.0.0）：A<a0>,V<v0>,A1<a1>,V1<v1>,S<c0><c1>,I<i0>
//   旧（单电机固件 v2.8.0）：A<a0>,V<v0>,M<mode>,E<ms>,S<c>,I<i0>
// 兼容旧帧是为了：万一电机主控还没刷成双电机版，第一次上电也能用它验证
// 「板间串口引脚/波特率是否正确」这一件事（此时只有 M0 有数据）。
void MotorLink::pumpRx() {
    while (_s->available()) {
        char ch = (char)_s->read();
        if (ch == '\n' || ch == '\r') {
            if (_rxBuf.length() > 0 && _rxBuf[0] == 'A') {
                float a0 = 0, v0 = 0, a1 = 0, v1 = 0, i0 = 0;
                char  c0 = '?', c1 = '?';
                bool  ok = false;

                if (sscanf(_rxBuf.c_str(), "A%f,V%f,A1%f,V1%f,S%c%c,I%f",
                           &a0, &v0, &a1, &v1, &c0, &c1, &i0) == 7) {
                    ok = true;
                } else {
                    int m0 = 0; unsigned long e0 = 0; char sc = '?';
                    if (sscanf(_rxBuf.c_str(), "A%f,V%f,M%d,E%lu,S%c,I%f",
                               &a0, &v0, &m0, &e0, &sc, &i0) == 6) {
                        c0 = (m0 == 1) ? 'R' : (m0 == 3) ? 'S' : (m0 == 4) ? 'O' : 'I';
                        ok = true;
                    }
                }

                if (ok) {
                    _data.angle[0] = a0;  _data.vel[0] = v0;
                    _data.angle[1] = a1;  _data.vel[1] = v1;
                    _data.iq[0]    = i0;
                    _data.state[0] = c0;  _data.state[1] = c1;
                    _lastRx = millis();
                    _online = true;
                }
            } else if (_rxBuf.length() > 0 && millis() < _echoUntil) {
                // ★v2.4.0：`@<line>` 转发后的 1.5 s 内，把 S1 的文本回复转印到 USB。
                //   没有这一条，`@W30` 到底被接受了没有就完全看不见（S1 的 [RX] 行原被丢弃）。
                Serial.print("[S1<] ");
                Serial.println(_rxBuf);
            }
            _rxBuf = "";
        } else {
            if (_rxBuf.length() < 72) _rxBuf += ch;
            else _rxBuf = "";               // 脏数据过长直接丢弃
        }
    }
    if (_online && millis() - _lastRx > 500) _online = false;
}
MotorData MotorLink::getBLDCData() {
    pumpRx();
    pumpTx();
    return _data;
}

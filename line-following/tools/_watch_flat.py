# -*- coding: utf-8 -*-
"""段二摆动专项监听（离线自启跑专用，零命令注入）。

为什么不能复用 _listen.py：
  _listen.py 检测不到帧时会发 `d2` 开帧 —— 对离线自启跑是干扰。
  本脚本【一个字节都不发】，纯附着；并且带自动重连，
  所以用户按 S3 的 RST（原生 USB 会重新枚举）也能从复位那一拍开始完整记录。

重点抓的两类行：
  [FLAT]      odo / e / eF / Kp / wCmd / gyro / arcff / eD / v指令 / v实测
  [FLAT-STAT] 摆幅 ±X mm / 周期 Y ms / 过零 N 次   ← 判定"摆"的性质就看这行

用法：
    python -u _watch_flat.py --port COM12 --sec 150 --out run_flat.log
（--port 默认 COM12，请按你的实际串口改；--baud 默认 921600）
"""
import argparse
import io
import os
import sys
import time

import serial

ap = argparse.ArgumentParser()
ap.add_argument("--port", default="COM12")
ap.add_argument("--baud", type=int, default=921600)
ap.add_argument("--sec", type=float, default=150.0)
ap.add_argument("--out", default="run_flat.log")
ap.add_argument("--reset", action="store_true",
                help="打开串口后发一次 RTS 复位脉冲，强制从复位那一拍开始录")
ap.add_argument("--idle", type=float, default=8.0,
                help="连续多少秒收不到数据就判定句柄已死并重开（默认 8 s）")
ap.add_argument("--reset-flag", default="",
                help="外部触发复位：该文件一旦出现就发一次 RTS 复位脉冲并删掉它")
a = ap.parse_args()

out = io.open(a.out, "w", encoding="utf-8", newline="\n")


def say(msg):
    try:
        sys.stdout.write(msg + "\n")
        sys.stdout.flush()
    except Exception:
        pass


def log(msg):
    out.write(msg + "\n")
    out.flush()
    say(msg)


log("# watch_flat start  port=%s baud=%d sec=%.0f  (零命令、自动重连)" % (a.port, a.baud, a.sec))

t0 = time.time()
s = None
nOpen = 0
nLine = 0
nFlat = 0
nStat = 0
lastData = time.time()
resetDone = False

while time.time() - t0 < a.sec:
    # ★外部触发复位：标记文件一出现就发一次 RTS 脉冲（可重复触发，不必抢串口）
    if a.reset_flag and os.path.exists(a.reset_flag):
        try:
            os.remove(a.reset_flag)
        except Exception:
            pass
        if s is not None:
            try:
                s.dtr = False
                s.rts = True
                time.sleep(0.15)
                s.rts = False
                lastData = time.time()
                log("# [reset] 外部触发 RTS 复位 t=%.2fs ⇒ 约 8.5 s 后自启" % (time.time() - t0))
            except Exception as e:
                log("# [reset fail] %r" % (e,))

    if s is None:
        try:
            ser = serial.Serial()
            ser.port = a.port
            ser.baudrate = a.baud
            ser.timeout = 0.2
            ser.write_timeout = 1.0
            ser.dtr = False          # ★不拉 DTR/RTS：避免把刚起来的固件又复位一次
            ser.rts = False
            ser.open()
            s = ser
            nOpen += 1
            lastData = time.time()
            log("# [open#%d] t=%.2fs" % (nOpen, time.time() - t0))
            # ★★★必须发一个空行！ESP32-S3 原生 USB-CDC 的 HWCDC 有个 m_connected 标志，
            #   只有主机【发过东西】它才会置位，否则 write() 直接返回 0（日志全被静默丢弃）。
            #   表现：固件完全活着、`s` 一叫就应，但纯监听的日志里一个字节都没有。
            #   空行对命令解析器是无害的（len==0 直接返回）。
            if not (a.reset and not resetDone):
                try:
                    ser.write(b"\nd1\n")
                    ser.flush()
                    log("# [d1] 已打开 20Hz 状态帧（后续 S, 帧会写进日志但不刷屏）")
                except Exception:
                    pass
            # ★强制复位：USB-Serial-JTAG 里 RTS→EN(复位)、DTR→GPIO0(下载模式)。
            #   只脉冲 RTS，DTR 保持 False ⇒ 复位后走正常 app 启动，不会进下载模式。
            #   ★只发一次（resetDone），避免重连时反复复位把起跑时序打乱。
            if a.reset and not resetDone:
                resetDone = True
                try:
                    ser.dtr = False
                    ser.rts = True
                    time.sleep(0.15)
                    ser.rts = False
                    lastData = time.time()
                    log("# [reset] RTS 脉冲已发 t=%.2fs ⇒ 约 8.5 s 后自启" % (time.time() - t0))
                except Exception as e:
                    log("# [reset fail] %r" % (e,))
        except Exception:
            s = None
            time.sleep(0.15)
            continue

    try:
        raw = s.readline()
    except Exception as e:                       # 拔插 / RST 会让句柄失效
        log("# [lost] t=%.2fs %r" % (time.time() - t0, e))
        try:
            s.close()
        except Exception:
            pass
        s = None
        continue

    if not raw:
        # ★句柄假死检测：设备重新枚举后 readline() 只返回空、**不抛异常**，
        #   旧代码会在这里无限空转（v3.15.19 第一次监听就是这样，40 s 一个字节都没进）。
        if time.time() - lastData > a.idle:
            log("# [idle] %.0fs 无数据 ⇒ 判定句柄已死，重开" % (time.time() - lastData))
            try:
                s.close()
            except Exception:
                pass
            s = None
            lastData = time.time()
        continue

    lastData = time.time()
    ln = raw.decode("utf-8", "replace").rstrip("\r\n")
    if not ln:
        continue

    dt = time.time() - t0
    out.write("%8.3f %s\n" % (dt, ln))
    nLine += 1

    if ln.startswith("S,") or ln.startswith("L,"):
        continue                                  # 高频状态帧不刷屏，但已存盘

    say("[%7.2f] %s" % (dt, ln))
    if ln.startswith("[FLAT-STAT]"):
        nStat += 1
    elif ln.startswith("[FLAT]"):
        nFlat += 1

log("# done  行数=%d  [FLAT]=%d  [FLAT-STAT]=%d" % (nLine, nFlat, nStat))
if s:
    try:
        s.close()
    except Exception:
        pass
out.close()

# -*- coding: utf-8 -*-
"""探活：往 COM12 发一条 `s`（停止/解除），看固件还有没有回应。

用途：区分「固件卡死/崩溃」和「只是串口句柄假死/掉线」。
  - 有回显（任何字节）⇒ CPU 活着，之前是 USB/句柄问题
  - 一个字节都没有     ⇒ CPU 已停（hard hang / panic 无输出）
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM12"
cmd = sys.argv[2] if len(sys.argv) > 2 else "s"
sec = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0

print("open %s ..." % port)
try:
    s = serial.Serial()
    s.port = port
    s.baudrate = 921600
    s.timeout = 0.2
    s.dtr = False
    s.rts = False
    s.open()
except Exception as e:
    print("OPEN_FAIL %r" % (e,))
    sys.exit(2)

print("open ok, send %r" % cmd)
try:
    s.write((cmd + "\n").encode())
    s.flush()
except Exception as e:
    print("WRITE_FAIL %r" % (e,))

n = 0
t0 = time.time()
while time.time() - t0 < sec:
    try:
        raw = s.readline()
    except Exception as e:
        print("READ_FAIL %r" % (e,))
        break
    if raw:
        n += 1
        print("RX %7.3f %s" % (time.time() - t0, raw.decode("utf-8", "replace").rstrip()))
s.close()
print("TOTAL_LINES=%d" % n)
print("VERDICT=%s" % ("ALIVE" if n > 0 else "SILENT"))

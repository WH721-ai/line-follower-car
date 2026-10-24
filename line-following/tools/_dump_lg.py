# -*- coding: utf-8 -*-
"""完整导出板载 RAM 运行记录。
用法: python _dump_lg.py COM12 out.csv [清空=0/1]
- 打开串口后先发 \n 唤醒 HWCDC，再发 D
- 等到 "[LG] end" 或超时，把 #LG 之后的行切成 CSV 写文件
"""
import sys, time, serial

port = sys.argv[1] if len(sys.argv) > 1 else 'COM12'
out = sys.argv[2] if len(sys.argv) > 2 else r'C:\platformio\lg_dump.txt'
do_clear = (len(sys.argv) > 3 and sys.argv[3] == '1')

sp = serial.Serial()
sp.port = port
sp.baudrate = 921600
sp.timeout = 0.25
sp.dtr = False
sp.rts = False
sp.open()
time.sleep(0.4)
sp.write(b'\nD\n')
t0 = time.time()
lines = []
done = False
while time.time() - t0 < 40.0:
    try:
        ln = sp.readline()
    except Exception:
        break
    if not ln:
        if done:
            break
        continue
    t = ln.decode('utf-8', 'replace').rstrip()
    if t:
        lines.append(t)
    if '[LG] end' in t:
        done = True
if do_clear:
    sp.write(b'\nD0\n')
    time.sleep(0.3)
sp.close()

with open(out, 'w', encoding='utf-8', newline='\n') as f:
    for t in lines:
        f.write(t + '\n')

csv = [t for t in lines if t and not t.startswith('[') and not t.startswith('#') and not t.startswith('[') and (t[0].isdigit() or t[0] == '-')]
print('total_lines=', len(lines))
print('csv_rows=', len(csv))
print('has_begin=', any('[LG] begin' in t for t in lines))
print('has_end=', done)
print('elapsed=%.2f' % (time.time() - t0))
if csv:
    print('FIRST|', csv[0])
    print('LAST |', csv[-1])

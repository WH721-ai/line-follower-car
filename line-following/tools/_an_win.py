# -*- coding: utf-8 -*-
"""Frame-by-frame dump of an odo window. Usage: python _an_win.py <dump> <lo_m> <hi_m> [step]"""
import sys

path = sys.argv[1] if len(sys.argv) > 1 else r'C:\platformio\lg_L4.txt'
lo = float(sys.argv[2]) * 1000 if len(sys.argv) > 2 else 7700
hi = float(sys.argv[3]) * 1000 if len(sys.argv) > 3 else 8800
step = int(sys.argv[4]) if len(sys.argv) > 4 else 1

rows = []
for ln in open(path, encoding='utf-8', errors='replace'):
    ln = ln.strip()
    if not ln or ln.startswith('[') or ln.startswith('#'):
        continue
    p = ln.split(',')
    if len(p) < 12:
        continue
    try:
        rows.append([int(x) for x in p[:12]])
    except ValueError:
        pass

print('   t      odo     e     edot   kp    turn   wcmd  trim  gyro  arcff  vcmd  vme flag')
k = 0
for i, r in enumerate(rows):
    if not (lo <= r[0] * 100 <= hi):
        continue
    k += 1
    if k % step:
        continue
    fl = r[11]
    tags = []
    if fl & 1:  tags.append('on')
    if fl & 2:  tags.append('FLAT')
    if fl & 4:  tags.append('SRCH')
    if fl & 8:  tags.append('CRN')
    if fl & 32: tags.append('vld')
    if fl & 64: tags.append('reacq')
    print('%6.2f %7.2f %+6.1f %+6.3f %5.1f %+6.2f %+6.2f %+4d %+6.3f %+6.2f %5.2f %5.2f %3d %s' % (
        i * 0.02, r[0] / 10.0, r[1] / 10.0, r[2] / 1000.0, r[3] / 10.0,
        r[4] / 1000.0, r[5] / 1000.0, r[6], r[7] / 1000.0, r[8] / 1000.0,
        r[9] / 1000.0, r[10] / 1000.0, fl, ','.join(tags)))

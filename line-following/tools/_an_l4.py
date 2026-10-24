# -*- coding: utf-8 -*-
"""L4 straight sway diagnosis. Usage: python _an_l4.py <dump.txt>
Fields: odo_dm(0.1m) e_mm10(0.1mm) edot(cms) kp10 turn(1e-3) wcmd(x1e-3) trim gyro(x1e-3) arcff vcmd vmeas flag
"""
import sys, math

path = sys.argv[1] if len(sys.argv) > 1 else r'C:\platformio\lg_L4.txt'
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

DT = 0.02
N = len(rows)
print('rows=%d  dur=%.2fs  odo %.2f -> %.2f m' % (N, N * DT, rows[0][0] / 10.0, rows[-1][0] / 10.0))

# --- speed phases ---
print('\n=== vcmd phases ===')
cur = None
for i, r in enumerate(rows):
    v = r[9]
    if cur is None or abs(v - cur) > 15:
        print('  t=%6.2fs odo=%6.2f m  vcmd %.2f -> %.2f' % (i * DT, r[0] / 10.0, (cur or 0) / 1000.0, v / 1000.0))
        cur = v

# --- per 0.5s summary ---
print('\n=== per 0.5 s ===')
print(' t      odo    vcmd vmeas  e_min  e_max  amp  zc  wcmdmx  turnmx edotmx kp  arcff sat')
for k in range(0, N, 25):
    seg = rows[k:k + 25]
    if len(seg) < 5:
        continue
    es = [r[1] / 10.0 for r in seg]
    zc = 0
    for j in range(1, len(seg)):
        if (seg[j - 1][1] < 0) != (seg[j][1] < 0):
            zc += 1
    amp = (max(es) - min(es)) / 2.0
    wm = max(abs(r[5]) for r in seg) / 1000.0
    tm = max(abs(r[4]) for r in seg) / 1000.0
    em = max(abs(r[2]) for r in seg) / 1000.0
    sat = sum(1 for r in seg if abs(r[4]) >= 1990)
    print('%5.2f %7.2f %5.2f %5.2f %+6.1f %+6.1f %5.1f %3d %6.2f %6.2f %5.2f %4.1f %+6.2f %3d' % (
        k * DT, seg[0][0] / 10.0, seg[-1][9] / 1000.0, sum(r[10] for r in seg) / len(seg) / 1000.0,
        min(es), max(es), amp, zc, wm, tm, em, seg[-1][3] / 10.0,
        sum(r[8] for r in seg) / len(seg) / 1000.0, sat))

# --- zero-crossing period estimate on the last 60% ---
print('\n=== zero-crossing periods (full run) ===')
idx = [i for i in range(1, N) if (rows[i - 1][1] < 0) != (rows[i][1] < 0)]
if len(idx) >= 3:
    per = [(idx[j] - idx[j - 1]) * DT * 1000.0 for j in range(1, len(idx))]
    per = [p for p in per if 60 < p < 3000]
    if per:
        print('  n=%d  mean=%.0f ms  min=%.0f  max=%.0f' % (len(per), sum(per) / len(per), min(per), max(per)))
    print('  crossings at odo: ' + ' '.join('%.2f' % (rows[i][0] / 10.0) for i in idx[:40]))
else:
    print('  too few crossings (%d)' % len(idx))

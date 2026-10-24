# -*- coding: utf-8 -*-
"""Integrate gyro heading per arc window -> total degrees + median yaw rate.
Usage: python _an_dir2.py <dump.csv>
Fields: odo_dm e_mm10 edot_mms kp10 turn wcmd trim gyro arcff vcmd vmeas flag
"""
import sys, math

path = sys.argv[1]
rows = []
for ln in open(path, encoding='utf-8', errors='replace'):
    ln = ln.strip()
    if not ln or ln[0] in '[#':
        continue
    p = ln.split(',')
    if len(p) < 12:
        continue
    try:
        rows.append([int(x) for x in p[:12]])
    except ValueError:
        pass

DT = 0.02
ARC = [
    ("R7 ",  7110,  7975, 500, -1, -90),
    ("R8 ",  8795, 11027, 685, +1, +180),
    ("R9 ", 11651, 13343, 342, +1, +270),
    ("R10", 14708, 15247, 292, -1, -90),
    ("R11", 15247, 16577, 392, -1, -180),
    ("R12", 18624, 19489, 500, -1, -90),
    ("R13", 19409, 21827, 500, +1, +268),
    ("R14", 21747, 22395, 600, -1, -54),
    ("R15", 22315, 22965, 500, +1, +65),
]

def med(v):
    v = sorted(v)
    n = len(v)
    return 0.0 if n == 0 else (v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2]))

print('rows=%d' % len(rows))
print('')
print('arc   n     dYaw_int   yaw_med   fw  expect   verdict')
for name, lo, hi, R, sgn, exp in ARC:
    seg = [r for r in rows if lo <= r[0] * 100 <= hi]
    if len(seg) < 8:
        print('%s  %4d  (too few)' % (name, len(seg)))
        continue
    dint = sum(r[7] for r in seg) / 1000.0 * DT * 57.2958
    ym = med([r[7] / 1000.0 for r in seg])
    ok = 'OK' if (dint * exp) > 0 else '*** DIR OPPOSITE ***'
    print('%s %4d  %+8.1f  %+7.3f  %+d  %+5d   %s'
          % (name, len(seg), dint, ym, sgn, exp, ok))

# print odo where the car actually loses the line (search flags) for reference
print('')
print('flag transitions (bit2=search/bit3=corner):')
prev = None
for i, r in enumerate(rows):
    f = r[11]
    key = (f >> 2) & 3
    if key != prev:
        print('  t=%6.2f odo=%6.2f m flag=%3d (bit2=%d bit3=%d) vcmd=%.2f vmeas=%.2f'
              % (i * DT, r[0] / 10.0, f, (f >> 2) & 1, (f >> 3) & 1,
                 r[9] / 1000.0, r[10] / 1000.0))
        prev = key

import sys

rows = []
with open(sys.argv[1], 'r', encoding='utf-8', errors='replace') as f:
    for ln in f:
        ln = ln.strip()
        if not ln or ln.startswith('#') or ln.startswith('=== '):
            continue
        parts = ln.split(',')
        if len(parts) < 12:
            continue
        try:
            r = [int(x) for x in parts[:12]]
        except ValueError:
            continue
        rows.append(r)

print('rows=%d' % len(rows))

# flag bit16 = c4On
prev = None
for i, r in enumerate(rows):
    on = (r[11] & 16) != 0
    if prev is None:
        prev = on
        print('first frame: odo=%.2f m c4On=%d vcmd=%.2f' % (r[0] / 100.0, on, r[10] / 1000.0))
        continue
    if on != prev:
        print('--- c4 %s at t=%.2f s odo=%.2f m' % ('ON' if on else 'OFF', i * 0.02, r[0] / 100.0))
        for j in range(max(0, i - 6), min(len(rows), i + 4)):
            q = rows[j]
            print('   t=%6.2f odo=%6.2f e=%+6.1f turn=%+5.2f wcmd=%+5.2f gyro=%+6.3f arcff=%+5.2f v=%4.2f/%4.2f flag=%3d' % (
                j * 0.02, q[0] / 100.0, q[1] / 10.0, q[4] / 1000.0, q[5] / 1000.0,
                q[7] / 1000.0, q[8] / 1000.0, q[9] / 1000.0, q[10] / 1000.0, q[11]))
        prev = on

# vcmd steps
print('\n=== vcmd changes ===')
pv = None
for i, r in enumerate(rows):
    v = r[9]
    if pv is None or abs(v - pv) >= 30:
        print('  t=%6.2f odo=%6.2f vcmd=%.2f vmeas=%.2f flag=%3d' % (
            i * 0.02, r[0] / 100.0, r[9] / 1000.0, r[10] / 1000.0, r[11]))
    pv = v

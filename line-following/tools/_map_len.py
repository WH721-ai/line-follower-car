# -*- coding: utf-8 -*-
"""逐段累计里程：从 track/field_track_named.geom 累加每一段长度。

用法：
    python _map_len.py                       # 自动找 ../track/field_track_named.geom
    python _map_len.py <path-to.geom>        # 或显式指定

★ 输出全部用 ASCII，避免 Windows PowerShell 代码页把中文标签写成乱码
  （乱码还可能导致整个文件被当成二进制、后续 diff 全失效）。
"""
import io
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT = os.path.normpath(os.path.join(HERE, "..", "..", "track", "field_track_named.geom"))

path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
if not os.path.exists(path):
    print("geom file not found: %s" % path)
    sys.exit(1)

lines = io.open(path, encoding="utf-8").read().splitlines()

cum = 0.0
seg = None
out = []

for ln in lines:
    s = ln.strip()

    # 注释行：抓段名 "#L3" / "#R7"，以及原地转角 "# A1 原地转 +90"
    if s.startswith("#"):
        m = re.match(r"#\s*(L\d+|R\d+)", s)
        if m:
            seg = m.group(1)
        m2 = re.match(r"#\s*A(\d+)\s+\S+\s+([+-]?[\d.]+)", s)
        if m2:
            out.append("  A%-2s  yaw %+7s deg   (odo=%.1f mm)" % (m2.group(1), m2.group(2), cum))
        continue

    t = s.split()
    if not t:
        continue

    if t[0] == "LINE":
        x1, y1, x2, y2 = map(float, t[1:5])
        L = math.hypot(x2 - x1, y2 - y1)

    elif t[0] == "ARC":
        cx, cy, r, a1, a2 = map(float, t[1:6])
        da = abs(a2 - a1)
        if da > 360.0:
            da -= 360.0
        L = r * math.radians(da)

    else:
        continue

    cum += L
    out.append("%-5s %8.1f mm  -> %8.1f mm" % (seg or "?", L, cum))
    seg = None

print("\n".join(out))
print("")
print("TOTAL = %.1f mm = %.3f m" % (cum, cum / 1000.0))

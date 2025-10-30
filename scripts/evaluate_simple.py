#!/usr/bin/env python3
import os
import sys


def read_tum_positions(path):
    ts, pos = [], []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            ts.append(float(parts[0]))
            pos.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return ts, pos


def l2(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return (dx*dx + dy*dy + dz*dz) ** 0.5


def main():
    if len(sys.argv) < 4:
        print('Usage: evaluate_simple.py ndt_tum.txt gnicp_tum.txt out_dir')
        sys.exit(1)

    ndt_path = sys.argv[1]
    gnicp_path = sys.argv[2]
    out_dir = sys.argv[3]
    os.makedirs(out_dir, exist_ok=True)

    t1, p1 = read_tum_positions(ndt_path)
    t2, p2 = read_tum_positions(gnicp_path)
    n = min(len(t1), len(t2))
    if n == 0:
        print('No data to compare.')
        sys.exit(2)

    # 仅做平移对齐：减去各自首帧位置
    o1 = p1[0]
    o2 = p2[0]
    p1r = [(x - o1[0], y - o1[1], z - o1[2]) for (x, y, z) in p1[:n]]
    p2r = [(x - o2[0], y - o2[1], z - o2[2]) for (x, y, z) in p2[:n]]

    errs = [l2(a, b) for a, b in zip(p1r, p2r)]
    rmse = (sum(e*e for e in errs) / len(errs)) ** 0.5
    mae = sum(errs) / len(errs)
    maxe = max(errs)

    out_path = os.path.join(out_dir, 'relative_simple_metrics.txt')
    with open(out_path, 'w') as f:
        f.write(f'frames: {n}\n')
        f.write(f'rmse[m]: {rmse:.6f}\n')
        f.write(f'mae[m]: {mae:.6f}\n')
        f.write(f'max[m]: {maxe:.6f}\n')

    print(f'Done. frames={n}, RMSE={rmse:.6f} m, MAE={mae:.6f} m, MAX={maxe:.6f} m')


if __name__ == '__main__':
    main()



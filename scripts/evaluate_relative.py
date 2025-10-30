#!/usr/bin/env python3
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_tum(path):
    ts, p = [], []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            ts.append(float(parts[0]))
            p.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(ts), np.array(p)


def umeyama_align(src, dst):
    # src -> dst
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst
    cov = src_c.T @ dst_c / src.shape[0]
    U, S, Vt = np.linalg.svd(cov)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = U @ Vt
    t = mu_dst - R @ mu_src
    return R, t


def rmse(a, b):
    d = a - b
    return float(np.sqrt((d * d).sum(axis=1).mean()))


def main():
    if len(sys.argv) < 4:
        print('Usage: evaluate_relative.py ndt_tum.txt gnicp_tum.txt out_dir')
        sys.exit(1)
    ndt_path = sys.argv[1]
    gnicp_path = sys.argv[2]
    out_dir = sys.argv[3]
    os.makedirs(out_dir, exist_ok=True)

    t1, p1 = read_tum(ndt_path)
    t2, p2 = read_tum(gnicp_path)
    n = min(len(t1), len(t2))
    if n == 0:
        print('No data to compare.')
        sys.exit(2)
    # 对齐前n帧（假设时间步一致）
    p1 = p1[:n]
    p2 = p2[:n]
    R, t = umeyama_align(p2, p1)
    p2_aligned = (R @ p2.T).T + t

    err = np.linalg.norm(p1 - p2_aligned, axis=1)
    rmse_val = float(np.sqrt((err ** 2).mean()))

    with open(os.path.join(out_dir, 'relative_metrics.txt'), 'w') as f:
        f.write(f'frames: {n}\n')
        f.write(f'rmse[m]: {rmse_val:.6f}\n')

    # 绘制轨迹与误差
    plt.figure(figsize=(8, 6))
    plt.plot(p1[:, 0], p1[:, 1], 'r-o', label='NDT')
    plt.plot(p2_aligned[:, 0], p2_aligned[:, 1], 'b-o', label='GN-ICP (aligned)')
    plt.axis('equal')
    plt.legend()
    plt.title('NDT vs GN-ICP (Aligned) - XY')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'relative_xy.png'), dpi=150)
    plt.close()

    plt.figure(figsize=(8, 3))
    plt.plot(err, 'k-o')
    plt.title(f'Per-frame position error (RMSE={rmse_val:.3f} m)')
    plt.xlabel('frame idx')
    plt.ylabel('error [m]')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'relative_error.png'), dpi=150)
    plt.close()

    print(f'Done. RMSE={rmse_val:.6f} m')


if __name__ == '__main__':
    main()



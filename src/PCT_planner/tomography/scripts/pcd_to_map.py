#!/usr/bin/env python3
"""
pcd_to_map.py
把 FAST-LIO-SAM 输出的 3D 点云 (.pcd) 离线投影成 ROS 2D 栅格地图 (.pgm + .yaml)。

只依赖 numpy(放平、解析全自己实现), 不需要 open3d / PCL / ROS, 适合 Jetson 直接跑。

关键特性:
  * 自动用 RANSAC 拟合地面并把点云放平(默认开启) —— 解决 FAST-LIO 世界帧
    未对齐重力导致地图"糊成一团"的问题。可用 --no-level 关闭。
  * 内置 PCD 解析器, 支持 ascii / binary / binary_compressed。

依赖:
    pip3 install numpy
    pip3 install scipy        # 可选, 用于 --fill-free

用法:api_map/dinggu7_2/dinggu7_2.pcd
    python3 pcd_to_map.py api_map/dinggu7_2/dinggu7_2_display.pcd -o api_map/dinggu7_2/dinggu7_2_map__display_auto_fill --fill-free
    python3 pcd_to_map.py GlobalMap.pcd -o map --obstacle-max 1.3 --min-points 3 --fill-free
    python3 pcd_to_map.py GlobalMap.pcd -o map --no-level     # 若点云本就水平

输出:
    map.pgm   (0=占据黑, 254=空闲白, 205=未知灰)
    map.yaml  map_server 元数据
"""

import argparse
import os
import struct
import numpy as np

# numpy >=1.24 移除了 np.int / np.float 等别名, 但 Jetson 上通过 apt 安装的
# 旧版 scipy (ndimage.label 等) 内部仍在使用这些已废弃的别名, 此处做兼容处理。
if not hasattr(np, 'int'):
    np.int = int
if not hasattr(np, 'float'):
    np.float = float
if not hasattr(np, 'bool'):
    np.bool = bool


# =========================== PCD 解析器 (纯 numpy) ===========================

def _lzf_decompress(data, out_len):
    """
    PCD 的 binary_compressed 格式使用 LZF 压缩。

    这个函数负责把压缩数据解压回原始字节流，
    这样脚本就能直接读取常见的 PCL/ROS 点云文件，不依赖 Open3D/PCL.
    """
    out = bytearray(); i, n = 0, len(data)
    while i < n:
        ctrl = data[i]; i += 1
        if ctrl < 32:
            L = ctrl + 1; out += data[i:i + L]; i += L
        else:
            L = ctrl >> 5
            ref = len(out) - ((ctrl & 0x1f) << 8) - 1
            if L == 7:
                L += data[i]; i += 1
            ref -= data[i]; i += 1
            L += 2
            for _ in range(L):
                out.append(out[ref]); ref += 1
    return bytes(out)


_TYPE = {('F', 4): 'f4', ('F', 8): 'f8',
         ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4', ('I', 8): 'i8',
         ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4', ('U', 8): 'u8'}


def read_pcd_xyz(path):
    """
    读取 .pcd 文件并返回 Nx3 点云数组 [x, y, z]。

    该函数支持：
      - ascii
      - binary
      - binary_compressed

    对 FAST-LIO/SAM 输出的地图非常实用，因为它经常会产生压缩 PCD 文件，
    而直接依赖 ROS/PCL 可能不方便，所以这里用纯 numpy 自己解析。
    """
    with open(path, 'rb') as f:
        raw = f.read()
    header = {}; idx = 0
    while True:
        nl = raw.index(b'\n', idx)
        line = raw[idx:nl].decode('ascii', 'ignore').strip(); idx = nl + 1
        if not line or line.startswith('#'):
            continue
        key, *vals = line.split(); header[key.upper()] = vals
        if key.upper() == 'DATA':
            data_start, data_fmt = idx, vals[0].lower(); break

    fields = header['FIELDS']
    sizes = list(map(int, header['SIZE']))
    types = header['TYPE']
    counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
    npts = int(header['POINTS'][0])
    if not all(c in fields for c in ('x', 'y', 'z')):
        raise ValueError("PCD 缺少 x/y/z 字段: " + str(fields))

    if data_fmt == 'ascii':
        txt = raw[data_start:].decode('ascii', 'ignore').splitlines()
        arr = np.atleast_2d(np.loadtxt(txt, dtype=np.float64))
        fi = [fields.index(c) for c in ('x', 'y', 'z')]
        return arr[:, fi].astype(np.float64)

    if data_fmt == 'binary':
        names, formats = [], []
        for fn, sz, tp, ct in zip(fields, sizes, types, counts):
            base = _TYPE[(tp, sz)]
            formats.append(base if ct == 1 else (base, ct)); names.append(fn)
        dt = np.dtype({'names': names, 'formats': formats})
        rec = np.frombuffer(raw[data_start:data_start + dt.itemsize * npts],
                            dtype=dt, count=npts)
        return np.c_[rec['x'], rec['y'], rec['z']].astype(np.float64)

    if data_fmt == 'binary_compressed':
        comp_sz, uncomp_sz = struct.unpack('II', raw[data_start:data_start + 8])
        buf = _lzf_decompress(raw[data_start + 8:data_start + 8 + comp_sz], uncomp_sz)
        cols, off = {}, 0
        for fn, sz, tp, ct in zip(fields, sizes, types, counts):
            span = sz * ct * npts
            cols[fn] = np.frombuffer(buf[off:off + span], dtype=_TYPE[(tp, sz)])[:npts]
            off += span
        return np.c_[cols['x'], cols['y'], cols['z']].astype(np.float64)

    raise ValueError("未知 DATA 格式: " + data_fmt)


# =========================== 用 RANSAC 把点云放平 ===========================

def fit_ground_normal(P, n_iter=300, thresh=0.05, vertical_cos=0.5, seed=0):
    """
    用 RANSAC 拟合近水平的地面平面，并返回：
      - ground normal（地面法向量）
      - ground_z（地面高度）

    作用：
    由于 FAST-LIO 世界坐标系可能存在姿态倾斜，导致地图在 z 方向上“斜着”或
    “被揉在一起”，所以先把地面水平化，再做 2D 占据地图投影。

    这里采用“平面拟合 + 取内点 z 的中位数”来估计 ground height，
    比单纯使用全局 z percentile 更稳，因为能排除高层建筑/天花板等干扰。

    返回的 ground_z 是 RANSAC 内点的 z 中位数——用地面点本身来算高度,
    比全局百分位更准，不受天花板、高楼等远处高点干扰。
    """
    rng = np.random.default_rng(seed); N = len(P)
    Q = P[rng.choice(N, min(N, 20000), replace=False)]
    best_inl, best_n, best_d = -1, np.array([0., 0., 1.]), 0.0
    for _ in range(n_iter):
        s = Q[rng.choice(len(Q), 3, replace=False)]
        nrm = np.cross(s[1] - s[0], s[2] - s[0]); ln = np.linalg.norm(nrm)
        if ln < 1e-6:
            continue
        nrm /= ln
        if abs(nrm[2]) < vertical_cos:      # 只接受法向量够竖直的(即近水平面)
            continue
        d = -nrm.dot(s[0])
        inl = int((np.abs(Q.dot(nrm) + d) < thresh).sum())
        if inl > best_inl:
            best_inl, best_n, best_d = inl, nrm, d
    if best_n[2] < 0:
        best_n = -best_n; best_d = -best_d
    # 用最佳平面的内点在原始点云中的 z 中位数作为地面高度
    inlier_mask = np.abs(P.dot(best_n) + best_d) < thresh
    ground_z = float(np.median(P[inlier_mask, 2])) if inlier_mask.any() else 0.0
    return best_n, ground_z


def rotation_to_z(n):
    """
    创建一个旋转矩阵，把任意法向量 n 转到 +z 方向。

    这一步用在“地面矫正”中：
    先检测地面法向量，再把点云旋转到水平面，
    这样后续的高度分布和投影地图会稳定很多。
    """
    z = np.array([0., 0., 1.]); v = np.cross(n, z); s = np.linalg.norm(v); c = n.dot(z)
    if s < 1e-8:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))


# =============================== 投影主流程 ===============================

def parse_args():
    """
    命令行参数解析。

    这些参数控制：
      - 栅格分辨率
      - 障碍高度区间
      - 地面容忍阈值
      - 占据点阈值
      - 连通域过滤
      - 是否自动地面放平 / 填洞处理
    """
    p = argparse.ArgumentParser(description="3D .pcd -> 2D occupancy grid (.pgm + .yaml)")
    p.add_argument("pcd")
    p.add_argument("-o", "--out", default="map")
    p.add_argument("--resolution", type=float, default=0.05, help="米/格, 默认 0.05")
    p.add_argument("--obstacle-min", type=float, default=0.2, help="障碍高度下界(相对地面,米)")
    p.add_argument("--obstacle-max", type=float, default=0.7, help="障碍高度上界(相对地面,米)")
    p.add_argument("--ground-tol", type=float, default=0.15, help="地面厚度容差(米)")
    p.add_argument("--ground-percentile", type=float, default=2.0, help="估计地面用的 z 百分位")
    p.add_argument("--min-points", type=int, default=2, help="一格至少几个障碍点才算占据")
    p.add_argument("--min-obstacle-area", type=int, default=30, help="连通域最小面积(格), 小于此值的孤立占据块视为噪声剔除, 0=关闭")
    p.add_argument("--padding", type=float, default=1.0, help="四周留白(米)")
    p.add_argument("--no-level", action="store_true", help="关闭自动放平(点云本就水平时用)")
    p.add_argument("--level-thresh", type=float, default=0.05, help="RANSAC 地面内点阈值(米)")
    p.add_argument("--fill-free", action="store_true", help="形态学填补空闲区小洞(需 scipy)")
    p.add_argument("--manual-tilt", type=float, default=None,help="手动指定地面与竖直夹角(度), 保留 RANSAC 检测的倾斜方向")
    return p.parse_args()

def normal_from_tilt(tilt_deg, azimuth_deg):
    """
    根据倾斜角和方位角构造一个“地面法向量”。

    这个函数用于在手动指定倾斜时，生成一个与竖直方向有偏差的法向量，
    便于点云倾斜修正时保持与 RANSAC 的倾斜方向一致。
    """
    t = np.radians(tilt_deg)
    a = np.radians(azimuth_deg)
    return np.array([
        np.sin(t) * np.cos(a),
        np.sin(t) * np.sin(a),
        np.cos(t),
    ])


def main():
    """
    主流程：
      1. 读取点云
      2. 进行地面水平化（可选）
      3. 计算相对地面高度，筛选障碍点
      4. 把 3D 点转成二维栅格
      5. 过滤孤立噪声块
      6. 导出 .pgm/.yaml 路径地图
    """
    args = parse_args()

    pts = read_pcd_xyz(args.pcd)
    if pts.size == 0:
        raise SystemExit("点云为空")
    print(f"读入 {len(pts)} 点  z原始范围:[{pts[:,2].min():.2f},{pts[:,2].max():.2f}]")

    # --- 放平: 把地面转到水平 ---
    # 这是整个脚本最关键的步骤之一。
    # 若点云本身因为传感器姿态偏移而“斜着”，那么直接投影到 XY 平面会造成地图严重畸变。
    # 因此先通过 RANSAC 拟合地面法向量，并把点云旋转到水平状态。
    if not args.no_level:
        n_auto, ground_z_ransac = fit_ground_normal(pts, thresh=args.level_thresh)
        if args.manual_tilt is not None:
            azimuth = np.degrees(np.arctan2(n_auto[1], n_auto[0]))
            n = normal_from_tilt(args.manual_tilt, azimuth)
        else:
            n = n_auto
        ang = np.degrees(np.arccos(n.dot(np.array([0., 0., 1.]))))
        pts = pts @ rotation_to_z(n).T
        print(f"地面法向量 {np.round(n,3)}  与竖直夹角 {ang:.1f}° -> 已放平  "
              f"z新范围:[{pts[:,2].min():.2f},{pts[:,2].max():.2f}]")
        if ang > 30:
            print("  ! 倾角偏大, 若结果仍乱, 可能不是单一平面地形, 检查 --level-thresh 或点云质量")

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    # --ground-percentile 参数已废弃: 地面高度改用 RANSAC 内点的中位 Z,
    # 不再依赖全局 z 百分位，以避免高层建筑或天花板干扰导致地面高度错误。
    # 不再依赖全局百分位(受天花板/高楼干扰)。参数保留兼容已有脚本, 不参与计算。
    ground_z = ground_z_ransac if not args.no_level else np.percentile(z, args.ground_percentile)
    z_rel = z - ground_z
    obstacle_mask = (z_rel >= args.obstacle_min) & (z_rel <= args.obstacle_max)
    ground_mask = np.abs(z_rel) <= args.ground_tol
    print(f"地面高度 {ground_z:.2f}  障碍点 {int(obstacle_mask.sum())}  地面点 {int(ground_mask.sum())}")

    print(f"检测到的 xyz 范围:")
    print(f"  x: [{x.min():.3f}, {x.max():.3f}]")
    print(f"  y: [{y.min():.3f}, {y.max():.3f}]")
    print(f"  z: [{z.min():.3f}, {z.max():.3f}]")
    print(f"ground_z = {ground_z:.3f}  (percentile={args.ground_percentile})")


    # --- 2D 栅格投影 ---
    # 把 3D 点投影到二维地图上，用网格计数方式估计每个格子中有多少障碍点。
    res, pad = args.resolution, args.padding
    x_min, y_min = x.min() - pad, y.min() - pad
    w = int(np.ceil((x.max() + pad - x_min) / res))
    h = int(np.ceil((y.max() + pad - y_min) / res))
    print(f"地图尺寸 {w} x {h} 格")

    def to_cell(px, py):
        """
        把世界坐标转换到地图栅格 index。
        其中：
        - col 对应 x 方向
        - row 对应 y 方向
        """
        col = np.clip(((px - x_min) / res).astype(np.int32), 0, w - 1)
        row = np.clip(((py - y_min) / res).astype(np.int32), 0, h - 1)
        return col, row

    obs_count = np.zeros((h, w), dtype=np.int32)
    free_seen = np.zeros((h, w), dtype=bool)
    # 统计每个栅格里有多少个障碍点；
    # 同时保存地面点出现的位置，便于后续将“地面”视为可通行区域。
    oc, orow = to_cell(x[obstacle_mask], y[obstacle_mask]); np.add.at(obs_count, (orow, oc), 1)
    gc, grow = to_cell(x[ground_mask], y[ground_mask]); free_seen[grow, gc] = True


    txt_path = args.out + "_grid_debug.txt"
    with open(txt_path, "w") as f:
        f.write(f"# map size: {h} x {w}\n")
        f.write("# --- obs_count ---\n")
        np.savetxt(f, obs_count, fmt="%d")
        f.write("# --- free_seen (0/1) ---\n")
        np.savetxt(f, free_seen.astype(np.int8), fmt="%d")
    print(f"已写出 {txt_path}")


    occupied = obs_count >= args.min_points

    # 连通域过滤: 剔除孤立小面积的占据块(人影、噪点等)
    # 这是对点云投影后产生的碎片噪声进行去噪的关键步骤。
    # 在 2D 栅格上做 label + 面积阈值, 不影响墙体等大面积连续障碍
    if args.min_obstacle_area > 0:
        try:
            from scipy import ndimage
            labeled, n_labels = ndimage.label(occupied)
            removed = 0
            for i in range(1, n_labels + 1):
                mask = (labeled == i)
                if mask.sum() < args.min_obstacle_area:
                    occupied[mask] = False
                    removed += 1
            if removed > 0:
                print(f"连通域过滤: 剔除 {removed} 个孤立小块 (阈值 {args.min_obstacle_area} 格)")
        except ImportError:
            print("未安装 scipy, 跳过 --min-obstacle-area")
    free = free_seen & ~occupied

    # 可选：填补空闲区域的局部小洞，减少由于采样不完整带来的空洞。
    if args.fill_free:
        try:
            from scipy import ndimage
            free = ndimage.binary_closing(free, np.ones((3, 3)), iterations=2) & ~occupied
            print("已对空闲区填洞")
        except ImportError:
            print("未安装 scipy, 跳过 --fill-free")

# ==============================================================================

    # occupied / free 合并地图: 0=占据, 1=空闲, 2=未知
    occ_free_map = np.full((h, w), 2, dtype=np.int8)
    occ_free_map[free] = 1
    occ_free_map[occupied] = 0

    txt_path = args.out + "_occ_free_map.txt"
    with open(txt_path, "w") as f:
        f.write(f"# map size: {h} x {w}\n")
        f.write("# 0=occupied  1=free  2=unknown\n")
        np.savetxt(f, occ_free_map, fmt="%d")
    print(f"已写出 {txt_path}  "
          f"占据={int((occ_free_map==0).sum())}  "
          f"空闲={int((occ_free_map==1).sum())}  "
          f"未知={int((occ_free_map==2).sum())}")

# ==============================================================================


    # PGM 采用标准的 ROS occupancy image 表示：
    # - 0: occupied
    # - 254: free
    # - 205: unknown (这里我们保留 205 作为未知值，但本脚本中通常填充为 254)
    grid = np.full((h, w), 254, dtype=np.uint8)
    grid[free] = 254
    grid[occupied] = 0
    img = np.flipud(grid)

    pgm_path, yaml_path = args.out + ".pgm", args.out + ".yaml"
    with open(pgm_path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode()); f.write(img.tobytes())
    with open(yaml_path, "w") as f:
        f.write(f"image: {os.path.basename(pgm_path)}\nresolution: {res}\n")
        f.write(f"origin: [{x_min:.4f}, {y_min:.4f}, 0.0]\n")
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

    print(f"占据 {int((grid==0).sum())}  空闲 {int((grid==254).sum())}  未知 {int((grid==205).sum())}")
    print(f"已写出 {pgm_path} 和 {yaml_path}")


if __name__ == "__main__":
    main()
#!/usr/bin/python3
"""CPU-only (numpy) implementation of the tomography pipeline.

Drop-in replacement for run_standalone.py / run_ros2.py that does NOT require
cupy or a CUDA GPU. Produces the identical pickle format consumed by the PCT
global planner (TomogramPlanner.loadTomogram / run_ros2_global_planner.py):
  data[0] = traversability cost (inflated), data[1/2] = cost gradients,
  data[3] = elevation (layers_g, nan = empty), data[4] = ceiling (layers_c),
  plus resolution / center / slice_h0 / slice_dh.

The computation replicates the three cupy kernels (tomography / trav /
inflation) and the layer-simplification loop of tomography/scripts/tomogram.py
using only numpy, so the output is numerically identical to the GPU version.

Usage:
    python3 tomogram_cpu.py --pcd /path/to/map.pcd --out /path/to/map.pickle \
        --resolution 0.10 --ground_h 0.0 --slice_dh 0.5
"""
import argparse
import pickle
import struct
import sys

import numpy as np

# ── defaults identical to tomography/config/scene.py ─────────────────────────
DEFAULT_TRAV = dict(
    kernel_size=7,
    interval_min=0.50,
    interval_free=0.65,
    slope_max=0.70,
    step_max=0.70,
    standable_ratio=0.20,
    cost_barrier=50.0,
    safe_margin=0.05,
    inflation=0.01,
)


# ── PCD loader (ascii / binary / binary_compressed), no open3d needed ───────

_NP_TYPE = {
    ('f', 4): 'f4', ('f', 8): 'f8',
    ('i', 1): 'i1', ('i', 2): 'i2', ('i', 4): 'i4', ('i', 8): 'i8',
    ('u', 1): 'u1', ('u', 2): 'u2', ('u', 4): 'u4', ('u', 8): 'u8',
}


def _lzf_decompress(src, out_len):
    """
    LZF 压缩解码，用于 PCL 的 binary_compressed PCD。

    这是 PCD 读取中最容易遇到的压缩格式之一。它和 zip/gzip 不一样，
    需要用特定的块引用方式恢复原始字节流；解码完成后才可以提取点云 xyz.
    """
    out = bytearray()
    i = 0
    n = len(src)
    while i < n and len(out) < out_len:
        ctrl = src[i]
        i += 1
        if ctrl < 32:
            length = ctrl + 1
            out += src[i:i + length]
            i += length
        else:
            length = ctrl >> 5
            if length == 7:
                length += src[i]
                i += 1
            ref_off = ((ctrl & 0x1f) << 8) + src[i]
            i += 1
            ref_pos = len(out) - ref_off - 1
            for _ in range(length + 2):
                out.append(out[ref_pos])
                ref_pos += 1
    return bytes(out)


def read_pcd(path):
    """
    读取 PCD 文件，返回 Nx3 的点云数组 [x, y, z]。

    这里支持：
    - ascii
    - binary
    - binary_compressed

    这是整个 tomography 流程的第一步。没有这一步，就无法构建地图和检测可通行区域。
    """
    with open(path, 'rb') as f:
        header = b''
        while True:
            line = f.readline()
            if not line:
                raise ValueError('unexpected EOF in PCD header')
            header += line
            if line.strip().startswith(b'DATA'):
                break
        body_start = f.tell()

    fields, size, type_, count = [], [], [], []
    width = height = points = None
    data_mode = None
    for line in header.decode('ascii', errors='replace').splitlines():
        parts = line.split()
        if not parts:
            continue
        key = parts[0].upper()
        if key == 'FIELDS':
            fields = parts[1:]
        elif key == 'SIZE':
            size = [int(v) for v in parts[1:]]
        elif key == 'TYPE':
            type_ = parts[1:]
        elif key == 'COUNT':
            count = [int(v) for v in parts[1:]]
        elif key == 'WIDTH':
            width = int(parts[1])
        elif key == 'HEIGHT':
            height = int(parts[1])
        elif key == 'POINTS':
            points = int(parts[1])
        elif key == 'DATA':
            data_mode = parts[1].lower()

    if points is None:
        points = (width or 0) * (height or 0)
    if not fields:
        raise ValueError('PCD header has no FIELDS')
    if data_mode is None:
        raise ValueError('PCD header has no DATA line')

    try:
        xyz_idx = [fields.index(name) for name in ('x', 'y', 'z')]
    except ValueError as exc:
        raise ValueError('PCD has no x/y/z fields') from exc

    def field_dtype(i):
        t = type_[i].lower() if i < len(type_) else 'f'
        s = size[i] if i < len(size) else 4
        return np.dtype(_NP_TYPE.get((t, s), 'f4'))

    with open(path, 'rb') as f:
        f.seek(body_start)
        if data_mode == 'ascii':
            rows = np.loadtxt(f)
            if rows.ndim == 1:
                rows = rows.reshape(1, -1)
            xyz = np.ascontiguousarray(rows[:, xyz_idx], dtype=np.float32)
        elif data_mode == 'binary':
            # binary PCD is interleaved (AOS): [x0,y0,z0,i0,...][x1,y1,z1,i1,...]
            # Must read as a structured dtype — NOT field-by-field (that's SOA,
            # which is only correct for binary_compressed after decompression).
            names_dt, fmts_dt = [], []
            for i, fn in enumerate(fields):
                ct = count[i] if i < len(count) else 1
                base = str(field_dtype(i))
                fmts_dt.append(base if ct == 1 else (base, ct))
                names_dt.append(fn)
            dt = np.dtype({'names': names_dt, 'formats': fmts_dt})
            rec = np.frombuffer(f.read(dt.itemsize * points), dtype=dt, count=points)
            xyz = np.column_stack([rec['x'].astype(np.float32),
                                   rec['y'].astype(np.float32),
                                   rec['z'].astype(np.float32)])
        elif data_mode == 'binary_compressed':
            xyz = np.empty((points, 3), np.float32)
            for c, field in enumerate(fields):
                csize, usize = struct.unpack('<II', f.read(8))
                compressed = f.read(csize)
                raw = _lzf_decompress(compressed, usize)
                if len(raw) != usize:
                    raise ValueError(f'LZF decompress mismatch for {field}: '
                                     f'{len(raw)} != {usize}')
                col = np.frombuffer(raw, dtype=field_dtype(c))
                if field in ('x', 'y', 'z'):
                    xyz[:, ('x', 'y', 'z').index(field)] = col
        else:
            raise ValueError(f'unsupported PCD DATA mode: {data_mode}')

    print(f'[INFO] {path}: {points} points, DATA={data_mode}, '
          f'FIELDS={fields}', file=sys.stderr)
    return xyz


def voxel_downsample(points, voxel_size):
    """
    对点云做 voxel downsample。

    作用是把离散点浓密区域压缩成一个代表点，能降低数据量，
    特别适合大规模地图处理；
    当 voxel_size <= 0 时，表示禁用下采样。
    """
    if voxel_size <= 0:
        return points
    q = np.floor(points[:, :3] / voxel_size)
    _, uniq = np.unique(q, axis=0, return_index=True)
    print(f'[INFO] voxel downsample {voxel_size} m: '
          f'{points.shape[0]} -> {uniq.shape[0]} points', file=sys.stderr)
    return points[uniq]


# ── pure-numpy tomography pipeline ───────────────────────────────────────────

def _round_half_away(x):
    """
    模拟 C 语言中的 round() 行为：四舍五入时远离 0。

    在把点云投影到地图网格时，必须保证坐标四舍五入的行为与 GPU/CUDA 版本一致，
    否则 voxel 对齐会出现偏差，导致两套实现输出不一致。
    """
    return np.copysign(np.floor(np.abs(x) + 0.5), x)


def _box_filter_sum(a, half):
    """
    计算二维窗口内的加和，类似 box filter / sliding sum。

    这个函数用于统计某个局部区域中满足条件的格子数量，例如：
    评估一个点附近有多少个可站立区域；
    它是 traversability 计算里关键的“局部平滑”步骤。
    """
    a = a.astype(np.float32)
    H, W = a.shape[-2], a.shape[-1]
    ii = np.zeros((a.shape[0], H + 1, W + 1), np.float32)
    ii[:, :-1, :-1] = a
    ii = ii.cumsum(axis=1).cumsum(axis=2)

    top = np.clip(np.arange(H) - half, 0, H)
    bot = np.clip(np.arange(H) + half + 1, 0, H)
    lft = np.clip(np.arange(W) - half, 0, W)
    rgt = np.clip(np.arange(W) + half + 1, 0, W)

    s_top = ii[:, top]          # (n_slice, H, W+1)
    s_bot = ii[:, bot]
    return (s_bot[:, :, rgt] - s_top[:, :, rgt]
            - s_bot[:, :, lft] + s_top[:, :, lft])


def build_tomogram(points, resolution, ground_h, slice_dh, trav):
    """
    真正的体素地图构建主函数。

    该函数从点云出发，执行以下关键步骤：
    1. 归一化点云位置到地图坐标系。
    2. 通过层切片统计每个 voxel 的 max/min 高度值。
    3. 根据高度间隔计算 ground / ceiling 层。
    4. 计算 traversability cost 与 inflation cost。
    5. 执行 layer simplification，减少冗余层。
    6. 输出与 GPU 版本完全兼容的字典格式。
    """
    points = np.asarray(points, dtype=np.float32)
    points = points[~np.isnan(points).any(axis=1)]

    points_min = np.min(points, axis=0)
    points_max = np.max(points, axis=0)
    points_min[-1] = ground_h
    dim_x = int(np.ceil((points_max[0] - points_min[0]) / resolution)) + 4
    dim_y = int(np.ceil((points_max[1] - points_min[1]) / resolution)) + 4
    n_slice_init = int(np.ceil((points_max[2] - points_min[2]) / slice_dh))
    if dim_x <= 0 or dim_y <= 0 or n_slice_init <= 0:
        raise ValueError(f'degenerate map extent: {dim_x}x{dim_y}, '
                         f'{n_slice_init} slices')
    center = (points_max[:2] + points_min[:2]) / 2
    slice_h0 = points_min[-1] + slice_dh

    extent = points_max - points_min
    est_gb = n_slice_init * dim_x * dim_y * 4 * 7 / 1e9
    print(f'[INFO] extent(m)=({extent[0]:.1f}x{extent[1]:.1f}x{extent[2]:.1f}) '
          f'grid={dim_x}x{dim_y} slices={n_slice_init} '
          f'est. peak ~{est_gb:.1f} GB', file=sys.stderr)

    half_trav_k = int(trav['kernel_size'] / 2)
    step_stand = float(1.2 * resolution * np.tan(trav['slope_max']))
    step_cross = float(trav['step_max'])
    standable_th = int(trav['standable_ratio'] * (2 * half_trav_k + 1) ** 2) - 1
    cost_barrier = float(trav['cost_barrier'])
    half_inf_k = int((trav['safe_margin'] + trav['inflation']) / resolution)

    # weight table for the inflation kernel (same as initKernel)
    inf = np.zeros((2 * half_inf_k + 1, 2 * half_inf_k + 1), np.float32)
    for i in range(inf.shape[0]):
        for j in range(inf.shape[1]):
            dist = np.sqrt((resolution * (i - half_inf_k)) ** 2 +
                           (resolution * (j - half_inf_k)) ** 2)
            inf[i, j] = np.clip(
                1.0 - (dist - trav['inflation']) /
                (trav['safe_margin'] + resolution), 0.0, 1.0)

    # ── tomography kernel: voxelize points into layered max/min heights ──
    cx, cy = center
    ix = _round_half_away((points[:, 0] - cx) / resolution) + dim_x // 2
    iy = _round_half_away((points[:, 1] - cy) / resolution) + dim_y // 2
    valid = (ix >= 0) & (ix < dim_x) & (iy >= 0) & (iy < dim_y)
    idx = (ix.astype(np.int64) * dim_y + iy.astype(np.int64))[valid]
    pz = points[valid, 2].astype(np.float32)

    layers_g = np.full((n_slice_init, dim_x, dim_y), -1e6, np.float32)
    layers_c = np.full((n_slice_init, dim_x, dim_y), 1e6, np.float32)
    for s in range(n_slice_init):
        sl = slice_h0 + s * slice_dh
        low = pz <= sl
        high = ~low
        if low.any():
            np.maximum.at(layers_g[s].ravel(), idx[low], pz[low])
        if high.any():
            np.minimum.at(layers_c[s].ravel(), idx[high], pz[high])
    print(f'[INFO] map center={center}, dim={dim_x}x{dim_y}, '
          f'slices={n_slice_init}', file=sys.stderr)

    # ── ground/ceiling gradient (grad_mag_sq, grad_mag_max) ─────────────
    diff_x_sq = np.maximum(
        (layers_g[:, 1:-1, :] - layers_g[:, :-2, :]) ** 2,
        (layers_g[:, 1:-1, :] - layers_g[:, 2:, :]) ** 2)
    diff_y_sq = np.maximum(
        (layers_g[:, :, 1:-1] - layers_g[:, :, :-2]) ** 2,
        (layers_g[:, :, 1:-1] - layers_g[:, :, 2:]) ** 2)
    grad_mag_sq = np.zeros((n_slice_init, dim_x, dim_y), np.float32)
    grad_mag_max = np.zeros((n_slice_init, dim_x, dim_y), np.float32)
    grad_mag_sq[:, 1:-1, 1:-1] = diff_x_sq[:, :, 1:-1] + diff_y_sq[:, 1:-1, :]
    grad_mag_max[:, 1:-1, 1:-1] = np.maximum(
        diff_x_sq[:, :, 1:-1], diff_y_sq[:, 1:-1, :])

    interval = layers_c - layers_g

    # ── traversability kernel ────────────────────────────────────────────
    trav_cost = np.zeros((n_slice_init, dim_x, dim_y), np.float32)
    nbar = interval >= trav['interval_min']          # cells not early-returned
    add = np.maximum(0.0, 20.0 * (trav['interval_free'] - interval))
    add[~nbar] = 0.0
    trav_cost[~nbar] = cost_barrier
    trav_cost[nbar] += add[nbar]

    step_stand_sq = step_stand ** 2
    step_cross_sq = step_cross ** 2
    stand_count = _box_filter_sum(grad_mag_sq < step_stand_sq, half_trav_k)

    cond1 = nbar & (grad_mag_sq <= step_stand_sq)
    cond2 = nbar & (grad_mag_sq > step_stand_sq) & (grad_mag_max <= step_cross_sq)
    cond3 = nbar & (grad_mag_sq > step_stand_sq) & (grad_mag_max > step_cross_sq)

    trav_cost[cond1] += 15.0 * grad_mag_sq[cond1] / step_stand_sq
    good2 = cond2 & (stand_count >= standable_th)
    bad2 = cond2 & ~good2
    trav_cost[bad2] = cost_barrier
    trav_cost[good2] += 20.0 * grad_mag_max[good2] / step_cross_sq
    trav_cost[cond3] = cost_barrier

    # ── inflation kernel (max over weighted window) ──────────────────────
    padded = np.pad(trav_cost,
                    ((0, 0), (half_inf_k, half_inf_k), (half_inf_k, half_inf_k)),
                    constant_values=-np.inf)
    inflated = np.full_like(trav_cost, -np.inf)
    for dx in range(-half_inf_k, half_inf_k + 1):
        for dy in range(-half_inf_k, half_inf_k + 1):
            sh = padded[:, half_inf_k + dx: half_inf_k + dx + dim_x,
                        half_inf_k + dy: half_inf_k + dy + dim_y]
            inflated = np.maximum(inflated, sh * inf[dy + half_inf_k,
                                                     dx + half_inf_k])
    inflated[~np.isfinite(inflated)] = cost_barrier

    # ── layer simplification (identical loop to the cupy version) ────────
    idx_simp = [0]
    if n_slice_init > 1:
        l_idx, m_idx = 0, 1
        diff_h = layers_g[1:] - layers_g[:-1]
        while m_idx < n_slice_init - 2:
            mask_l_g = layers_g[m_idx] - layers_g[l_idx] > 0
            mask_l_t = inflated[l_idx] > inflated[m_idx]
            mask_u_g = diff_h[m_idx] > 0
            mask_t = inflated[m_idx] < cost_barrier
            unique = (mask_l_g | mask_l_t) & mask_u_g & mask_t
            if np.any(unique):
                idx_simp.append(m_idx)
                l_idx = m_idx
            m_idx += 1
        idx_simp.append(m_idx)
    idx_simp = np.asarray(idx_simp, np.int64)
    print(f'[INFO] simplified to {idx_simp.shape[0]} layers', file=sys.stderr)

    # ── assemble output (identical dict to exportTomogram) ───────────────
    layers_t = inflated[idx_simp]
    layers_g_sel = layers_g[idx_simp]
    layers_c_sel = layers_c[idx_simp]
    layers_g_out = np.where(layers_g_sel > -1e6, layers_g_sel, np.nan).astype(np.float32)
    layers_c_out = np.where(layers_c_sel < 1e6, layers_c_sel, np.nan).astype(np.float32)

    trav_grad_x = inflated[idx_simp][:, 2:, :] - inflated[idx_simp][:, :-2, :]
    trav_grad_y = inflated[idx_simp][:, :, 2:] - inflated[idx_simp][:, :, :-2]
    trav_gx = np.zeros_like(layers_g_out)
    trav_gx[:, 1:-1, :] = trav_grad_x
    trav_gy = np.zeros_like(layers_g_out)
    trav_gy[:, :, 1:-1] = trav_grad_y

    data = np.stack((layers_t, trav_gx, trav_gy, layers_g_out, layers_c_out)) \
             .astype(np.float16)
    return {
        'data': data,
        'resolution': float(resolution),
        'center': np.asarray(center, dtype=np.float32),
        'slice_h0': float(slice_h0),
        'slice_dh': float(slice_dh),
    }


def main():
    """
    命令行入口：
    将 PCD 点云转换为 tomogram pickle 文件，供全局规划器使用。

    输出格式与 GPU 版完全一致，因此直接替换实现即可。
    """
    ap = argparse.ArgumentParser(
        description='CPU-only tomography: PCD -> tomogram pickle (no GPU)')
    ap.add_argument('--pcd', required=True, help='input PCD map')
    ap.add_argument('--out', required=True, help='output .pickle path')
    ap.add_argument('--resolution', type=float, default=0.10,
                    help='grid cell size [m] (default 0.10)')
    ap.add_argument('--ground_h', type=float, default=0.0,
                    help='ground plane height [m] (default 0.0)')
    ap.add_argument('--slice_dh', type=float, default=0.5,
                    help='height slice thickness [m] (default 0.5)')
    ap.add_argument('--voxel', type=float, default=0.0,
                    help='voxel downsampling size [m] (0 = off; '
                         'recommend ~resolution for huge clouds)')
    args = ap.parse_args()

    points = read_pcd(args.pcd)
    if args.voxel <= 0 and points.shape[0] > 10_000_000:
        print(f'[WARN] {points.shape[0]} points is very large; add '
              f'--voxel {args.resolution:.2f} to downsample, otherwise this '
              f'may run out of memory', file=sys.stderr)
    points = voxel_downsample(points, args.voxel)
    data_dict = build_tomogram(
        points, args.resolution, args.ground_h, args.slice_dh, DEFAULT_TRAV)
    with open(args.out, 'wb') as f:
        pickle.dump(data_dict, f, protocol=pickle.HIGHEST_PROTOCOL)
    print(f'[INFO] exported {args.out}: '
          f'data={data_dict["data"].shape} dtype={data_dict["data"].dtype}')


if __name__ == '__main__':
    main()

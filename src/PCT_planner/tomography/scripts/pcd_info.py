#!/usr/bin/env python3
"""Print the XYZ range (min/max/extent) of a PCD file.

Usage:
    python3 pcd_info.py /path/to/map.pcd
    python3 pcd_info.py /path/to/map.pcd --voxel 0.10   # downsample before stat
"""
import argparse
import struct
import sys

import numpy as np


# ── PCD reader (ascii / binary / binary_compressed) ──────────────────────────
# Replicates the reader in tomogram_cpu.py — no Open3D needed.

_NP_TYPE = {
    ('f', 4): 'f4', ('f', 8): 'f8',
    ('i', 1): 'i1', ('i', 2): 'i2', ('i', 4): 'i4', ('i', 8): 'i8',
    ('u', 1): 'u1', ('u', 2): 'u2', ('u', 4): 'u4', ('u', 8): 'u8',
}


def _lzf_decompress(src, out_len):
    out = bytearray()
    i = 0
    n = len(src)
    while i < n and len(out) < out_len:
        ctrl = src[i]; i += 1
        if ctrl < 32:
            length = ctrl + 1
            out += src[i:i + length]; i += length
        else:
            length = ctrl >> 5
            if length == 7:
                length += src[i]; i += 1
            ref_off = ((ctrl & 0x1f) << 8) + src[i]; i += 1
            ref_pos = len(out) - ref_off - 1
            for _ in range(length + 2):
                out.append(out[ref_pos]); ref_pos += 1
    return bytes(out)


def read_pcd(path):
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
        if key == 'FIELDS':    fields = parts[1:]
        elif key == 'SIZE':    size = [int(v) for v in parts[1:]]
        elif key == 'TYPE':    type_ = parts[1:]
        elif key == 'COUNT':   count = [int(v) for v in parts[1:]]
        elif key == 'WIDTH':   width = int(parts[1])
        elif key == 'HEIGHT':  height = int(parts[1])
        elif key == 'POINTS':  points = int(parts[1])
        elif key == 'DATA':    data_mode = parts[1].lower()

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
                    raise ValueError(
                        f'LZF decompress mismatch for {field}: {len(raw)} != {usize}')
                col = np.frombuffer(raw, dtype=field_dtype(c))
                if field in ('x', 'y', 'z'):
                    xyz[:, ('x', 'y', 'z').index(field)] = col
        else:
            raise ValueError(f'unsupported PCD DATA mode: {data_mode}')

    # Remove NaN/Inf points
    valid = np.isfinite(xyz).all(axis=1)
    n_invalid = int((~valid).sum())
    xyz = xyz[valid]
    return xyz, n_invalid, data_mode, fields


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description='Print XYZ range of a PCD file.')
    ap.add_argument('pcd', help='path to .pcd file')
    ap.add_argument('--voxel', type=float, default=0.0,
                    help='optional voxel downsample size [m] before computing stats (0=off)')
    args = ap.parse_args()

    print(f'File : {args.pcd}')
    xyz, n_invalid, data_mode, fields = read_pcd(args.pcd)
    print(f'DATA : {data_mode}   FIELDS : {fields}')
    print(f'Points (total): {xyz.shape[0] + n_invalid}')
    if n_invalid:
        print(f'  NaN/Inf removed: {n_invalid}')
    print(f'Points (valid): {xyz.shape[0]}')

    if args.voxel > 0:
        q = np.floor(xyz / args.voxel)
        _, uniq = np.unique(q, axis=0, return_index=True)
        xyz = xyz[uniq]
        print(f'Points (after voxel {args.voxel} m): {xyz.shape[0]}')

    mn = xyz.min(axis=0)
    mx = xyz.max(axis=0)
    extent = mx - mn
    center = (mn + mx) / 2

    print()
    print(f'{"":6}  {"min":>12}  {"max":>12}  {"extent":>12}  {"center":>12}')
    print(f'{"":6}  {"────────────":>12}  {"────────────":>12}  {"────────────":>12}  {"────────────":>12}')
    for axis, i in (('X', 0), ('Y', 1), ('Z', 2)):
        print(f'{axis:6}  {mn[i]:12.4f}  {mx[i]:12.4f}  {extent[i]:12.4f}  {center[i]:12.4f}')
    print()
    print(f'Suggested tomogram_cpu.py args:')
    print(f'  --resolution 0.10 --ground_h {mn[2]:.4f} --slice_dh 0.5')


if __name__ == '__main__':
    main()

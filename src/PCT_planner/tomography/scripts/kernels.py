import string
import cupy as cp


def utils_point(resolution, n_row, n_col):
    """
    2D 点到地图索引转换所需的 CUDA helper 函数。

    作用：
    - 把 (x, y) 转成网格索引
    - 把图层索引和 2D index 组合成 multi-layer 中的一维索引
    - 提供 atomicMax / atomicMin，用于更稳地更新 ground / ceiling
    """
    util_preamble = string.Template(
        '''
        __device__ int getIndexLine(float x, float center)
        {
            int i = round((x - center) / ${resolution});
            return i;
        }

        __device__ int getIndexMap_1d(float x, float y, float cx, float cy)
        {
            // Return 1D index of a point (x, y) in a layer
            int idx_x = getIndexLine(x, cx) + ${n_row} / 2;
            int idx_y = getIndexLine(y, cy) + ${n_col} / 2;

            // Check if the index is inside the map
            if (idx_x < 0 || idx_x >= ${n_row} || idx_y < 0 || idx_y >= ${n_col})
            {
                return -1;
            }
            return ${n_col} * idx_x + idx_y;
        }

        __device__ int getIndexBlock_1d(int idx, int layer_n)
        {
            // Return 1D index of a point (x, y) in multi-layer map block
            return (int)${layer_size} * layer_n + idx;
        }

        __device__ static float atomicMaxFloat(float* address, float val)
        {
            int* address_as_i = (int*) address;
            int old = *address_as_i, assumed;
            do {
                assumed = old;
                old = ::atomicCAS(address_as_i, assumed,
                    __float_as_int(::fmaxf(val, __int_as_float(assumed))));
            } while (assumed != old);

            return __int_as_float(old);
        }

        __device__ static float atomicMinFloat(float* address, float val)
        {
            int* address_as_i = (int*) address;
            int old = *address_as_i, assumed;
            do {
                assumed = old;
                old = ::atomicCAS(address_as_i, assumed,
                    __float_as_int(::fminf(val, __int_as_float(assumed))));
            } while (assumed != old);

            return __int_as_float(old);
        }
        '''
    ).substitute(
        resolution=resolution,
        n_row=n_row, 
        n_col=n_col,
        layer_size=n_row*n_col
    )

    return util_preamble


def utils_map(n_row, n_col):
    """
    2D/3D 局部邻域访问辅助函数。

    主要用于 inflation 和局部邻域遍历，
    例如查看某个点周围的滤波窗口，计算邻域成本或是否可站立。
    """
    util_preamble=string.Template(
        '''
        __device__ int getIdxRelative(int idx, int dx, int dy) 
        {
            // Return 1D index of the relative point (x+dx, y+dy) in multi-layer map block
            int idx_2d = idx % (int)${layer_size};
            int idx_x = idx_2d / ${n_col};
            int idx_y = idx_2d % ${n_col};
            int idx_rx = idx_x + dx;
            int idx_ry = idx_y + dy;

            if ( idx_rx < 0 || idx_rx > (${n_row} - 1) ) 
                return -1;
            if ( idx_ry < 0 || idx_ry > (${n_col} - 1) )
                return -1;

            return ${n_col} * dx + dy + idx;
        }
        '''
    ).substitute(
        n_row=n_row, 
        n_col=n_col,
        layer_size=n_row*n_col
    )

    return util_preamble


def tomographyKernel(resolution, n_row, n_col, n_slice, slice_h0, slice_dh):
    """
    将每个 3D 点映射到不同高度层，并维护每个 voxel 的：
    - ground: 该层中最低/最高的地面高度
    - ceiling: 该层中最高的天花板高度

    也就是每个体素位置在各层里可以看到一个“地形上界/下界”。
    """
    tomography_kernel = cp.ElementwiseKernel(
        in_params='raw U points, raw U center',
        out_params='raw U layers_g, raw U layers_c',
        preamble=utils_point(resolution, n_row, n_col),
        operation=string.Template(
            '''
            U px = points[i * 3];
            U py = points[i * 3 + 1];
            U pz = points[i * 3 + 2];

            int idx = getIndexMap_1d(px, py, center[0], center[1]);
            if ( idx < 0 ) 
                return; 
            for ( int s_idx = 0; s_idx < ${n_slice}; s_idx ++ )
            {
                U slice = ${slice_h0} + s_idx * ${slice_dh};
                if ( pz <= slice )
                    atomicMaxFloat(&layers_g[getIndexBlock_1d(idx, s_idx)], pz);
                else
                    atomicMinFloat(&layers_c[getIndexBlock_1d(idx, s_idx)], pz);
            }
            '''
        ).substitute(
            n_slice=n_slice,
            slice_h0=slice_h0,
            slice_dh=slice_dh
        ),
        name='tomography_kernel'
    )
                            
    return tomography_kernel


def travKernel(
    n_row, n_col, half_kernel_size,
    interval_min, interval_free, step_cross, step_stand, standable_th, cost_barrier
    ):
    """
    traversability kernel.

    基于高度层间距、坡度和局部可站立性来计算每个栅格的代价，
    它决定了地图上哪些位置更容易走、哪些位置更危险。
    """
    trav_kernel = cp.ElementwiseKernel(
        in_params='raw U interval, raw U grad_mag_sq, raw U grad_mag_max',
        out_params='raw U trav_cost',
        preamble=utils_map(n_row, n_col),
        operation=string.Template(
            '''
            if ( interval[i] < ${interval_min} )
            {
                trav_cost[i] = ${cost_barrier};
                return;
            }
            else
                trav_cost[i] += max(0.0, 20 * (${interval_free} - interval[i]));
            if ( grad_mag_sq[i] <= ${step_stand_sq} )
            {
                trav_cost[i] += 15 * grad_mag_sq[i] / ${step_stand_sq};
                return;
            }
            else 
            {
                if ( grad_mag_max[i] <= ${step_cross_sq} )
                {
                    int standable_grids = 0;
                    for ( int dy = -${half_kernel_size}; dy <= ${half_kernel_size}; dy++ ) 
                    {
                        for ( int dx = -${half_kernel_size}; dx <= ${half_kernel_size}; dx++ ) 
                        {
                            int idx = getIdxRelative(i, dx, dy);
                            if ( idx < 0 )
                                continue;
                            if ( grad_mag_sq[idx] < ${step_stand_sq} )
                                standable_grids += 1;
                        }
                    }
                    if ( standable_grids < ${standable_th} )
                    {
                        trav_cost[i] = ${cost_barrier};
                        return;
                    }
                    else
                        trav_cost[i] += 20 * grad_mag_max[i] / ${step_cross_sq};
                }
                else
                {
                    trav_cost[i] = ${cost_barrier};
                    return;
                }
            } 
            '''
        ).substitute(
            half_kernel_size=half_kernel_size,
            interval_min=interval_min,
            interval_free=interval_free,
            step_cross_sq=step_cross ** 2,
            step_stand_sq=step_stand ** 2,
            standable_th=standable_th,
            cost_barrier=cost_barrier
        ),
        name='trav_kernel'
    )
                            
    return trav_kernel


def inflationKernel(n_row, n_col, half_kernel_size):
    """
    对 traversability cost 做膨胀处理，
    让障碍临近区域也获得更高代价以保证安全裕度。
    """
    inflation_kernel = cp.ElementwiseKernel(
        in_params='raw U trav_cost, raw U score_table',
        out_params='raw U inflated_cost',
        preamble=utils_map(n_row, n_col),
        operation=string.Template(
            '''
            int counter = 0;
            for ( int dy = -${half_kernel_size}; dy <= ${half_kernel_size}; dy++ ) 
            {
                for ( int dx = -${half_kernel_size}; dx <= ${half_kernel_size}; dx++ ) 
                {
                    int idx = getIdxRelative(i, dx, dy);
                    if ( idx >= 0 )
                        inflated_cost[i] = max(inflated_cost[i], trav_cost[idx] * score_table[counter]);
                    counter += 1;
                }
            }
            '''
        ).substitute(
            half_kernel_size=half_kernel_size
        ),
        name='inflation_kernel'
    )
                            
    return inflation_kernel
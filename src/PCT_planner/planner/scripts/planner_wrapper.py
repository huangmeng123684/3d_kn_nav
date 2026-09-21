import os
import sys
import pickle
import numpy as np
from scipy.ndimage import distance_transform_edt, maximum_filter

from utils import *

sys.path.append('../')
from lib import a_star, ele_planner, traj_opt

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


class TomogramPlanner(object):
    """
    TomogramPlanner
    --------------
    这是 PCT（Probability/Cost Tomogram）路径规划器的核心封装类。

    它负责以下几部分工作：
    1. 读取 tomogram（地形/高度/可 traversable 成本）数据。
    2. 构建层级地图与 gateway 约束。
    3. 计算 clearance cost，提升规划在边界附近的安全性。
    4. 调用底层 A* / GPMP/trajectory optimizer 进行路径搜索与平滑。
    5. 把优化后的结果转换回地图坐标系，输出 3D 轨迹。

    整个类属于 planner 模块中的核心算法封装层，负责把原始体数据
    转成可导航 traj。
    """

    def __init__(self, cfg):
        self.cfg = cfg

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate
        self.a_star_cost_threshold = self.cfg.planner.a_star_cost_threshold
        self.step_cost_weight = self.cfg.planner.step_cost_weight
        self.optimizer_cost_threshold = self.cfg.planner.optimizer_cost_threshold
        self.use_clearance_cost = self.cfg.planner.use_clearance_cost
        self.clearance_cost_mode = getattr(
            self.cfg.planner, 'clearance_cost_mode', 'absolute'
        )
        self.clearance_cost_weight = self.cfg.planner.clearance_cost_weight
        self.clearance_cost_decay = self.cfg.planner.clearance_cost_decay
        self.clearance_cost_local_radius = getattr(
            self.cfg.planner, 'clearance_cost_local_radius', 1.0
        )
        self.clearance_cost_cap = getattr(
            self.cfg.planner, 'clearance_cost_cap',
            self.clearance_cost_weight
        )

        self.tomo_dir = rsg_root + self.cfg.wrapper.tomo_dir

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)
        self.elev_g = None
        self.raw_trav = None
        self.planning_trav = None
        self.clearance = None
        self.last_astar_traj = None

    def loadTomogram(self, tomo_file):
        """
        加载 tomogram 数据文件。

        数据文件中通常包含：
        - data: 体数据（多层切片）
        - resolution: 每个 voxel 的真实距离
        - center: 地图中心点
        - slice_h0 / slice_dh: 层切片高度参数

        这个函数会将体数据拆分为：
        - trav: traversability / cost map
        - elev_g: 地形高度场
        - elev_c: 与优化相关的高度/代价信息
        然后调用 initPlanner 完成地图准备工作。
        """
        with open(self.tomo_dir + tomo_file + '.pickle', 'rb') as handle:
            data_dict = pickle.load(handle)

            tomogram = np.asarray(data_dict['data'], dtype=np.float32)

            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            self.n_slice = tomogram.shape[1]
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        trav = tomogram[0]
        trav_gx = tomogram[1]
        trav_gy = tomogram[2]
        self.raw_trav = trav.copy()
        elev_g_raw = tomogram[3]
        self.elev_g = elev_g_raw.copy()
        elev_g = np.nan_to_num(elev_g_raw, nan=-100)
        elev_c = tomogram[4]
        elev_c = np.nan_to_num(elev_c, nan=1e6)

        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)
        
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        """
        初始化规划器的地图表示。

        这里有两个关键步骤：
        1. 计算 gateway：识别层间发生明显高度/通行性变化的位置，用于
           约束 A* 或优化器跨层搜索时的“门槛”/“过渡区域”。
        2. 将地图 cost 和梯度加入 clearance cost，并传给底层 ele_planner.

        这样做的目的是让规划器不仅能搜索一条路径，还能在地形边界、
        斜坡和高低差较大的区域附近保留更合理的安全裕度。
        """
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        planning_trav, planning_gx, planning_gy = self.add_clearance_cost(
            trav, trav_gx, trav_gy
        )
        self.planning_trav = planning_trav

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        try:
            self.planner.init_map(
                self.a_star_cost_threshold,
                self.optimizer_cost_threshold,
                self.resolution,
                self.n_slice,
                self.step_cost_weight,
                trav.reshape(-1, trav.shape[-1]).astype(np.double),
                planning_trav.reshape(-1, planning_trav.shape[-1]).astype(np.double),
                elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
                elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
                gateway.reshape(-1, gateway.shape[-1]),
                trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
                -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double)
            )
        except TypeError:
            self.planner.init_map(
                self.a_star_cost_threshold,
                self.optimizer_cost_threshold,
                self.resolution,
                self.n_slice,
                self.step_cost_weight,
                trav.reshape(-1, trav.shape[-1]).astype(np.double),
                elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
                elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
                gateway.reshape(-1, gateway.shape[-1]),
                trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
                -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double)
            )

    def add_clearance_cost(self, trav, trav_gx, trav_gy):
        """
        为可通行成本图添加 clearance cost（安全距离惩罚）。

        作用：
        - 对接近障碍物、边界和低可通行区域的点增加代价；
        - 引导规划器优先走更“安全”的路径；
        - 避免路径贴着危险边界运行。

        这里使用的是距离变换：
        distance_transform_edt 会给每个点计算到最近不可通行区域的距离，
        距离越小，说明该点越靠近障碍，因此罚值越大。

        mode 还支持两种策略：
        - absolute: 绝对距离衰减惩罚
        - relative: 相对局部窗口的清晰度惩罚
        """
        planning_trav = trav.copy()
        planning_gx = trav_gx.copy()
        planning_gy = trav_gy.copy()
        self.clearance = np.zeros_like(trav, dtype=np.float32)

        if not self.use_clearance_cost or self.clearance_cost_weight <= 0.0:
            return planning_trav, planning_gx, planning_gy

        traversable = np.isfinite(trav) & (trav <= self.a_star_cost_threshold)
        clearance_free = np.isfinite(trav) & (trav <= self.optimizer_cost_threshold)
        for layer in range(trav.shape[0]):
            self.clearance[layer] = distance_transform_edt(
                clearance_free[layer]
            ).astype(np.float32) * self.resolution

        mode = str(self.clearance_cost_mode).lower()
        if mode == 'absolute':
            if self.clearance_cost_decay <= 0.0:
                raise ValueError('clearance_cost_decay must be greater than zero')
            clearance_cost = self.clearance_cost_weight * np.exp(
                -self.clearance / self.clearance_cost_decay
            )
        elif mode == 'relative':
            local_radius_cells = max(
                1,
                int(np.ceil(self.clearance_cost_local_radius / self.resolution))
            )
            window = 2 * local_radius_cells + 1
            local_width = maximum_filter(
                self.clearance,
                size=(1, window, window),
                mode='nearest'
            )
            relative_clearance = self.clearance / (local_width + 1e-6)
            relative_clearance = np.clip(relative_clearance, 0.0, 1.0)
            clearance_cost = self.clearance_cost_weight * (
                1.0 - relative_clearance
            ) ** 2
            clearance_cost = np.clip(
                clearance_cost,
                0.0,
                self.clearance_cost_cap
            )
        else:
            raise ValueError(
                "clearance_cost_mode must be 'relative' or 'absolute'"
            )

        clearance_cost[~traversable] = 0.0
        planning_trav += clearance_cost

        clearance_gx = np.zeros_like(clearance_cost)
        clearance_gy = np.zeros_like(clearance_cost)
        clearance_gx[:, 1:-1, :] = (
            clearance_cost[:, 2:, :] - clearance_cost[:, :-2, :]
        )
        clearance_gy[:, :, 1:-1] = (
            clearance_cost[:, :, 2:] - clearance_cost[:, :, :-2]
        )
        planning_gx += clearance_gx
        planning_gy += clearance_gy

        return planning_trav, planning_gx, planning_gy

    def plan(self, start_pos, end_pos):
        """
        总体规划入口函数。

        流程分为三段：
        1. 把起点/终点转成地图索引，并设置 A* 搜索起末点。
        2. 调用底层 planner.plan 做全局/层级搜索，拿到 A* 路径。
        3. 对 A* 路径做优化与高度重采样，输出最终 3D 轨迹。

        输出为 Nx3 的轨迹数组，通常每行为 [x, y, z]。
        """
        self.last_astar_traj = None
        self.start_idx[0] = self.pos2layer(start_pos)
        self.end_idx[0] = self.pos2layer(end_pos)
        self.start_idx[1:] = self.pos2idx(start_pos[:2])
        self.end_idx[1:] = self.pos2idx(end_pos[:2])

        print("Start idx:", self.start_idx, "End idx:", self.end_idx)

        plan_success = self.planner.plan(self.start_idx, self.end_idx, True)
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if not plan_success or len(path) == 0:
            return None
        self.last_astar_traj = self.astar_path_to_map(path)

        if len(path) == 1:
            start_pos = np.asarray(start_pos, dtype=np.float64)
            end_pos = np.asarray(end_pos, dtype=np.float64)
            if np.linalg.norm(end_pos - start_pos) <= np.finfo(np.float64).eps:
                return end_pos.reshape(1, 3)
            return np.stack([start_pos, end_pos])

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()

        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        heights = self.sample_traj_heights(
            layers, traj[:, 0], traj[:, y_idx], heights
        )
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)

        return traj_3d

    def getLastAstarPath(self):
        return self.last_astar_traj

    def sample_traj_heights(self, layers, cols, rows, fallback_heights):
        """
        根据优化后的 XY 位置重新采样高度（z）值。

        这是一个关键修正：
        优化器输出的轨迹可能存在高度过平、或未准确贴合地形的情况。
        因此我们在每个轨迹点附近，从 elev_g（地形高程图）中寻找最接近
        的高程值作为真实 z，避免路径“飘在空中”或“压在地面下”。
        """
        sampled = np.asarray(fallback_heights, dtype=np.float64).copy()
        if self.elev_g is None:
            return sampled

        layers = np.rint(layers).astype(np.int32)
        rows = np.rint(rows).astype(np.int32)
        cols = np.rint(cols).astype(np.int32)
        for i in range(sampled.shape[0]):
            h = self.nearest_elevation(layers[i], rows[i], cols[i])
            if h is not None:
                sampled[i] = h
        return sampled

    def nearest_elevation(self, layer, row, col, search_radius=2):
        """
        在局部窗口内搜索最接近的地形高程值。

        如果当前点的 elev_g 已经有有效值，就直接返回；
        否则在附近半径范围内搜索最靠近的高程点，确保轨迹高度连续。
        """
        layer = int(np.clip(layer, 0, self.n_slice - 1))
        row = int(np.clip(row, 0, self.map_dim[0] - 1))
        col = int(np.clip(col, 0, self.map_dim[1] - 1))

        h = self.elev_g[layer, row, col]
        if np.isfinite(h):
            return float(h)

        x0 = max(0, row - search_radius)
        x1 = min(self.map_dim[0], row + search_radius + 1)
        y0 = max(0, col - search_radius)
        y1 = min(self.map_dim[1], col + search_radius + 1)
        local_elev = self.elev_g[layer, x0:x1, y0:y1]
        finite = np.isfinite(local_elev)
        if not np.any(finite):
            return None

        local_rows, local_cols = np.where(finite)
        dist_sq = (
            (local_rows + x0 - row) ** 2 +
            (local_cols + y0 - col) ** 2
        )
        nearest = int(np.argmin(dist_sq))
        return float(local_elev[local_rows[nearest], local_cols[nearest]])

    def astar_path_to_map(self, path):
        """
        把 A* 搜索得到的网格路径转换回地图/world 坐标系。

        这里的 path 是以 voxel 索引表示的网格路径，内部存储形式通常是
        [layer, row, col] 或类似索引组合。我们需要把它重新换算成真实坐标，
        以便在 RViz 或外部导航中可视化和进一步用于参考轨迹。
        """
        path_idx = np.rint(path).astype(np.int32)
        layers = path_idx[:, 0]
        rows = path_idx[:, 1]
        cols = path_idx[:, 2]
        heights = self.elev_g[layers, rows, cols]
        traj_grid = np.stack(
            [cols, rows, heights / self.resolution], axis=1
        ).astype(np.float64)
        return transTrajGrid2Map(
            self.map_dim, self.center, self.resolution, traj_grid
        )
    
    def pos2idx(self, pos):
        """
        将 XY 坐标转换成地图索引。

        这里使用 grid 坐标系和 world 坐标系之间的换算：
        - world 坐标：真实位置
        - grid 坐标：栅格索引

        输出格式为 [col, row] 这种二维索引，适合底层 A* 使用。
        """
        idx = self.pos2array_idx(pos)
        idx = np.array([idx[1], idx[0]], dtype=np.int32)
        return idx

    def pos2array_idx(self, pos):
        """
        把世界坐标转换成 tomogram 体素索引。

        计算方法：
        - 先减去地图中心 center
        - 再除以 resolution，将米转换为格子数
        - 最后加上 offset 使坐标从中心点对齐

        所以这个函数是整个规划流程中最基础的坐标转换函数。
        """
        pos = np.asarray(pos, dtype=np.float64) - self.center
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        return idx

    def pos2layer(self, pos):
        """
        根据 3D 点位置，选择最合适的 tomogram 层。

        这是一个关键的层级选择函数：
        - 通过 XY 坐标找到相邻栅格；
        - 再根据 z 的高度从 elev_g 的各层中找出最接近的 slice layer；
        - 如果点超出地图范围，则回退到 z 对应的层索引。

        这样可以把 3D 任务点投影到最合适的地形层上，供底层搜索器使用。
        """
        pos = np.asarray(pos, dtype=np.float64)
        if pos.shape[0] < 3 or not np.isfinite(pos[2]) or self.elev_g is None:
            return 0

        idx = self.pos2array_idx(pos[:2])
        if (
            idx[0] < 0 or idx[0] >= self.map_dim[0] or
            idx[1] < 0 or idx[1] >= self.map_dim[1]
        ):
            fallback = self.z2slice_layer(pos[2])
            print("Clicked point outside tomogram grid, fallback layer:", fallback)
            return fallback

        layer = self.nearest_layer_at_idx(idx, pos[2])
        print(
            "Selected layer %d for clicked z %.3f at grid [%d, %d]" %
            (layer, pos[2], idx[0], idx[1])
        )
        return layer

    def nearest_layer_at_idx(self, idx, z):
        """
        在局部 XY 邻域中找到和 z 最相近的高度层。

        这一步非常重要，因为 tomogram 是按层存储的，
        而用户点击的点是 3D 坐标，所以需要决定该点属于哪一层。
        """
        search_radius = 2
        x0 = max(0, idx[0] - search_radius)
        x1 = min(self.map_dim[0], idx[0] + search_radius + 1)
        y0 = max(0, idx[1] - search_radius)
        y1 = min(self.map_dim[1], idx[1] + search_radius + 1)

        local_elev = self.elev_g[:, x0:x1, y0:y1]
        finite = np.isfinite(local_elev)
        if not np.any(finite):
            return self.z2slice_layer(z)

        scores = np.abs(local_elev - z)
        scores[~finite] = np.inf
        layer = int(np.unravel_index(np.argmin(scores), scores.shape)[0])
        return layer

    def z2slice_layer(self, z):
        """
        把真实高度 z 转换成 tomogram 采样层索引。

        这是高度到切片层之间的换算，主要用于：
        - 目标点超出边界时的回退
        - 层索引的快速估计
        """
        layer = int(np.round((z - self.slice_h0) / self.slice_dh))
        return int(np.clip(layer, 0, self.n_slice - 1))

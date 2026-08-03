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
                planning_gy.reshape(-1, planning_gy.shape[-1]).astype(np.double),
                -planning_gx.reshape(-1, planning_gx.shape[-1]).astype(np.double)
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
        """Sample tomogram elevation along optimized XY to avoid flat z output."""
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
        idx = self.pos2array_idx(pos)
        idx = np.array([idx[1], idx[0]], dtype=np.int32)
        return idx

    def pos2array_idx(self, pos):
        pos = np.asarray(pos, dtype=np.float64) - self.center
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        return idx

    def pos2layer(self, pos):
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
        layer = int(np.round((z - self.slice_h0) / self.slice_dh))
        return int(np.clip(layer, 0, self.n_slice - 1))

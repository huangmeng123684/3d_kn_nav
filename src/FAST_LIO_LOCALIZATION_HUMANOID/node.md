# 1. launch
```bash 
open3d_loc/launch/open3d_loc_g1.launch.py中如下参数修改为：
'voxelsize_coarse': 0.2,
'voxelsize_fine': 0.01, 
```

# 2. 注意
* 建议先单独使用fastlio进行建图测试，看看fastlio的定位效果

docker run -it --rm \
  --network host \
  --ipc host \
  --privileged \
  -v /home/kangneng/xq/pcd:/home/kangneng/xq/pcd \
  fast-lio-localization-humble-local:latest \
  bash


# 测试问题
## 6月5日
1. 当前镜像问题，fastlio需要Log目录，livox驱动需要更改ip，oped3d-loc关闭rviz

2. 笔记本接受不到来自docker的消息，需要加入以下环境变量
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

3. MID360S的 `host_ip: 192.168.124.162, lidar_config/lidar_ip: 192.168.124.188`

## 6月8日
1. 修改faslio中参数：`extrinsic_T: [-0.011, -0.02329, 0.04412]`,`extrinsic_est_en: false`

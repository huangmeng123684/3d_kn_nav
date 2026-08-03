```bash
# 镜像制作
cd ~/xq/code

docker buildx build \
  --network=host \
  --platform=linux/amd64 \
  --load \
  --build-arg GIT_REFRESH="$(date -u +%Y%m%d%H%M%S)" \
  -f work_space/kn_nav_ws/docker/amd/Dockerfile \
  -t cross-floor-nav:amd-v1 .

# 镜像导出
  docker save cross-floor-nav:amd-v1 > cross-floor-nav-amd-v1.tar

# 镜像导入
  docker load < cross-floor-nav-amd-v1.tar
```


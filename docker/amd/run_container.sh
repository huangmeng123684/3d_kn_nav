#!/usr/bin/env bash
set -euo pipefail

IMAGE="${1:-cross-floor-nav:amd-v1}"
CONTAINER_NAME="${CONTAINER_NAME:-kn_nav_humble}"
NAV_MAP_DIR="${NAV_MAP_DIR:-/home/unitree/nav_map}"

docker run -d \
  --name "${CONTAINER_NAME}" \
  --restart unless-stopped \
  --network host \
  --ipc host \
  --privileged \
  -v "${NAV_MAP_DIR}:/home/nav_map:ro" \
  "${IMAGE}"

echo "Container started: ${CONTAINER_NAME}"
echo "Enter it with: docker exec -it ${CONTAINER_NAME} bash"

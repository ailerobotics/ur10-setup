#!/usr/bin/env bash
# Build the cuRobo MPC evaluation image (task plan-mpc-curobo-v1).
# SIMULATION / EVALUATION ONLY. Records image size and build time.
#
# The image is named with the task prefix required by the brief; it is ours to
# create and remove. Never reuses the cell's names (ur10-cumotion, shelfbot_ros).
set -euo pipefail

IMAGE="fm-plan-mpc-curobo-v1:latest"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "==> Building ${IMAGE} from ${HERE}/Dockerfile"
START=$(date +%s)
DOCKER_BUILDKIT=1 docker build -t "${IMAGE}" -f "${HERE}/Dockerfile" "${HERE}"
END=$(date +%s)

echo "==> Build finished in $((END - START)) s"
docker image inspect "${IMAGE}" --format 'image: {{.RepoTags}}  size: {{.Size}} bytes' \
    | sed 's/\([0-9]\{9,\}\) bytes/& /'
docker images "${IMAGE}" --format '  {{.Repository}}:{{.Tag}}  {{.Size}}'

#!/usr/bin/env bash
# Build both Orin perception images. Run ON the Jetson Orin (aarch64, JetPack 5.1.2).
# Builds from this docker/ dir so the build context stays tiny (no models/frames sent).
set -euo pipefail
cd "$(dirname "$0")"

echo ">> building andrea_deploy:latest (NLF + YOLO-pose + wound)"
docker build -t andrea_deploy:latest -f Dockerfile .

echo ">> building andrea_mp:latest (isolated MediaPipe)"
docker build -t andrea_mp:latest -f Dockerfile.mediapipe .

echo ">> done:"
docker images | grep -E "andrea_deploy|andrea_mp"

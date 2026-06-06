#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
MODEL_URL="https://github.com/isarandi/nlf/releases/download/v0.3.2/nlf_s.torchscript"
MODEL_PATH="$WORKSPACE_ROOT/nlf_s.torchscript"

if [ -f "$MODEL_PATH" ]; then
    echo "Model already exists at $MODEL_PATH"
    exit 0
fi

echo "Downloading NLF model from $MODEL_URL ..."
wget -O "$MODEL_PATH" "$MODEL_URL" || curl -L -o "$MODEL_PATH" "$MODEL_URL"
echo "Done. Model saved to $MODEL_PATH"

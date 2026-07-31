#!/usr/bin/env bash
set -euo pipefail

# Backward-compatible defaults for callers that source this legacy shell helper.
export DASHSCOPE_BASE_URL="${DASHSCOPE_BASE_URL:-https://dashscope.aliyuncs.com/compatible-mode/v1}"
export DASHSCOPE_MODEL="${DASHSCOPE_MODEL:-qwen3.6-flash}"

#!/usr/bin/env bash
set -euo pipefail

# Defaults for DashScope OpenAI-compatible endpoint.
export DASHSCOPE_BASE_URL="${DASHSCOPE_BASE_URL:-https://dashscope.aliyuncs.com/compatible-mode/v1}"
export DASHSCOPE_MODEL="${DASHSCOPE_MODEL:-qwen3.6-flash}"

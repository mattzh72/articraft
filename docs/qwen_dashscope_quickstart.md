# Qwen (DashScope) Quickstart

## 1) Configure environment

Create a local `.env.dashscope` at repo root:

```bash
cat > .env.dashscope <<'EOF'
DASHSCOPE_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
DASHSCOPE_MODEL=qwen3.6-flash
DASHSCOPE_API_KEY=
EOF
```

Then fill your real key locally in `.env.dashscope`.

Do not commit `.env.dashscope` or real API keys.

## 2) Verify the API connection

```bash
just dashscope-test
```

Expected output:

```text
ok
```

## 3) Run generation with Qwen

```bash
just dashscope-generate "Create a compact articulated desk fan."
```

Or directly:

```bash
uv run articraft generate --provider dashscope --model qwen3.6-flash "Create a compact articulated desk fan."
```

## 4) View results

```bash
just viewer
```

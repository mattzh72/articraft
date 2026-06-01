# Qwen (DashScope) Quickstart

## 1) Configure Environment

Create or update the repo-local `.env`:

```bash
uv run articraft env bootstrap
```

Then set these values in `.env`:

```bash
DASHSCOPE_API_KEY=
DASHSCOPE_MODEL=qwen3.6-flash
DASHSCOPE_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
```

Do not commit `.env` or real API keys.

## 2) Verify the API Connection

```bash
just dashscope-test
```

Expected output:

```text
ok
```

## 3) Run Generation with Qwen

```bash
just dashscope-generate "Create a compact articulated desk fan."
```

Or directly:

```bash
uv run articraft generate --provider dashscope --model qwen3.6-flash "Create a compact articulated desk fan."
```

## 4) View Results

```bash
just viewer
```

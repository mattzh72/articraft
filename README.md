# Articraft

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Python versions](https://img.shields.io/badge/Python-3.11%20%7C%203.12-blue)](https://www.python.org/)
[![CI](https://github.com/mattzh72/articraft/actions/workflows/ci.yml/badge.svg)](https://github.com/mattzh72/articraft/actions/workflows/ci.yml)

**An Agentic System for Scalable Articulated 3D Asset Generation.**

[Paper](https://arxiv.org/abs/2605.15187) | [Project Page](https://articraft3d.github.io/) | [Dataset](https://github.com/mattzh72/articraft-data)

Articraft transforms the creation of articulated 3D assets into a programmatic, code-generation workflow powered by LLMs. It is now a local-first harness: this repo contains the generation/viewer logic, while the public dataset lives separately at [`mattzh72/articraft-data`](https://github.com/mattzh72/articraft-data).

![Articraft viewer showing an articulated desk lamp with joint controls and library metadata](docs/images/viewer-demo.png)

> **Security Note:** Articraft compiles and inspects generated records by executing their `model.py` files as Python code. Only run generated records and model scripts from trusted sources.

---

## Quickstart

### 1. Prerequisites
- Python 3.12 recommended (or 3.11). *Note: 3.13+ is not currently supported.*
- [`uv`](https://docs.astral.sh/uv/) for incredibly fast Python package management.
- [`just`](https://github.com/casey/just) as the command runner.
- [`npm`](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm) (optional, but needed for local viewer frontend).

### 2. Setup
From the repo root, run:
```bash
just setup
```
To set up a checkout from another working directory, pass the repository root:
```bash
just setup ./path/to/checkout
```

Articraft stores records in a gitignored data root. By default that is `<repo-root>/data`. To browse the released dataset, clone [`mattzh72/articraft-data`](https://github.com/mattzh72/articraft-data) and point Articraft at it:

```bash
git clone https://github.com/mattzh72/articraft-data.git ../articraft-data
export ARTICRAFT_DATA_DIR="$(cd ../articraft-data && pwd)"
uv run articraft status
uv run articraft library check --require-records
```

You can also pass any data folder explicitly with `--data-dir`.

### 3. Add API Keys
Open `.env` and set one or more provider keys (e.g. `OPENAI_API_KEY`, `GEMINI_API_KEYS`, `ANTHROPIC_API_KEYS`, `DASHSCOPE_API_KEY`).

> **No API Keys?** No problem. You can use external AI agents like Claude Code, Codex, or Cursor instead. For Codex setup, including how to add the Codex plugin, see [Codex Plugin Setup](docs/codex_plugin.md). Then point the agent at this repository and prompt it:
> 
> *"Create a realistic articulated [object name] in Articraft. Follow EXTERNAL_AGENT_DATA.md."*

### 4. Create an Asset

Generate your first model directly from a prompt using `articraft generate`:
```bash
uv run articraft generate "Create a realistic articulated desk lamp with a weighted base, two hinged arms, and an adjustable lamp head."
```

If you specify no overrides, it uses `ARTICRAFT_MODEL` and `ARTICRAFT_THINKING_LEVEL` from `.env` when present, otherwise `--model gpt-5.5-2026-04-23 --thinking-level high`. You can change models and caps:
```bash
uv run articraft generate --max-cost-usd 1.5 "Create a compact desk fan with adjustable tilt."
```

To generate from a reference image, see [Image-Conditioned Generation](docs/image_conditioned_generation.md).

### 5. Open the Viewer
Browse the objects you just generated. The local viewer API and React frontend can be started with:
```bash
just viewer
```

To browse an external data folder explicitly:

```bash
uv run articraft viewer --data-dir /Users/mzhou/articraft-data
```

### 6. Edit an Existing Asset
Fork an existing record when you want to modify it:
```bash
uv run articraft fork <record_id> "make the handle longer"
```

Forking creates a new child record and leaves the parent unchanged. See [Editing Existing Records](docs/record_editing.md) for model options and history viewing.

---

## Local Library

Use the compact library surface to inspect and maintain the data folder:

```bash
uv run articraft library list
uv run articraft library rebuild-manifest
uv run articraft library check --require-records
uv run articraft library set-category <record_id> <category_slug>
```

**Data Usage & Licensing**  
By contributing data to the Articraft project, you acknowledge and agree that your submissions will be used to build, evaluate, and improve machine learning models, and may be distributed publicly as part of Articraft data releases. You explicitly agree that all contributed data is released under the **[Creative Commons Attribution 4.0 International (CC-BY 4.0)](https://creativecommons.org/licenses/by/4.0/)** license.

---

## Documentation & Advanced Usage

- **[Architecture & Project Structure](docs/architecture.md)**
- **[Qwen / DashScope Quickstart](docs/qwen_dashscope_quickstart.md)**
- **[Codex Plugin Setup](docs/codex_plugin.md)**
- **[Editing Existing Records](docs/record_editing.md)**
- **[Image-Conditioned Generation](docs/image_conditioned_generation.md)**
- **[Contributing Standards & Workflow](CONTRIBUTING.md)**
- **[Security Policy](SECURITY.md)**

## Citation

```bibtex
@article{zhou2026articraft,
  title     = {Articraft: An Agentic System for Scalable Articulated 3D Asset Generation},
  author    = {Zhou, Matt and Li, Ruining and Lyu, Xiaoyang and Song, Zhaomou and Huang, Zhening and Zheng, Chuanxia and Rupprecht, Christian and Vedaldi, Andrea and Wu, Shangzhe},
  journal   = {arXiv preprint arXiv:2605.15187},
  year      = {2026}
}
```

This repository is licensed under the [Apache-2.0 License](LICENSE).

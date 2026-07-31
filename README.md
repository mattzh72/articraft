<h1 align="center">Articraft</h1>

<p align="center">A tool for generating and viewing articulated 3D assets.</p>

<p align="center">
  <a href="https://github.com/articraftresearch/Articraft"><img src="https://img.shields.io/badge/current%20project-articraftresearch%2FArticraft-24292f?style=flat-square&logo=github&logoColor=white" alt="Open the current Articraft project"></a>
</p>

<p align="center">
  <a href="https://arxiv.org/abs/2605.15187">Paper</a>
  ·
  <a href="https://articraft3d.github.io/">Project page</a>
  ·
  <a href="https://github.com/mattzh72/articraft-data">Dataset</a>
  ·
  <a href="LICENSE">Apache 2.0</a>
</p>

> [!IMPORTANT]
> **This repository has been superseded by [articraftresearch/Articraft](https://github.com/articraftresearch/Articraft).**
> Please use the new repository for current development and support. A volunteer group of researchers and engineers maintains the project.

Articraft generates articulated 3D assets from prompts. This repo contains the generation code, viewer, SDK, and command line tools. The public dataset lives at [`mattzh72/articraft-data`](https://github.com/mattzh72/articraft-data).

![Articraft viewer showing an articulated desk lamp with joint controls and library metadata](docs/images/viewer-demo.png)

> **Security note:** Articraft compiles and inspects generated records by running their `model.py` files as Python code. Only run generated records and model scripts from sources you trust.

---

## Quickstart

### 1. Prerequisites
- Python 3.12 recommended (or 3.11). *Note: 3.13+ is not currently supported.*
- [`uv`](https://docs.astral.sh/uv/) to install and run the Python package.
- [`just`](https://github.com/casey/just) to run project commands.
- [`npm`](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm) if you want to run the local viewer frontend.

### 2. Setup
From the repo root, run:
```bash
just setup
```
To set up a checkout from another working directory, pass the repository root:
```bash
just setup ./path/to/checkout
```

On Windows, install `just` with `winget install --id Casey.Just --exact`, reopen
PowerShell, and run the same `just` commands. The task recipes are compatible with both
PowerShell and Unix shells. Without `just`, the equivalent setup commands are:

```powershell
uv sync --frozen --group dev
npm --prefix .\viewer\web ci
npm --prefix .\viewer\web run typecheck
uv run --frozen articraft init
```

Articraft stores records in a gitignored data root. By default that is `<repo-root>/data`. To browse the released dataset, clone [`mattzh72/articraft-data`](https://github.com/mattzh72/articraft-data) and point Articraft at it:

```bash
git clone https://github.com/mattzh72/articraft-data.git ../articraft-data
export ARTICRAFT_DATA_DIR="$(cd ../articraft-data && pwd)"
uv run articraft status
uv run articraft library check --require-records
```

PowerShell equivalent:

```powershell
git clone https://github.com/mattzh72/articraft-data.git ..\articraft-data
$env:ARTICRAFT_DATA_DIR = (Resolve-Path ..\articraft-data).Path
uv run articraft status
uv run articraft library check --require-records
```

You can also pass any data folder explicitly with `--data-dir`.

### 3. Add API keys
Open `.env` and set one or more provider keys (e.g. `OPENAI_API_KEY`, `GEMINI_API_KEYS`, `ANTHROPIC_API_KEYS`, `DASHSCOPE_API_KEY`).

> If you do not have API keys, you can use external AI agents like Claude Code, Codex, or Cursor instead. For Codex setup, including how to add the Codex plugin, see [Codex plugin setup](docs/codex_plugin.md). Then point the agent at this repository and prompt it:
> 
> *"Create a realistic articulated [object name] in Articraft. Follow EXTERNAL_AGENT_DATA.md."*

### 4. Create an asset

Generate a model from a prompt with `articraft generate`:
```bash
uv run articraft generate "Create a realistic articulated desk lamp with a weighted base, two hinged arms, and an adjustable lamp head."
```

If you specify no overrides, it uses `ARTICRAFT_MODEL` and `ARTICRAFT_THINKING_LEVEL` from `.env` when present, otherwise `--model gpt-5.6-sol --thinking-level high`. You can change models and caps:
```bash
uv run articraft generate --max-cost-usd 1.5 "Create a compact desk fan with adjustable tilt."
```

To generate from a reference image, see [Image conditioned generation](docs/image_conditioned_generation.md).

### 5. Open the viewer
Browse the objects you just generated. The local viewer API and React frontend can be started with:
```bash
just viewer
```

The command prints the URL to open, normally <http://127.0.0.1:8765>. It works from
PowerShell as well. Without `just`, run `uv run --frozen articraft viewer`. For frontend
development with hot reload, use `just viewer-dev` or
`uv run --frozen articraft viewer --dev`, then open <http://127.0.0.1:5173>.

To browse an external data folder explicitly:

```bash
uv run articraft viewer --data-dir /path/to/articraft-data
```

```powershell
uv run articraft viewer --data-dir C:\path\to\articraft-data
```

### 6. Edit an existing asset
Fork an existing record when you want to modify it:
```bash
uv run articraft fork <record_id> "make the handle longer"
```

Forking creates a new child record and leaves the parent unchanged. See [Editing existing records](docs/record_editing.md) for model options and history viewing.

---

## Local library

Use these commands to inspect and maintain the data folder:

```bash
uv run articraft library list
uv run articraft library rebuild-manifest
uv run articraft library check --require-records
uv run articraft library set-category <record_id> <category_slug>
```

**Data usage and licensing**
By contributing data to the Articraft project, you acknowledge and agree that your submissions will be used to build, evaluate, and improve machine learning models, and may be distributed publicly as part of Articraft data releases. You agree that all contributed data is released under the **[Creative Commons Attribution 4.0 International (CC BY 4.0)](https://creativecommons.org/licenses/by/4.0/)** license.

---

## Documentation and advanced usage

- **[Architecture and project structure](docs/architecture.md)**
- **[Qwen / DashScope quickstart](docs/qwen_dashscope_quickstart.md)**
- **[Codex plugin setup](docs/codex_plugin.md)**
- **[Editing existing records](docs/record_editing.md)**
- **[Image conditioned generation](docs/image_conditioned_generation.md)**
- **[Contributing standards and workflow](CONTRIBUTING.md)**
- **[Security policy](SECURITY.md)**

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

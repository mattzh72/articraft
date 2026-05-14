# Articraft

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Python versions](https://img.shields.io/badge/Python-3.11%20%7C%203.12-blue)](https://www.python.org/)
[![CI](https://github.com/mattzh72/articraft/actions/workflows/ci.yml/badge.svg)](https://github.com/mattzh72/articraft/actions/workflows/ci.yml)

**An Agentic System for Scalable Articulated 3D Asset Generation.**

Articraft transforms the creation of articulated 3D assets into a programmatic, code-generation workflow powered by LLMs. Engineered for large-scale dataset generation, it bypasses heavyweight manual tools to rapidly produce objects with semantic parts, robust geometry, and physical joints.

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

### 3. Add API Keys
Open `.env` and set one or more provider keys (e.g. `OPENAI_API_KEY`, `GEMINI_API_KEYS`, `ANTHROPIC_API_KEYS`).

> **No API Keys?** No problem! If you don't have API keys set up, you can use external AI agents like Claude Code, Codex, or Cursor. Just point them to this repository and prompt them:
> 
> *"Create a realistic articulated [object name] and add it to the Articraft dataset. Follow EXTERNAL_AGENT_DATA.md."*

### 4. Create an Asset

Generate your first model directly from a prompt using `articraft generate`:
```bash
uv run articraft generate "Create a realistic articulated desk lamp with a weighted base, two hinged arms, and an adjustable lamp head."
```

If you specify no overrides, it defaults to `--model gpt-5.5-2026-04-23 --thinking-level high`. You can change models and caps:
```bash
uv run articraft generate --model gemini-3-flash-preview --max-cost-usd 1.5 "Create a compact desk fan with adjustable tilt."
```

### 5. Open the Viewer
Browse the objects you just generated. The local viewer API and React frontend can be started with:
```bash
just viewer
```

---

## Contribute Data

A huge part of Articraft's mission is crowdsourcing a diverse, massive dataset of articulated 3D models. We welcome generation via our CLI, batch processing, or through external AI agents (like Claude Code or Codex). 

For full details on our data pipelines, generation guides, and opening pull requests, please read the complete **[Data Contribution Workflow in CONTRIBUTING.md](CONTRIBUTING.md)**.

**Data Usage & Licensing**  
By contributing data to the Articraft project, you acknowledge and agree that your submissions will be used to build, evaluate, and improve machine learning models, and will be distributed publicly as part of our datasets. You explicitly agree that all contributed data is released under the **[Creative Commons Attribution 4.0 International (CC-BY 4.0)](https://creativecommons.org/licenses/by/4.0/)** license.

---

## Documentation & Advanced Usage

- **[Architecture & Project Structure](docs/architecture.md)**
- **[Dataset Generation & Batch Processing](docs/dataset_generation.md)**
- **[Contributing Standards & Workflow](CONTRIBUTING.md)**
- **[Security Policy](SECURITY.md)**

## Citation

```bibtex
@misc{zhou2026articraft,
  title = {{Articraft}: An Agentic System for Scalable Articulated 3D Asset Generation},
  author = {Zhou, Matt and Li, Ruining and Lyu, Xiaoyang and Song, Zhaomou and Huang, Zhening and Zheng, Chuanxia and Rupprecht, Christian and Vedaldi, Andrea and Wu, Shangzhe},
  year = {2026},
  note = {Preprint},
  howpublished = {\url{https://github.com/mattzh72/articraft}}
}
```

This repository is licensed under the [Apache-2.0 License](LICENSE).

# Articraft

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Python versions](https://img.shields.io/badge/Python-3.11%20%7C%203.12-blue)](https://www.python.org/)
[![CI](https://github.com/mattzh72/articraft/actions/workflows/ci.yml/badge.svg)](https://github.com/mattzh72/articraft/actions/workflows/ci.yml)

**An Agentic System for Scalable Articulated 3D Asset Generation.**

Articraft transforms the creation of articulated 3D assets into a programmatic, code-generation workflow. Instead of relying on manual 3D modeling tools, Articraft enables LLMs to write compact Python programs against an articulation-focused SDK (the Articraft SDK). The harness executes the generated program, validates the mechanical structure, and autonomously feeds errors back to the model for iterative refinement. The final output is an object complete with semantic parts, robust geometry, physical joints, and rigid motion limits.

Engineered for scalable, high-throughput dataset generation, Articraft bypasses heavyweight graphics pipelines and slow visual-feedback loops. It natively integrates with major LLM providers (OpenAI, Anthropic, Gemini, OpenRouter) and features lightweight, comprehensive tooling for local storage, iterative compilation, visual inspection, and curation. This very stack powered the creation of **Articraft-10K**: a curated dataset comprising over 10,000 highly-articulated 3D assets spanning 245 everyday object categories.

### Security Note
Articraft compiles and inspects generated records by executing their `model.py` files as Python code. Only run generated records and model scripts from trusted sources. Review them first, or run Articraft in an isolated container/disposable environment when working with untrusted inputs.

---

## Quickstart

### 1. Prerequisites
- Python 3.12 recommended (or 3.11). *Note: 3.13+ is not currently supported due to `cadquery` dependency wheels.*
- [`uv`](https://docs.astral.sh/uv/) for incredibly fast Python package management.
- [`just`](https://github.com/casey/just) as the command runner.
- [`npm`](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm) (optional, but needed for local viewer frontend).

### 2. Setup
From the repo root, run:
```bash
just setup
```
This initializes the `.env` file, sets up the Python tools, installs the viewer dependencies (`npm`), sets up storage, and installs canonical git hooks for database synchronization.

### 3. Add API Keys
Open `.env` and set one or more provider keys (e.g. `OPENAI_API_KEY`, `GEMINI_API_KEYS`, `ANTHROPIC_API_KEYS`).

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
Browse the objects you generated. First compile the records:
```bash
uv run articraft compile-all
```
Then start the viewer API and React frontend:
```bash
just viewer
```

### 6. Contribute Data

A huge part of Articraft's mission is crowdsourcing a diverse, massive dataset of articulated 3D models. When you have generated objects that meet our quality bar, there are several pathways to contribute!

**A. Targeted Generation (CLI)**
If you want to author specific items directly into a dataset category, use the harness:
```bash
uv run articraft dataset run "Create a backyard gas grill..." --category-slug grill
```

**B. Large-Scale Batch Generation**
For contributing hundreds or thousands of variations, we lean heavily on automated batch specs. See our [Dataset Generation Guide](docs/dataset_generation.md) to learn how to drive batch curation.

**C. No API Keys? Crowdsource with Claude Code or Codex**
If you don't have API keys set up, or prefer to use external AI agents, simply point Claude Code, Codex, or Cursor to this repository! Just ask the agent to follow our authoring guidelines:
```text
Create a realistic articulated washing machine and add it to the Articraft dataset. Follow EXTERNAL_AGENT_DATA.md.
```
Your external agent will automatically use the `articraft external` CLI to author, compile, validate, and promote the asset into the repository. Open a Pull Request with the finalized record directory to contribute it upstream!

Before submitting your PR, make sure to rate all assets that you generate in the viewer!

---

## Documentation & Advanced Usage

Articraft scales to massive pipelines using CSV batch specifications. For detailed architectural background and how to run dataset workflows, see our documentation:

- **[Architecture & Project Structure](docs/architecture.md)**
- **[Dataset Generation & Batch Processing](docs/dataset_generation.md)**
- **[Contributing Standards & Workflow](CONTRIBUTING.md)**
- **[Security Policy](SECURITY.md)**

## Contributing
We welcome improvements to Articraft! Please refer to the [Contributor Guidelines](CONTRIBUTING.md) to get started on setting up a dev environment and submitting Pull Requests. Be sure to review our [Code of Conduct](CODE_OF_CONDUCT.md).

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

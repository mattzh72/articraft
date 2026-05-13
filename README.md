# Articraft

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Python versions](https://img.shields.io/badge/Python-3.11%20%7C%203.12-blue)](https://www.python.org/)

**An Agentic System for Scalable Articulated 3D Asset Generation.**

Articraft allows you to generate high-quality articulated 3D assets purely from natural language prompts, inspect them locally with a dedicated viewer, and systematically curate large-scale 3D asset datasets using major LLM providers (OpenAI, Anthropic, Gemini, OpenRouter).

### Security Note
⚠️ Articraft compiles and inspects generated records by executing their `model.py` files as Python code. Only run generated records and model scripts from trusted sources. Review them first, or run Articraft in an isolated container/disposable environment when working with untrusted inputs.

---

## ⚡️ Quickstart

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

### 6. Contribute Data With Claude Code Or Codex

You can use Claude Code or Codex to create Articraft records. Start the agent in this repo and ask it to follow [Data Authoring Guidelines](EXTERNAL_AGENT_DATA.md).

For a dataset contribution, use a prompt like:

```text
Create a realistic articulated washing machine and add it to the Articraft dataset. Follow EXTERNAL_AGENT_DATA.md.
```

For local inspection only, ask for a workbench record instead:

```text
Create a realistic articulated washing machine for local workbench inspection. Follow EXTERNAL_AGENT_DATA.md.
```

To contribute data, open a pull request with the finalized dataset record. Include the prompt, category, external agent used, and the checks you ran. Workbench-only records are local drafts and should not be committed.

---

## 📚 Documentation & Advanced Usage

Articraft scales to massive pipelines using CSV batch specifications. For detailed architectural background and how to run dataset workflows, see our documentation:

- **[Architecture & Project Structure](docs/architecture.md)**
- **[Dataset Generation & Batch Processing](docs/dataset_generation.md)**
- **[Contributing Standards & Workflow](CONTRIBUTING.md)**
- **[Security Policy](SECURITY.md)**

## 🤝 Contributing
We welcome improvements to Articraft! Please refer to the [Contributor Guidelines](CONTRIBUTING.md) to get started on setting up a dev environment and submitting Pull Requests. Be sure to review our [Code of Conduct](CODE_OF_CONDUCT.md).

## 📄 Citation

```bibtex
@misc{articraft2026,
  title={{Articraft}: An Agentic System for Scalable Articulated 3D Asset Generation},
  author={{Articraft contributors}},
  howpublished={\url{https://github.com/mattzh72/articraft}},
  year={2026}
}
```

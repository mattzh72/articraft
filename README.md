# Articraft

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Python versions](https://img.shields.io/badge/Python-3.11%20%7C%203.12-blue)](https://www.python.org/)
[![CI](https://github.com/mattzh72/articraft/actions/workflows/ci.yml/badge.svg)](https://github.com/mattzh72/articraft/actions/workflows/ci.yml)

**An Agentic System for Scalable Articulated 3D Asset Generation.**

[Paper](https://arxiv.org/abs/2605.15187) | [Project Page](https://articraft3d.github.io/)

Articraft transforms the creation of articulated 3D assets into a programmatic, code-generation workflow powered by LLMs. Engineered for large-scale dataset generation, it bypasses heavyweight manual tools to rapidly produce objects with semantic parts, robust geometry, and physical joints.

![Articraft viewer showing an articulated desk lamp with joint controls and dataset metadata](docs/images/viewer-demo.png)

> **Security Note:** Articraft compiles and inspects generated records by executing their `model.py` files as Python code. Only run generated records and model scripts from trusted sources.

---

## Quickstart

### 1. Prerequisites
- Python 3.12 recommended (or 3.11). *Note: 3.13+ is not currently supported.*
- [`uv`](https://docs.astral.sh/uv/) for incredibly fast Python package management.
- [`just`](https://github.com/casey/just) as the command runner.
- [`Git LFS`](https://git-lfs.com/) for hydrating dataset records on demand.
- [`npm`](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm) (optional, but needed for local viewer frontend).

### 2. Setup
From the repo root, run:
```bash
just setup
```

Fresh clones are code-first: `data/records/**` is stored with Git LFS and excluded from automatic LFS fetch by `.lfsconfig`, so it does not need to be hydrated before developing the code or browsing indexed metadata. Hydrate records only when you want to inspect, render, or edit their payloads:

```bash
uv run articraft data hydrate --record <record_id>
uv run articraft data hydrate --category <category_slug>
uv run articraft data hydrate --time-from 2026-04-01 --time-to 2026-04-07
uv run articraft data hydrate --last 7d
uv run articraft data hydrate --all
```

### 3. Choose Model Access
Open `.env` and set one or more provider keys when using direct API providers such as `OPENAI_API_KEY`, `GEMINI_API_KEYS`, or `ANTHROPIC_API_KEYS`.

For no-key Codex generation, install/login to the Codex CLI and use Articraft's `codex-cli` provider. This keeps Articraft in control of the generation loop, compile feedback, turn count, tool-call count, compile-attempt count, record persistence, and trajectory. Articraft records the total token count when Codex CLI emits its `tokens used` line; dollar cost remains unavailable because Codex CLI does not expose billable cost accounting.

### 4. Create an Asset

Generate your first model directly from a prompt using `articraft generate`:
```bash
uv run articraft generate "Create a realistic articulated desk lamp with a weighted base, two hinged arms, and an adjustable lamp head."
```

If you specify no overrides, it defaults to `--model gpt-5.5-2026-04-23 --thinking-level high`. You can change models and caps:
```bash
uv run articraft generate --model gemini-3-flash-preview --max-cost-usd 1.5 "Create a compact desk fan with adjustable tilt."
```

Run the same internal Articraft harness through no-key Codex:

```bash
uv run articraft generate --provider codex-cli "Create a compact desk fan with adjustable tilt."
```

Use an explicit Codex model when needed:

```bash
uv run articraft generate --provider codex-cli --model gpt-5.5 "Create a compact desk fan with adjustable tilt."
```

Reference images also stay inside the internal harness:

```bash
uv run articraft generate --provider codex-cli --image reference.png "Create an articulated object matching this reference."
```

### 5. Open the Viewer
Browse the objects you just generated. The local viewer API and React frontend can be started with:
```bash
just viewer
```

The viewer can browse/search the dataset from `data/records_index.jsonl` before record payloads are hydrated. When you select an unhydrated record, use the "Hydrate record" action before opening source files, traces, or rendered assets.

### 6. Edit an Existing Asset
Fork an existing record when you want to modify it:
```bash
uv run articraft fork data/records/<record_id> "make the handle longer"
```

Forking creates a new child record and leaves the parent unchanged. See [Editing Existing Records](docs/record_editing.md) for model options, dataset behavior, and history viewing.

---

## Codex Plugin Setup

This repository includes a repo-local Codex plugin under `plugins/articraft/` for Articraft-specific Codex workflows.

After completing the Quickstart setup above on a new computer:

1. Open this repository in Codex.
2. Restart or reload Codex so it can discover `.agents/plugins/marketplace.json`.
3. In the Codex plugin or marketplace UI, install **Articraft** from **Articraft Local**.

The plugin itself adds Codex guidance and does not introduce any additional credential requirements beyond the Articraft workflows you choose to run.

Use `uv run articraft generate --provider codex-cli "<prompt>"` for no-key Codex generation with the internal Articraft harness. Use direct API providers when you need billable cost accounting or provider-native usage breakdowns. The legacy `articraft external ...` workflow remains available for manual external-agent drafting, but it is not the quality-parity path.

---

## Contribute Data

A huge part of Articraft's mission is crowdsourcing a diverse, massive dataset of articulated 3D models. We welcome generation via our CLI, batch processing, or through external AI agents (like Claude Code, Codex, or Cursor).

For full details on our data pipelines, generation guides, and opening pull requests, please read the complete **[Data Contribution Workflow in CONTRIBUTING.md](CONTRIBUTING.md)**.

**Data Usage & Licensing**  
By contributing data to the Articraft project, you acknowledge and agree that your submissions will be used to build, evaluate, and improve machine learning models, and will be distributed publicly as part of our datasets. You explicitly agree that all contributed data is released under the **[Creative Commons Attribution 4.0 International (CC-BY 4.0)](https://creativecommons.org/licenses/by/4.0/)** license.

---

## Documentation & Advanced Usage

- **[Architecture & Project Structure](docs/architecture.md)**
- **[Editing Existing Records](docs/record_editing.md)**
- **[Dataset Generation & Batch Processing](docs/dataset_generation.md)**
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

# Agent Component Ablations

This directory is experimental. Do not merge this branch.

The experiment runs ten object prompts under five conditions. Every request uses
`gpt-5.6-sol` with high thinking. The conditions are the full iterative baseline,
the primitives only SDK, no compile feedback with no testing SDK, no example
retrieval, and a one request single pass with no tools or examples.

Run the complete experiment from the repository root:

```shell
uv run python -m experiments.agent_component_ablations.run \
  --run-id gpt56sol-high-v1 \
  --concurrency 4
```

Results are stored under
`performance/results/agent_component_ablations/gpt56sol-high-v1/`. That path is
ignored by Git. Each cell stores its prompt, condition, exact system prompt,
provider output, final Python file, trace, usage, compile evaluation, exported
URDF when available, and status.

The same command can be run again after an interruption. Completed cells are
kept and unfinished or failed cells run again. Add `--force` only when every
scheduled cell should run again.

Check the complete schedule without making model requests:

```shell
uv run python -m experiments.agent_component_ablations.run \
  --run-id schedule-check \
  --dry-run
```

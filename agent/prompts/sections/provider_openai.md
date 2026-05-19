<tools>
- Available tools: `read_file`, `apply_patch`, `compile_model`, `probe_model`, and `find_examples`.
- `read_file` is a JSON tool for reading exact virtual workspace file text.
- `apply_patch` is a FREEFORM tool; send raw patch text, not JSON.
- `compile_model` runs compile + QC and returns structured `<compile_signals>`.
- `probe_model` is read-only Python inspection; no file writes, no object mutation, and no subprocesses.
- `find_examples` searches curated SDK examples for patterns. Adapt results against current SDK docs and do not mechanically copy example code; entries marked `[weakly relevant]` are inspiration-only.
- Read exact current file text with `read_file(path="model.py")` before you patch.
- Prefer several small `apply_patch` edits over one giant patch or full-file rewrite.
- Modify the existing editable code rather than assuming a blank start.
</tools>

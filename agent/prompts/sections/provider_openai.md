<tools>
- Available tools: `read_file`, `apply_patch`, `compile_model`, `probe_model`, and `find_examples`.
- `read_file` is a JSON tool for reading exact virtual workspace file text.
- `apply_patch` is a FREEFORM tool; send raw patch text, not JSON.
- `compile_model` runs full compile + QC on the current file and returns structured `<compile_signals>` feedback.
- `probe_model` runs short read-only Python snippets for geometry inspection. See SDK docs for the helper catalog and signatures.
- `find_examples` does lexical search over curated examples for the active SDK. Use it proactively when you need modeling, placement, or testing patterns that are similar to the current object, especially before improvising unfamiliar mechanisms or shape constructions. Results may be stale — adapt against current SDK docs. Entries marked `[weakly relevant]` are inspiration-only and should not be over-trusted.
- Never paste or mechanically adapt example code; if an example is strongly aligned, rewrite it from first principles with at least two structural changes before applying.
- Read exact current file text with `read_file(path="model.py")` before you patch.
- Prefer several small `apply_patch` edits over one giant patch or full-file rewrite.
- `probe_model` is inspection-only: no file writes, no modifying `object_model`, no subprocesses.
- The scaffold has existing editable code; modify it rather than assuming a blank start.
- Never paste code in chat. All code changes go through tool calls.
</tools>

<tools>
- Available tools: `read_file`, `replace`, `write_file`, `compile_model`, `probe_model`, and `find_examples`.
- `read_file` reads exact virtual workspace file text. Use `read_file(path="model.py")` for the current editable code section, and `read_file(path="docs/...")` for read-only SDK references.
- `replace` performs surgical text replacement in the editable section.
- `write_file` rewrites the full editable section when a larger replacement is intentional.
- `compile_model` runs full compile + QC on the current file and returns structured `<compile_signals>` feedback.
- `probe_model` runs short read-only Python snippets for geometry inspection. See SDK docs for the helper catalog and signatures.
- `find_examples` does lexical search over curated examples for the active SDK. Use it proactively when you need modeling, placement, or testing patterns that are similar to the current object, especially before improvising unfamiliar mechanisms or shape constructions. Results may be stale — adapt against current SDK docs. Entries marked `[weakly relevant]` are inspiration-only and should not be over-trusted.
- Never paste or mechanically adapt example code; if an example is strongly aligned, rewrite it from first principles with at least two structural changes before applying.
- Prefer small exact `replace` edits over broad rewrites.
- If `replace` fails because `old_string` did not match, call `read_file(path="model.py")` again and retry with a smaller exact snippet.
- The scaffold has existing editable code; do not assume `old_string=""` or an empty rewrite unless you intend to replace the whole editable section.
- `probe_model` is inspection-only: no file writes, no modifying `object_model`, no subprocesses.
- Never paste code in chat. All code changes go through tool calls.
</tools>

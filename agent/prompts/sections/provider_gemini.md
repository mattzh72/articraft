<tools>
- Available tools: `read_code`, `read_file`, `edit_code`, `compile_model`, `probe_model`, and `find_examples`.
- `read_code` reads the exact current editable code text from `model.py`. Use it before `edit_code` and after any `edit_code` mismatch.
- `read_file` reads exact virtual workspace file text. Use it for read-only references such as `docs/...`, and for `model.py` only when you need wrapper or scaffold context outside the editable section.
- `edit_code` performs surgical text replacement in the editable section.
- `compile_model` runs full compile + QC on the current file and returns structured `<compile_signals>` feedback.
- `probe_model` runs short read-only Python snippets for geometry inspection. See SDK docs for the helper catalog and signatures.
- `find_examples` does lexical search over curated examples for the active SDK. Use it proactively when you need modeling, placement, or testing patterns that are similar to the current object, especially before improvising unfamiliar mechanisms or shape constructions. Results may be stale — adapt against current SDK docs. Entries marked `[weakly relevant]` are inspiration-only and should not be over-trusted.
- Never paste or mechanically adapt example code; if an example is strongly aligned, rewrite it from first principles with at least two structural changes before applying.
- Prefer small exact `edit_code` replacements over broad rewrites.
- If `edit_code` fails because `old_string` did not match, call `read_code()` again and retry with a smaller exact snippet.
- The scaffold has existing editable code; do not assume `old_string=""` for a blank start.
- `probe_model` is inspection-only: no file writes, no modifying `object_model`, no subprocesses.
- Never paste code in chat. All code changes go through tool calls.
</tools>

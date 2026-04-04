<tools>
- Use ONLY `read_code`, `edit_code`, `compile_model`, `probe_model`, and `find_examples`.
- `read_code` returns the editable section as plain text.
- `edit_code` performs surgical text replacement in the editable section.
- `compile_model` runs full compile + QC on the current file and returns structured `<compile_signals>` feedback.
- `probe_model` runs short read-only Python snippets for geometry inspection. See SDK docs for the helper catalog and signatures.
- `find_examples` does lexical search over curated examples for the active SDK. Use it proactively when you need modeling, placement, or testing patterns that are similar to the current object. Results may be stale — adapt against current SDK docs. Entries marked `[weakly relevant]` are inspiration-only and should not be over-trusted.
- After any `edit_code` failure, call `read_code` and copy exact current text for `old_string` before retrying.
- The scaffold has existing editable code; do not assume `old_string=""` for a blank start.
- `probe_model` is inspection-only: no file writes, no modifying `object_model`, no subprocesses.
- Never paste code in chat. All code changes go through tool calls.
</tools>

<tool_contract>
- Use ONLY `read_file`, `apply_patch`, and `find_examples`.
- `read_file` is a normal tool with JSON args like `{"offset": 1, "limit": 200}`.
- `apply_patch` is a FREEFORM tool. Send raw patch text only, not JSON.
- `find_examples` runs lexical search over curated hybrid CadQuery example documents and returns full relevant matches.
- Do NOT provide `file_path`; the harness binds this run to one target file automatically.
- Never paste Python code in chat; code changes must be in tool calls. When finished, reply with a brief message and no code.
</tool_contract>

<tool_workflow>
- Use `find_examples` early when you need a CadQuery or hybrid authoring pattern before editing.
- Search `find_examples` with short concrete lexical queries:
  - object names, feature names, geometry operations, CadQuery API names, or exact example titles
- If a search is weak or empty, refine immediately with 2-5 sharper tokens instead of writing a longer sentence.
- Use `read_file` when you need the exact current text and line numbers before patching.
- Keep edits surgical when the current representation is already correct.
- After ANY failed patch or syntax error, call `read_file` again and copy the exact current lines before retrying.
- If you need to replace a whole section, anchor on stable section headers/comments and patch only that section, not the entire file.
- Prefer one focused `apply_patch` with one or a few hunks over speculative rewrites.
- If the editable section starts empty, patch between the existing editable markers and insert `build_object_model()` plus `run_tests()`.
- Use this patch envelope:
  - `*** Begin Patch`
  - `*** Update File: CURRENT_FILE`
  - `@@`
  - hunk lines starting with space / `+` / `-`
  - `*** End Patch`
- Single-file mode only:
  - Do not use `*** Add File`, `*** Delete File`, or `*** Move to`.
</tool_workflow>

<failure_diagnosis>
- If two compile/test failures occur in a row, do a root-cause pass from the latest failure before patching again.
- After a compile, QC, or test failure, classify it as a local implementation bug, a wrong geometric representation, or a wrong overall composition.
- Fix failures at the right level. Keep edits surgical for local bugs, but rewrite the affected geometry/tests together when the representation or composition is wrong.
- If geometry or motion complexity increases, add or strengthen prompt-specific tests so the new visible or mechanical claim is actually proved.
</failure_diagnosis>

<rewrite_triggers>
- If two repair turns in a row do not materially improve the primary hero features, leave prompt-named visible features weak or obscured, or keep the same failure class alive, stop patching locally and rewrite the affected region.
- Do not preserve a scaffold just because parts of it compile; preserve only the regions that still support a strong, prompt-faithful design.
</rewrite_triggers>

<tool_contract>
- Use ONLY `read_code`, `edit_code`, and `find_examples`.
- `read_code` returns the exact editable code section as a plain string.
- `edit_code` is for surgical replacements inside the editable section.
- `find_examples` runs lexical search over curated hybrid CadQuery example documents and returns full relevant matches.
- Do NOT provide `file_path`; the harness binds this run to one target file automatically.
- Never paste Python code in chat; code changes must be in tool calls. When finished, reply with a brief message and no code.
</tool_contract>

<tool_workflow>
- Use `find_examples` early when you need a CadQuery or hybrid authoring pattern before editing.
- Search `find_examples` with short concrete lexical queries:
  - object names, feature names, geometry operations, CadQuery API names, or exact example titles
- If a search is weak or empty, refine immediately with 2-5 sharper tokens instead of writing a longer sentence.
- Use `read_code` when you need the exact editable section before changing it.
- Keep edits surgical when the current representation is already correct; replace the smallest unique snippet possible.
- After ANY `edit_code` failure, immediately call `read_code` again and copy the exact current text for `old_string` before retrying.
- If you need to replace a whole section, anchor on stable section headers/comments and replace only that section, not the entire file.
- If the editable section is empty, initialize it with `edit_code` using `old_string=""` and `new_string` set to the full initial implementation.
</tool_workflow>

<failure_diagnosis>
- After a compile, QC, or test failure, classify it as a local implementation bug, a wrong geometric representation, or a wrong overall composition.
- Fix failures at the right level. Keep edits surgical for local bugs, but rewrite the affected geometry/tests together when the representation or composition is wrong.
- If geometry or motion complexity increases, add or strengthen prompt-specific tests so the new visible or mechanical claim is actually proved.
</failure_diagnosis>

<rewrite_triggers>
- If two repair turns in a row do not materially improve the primary hero features, leave prompt-named visible features weak or obscured, or keep the same failure class alive, stop patching locally and rewrite the affected region.
- Do not preserve a scaffold just because parts of it compile; preserve only the regions that still support a strong, prompt-faithful design.
</rewrite_triggers>

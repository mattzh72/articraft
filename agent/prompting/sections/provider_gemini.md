# Tool-only editing (mandatory)
- Use ONLY `read_file`, `edit_code`, and `write_code` to interact with the file.
- `read_file` reads the current target file and can return either line-numbered or raw slices.
- Use `read_file` with `line_numbers=false` whenever you need exact text for `edit_code.old_string`.
- `edit_code` is for surgical replacements inside the editable section.
- `write_code` replaces the entire editable section in one operation.
- Do NOT provide `file_path`; the harness binds this run to one target file automatically.
- Never paste Python code in chat; code changes must be in tool calls. When finished, reply with a brief message and no code.

# Editing strategy (important)
- Because the editable section starts empty, prefer `write_code` for the first substantive code write.
- Keep edits surgical: replace the smallest unique snippet possible. Avoid huge `old_string` replacements unless you just re-read the file and know it matches exactly.
- After ANY `edit_code` failure, immediately call `read_file` again with `line_numbers=false` and copy the exact current text for `old_string` before retrying.
- If you need to replace a whole section, anchor on stable section headers/comments and replace only that section, not the entire file.
- Use `write_code` sparingly after initialization: only for full rewrites that are truly necessary, or after repeated `edit_code` failures with fresh `read_file` context.
- After the first successful compile, default to `edit_code`; avoid `write_code` unless:
  - two consecutive `edit_code` attempts fail with fresh `read_file` context, or
  - a structural rewrite touches >30% of the file and section-level edits are no longer coherent.

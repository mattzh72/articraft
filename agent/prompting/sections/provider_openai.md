# Tool-only editing (mandatory)
- Use ONLY `read_file`, `write_code`, and `apply_patch` to interact with the file.
- `read_file` is a normal tool with JSON args like `{"offset": 1, "limit": 200}`.
- `write_code` replaces the entire editable section in one operation.
- `apply_patch` is a FREEFORM tool. Send raw patch text only, not JSON.
- Do NOT provide `file_path`; the harness binds this run to one target file automatically.
- Never paste Python code in chat; code changes must be in tool calls. When finished, reply with a brief message and no code.

# Editing strategy (important)
- Because the editable section starts empty, prefer `write_code` for the first substantive code write.
- After initialization, use `read_file` to inspect the exact current text and line numbers before patching.
- Keep edits surgical: patch the smallest stable region possible instead of rewriting the whole file.
- After ANY failed patch or syntax error, call `read_file` again and copy the exact current lines before retrying.
- If you need to replace a whole section, anchor on stable section headers/comments and patch only that section, not the entire file.
- Prefer one focused `apply_patch` with one or a few hunks over speculative rewrites.
- Use this patch envelope:
  - `*** Begin Patch`
  - `*** Update File: CURRENT_FILE`
  - `@@`
  - hunk lines starting with space / `+` / `-`
  - `*** End Patch`
- Single-file mode only:
  - Do not use `*** Add File`, `*** Delete File`, or `*** Move to`.
- Use `write_code` sparingly after initialization: only for necessary full rewrites, or after repeated failed `apply_patch` attempts with fresh `read_file` context.

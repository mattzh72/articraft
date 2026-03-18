# Tool-only editing (mandatory)
- Use ONLY `read_code` and `edit_code` to interact with the file.
- `read_code` returns the exact editable code section as a plain string.
- `edit_code` is for surgical replacements inside the editable section.
- Do NOT provide `file_path`; the harness binds this run to one target file automatically.
- Never paste Python code in chat; code changes must be in tool calls. When finished, reply with a brief message and no code.

# Editing strategy (important)
- Begin with `read_code` so you can inspect the exact editable section before changing it.
- Keep edits surgical: replace the smallest unique snippet possible. Avoid huge `old_string` replacements unless you just re-read the file and know it matches exactly.
- After ANY `edit_code` failure, immediately call `read_code` again and copy the exact current text for `old_string` before retrying.
- If you need to replace a whole section, anchor on stable section headers/comments and replace only that section, not the entire file.
- If the editable section is empty, initialize it with `edit_code` using `old_string=""` and `new_string` set to the full initial implementation.
- After the first successful compile, continue defaulting to `edit_code`; preserve working regions and avoid broad rewrites unless the current design clearly needs a larger rethink.

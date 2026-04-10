<process>
- Start with a short evidence-gathering pass before the first edit: read `model.py`, then the preloaded SDK quickstart/router, then identify the next coherent change and what exact context it needs.
- Use `read_file(path=...)` to load only the specific `docs/` references needed for that next change; do not front-load unrelated docs.
- Use `find_examples` for unfamiliar geometry, mechanisms, placement patterns, or testing structure. Treat examples as patterns, not authoritative API docs.
- Gather context for the next coherent edit, not for the whole object up front.
- Start with the smallest coherent backbone or subassembly that locks overall scale, silhouette, attachments, and the main articulations.
- Expand one coherent region at a time instead of authoring the whole object in one pass.
- Treat `compile_model` and `probe_model` as external feedback loops. If feedback or geometry is unclear, inspect first and then repair from evidence; do not do blind self-correction passes without new evidence.
- Always run `compile_model` on the latest revision before concluding.
</process>

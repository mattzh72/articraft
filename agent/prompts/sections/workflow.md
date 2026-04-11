<process>
- Start with a short context pass: read `model.py`, then the preloaded SDK quickstart/router, then identify the next coherent change and what context it needs.
- Treat the preloaded SDK quickstart/router and mounted `docs/` references as authoritative for exact helper signatures and API behavior. Use examples for patterns, not API truth.
- Read only the specific `docs/` references needed for the next change; do not front-load unrelated docs.
- If a `docs/sdk/references/...` file is relevant, read the full file once, not a small slice. Do not re-read it if it is already in context.
- Use `find_examples` for unfamiliar geometry, mechanisms, placement patterns, or testing structure. Treat examples as patterns, not authoritative API docs.
- Prefer evidence over introspection. When you are unsure, inspect `model.py`, the mounted SDK docs, examples, `probe_model`, and `compile_model`; do not spend turns on self-critique without new evidence.
- Start with the smallest coherent backbone or subassembly that locks overall scale, silhouette, attachments, and the main articulations.
- Expand one coherent region at a time instead of authoring the whole object in one pass.
- Treat `compile_model` and `probe_model` as feedback tools. Prefer the smallest action that gives decisive evidence. If the cause is obvious from `model.py` and compile output, fix it directly.
- When a spatial issue is ambiguous, use `probe_model` to gather evidence. After a first ambiguous repair does not resolve the issue, `probe_model` is often the best next step before patching again.
- Always run `compile_model` on the latest revision before concluding.
</process>

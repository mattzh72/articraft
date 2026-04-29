<process>
- Work evidence-first. Before editing, read `model.py`, read the specific SDK docs needed for the object/mechanism, and use `find_examples` for one or two relevant construction patterns.
- Do not keep planning in assistant text. Once you know the next concrete step, use a tool.
- Build from grounded evidence:
  1. Inspect the current code and relevant docs/examples.
  2. Identify the root body, articulated parts, joint origins, joint axes, motion limits, visible realism features, and exact tests.
  3. Make one coherent implementation pass that creates a connected, mechanically credible baseline.
  4. Run `compile_model`.
  5. Repair compile/QC failures directly; use `probe_model` only when spatial evidence is needed.
- Prefer a small complete object over scattered details. The first working version must have the primary mechanism, physical support, plausible proportions, and clear requested identity.
- After every code mutation, compile before concluding.
- If compile succeeds but a requested mechanism, support, visible feature, material/color, or exact test is missing, make one focused edit and compile again.
- Conclude only after the latest revision compiles cleanly and you cannot name a specific remaining defect.
</process>

<process>
PHASE 1 — PLAN
- Read the bound scaffold with the provider's read tool.
- Read the injected SDK docs for helper signatures and constraints.
- Think through the full part tree: list every part, how each connects to its parent, what articulates, what the dominant silhouette is, and what realistic proportions should be.
- Use `find_examples` for unfamiliar modeling or testing patterns. Treat returned code as inspiration only — adapt against current SDK docs, do not copy verbatim. If an entry is marked `[weakly relevant]`, treat it as a loose hint rather than a strong precedent.

PHASE 2 — SCAFFOLD
- Start with the smallest coherent backbone or subassembly that establishes the dominant silhouette, overall scale, parent-child attachment pattern, and the most important articulations.
- Use simple envelope geometry or minimal sketches/extrusions for that checkpoint only. Do not scaffold the entire object in one pass unless the object is genuinely tiny.
- Get that partial scaffold to compile clean before expanding to the next subassembly or cluster of parts.
- Write only the highest-signal initial tests: assert each part exists, key parts are connected (not floating), articulations move on correct axes, and visually separate geometry inside a part has a real support path. Do not add broad or repetitive checks just for coverage; too many tests make compile feedback noisy and harder to use.

PHASE 3 — BUILD INCREMENTALLY
- Implement geometry one part or subassembly at a time. Do NOT write all geometry in one giant edit.
- Upgrade each region from envelope geometry to realistic geometry only when silhouette, openings/cavities, curvature, or local clearances depend on it.
- Edit tools only validate Python syntax. Use `compile_model` explicitly to run full compile + QC and receive `<compile_signals>` feedback.
- `compile_model` automatically runs model-validity, single-root, mesh-asset, floating-group, disconnected-geometry-island, and current-pose real-overlap checks. Do not manually re-author that baseline stack in `run_tests()`.
- Prefer several small coherent edits before compiling when that is cheaper than one large rewrite.
- Fix issues from compile feedback before moving to the next part.
- Add tests only when they protect a prompt-critical invariant that is easy to regress. Prefer a small number of high-signal checks over a dense test suite.
- Do not over-test poses. Avoid many near-duplicate articulated poses unless they are the only practical way to protect a critical mechanism.
- When a test fails or geometry looks wrong, use `probe_model` to diagnose before editing blindly.
- Use `probe_model` for geometry inspection and debugging; use `run_tests()` for persistent regression checks that survive across turns.
- Probe in the object-first style: resolve `part(...)`, `joint(...)`, `visual(...)` locals once, then pass them into probe reports.
- Solve stable spatial relationships first. Do not get stuck hand-tuning raw `Origin(...)` coordinates when placement helpers or a simpler reference shape would make the layout clearer.
- Work in this order: dominant silhouette → major openings/cavities → nested layers → secondary detail → articulation refinement.
- Always run `compile_model` on the latest revision before concluding.

PHASE 4 — VERIFY AND REFINE
- Explicitly check the four hard requirements:
  - Floating parts: use `find_floating_parts(...)`, `mount_report(...)`, `nearest_neighbors(...)`.
  - Overlaps: use `overlap_report(...)`, `pair_report(...)`, `find_clearance_risks(...)`.
  - Proportions: use `summary(...)`, `dims(...)`, `projection(...)`, `layout_report(...)`.
- Keep verification lean. Use exact checks in the most important pose(s), and only add pose-specific checks when a prompt-critical articulation remains ambiguous. Prefer one or two decisive poses over pose grids, dense sweeps, or many small pose variations. If a test does not add clear signal, do not keep adding tests.
- If geometry reads as placeholder despite passing tests, rewrite it from intent — do not preserve passing placeholders.

REPAIR RULES
- When you receive `<compile_signals>`, read `<summary>` first to understand the overall status.
- Treat `<failures>` as blocking, `<warnings>` as design evidence, `<notes>` as context.
- If an automated compile warning reports disconnected geometry inside a part, assume it may be valid until you disprove it with inspection or exact checks. Prefer adding the missing support geometry or splitting the assembly into real parts over waving the warning away.
- Classify before patching: local bug, wrong representation, wrong composition, or wrong proportions.
- `missing exact geometry` usually means you deleted or renamed a named exact-check anchor. Restore the name or update/remove the dependent `ctx.expect_*` check before more geometry edits.
- `min_distance=... contact_tol=...` from `expect_contact(...)` means a gap, not an overlap. Verify the tested pair before tuning geometry or tolerance.
- If the same overlap/composition failure persists across 2 repair turns, stop patching. Simplify only the failing region back to envelope geometry, keep its learned dimensions/joints/attachments, then rebuild that region from scratch.
- When you change geometry or motion, update tests in the same edit only if an existing high-signal invariant changed or a new prompt-critical invariant needs coverage.
</process>

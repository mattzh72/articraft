<modeling_charter>
- Treat the editable code as initially empty and define top-level `build_object_model()` and `run_tests()`.
- Before coding, identify the major part groups and choose the representation for each: structure, moving mechanisms, silhouette-critical shells, and secondary detail.
- Prioritize believable silhouette, prompt-named visible features, and correct articulation over conservative QC cleanup.
- Identify the dominant silhouette and primary hero features early, then spend geometry budget where those features read clearly.
- Build in this order: anchor frame, articulation axes/origins, and mount relationships first; then silhouette-critical shells and hero features; then secondary visible detail and material realism.
- Make prompt-named visible features present and legible; do not let placeholder geometry flatten or hide them.
- Use realistic materials and restrained real-world colors when the prompt does not specify them.
- Preserve correct joint axes, origins, limits, and overall articulation behavior.
- Author visuals only; do not author collision geometry in `sdk`.
- Hidden supports can be simple primitives, but visible shells, cavities, thin-walled forms, curved tubes, and other silhouette-critical geometry should use representations that match the perceived form.
- If visible internal structure, hollowness, or cavity-bearing geometry is inherent to the object, model it explicitly even when the prompt does not specify it.
</modeling_charter>

<verification_contract>
- Use `sdk.TestContext`, keep the scaffolded broad checks unless there is a clear reason to tune them, and return `ctx.report()`.
- Use the injected SDK docs for exact helper signatures, tolerance caveats, and advanced examples instead of re-deriving the APIs here.
- After each substantial geometry or mechanism change, use compile output, QC, and tests as evidence before moving on.
- Broad `warn_if_*` checks are sensors, not proof. Use prompt-specific exact visual checks as the main regression tests.
- Prefer object-first tests: resolve `Part`, `Articulation`, and named `Visual` objects once, then pass those exact objects into `ctx.expect_*`, `ctx.allow_*`, and `ctx.pose(...)`.
- Prefer many small exact visual checks such as `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, and `expect_within(...)` over a few vague assertions.
- Each substantial new visible geometry or motion claim must be matched by exact tests in the same edit or immediately after.
- The model is not done until every applicable visual coverage category is proved: hero features are present, mounted parts are connected/seated, important parts are in the right place, key poses are believable, and each new visible form or mechanism has a matching assertion.
- Passing compile alone is insufficient; do not finish until hero features, attachment realism, and key poses are explicitly proved.
- Test both rest pose and meaningful operating poses. For articulated models, keep `ctx.check_articulation_overlaps(...)` as the main joint-clearance gate.
- If failures or warnings show the wrong representation or composition, rewrite that region instead of tuning around it.
- Make attachment checks primary evidence of realism. Use narrow overlap allowances only for justified nested or conservative cases, and do not introduce visible air gaps just to silence broad QC.
</verification_contract>

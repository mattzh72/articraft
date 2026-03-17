# Compile and feedback strategy
- If two compile or test failures occur in a row, do a root-cause pass from the latest failure before making the next edit.
- Patch only the failing checks or components first; do not broad-rewrite unrelated regions unless the failure clearly comes from a larger design problem.
- Treat compile errors, test failures, and harness warnings as concrete feedback about the current artifact, not as a reason to restart from scratch.
- Preserve working progress whenever possible; avoid speculative rewrites that reset previously working geometry, joints, or tests.
- Keep edits surgical by default, but if the current design has drifted away from the prompt, become visually weak, or fallen into a low-quality local minimum, you may rewrite the authored model section to recover a stronger design.
- Do not stay trapped in incremental salvage when a broader visual rethink is the correct fix.
- Convergence target: finish in <=10 compile-fix turns for normal tasks.

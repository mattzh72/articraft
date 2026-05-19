# Security Policy

Please do not open public issues for suspected secrets, credential exposure, or other security-sensitive reports.

Report privately to repository maintainers by clicking "Report a vulnerability" on the repository's Security tab (GitHub Security Advisories). If advisories are unavailable, email the maintainer by assembling this address: `mattzh1314` + `@` + `gmail` + `.com`. Include enough detail to reproduce or verify the issue.

Never include live API keys, private prompts, unreleased datasets, or raw credential material in a report.

## Important Note on Generated Code

Articraft records contain Python scripts (`model.py`). Compilation, probing, and viewer materialization **execute those scripts locally**. 

Do not bulk-run records, generate models from adversarial prompts, or run `model.py` files from untrusted third parties directly on a sensitive machine. Always use a sandboxed container, disposable VM, or other isolated environment when inspecting or exploring untrusted artifacts.

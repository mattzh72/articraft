# Security Policy

Please do not open public issues for suspected secrets, credential exposure, or other security-sensitive reports.

Report privately through GitHub Security Advisories for this repository. If advisories are unavailable, contact the repository owner through their GitHub profile and include enough detail to reproduce or verify the issue.

Never include live API keys, private prompts, unreleased datasets, or raw credential material in a report.

## Generated Code

Articraft records contain Python model scripts. Compilation, probing, and viewer
materialization execute those scripts locally. Do not run records or generated
`model.py` files from untrusted sources on a sensitive machine. Use a sandboxed
container, disposable checkout, or other isolated environment when inspecting
untrusted artifacts.

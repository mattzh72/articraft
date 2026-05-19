from __future__ import annotations

import argparse
import sys
from pathlib import Path

from agent.prompts.spec import GENERATED_DIR, PromptVariant, iter_prompt_variants


def compile_prompt_variant(variant: PromptVariant) -> str:
    parts: list[str] = []
    for section in variant.sections:
        text = section.read_text(encoding="utf-8")
        if text and not text.endswith("\n"):
            text += "\n"
        parts.append(text.rstrip("\n"))
    compiled = "\n\n".join(part for part in parts if part)
    if not compiled.endswith("\n"):
        compiled += "\n"
    return compiled


def write_compiled_prompt(variant: PromptVariant) -> bool:
    compiled = compile_prompt_variant(variant)
    variant.output.parent.mkdir(parents=True, exist_ok=True)
    current = None
    if variant.output.exists():
        current = variant.output.read_text(encoding="utf-8")
    if current == compiled:
        return False
    variant.output.write_text(compiled, encoding="utf-8")
    return True


def compile_all_prompts() -> list[Path]:
    written: list[Path] = []
    GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    for variant in iter_prompt_variants():
        if write_compiled_prompt(variant):
            written.append(variant.output)
    return written


def find_stale_prompts() -> list[PromptVariant]:
    stale: list[PromptVariant] = []
    for variant in iter_prompt_variants():
        compiled = compile_prompt_variant(variant)
        if not variant.output.exists():
            stale.append(variant)
            continue
        current = variant.output.read_text(encoding="utf-8")
        if current != compiled:
            stale.append(variant)
    return stale


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Compile prompt sources into generated prompt files."
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Exit non-zero if generated prompts are stale instead of rewriting them.",
    )
    args = parser.parse_args(argv)

    if args.check:
        stale = find_stale_prompts()
        if not stale:
            print("Prompt artifacts are up to date.")
            return 0
        for variant in stale:
            print(f"Stale prompt artifact: {variant.output}")
        print("Run `python -m agent.prompts.compile` to refresh generated prompts.")
        return 1

    written = compile_all_prompts()
    if not written:
        print("Prompt artifacts already up to date.")
        return 0
    for path in written:
        print(f"Wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

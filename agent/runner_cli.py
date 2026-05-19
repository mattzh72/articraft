"""
Command-line interface for single Articraft generation runs.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import sys
from pathlib import Path
from typing import Awaitable, Callable, Optional

from dotenv import load_dotenv

from agent.cost import max_cost_usd_from_env, parse_max_cost_usd
from agent.payload_preview import build_provider_payload_preview
from agent.prompts import normalize_sdk_package
from agent.providers.factory import validate_provider_credentials
from agent.run_context import _default_model_id
from agent.single_run import run_from_input
from agent.tools import build_initial_user_content as _build_initial_user_content
from agent.tools import resolve_image_path as _resolve_image_path
from agent.tui.single_run import LLMWaitAwareStreamHandler
from articraft.values import PROVIDER_VALUES, THINKING_LEVEL_VALUES, ProviderName


def _load_qc_blurb_text(qc_blurb_path: Optional[str], *, repo_root: Path) -> Optional[str]:
    if not qc_blurb_path:
        return None

    path = Path(qc_blurb_path)
    if not path.is_absolute():
        path = (repo_root / path).resolve()
    if not path.exists():
        raise FileNotFoundError(f"QC blurb file not found: {path}")

    text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
    if text and not text.endswith("\n"):
        text += "\n"
    return text


def _build_prompt_with_qc(prompt: str, qc_blurb_text: Optional[str]) -> str:
    if not qc_blurb_text:
        return prompt

    return (
        f"{prompt.rstrip()}\n\n"
        "-----\n\n"
        "The following is a QC checklist to use as a final pass before declaring the model finished:\n\n"
        f"{qc_blurb_text}"
    )


def main(
    argv: list[str] | None = None,
    *,
    run_from_input_func: Callable[..., Awaitable[int]] = run_from_input,
    build_provider_payload_preview_func: Callable[..., dict] = build_provider_payload_preview,
    load_dotenv_func: Callable[[], object] = load_dotenv,
) -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s: %(message)s",
        handlers=[LLMWaitAwareStreamHandler()],
    )
    load_dotenv_func()

    parser = argparse.ArgumentParser(
        description="Generate an articulated object and persist it to workbench or dataset storage."
    )
    parser.add_argument("--prompt", required=True, help="Text prompt for the object.")
    parser.add_argument(
        "--image",
        default=None,
        help="Optional reference image to augment --prompt.",
    )
    parser.add_argument(
        "--provider",
        default="openai",
        choices=PROVIDER_VALUES,
        help="LLM provider.",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="Repository root where data/records and data/cache/runs will be written.",
    )
    parser.add_argument(
        "--label", default=None, help="Optional workbench label for the saved record."
    )
    parser.add_argument(
        "--tag", action="append", default=[], help="Optional workbench tag. Repeatable."
    )
    parser.add_argument(
        "--collection",
        default="workbench",
        choices=["workbench", "dataset"],
        help="Target collection for the generated record. Defaults to workbench.",
    )
    parser.add_argument(
        "--dataset-id",
        default=None,
        help="Stable dataset identifier to assign when --collection dataset is used.",
    )
    parser.add_argument(
        "--category",
        default=None,
        help="Optional category slug to attach to the record. Required for --collection dataset.",
    )
    parser.add_argument("--model", default=None, help="Model id (provider-specific).")
    parser.add_argument(
        "--openai-transport",
        default="http",
        choices=["http", "websocket"],
        help=(
            "Transport for --provider openai. "
            "`websocket` uses Responses WebSocket mode and enables response storage."
        ),
    )
    parser.add_argument(
        "--thinking",
        default="high",
        choices=THINKING_LEVEL_VALUES,
        help="Thinking budget level.",
    )
    parser.add_argument("--max-turns", type=int, default=None)
    parser.add_argument(
        "--max-cost-usd",
        type=float,
        default=None,
        help="Optional per-run USD budget. Stops after the first response that pushes cumulative spend above this threshold.",
    )
    parser.add_argument(
        "--system-prompt",
        default="designer_system_prompt.txt",
        help=(
            "Path or generated prompt name for the system prompt file. "
            "Standard designer prompt names resolve to provider-specific generated files automatically."
        ),
    )
    parser.add_argument(
        "--qc-blurb",
        default=None,
        help="Path to a markdown QC checklist to append to the prompt.",
    )
    parser.add_argument(
        "--dump-provider-payload",
        action="store_true",
        help="Print the provider request payload for turn 1 and exit (no API call).",
    )
    parser.add_argument(
        "--dump-provider-payload-out",
        default=None,
        help="Write the payload JSON to this path instead of stdout.",
    )
    parser.add_argument(
        "--dump-provider-payload-indent",
        type=int,
        default=2,
        help="JSON indent for --dump-provider-payload (default: 2).",
    )
    parser.add_argument(
        "--sdk-package",
        default="sdk",
        help=argparse.SUPPRESS,
    )
    args = parser.parse_args(argv)
    if args.collection == "dataset":
        if not args.dataset_id:
            parser.error("--dataset-id is required when --collection dataset.")
        if not args.category:
            parser.error("--category is required when --collection dataset.")
    elif args.dataset_id:
        parser.error("--dataset-id is only supported with --collection dataset.")

    try:
        sdk_package = normalize_sdk_package(args.sdk_package)
        max_cost_usd = (
            parse_max_cost_usd(args.max_cost_usd, label="--max-cost-usd")
            if args.max_cost_usd is not None
            else max_cost_usd_from_env()
        )
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    openai_reasoning_summary = "auto"

    if args.provider != ProviderName.OPENAI.value and args.openai_transport != "http":
        print("--openai-transport is only supported for --provider openai.", file=sys.stderr)
        return 1

    repo_root = args.repo_root.resolve()
    try:
        qc_blurb_text = _load_qc_blurb_text(args.qc_blurb, repo_root=repo_root)
    except Exception as exc:
        print(f"Failed to load qc blurb: {exc}", file=sys.stderr)
        return 1

    try:
        image_path = _resolve_image_path(args.image, provider=args.provider)
    except Exception as exc:
        print(f"Failed to load image: {exc}", file=sys.stderr)
        return 1

    prompt_with_qc = _build_prompt_with_qc(args.prompt, qc_blurb_text)
    user_content = _build_initial_user_content(prompt_with_qc, image_path=image_path)

    if args.dump_provider_payload:
        model_id = _default_model_id(
            provider=args.provider,
            model_id=args.model,
            thinking_level=args.thinking,
            openai_transport=args.openai_transport,
            openai_reasoning_summary=openai_reasoning_summary,
        )

        payload = build_provider_payload_preview_func(
            user_content,
            provider=args.provider,
            model_id=model_id,
            openai_transport=args.openai_transport,
            thinking_level=args.thinking,
            system_prompt_path=args.system_prompt,
            sdk_package=sdk_package,
            openai_reasoning_summary=openai_reasoning_summary,
        )
        text = json.dumps(payload, indent=args.dump_provider_payload_indent, ensure_ascii=False)
        if args.dump_provider_payload_out:
            out_path = Path(args.dump_provider_payload_out)
            if not out_path.is_absolute():
                out_path = (Path.cwd() / out_path).resolve()
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(text, encoding="utf-8")
        else:
            print(text)
        return 0

    try:
        validate_provider_credentials(args.provider)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    return asyncio.run(
        run_from_input_func(
            user_content,
            prompt_text=prompt_with_qc,
            display_prompt=args.prompt,
            repo_root=args.repo_root.resolve(),
            image_path=image_path,
            provider=args.provider,
            model_id=args.model,
            openai_transport=args.openai_transport,
            thinking_level=args.thinking,
            max_turns=args.max_turns,
            system_prompt_path=args.system_prompt,
            sdk_package=sdk_package,
            openai_reasoning_summary=openai_reasoning_summary,
            max_cost_usd=max_cost_usd,
            label=args.label,
            tags=list(args.tag or []),
            collection=args.collection,
            category_slug=args.category,
            dataset_id=args.dataset_id,
        )
    )

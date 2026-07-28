from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import random
import re
import shutil
import sys
import traceback
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from dotenv import load_dotenv

from agent.compiler import compile_urdf_report_maybe_timeout
from agent.feedback import compile_signal_bundle_from_exception
from agent.harness import ArticraftAgent
from agent.models import AgentResult
from agent.prompts import load_system_prompt_text
from agent.providers.openai import OpenAILLM, openai_api_keys_from_env
from articraft.values import THINKING_LEVEL_VALUES
from sdk._profiles import get_sdk_profile

REPO_ROOT = Path(__file__).resolve().parents[2]
EXPERIMENT_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "performance" / "results" / "agent_component_ablations"
DEFAULT_MODEL = "gpt-5.6-sol"
DEFAULT_THINKING_LEVEL = "high"
PROMPT_FILE = EXPERIMENT_DIR / "base_prompts.json"
BASE_SYSTEM_PROMPT = (
    REPO_ROOT / "agent" / "prompts" / "generated" / ("designer_system_prompt_openai.txt")
)
NO_COMPILE_RUNTIME_GUIDANCE = """<runtime_task_guidance>
- Read the current `model.py` before editing.
- Plan a realistic rooted assembly and then edit the file until the requested object is complete.
- The compile tool and testing SDK are intentionally unavailable in this condition.
- Use `probe_model` only for read-only API or model inspection.
- Conclude when the file represents the requested object.
</runtime_task_guidance>"""


@dataclass(frozen=True)
class Condition:
    id: str
    label: str
    mode: str
    sdk_package: str
    compile_feedback: bool
    example_retrieval: bool
    require_compile_before_finish: bool


CONDITIONS = (
    Condition(
        id="full_baseline",
        label="Full SDK, compile feedback, tests, retrieval, iterative",
        mode="iterative",
        sdk_package="sdk",
        compile_feedback=True,
        example_retrieval=True,
        require_compile_before_finish=True,
    ),
    Condition(
        id="primitives_only",
        label="Primitive SDK only",
        mode="iterative",
        sdk_package="sdk_primitives",
        compile_feedback=True,
        example_retrieval=True,
        require_compile_before_finish=True,
    ),
    Condition(
        id="no_compile_feedback",
        label="No compile feedback and no testing SDK",
        mode="iterative",
        sdk_package="sdk_no_testing",
        compile_feedback=False,
        example_retrieval=True,
        require_compile_before_finish=False,
    ),
    Condition(
        id="no_example_retrieval",
        label="Full baseline without example retrieval",
        mode="iterative",
        sdk_package="sdk",
        compile_feedback=True,
        example_retrieval=False,
        require_compile_before_finish=True,
    ),
    Condition(
        id="single_pass",
        label="One request, no tools, no examples",
        mode="single_pass",
        sdk_package="sdk",
        compile_feedback=False,
        example_retrieval=False,
        require_compile_before_finish=False,
    ),
)
CONDITIONS_BY_ID = {condition.id: condition for condition in CONDITIONS}


def _now() -> str:
    return datetime.now(UTC).isoformat()


def _json_default(value: object) -> str:
    if isinstance(value, Path):
        return str(value)
    return repr(value)


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, default=_json_default) + "\n",
        encoding="utf-8",
    )


def _load_prompts() -> tuple[dict[str, Any], list[dict[str, str]]]:
    payload = json.loads(PROMPT_FILE.read_text(encoding="utf-8"))
    prompts = payload.get("prompts")
    if not isinstance(prompts, list) or len(prompts) != 10:
        raise ValueError(f"Expected exactly 10 prompts in {PROMPT_FILE}")
    normalized: list[dict[str, str]] = []
    for item in prompts:
        if not isinstance(item, dict):
            raise TypeError("Every prompt must be an object")
        normalized.append(
            {
                "id": str(item["id"]),
                "category": str(item["category"]),
                "prompt": str(item["prompt"]),
            }
        )
    return payload, normalized


def _filter_prompt_lines(text: str, blocked_terms: tuple[str, ...]) -> str:
    lines = [
        line
        for line in text.splitlines()
        if not any(term.lower() in line.lower() for term in blocked_terms)
    ]
    return "\n".join(lines).strip() + "\n"


def system_prompt_for_condition(condition: Condition) -> str:
    _, base = load_system_prompt_text(
        str(BASE_SYSTEM_PROMPT),
        provider="openai",
        sdk_package="sdk",
    )
    if condition.id == "full_baseline":
        return base

    if condition.id == "no_example_retrieval":
        return _filter_prompt_lines(base, ("find_examples", "examples are admissible"))

    if condition.id == "primitives_only":
        prompt = base.replace("from `sdk`", "from `sdk_primitives`")
        return (
            prompt
            + """
<experimental_condition>
This is the primitives-only condition. Import public authoring APIs only from
`sdk_primitives`. Use only Box, Cylinder, and Sphere for all visible geometry.
CadQuery, meshes, asset builders, and higher-level geometry helpers are
unavailable. This restriction overrides any earlier geometry recommendation.
Retrieved examples come only from the compatible primitives example set.
</experimental_condition>
"""
        )

    if condition.id == "no_compile_feedback":
        prompt = _filter_prompt_lines(
            base,
            (
                "compile_model",
                "compile output",
                "run_tests",
                "testcontext",
                "testing",
                "ctx.",
                "allow_overlap",
                "passes validation",
            ),
        )
        prompt = prompt.replace("from `sdk`", "from `sdk_no_testing`")
        return (
            prompt
            + """
<experimental_condition>
This condition intentionally provides no compile tool, no compile feedback, and
no testing SDK. Import from `sdk_no_testing`. Do not define run_tests and do not
import testing types. Build the model through the normal iterative editing loop,
then return a visible final response when the object is complete.
</experimental_condition>
"""
        )

    if condition.id == "single_pass":
        prompt = re.sub(r"<tools>.*?</tools>\s*", "", base, flags=re.DOTALL)
        prompt = _filter_prompt_lines(
            prompt,
            (
                "tool",
                "find_examples",
                "examples are admissible",
                "never answer with code",
                "compile output",
            ),
        )
        return (
            prompt
            + """
<single_pass_output_contract>
You receive the object request, the initial scaffold, and the full SDK
documentation in one message. Examples are intentionally absent. Produce the
complete contents of main.py in this one response. Return raw Python only. Do not
use Markdown fences and do not include explanations before or after the code.
There is no repair turn and no tool feedback.
</single_pass_output_contract>
"""
        )

    raise ValueError(f"Unknown condition: {condition.id}")


def _single_pass_input(prompt: dict[str, str]) -> str:
    profile = get_sdk_profile("sdk")
    parts = [
        "<object_request>",
        prompt["prompt"],
        "</object_request>",
        "",
        "<initial_scaffold>",
        (REPO_ROOT / profile.scaffold_path).read_text(encoding="utf-8"),
        "</initial_scaffold>",
        "",
        "<sdk_documentation>",
    ]
    for rel_path in profile.docs_full:
        parts.extend(
            [
                f"\n## {rel_path.as_posix()}",
                (REPO_ROOT / rel_path).read_text(encoding="utf-8"),
            ]
        )
    parts.append("</sdk_documentation>")
    return "\n".join(parts)


def _strip_code_fence(text: str) -> str:
    candidate = text.strip()
    match = re.fullmatch(r"```(?:python|py)?\s*\n(?P<code>.*)\n```\s*", candidate, re.DOTALL)
    if match:
        candidate = match.group("code")
    return candidate.rstrip() + "\n"


def _agent_result_dict(result: AgentResult) -> dict[str, Any]:
    payload = asdict(result)
    payload["reason"] = str(result.reason)
    return payload


def _compile_stage(
    script_path: Path,
    *,
    sdk_package: str,
    run_checks: bool,
    urdf_path: Path | None = None,
) -> dict[str, Any]:
    started_at = _now()
    try:
        report = compile_urdf_report_maybe_timeout(
            script_path,
            sdk_package=sdk_package,
            run_checks=run_checks,
            rewrite_visual_glb=False,
        )
        if urdf_path is not None:
            urdf_path.write_text(report.urdf_xml, encoding="utf-8")
        return {
            "status": "success",
            "started_at": started_at,
            "finished_at": _now(),
            "warnings": report.warnings,
            "signals": report.signal_bundle.to_dict(),
            "urdf_path": str(urdf_path) if urdf_path is not None else None,
        }
    except BaseException as exc:
        return {
            "status": "failure",
            "started_at": started_at,
            "finished_at": _now(),
            "error_type": type(exc).__name__,
            "error": str(exc),
            "signals": compile_signal_bundle_from_exception(exc).to_dict(),
            "traceback": traceback.format_exc(),
        }


def evaluate_cell(script_path: Path, *, sdk_package: str) -> dict[str, Any]:
    cell_dir = script_path.parent
    export_result = _compile_stage(
        script_path,
        sdk_package=sdk_package,
        run_checks=False,
        urdf_path=cell_dir / "model.urdf",
    )
    native_result = _compile_stage(
        script_path,
        sdk_package=sdk_package,
        run_checks=True,
    )

    source = script_path.read_text(encoding="utf-8")
    standardized_path = cell_dir / "standardized_evaluation.py"
    standardized_path.write_text(
        source.rstrip()
        + """


# Experimental hidden evaluation. This function is never shown to the agent.
from sdk import TestContext as _AblationTestContext


def run_tests():
    return _AblationTestContext(object_model).report()
""",
        encoding="utf-8",
    )
    standardized_result = _compile_stage(
        standardized_path,
        sdk_package="sdk",
        run_checks=True,
    )
    return {
        "evaluated_at": _now(),
        "export_only": export_result,
        "native_full_compile": native_result,
        "standardized_baseline_qc": standardized_result,
    }


async def _run_iterative(
    *,
    cell_dir: Path,
    prompt: dict[str, str],
    condition: Condition,
    system_prompt_path: Path,
    max_turns: int | None,
    max_cost_usd: float | None,
    model: str,
    thinking_level: str,
) -> dict[str, Any]:
    profile = get_sdk_profile(condition.sdk_package)
    script_path = cell_dir / "main.py"
    shutil.copyfile(REPO_ROOT / profile.scaffold_path, script_path)
    agent = ArticraftAgent(
        file_path=str(script_path),
        provider="openai",
        model_id=model,
        thinking_level=thinking_level,
        max_turns=max_turns,
        max_cost_usd=max_cost_usd,
        system_prompt_path=str(system_prompt_path),
        trace_dir=str(cell_dir / "trace"),
        checkpoint_urdf_path=cell_dir / "agent_checkpoint.urdf",
        sdk_package=condition.sdk_package,
        display_enabled=False,
        enable_compile_feedback=condition.compile_feedback,
        enable_example_retrieval=condition.example_retrieval,
        require_compile_before_finish=condition.require_compile_before_finish,
        runtime_guidance_text=(
            NO_COMPILE_RUNTIME_GUIDANCE if not condition.compile_feedback else None
        ),
    )
    try:
        result = await agent.run(prompt["prompt"])
        payload = _agent_result_dict(result)
        _write_json(cell_dir / "agent_result.json", payload)
        return payload
    finally:
        await agent.close()


async def _run_single_pass(
    *,
    cell_dir: Path,
    prompt: dict[str, str],
    system_prompt: str,
    model: str,
    thinking_level: str,
) -> dict[str, Any]:
    client = OpenAILLM(model_id=model, thinking_level=thinking_level)
    input_text = _single_pass_input(prompt)
    (cell_dir / "single_pass_input.txt").write_text(input_text, encoding="utf-8")
    try:
        response = await client.generate_with_tools(
            system_prompt=system_prompt,
            messages=[{"role": "user", "content": input_text}],
            tools=[],
        )
        _write_json(cell_dir / "provider_response.json", response)
        code = _strip_code_fence(str(response.get("content") or ""))
        (cell_dir / "main.py").write_text(code, encoding="utf-8")
        return {
            "success": bool(code.strip()),
            "reason": "SINGLE_PASS_RESPONSE",
            "message": "One tool-free provider request completed.",
            "turn_count": 1,
            "tool_call_count": 0,
            "compile_attempt_count": 0,
            "usage": response.get("usage"),
            "provider_diagnostics": response.get("provider_diagnostics"),
        }
    finally:
        await client.close()


def _cell_id(prompt_id: str, condition_id: str) -> str:
    return f"{prompt_id}__{condition_id}"


async def _run_cell(
    *,
    run_dir: Path,
    prompt: dict[str, str],
    condition: Condition,
    max_turns: int | None,
    max_cost_usd: float | None,
    model: str,
    thinking_level: str,
    force: bool,
) -> dict[str, Any]:
    cell_id = _cell_id(prompt["id"], condition.id)
    cell_dir = run_dir / "cells" / cell_id
    status_path = cell_dir / "status.json"
    if status_path.exists() and not force:
        prior = json.loads(status_path.read_text(encoding="utf-8"))
        if prior.get("status") == "complete":
            return prior

    cell_dir.mkdir(parents=True, exist_ok=True)
    system_prompt = system_prompt_for_condition(condition)
    system_prompt_path = cell_dir / "system_prompt.txt"
    system_prompt_path.write_text(system_prompt, encoding="utf-8")
    _write_json(cell_dir / "prompt.json", prompt)
    _write_json(cell_dir / "condition.json", asdict(condition))
    started_at = _now()
    _write_json(
        status_path,
        {"cell_id": cell_id, "status": "running", "started_at": started_at},
    )
    try:
        if condition.mode == "single_pass":
            generation = await _run_single_pass(
                cell_dir=cell_dir,
                prompt=prompt,
                system_prompt=system_prompt,
                model=model,
                thinking_level=thinking_level,
            )
        else:
            generation = await _run_iterative(
                cell_dir=cell_dir,
                prompt=prompt,
                condition=condition,
                system_prompt_path=system_prompt_path,
                max_turns=max_turns,
                max_cost_usd=max_cost_usd,
                model=model,
                thinking_level=thinking_level,
            )
        _write_json(cell_dir / "generation_result.json", generation)
        evaluation = await asyncio.to_thread(
            evaluate_cell,
            cell_dir / "main.py",
            sdk_package=condition.sdk_package,
        )
        _write_json(cell_dir / "evaluation.json", evaluation)
        status = {
            "cell_id": cell_id,
            "prompt_id": prompt["id"],
            "condition_id": condition.id,
            "status": "complete",
            "started_at": started_at,
            "finished_at": _now(),
            "generation_success": bool(generation.get("success")),
            "export_success": evaluation["export_only"]["status"] == "success",
            "standardized_qc_success": (
                evaluation["standardized_baseline_qc"]["status"] == "success"
            ),
        }
    except BaseException as exc:
        status = {
            "cell_id": cell_id,
            "prompt_id": prompt["id"],
            "condition_id": condition.id,
            "status": "failed",
            "started_at": started_at,
            "finished_at": _now(),
            "error_type": type(exc).__name__,
            "error": str(exc),
            "traceback": traceback.format_exc(),
        }
    _write_json(status_path, status)
    return status


def _schedule(
    prompts: list[dict[str, str]],
    conditions: list[Condition],
    *,
    seed: int,
) -> list[tuple[dict[str, str], Condition]]:
    cells = [(prompt, condition) for prompt in prompts for condition in conditions]
    random.Random(seed).shuffle(cells)
    return cells


async def _run_schedule(
    *,
    run_dir: Path,
    schedule: list[tuple[dict[str, str], Condition]],
    concurrency: int,
    max_turns: int | None,
    max_cost_usd: float | None,
    model: str,
    thinking_level: str,
    force: bool,
) -> list[dict[str, Any]]:
    semaphore = asyncio.Semaphore(concurrency)
    result_lock = asyncio.Lock()
    results_path = run_dir / "results.jsonl"

    async def run_one(index: int, prompt: dict[str, str], condition: Condition):
        async with semaphore:
            print(
                f"[{index + 1}/{len(schedule)}] {prompt['id']} / {condition.id}",
                flush=True,
            )
            status = await _run_cell(
                run_dir=run_dir,
                prompt=prompt,
                condition=condition,
                max_turns=max_turns,
                max_cost_usd=max_cost_usd,
                model=model,
                thinking_level=thinking_level,
                force=force,
            )
            async with result_lock:
                with results_path.open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps(status, sort_keys=True) + "\n")
            print(f"  {status['status']}: {status['cell_id']}", flush=True)
            return status

    return await asyncio.gather(
        *(run_one(index, prompt, condition) for index, (prompt, condition) in enumerate(schedule))
    )


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the Articraft agent component ablations.")
    parser.add_argument("--run-id", help="Reusable output run ID. Defaults to a UTC timestamp.")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--concurrency", type=int, default=4)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument(
        "--thinking-level",
        choices=THINKING_LEVEL_VALUES,
        default=DEFAULT_THINKING_LEVEL,
    )
    parser.add_argument("--seed", type=int, default=20260727)
    parser.add_argument("--prompt", action="append")
    parser.add_argument("--condition", action="append", choices=sorted(CONDITIONS_BY_ID))
    parser.add_argument("--limit", type=int)
    parser.add_argument("--max-turns", type=int)
    parser.add_argument("--max-cost-usd", type=float)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.concurrency < 1:
        raise ValueError("--concurrency must be at least 1")
    if args.limit is not None and args.limit < 1:
        raise ValueError("--limit must be at least 1")

    prompt_metadata, prompts = _load_prompts()
    if args.prompt:
        requested_prompt_ids = set(args.prompt)
        known_prompt_ids = {prompt["id"] for prompt in prompts}
        unknown_prompt_ids = requested_prompt_ids - known_prompt_ids
        if unknown_prompt_ids:
            raise ValueError(f"Unknown prompt IDs: {', '.join(sorted(unknown_prompt_ids))}")
        prompts = [prompt for prompt in prompts if prompt["id"] in requested_prompt_ids]
    condition_ids = args.condition or [condition.id for condition in CONDITIONS]
    conditions = [CONDITIONS_BY_ID[condition_id] for condition_id in condition_ids]
    schedule = _schedule(prompts, conditions, seed=args.seed)
    if args.limit is not None:
        schedule = schedule[: args.limit]

    run_id = args.run_id or datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    run_dir = args.output_root.expanduser().resolve() / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        "experiment": "agent_component_ablations",
        "experimental_only": True,
        "do_not_merge": True,
        "created_at": _now(),
        "run_id": run_id,
        "model": args.model,
        "thinking_level": args.thinking_level,
        "seed": args.seed,
        "concurrency": args.concurrency,
        "max_turns": args.max_turns,
        "max_cost_usd_per_cell": args.max_cost_usd,
        "prompt_source_sha256": hashlib.sha256(PROMPT_FILE.read_bytes()).hexdigest(),
        "prompt_metadata": prompt_metadata,
        "conditions": [asdict(condition) for condition in conditions],
        "schedule": [
            {
                "cell_id": _cell_id(prompt["id"], condition.id),
                "prompt_id": prompt["id"],
                "condition_id": condition.id,
            }
            for prompt, condition in schedule
        ],
        "cell_count": len(schedule),
        "dry_run": bool(args.dry_run),
    }
    _write_json(run_dir / "manifest.json", manifest)
    for condition in conditions:
        prompt_text = system_prompt_for_condition(condition)
        condition_dir = run_dir / "condition_inputs" / condition.id
        condition_dir.mkdir(parents=True, exist_ok=True)
        (condition_dir / "system_prompt.txt").write_text(prompt_text, encoding="utf-8")
        _write_json(condition_dir / "condition.json", asdict(condition))
    if any(condition.mode == "single_pass" for condition in conditions):
        _write_json(
            run_dir / "condition_inputs" / "single_pass" / "input_sizes.json",
            {prompt["id"]: len(_single_pass_input(prompt)) for prompt in prompts},
        )

    print(f"Run directory: {run_dir}")
    print(f"Scheduled cells: {len(schedule)}")
    if args.dry_run:
        print("Dry run complete. No model requests were made.")
        return 0

    load_dotenv(REPO_ROOT / ".env", override=False)
    if not openai_api_keys_from_env():
        raise ValueError("Set OPENAI_API_KEY or OPENAI_API_KEYS before starting the experiment.")
    results = asyncio.run(
        _run_schedule(
            run_dir=run_dir,
            schedule=schedule,
            concurrency=args.concurrency,
            max_turns=args.max_turns,
            max_cost_usd=args.max_cost_usd,
            model=args.model,
            thinking_level=args.thinking_level,
            force=args.force,
        )
    )
    complete = sum(result.get("status") == "complete" for result in results)
    failed = len(results) - complete
    _write_json(
        run_dir / "summary.json",
        {
            "finished_at": _now(),
            "cell_count": len(results),
            "complete": complete,
            "failed": failed,
            "results": results,
        },
    )
    print(f"Finished. Complete: {complete}. Failed: {failed}.")
    print(f"Results: {run_dir}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

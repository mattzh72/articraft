from __future__ import annotations

import asyncio
import json
import sys
from pathlib import Path

from agent.prompts import normalize_sdk_package
from agent.runtime_limits import BatchRuntimeLimits, local_work_slot
from agent.tools.base import (
    BaseDeclarativeTool,
    BoundFileToolInvocation,
    ToolParamsModel,
    ToolResult,
    make_tool_schema,
)
from agent.tools.probe_model.description import PROBE_MODEL_DESCRIPTION


class ProbeModelParams(ToolParamsModel):
    code: str
    timeout_ms: int = 10_000
    include_stdout: bool = False


class ProbeModelInvocation(BoundFileToolInvocation[ProbeModelParams, dict[str, object]]):
    def __init__(
        self,
        params: ProbeModelParams,
        *,
        sdk_package: str,
        runtime_limits: BatchRuntimeLimits | None = None,
    ) -> None:
        super().__init__(params)
        self.sdk_package = normalize_sdk_package(sdk_package)
        self.runtime_limits = runtime_limits

    def get_description(self) -> str:
        preview = self.params.code[:50].replace("\n", "\\n")
        if len(self.params.code) > 50:
            preview += "..."
        return f"Probe geometry in current target file: '{preview}'"

    async def execute(self) -> ToolResult:
        if not self.file_path:
            return ToolResult(error="file_path is required")
        if self.params.timeout_ms < 100:
            return ToolResult(error="timeout_ms must be >= 100")
        file_path = Path(self.file_path).resolve()
        if not file_path.exists():
            return ToolResult(error=f"File {file_path} not found")

        request = {
            "file_path": str(file_path),
            "sdk_package": self.sdk_package,
            "code": self.params.code,
        }
        repo_root = Path(__file__).resolve().parents[3]
        started = asyncio.get_running_loop().time()
        async with local_work_slot(self.runtime_limits):
            process = await asyncio.create_subprocess_exec(
                sys.executable,
                "-m",
                "agent.tools.probe_model.runner",
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                cwd=str(repo_root),
            )
            try:
                stdout_bytes, stderr_bytes = await asyncio.wait_for(
                    process.communicate(json.dumps(request).encode("utf-8")),
                    timeout=self.params.timeout_ms / 1000.0,
                )
            except asyncio.TimeoutError:
                process.kill()
                await process.communicate()
                elapsed_ms = (asyncio.get_running_loop().time() - started) * 1000.0
                return ToolResult(
                    output={
                        "ok": False,
                        "error": {
                            "type": "timeout",
                            "message": f"probe_model timed out after {self.params.timeout_ms} ms",
                        },
                        "elapsed_ms": float(elapsed_ms),
                    },
                )

        stdout_text = stdout_bytes.decode("utf-8", errors="replace").strip()
        stderr_text = stderr_bytes.decode("utf-8", errors="replace").strip()
        elapsed_ms = (asyncio.get_running_loop().time() - started) * 1000.0
        if process.returncode not in (0, None):
            payload = {
                "ok": False,
                "error": {
                    "type": "runner_process_error",
                    "message": f"probe_model runner exited with code {process.returncode}",
                },
                "runner_stdout": stdout_text,
                "runner_stderr": stderr_text,
            }
        elif not stdout_text:
            payload = {
                "ok": False,
                "error": {
                    "type": "invalid_runner_output",
                    "message": "probe_model runner returned no JSON output",
                },
                "runner_stdout": stdout_text,
                "runner_stderr": stderr_text,
            }
        else:
            try:
                payload = json.loads(stdout_text)
            except json.JSONDecodeError:
                payload = {
                    "ok": False,
                    "error": {
                        "type": "invalid_runner_output",
                        "message": "probe_model runner returned invalid JSON",
                    },
                    "runner_stdout": stdout_text,
                    "runner_stderr": stderr_text,
                }
        if not isinstance(payload, dict):
            payload = {
                "ok": False,
                "error": {
                    "type": "invalid_runner_output",
                    "message": "probe_model runner did not return a JSON object",
                },
                "runner_stdout": stdout_text,
                "runner_stderr": stderr_text,
            }
        elif not isinstance(payload.get("ok"), bool):
            payload = {
                "ok": False,
                "error": {
                    "type": "invalid_runner_output",
                    "message": "probe_model runner returned malformed JSON payload",
                },
                "runner_stdout": stdout_text,
                "runner_stderr": stderr_text,
            }
        payload.setdefault("elapsed_ms", float(elapsed_ms))
        if stderr_text and "stderr" not in payload:
            payload["stderr"] = stderr_text
        if not self.params.include_stdout:
            payload.pop("stdout", None)
            payload.pop("stderr", None)
            payload.pop("runner_stdout", None)
            payload.pop("runner_stderr", None)
        return ToolResult(output=payload)


class ProbeModelTool(BaseDeclarativeTool):
    def __init__(
        self,
        *,
        sdk_package: str,
        runtime_limits: BatchRuntimeLimits | None = None,
    ) -> None:
        self.sdk_package = normalize_sdk_package(sdk_package)
        self.runtime_limits = runtime_limits
        schema = make_tool_schema(
            name="probe_model",
            description=PROBE_MODEL_DESCRIPTION,
            parameters={
                "code": {
                    "type": "string",
                    "description": (
                        "Python snippet to execute against the current bound model. "
                        "Call emit(value) exactly once with a JSON-serializable result."
                    ),
                },
                "timeout_ms": {
                    "type": "integer",
                    "description": "Execution timeout in milliseconds (default: 10000).",
                },
                "include_stdout": {
                    "type": "boolean",
                    "description": (
                        "When true, include captured print/stdout and stderr text in the result."
                    ),
                },
            },
            required=["code"],
        )
        super().__init__("probe_model", schema)

    async def build(self, params: dict) -> ProbeModelInvocation:
        validated = ProbeModelParams(**params)
        return ProbeModelInvocation(
            validated,
            sdk_package=self.sdk_package,
            runtime_limits=self.runtime_limits,
        )

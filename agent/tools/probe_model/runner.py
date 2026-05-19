from __future__ import annotations

import importlib
import io
import json
import sys
import traceback
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from typing import Any

from agent.compiler import load_model_globals
from agent.tools.probe_model.helpers import ProbeLookupError, ProbeSession
from sdk._core.v0.assets import (
    activate_asset_session,
    asset_session_for_script,
    get_active_asset_session,
)


class _EmitContractError(RuntimeError):
    pass


def _jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(key): _jsonable(val) for key, val in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "xyz") and hasattr(value, "rpy"):
        return {
            "xyz": [float(v) for v in getattr(value, "xyz")],
            "rpy": [float(v) for v in getattr(value, "rpy")],
        }
    return value


def _payload(
    *,
    ok: bool,
    result: Any = None,
    error_type: str | None = None,
    message: str | None = None,
    stdout: str = "",
    stderr: str = "",
    traceback_text: str | None = None,
) -> dict[str, Any]:
    output: dict[str, Any] = {
        "ok": ok,
    }
    if ok:
        output["result"] = result
    else:
        output["error"] = {
            "type": str(error_type or "unknown_error"),
            "message": str(message or "Unknown probe_model failure"),
        }
        if traceback_text:
            output["error"]["traceback"] = traceback_text
    if stdout:
        output["stdout"] = stdout
    if stderr:
        output["stderr"] = stderr
    return output


def _asset_root_from_globals(globals_dict: dict[str, Any], *, file_path: Path) -> Path:
    assets = globals_dict.get("ASSETS")
    asset_root = getattr(assets, "asset_root", None)
    if asset_root is not None:
        return Path(asset_root).resolve()
    asset_session = get_active_asset_session()
    if asset_session is not None:
        return asset_session.asset_root
    return file_path.parent.resolve()


def main() -> int:
    raw = sys.stdin.read()
    stdout_buffer = io.StringIO()
    stderr_buffer = io.StringIO()
    try:
        request = json.loads(raw)
        if not isinstance(request, dict):
            raise ValueError("probe_model runner input must be a JSON object")
        file_path = Path(str(request["file_path"])).resolve()
        code = str(request["code"])
        sdk_package = str(request.get("sdk_package") or "sdk")
    except Exception as exc:
        payload = _payload(
            ok=False,
            error_type="invalid_request",
            message=str(exc),
        )
        sys.stdout.write(json.dumps(payload))
        return 0

    emitted_value: Any = None
    emit_count = 0

    def emit(value: Any) -> None:
        nonlocal emit_count, emitted_value
        emit_count += 1
        if emit_count > 1:
            raise _EmitContractError("emit(value) must be called exactly once")
        emitted_value = value

    stage = "load"
    try:
        with activate_asset_session(asset_session_for_script(file_path)):
            globals_dict = load_model_globals(file_path, sdk_package=sdk_package)
            object_model = globals_dict.get("object_model")
            if object_model is None:
                raise ValueError("Loaded script did not define `object_model`")
            sdk_module = importlib.import_module(sdk_package)
            test_context_type = getattr(sdk_module, "TestContext")
            asset_root = _asset_root_from_globals(
                globals_dict,
                file_path=file_path,
            )
            ctx = test_context_type(object_model, asset_root=asset_root)
            session = ProbeSession(object_model, ctx)
            namespace = {
                "__name__": "__probe_model__",
                "__file__": str(file_path),
                "Origin": getattr(sdk_module, "Origin"),
            }
            namespace.update(session.build_namespace(emit=emit))
            stage = "exec"
            compiled = compile(code, str(file_path.with_suffix(".probe.py")), "exec")
            with redirect_stdout(stdout_buffer), redirect_stderr(stderr_buffer):
                exec(compiled, namespace, namespace)
            if emit_count == 0:
                raise _EmitContractError("emit(value) was not called")
            emitted_value = _jsonable(emitted_value)
            json.dumps(emitted_value)
            payload = _payload(
                ok=True,
                result=emitted_value,
                stdout=stdout_buffer.getvalue(),
                stderr=stderr_buffer.getvalue(),
            )
    except ProbeLookupError as exc:
        payload = _payload(
            ok=False,
            error_type="lookup_failure",
            message=str(exc),
            stdout=stdout_buffer.getvalue(),
            stderr=stderr_buffer.getvalue(),
        )
    except _EmitContractError as exc:
        payload = _payload(
            ok=False,
            error_type="emit_contract",
            message=str(exc),
            stdout=stdout_buffer.getvalue(),
            stderr=stderr_buffer.getvalue(),
        )
    except TypeError as exc:
        if "not JSON serializable" in str(exc):
            payload = _payload(
                ok=False,
                error_type="non_serializable_result",
                message=str(exc),
                stdout=stdout_buffer.getvalue(),
                stderr=stderr_buffer.getvalue(),
            )
        else:
            payload = _payload(
                ok=False,
                error_type="snippet_exception",
                message=str(exc),
                stdout=stdout_buffer.getvalue(),
                stderr=stderr_buffer.getvalue(),
                traceback_text=traceback.format_exc(),
            )
    except Exception as exc:
        error_type = "snippet_exception" if stage == "exec" else "load_failure"
        payload = _payload(
            ok=False,
            error_type=error_type,
            message=str(exc),
            stdout=stdout_buffer.getvalue(),
            stderr=stderr_buffer.getvalue(),
            traceback_text=traceback.format_exc(),
        )

    sys.stdout.write(json.dumps(payload))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

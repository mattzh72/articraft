from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from agent import runner
from agent.models import AgentResult, TerminateReason
from agent.prompts import DESIGNER_PROMPT_NAME


class FakeAgent:
    def __init__(
        self,
        *,
        file_path: str,
        trace_dir: str | None = None,
        provider: str = "openai",
        model_id: str | None = None,
        **_: object,
    ) -> None:
        self.file_path = Path(file_path)
        self.trace_dir = Path(trace_dir) if trace_dir else None
        self.provider = provider
        self.loaded_system_prompt_path = str(
            runner.resolve_system_prompt_path(
                DESIGNER_PROMPT_NAME,
                provider=provider,
                repo_root=Path(__file__).resolve().parents[1],
            )
        )
        self.llm = SimpleNamespace(model_id=model_id or "gpt-5.5-2026-04-23")

    async def __aenter__(self) -> "FakeAgent":
        return self

    async def __aexit__(self, exc_type: object, exc: object, tb: object) -> None:
        return None

    async def run(self, user_content: object) -> AgentResult:
        self.file_path.parent.mkdir(parents=True, exist_ok=True)
        self.file_path.write_text(
            "from __future__ import annotations\n\nobject_model = None\n",
            encoding="utf-8",
        )
        meshes_dir = self.file_path.parent / "assets" / "meshes"
        meshes_dir.mkdir(parents=True, exist_ok=True)
        (meshes_dir / "part.obj").write_text("# mesh\n", encoding="utf-8")
        cost_path = self.file_path.parent / "cost.json"
        cost_path.write_text(
            json.dumps({"total": {"costs_usd": {"total": 0.123456}}}, indent=2),
            encoding="utf-8",
        )
        if self.trace_dir is not None:
            self.trace_dir.mkdir(parents=True, exist_ok=True)
            (self.trace_dir / "trajectory.jsonl").write_text(
                '{"type":"message","message":{"role":"assistant","content":"done"}}\n',
                encoding="utf-8",
            )
        return AgentResult(
            success=True,
            reason=TerminateReason.CODE_VALID,
            message="done",
            conversation=[{"role": "user", "content": user_content}],
            final_code=self.file_path.read_text(encoding="utf-8"),
            urdf_xml="<robot name='test'/>",
            compile_warnings=["warning: test"],
            turn_count=3,
            tool_call_count=5,
            compile_attempt_count=2,
            usage={"prompt_tokens": 10, "candidates_tokens": 5, "total_tokens": 15},
        )

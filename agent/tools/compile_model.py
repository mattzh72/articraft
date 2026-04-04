"""
CompileModel tool - Advertise explicit compile/QC to the model.
"""

from __future__ import annotations

from agent.tools.base import (
    BaseDeclarativeTool,
    BaseToolInvocation,
    ToolParamsModel,
    ToolResult,
    make_tool_schema,
)


class CompileModelParams(ToolParamsModel):
    """Parameters for compile_model tool."""


class CompileModelInvocation(BaseToolInvocation[CompileModelParams, str]):
    """Invocation placeholder for compile_model.

    The harness intercepts this tool and runs the actual compile/QC flow so it
    can update compile freshness state and persist checkpoints.
    """

    def get_description(self) -> str:
        return "Compile current model file"

    async def execute(self) -> ToolResult:
        return ToolResult(error="compile_model must be handled by the harness")


class CompileModelTool(BaseDeclarativeTool):
    """Declarative tool schema for explicit compile/QC."""

    def __init__(self) -> None:
        schema = make_tool_schema(
            name="compile_model",
            description=(
                "Run full model compile and QC on the current bound file.\n\n"
                "Use this after one or more edits when you want full compile/test feedback.\n"
                "The harness automatically runs baseline sanity/QC checks for model validity, "
                "exactly one root part, mesh assets, floating disconnected part groups, "
                "disconnected geometry islands inside a part, and current-pose real 3D overlaps.\n"
                "Do not manually author that baseline stack in `run_tests()`; reserve `run_tests()` "
                "for exact checks, targeted poses, and explicit allowances.\n\n"
                "The tool returns the same structured `<compile_signals>` block used by the harness "
                "for clean success, warnings, or failures."
            ),
            parameters={},
            required=[],
        )
        super().__init__("compile_model", schema)

    async def build(self, params: dict) -> CompileModelInvocation:
        validated = CompileModelParams(**params)
        return CompileModelInvocation(validated)

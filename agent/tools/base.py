"""
Base classes for tools
"""

from abc import ABC, abstractmethod
from typing import Any, Generic, Optional, TypeVar

from pydantic import BaseModel, ConfigDict

TParams = TypeVar("TParams")
TResult = TypeVar("TResult")


# Type alias for OpenAI-style tool schema
ToolSchema = dict[str, Any]


def make_tool_schema(
    name: str,
    description: str,
    parameters: Optional[dict[str, Any]] = None,
    required: Optional[list[str]] = None,
) -> ToolSchema:
    """
    Create an OpenAI-compatible tool schema.

    Args:
        name: Tool name
        description: Tool description
        parameters: Dict of parameter definitions (JSON Schema format)
        required: List of required parameter names

    Returns:
        Tool schema dict in OpenAI format
    """
    schema: ToolSchema = {
        "type": "function",
        "function": {
            "name": name,
            "description": description,
            "parameters": {
                "type": "object",
                "properties": parameters or {},
                "required": required or [],
                "additionalProperties": False,  # Required for OpenAI Responses API strict mode
            },
        },
    }
    return schema


class ToolResult:
    """Result from tool execution"""

    def __init__(
        self,
        output: Any = None,
        error: Optional[str] = None,
        compilation: Optional[dict] = None,
        tool_call_id: Optional[str] = None,
    ):
        self.output = output
        self.error = error
        self.compilation = compilation
        self.tool_call_id = tool_call_id

    def is_success(self) -> bool:
        return self.error is None

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for SSE event / function response"""
        result: dict[str, Any] = {}
        if self.tool_call_id:
            result["tool_call_id"] = self.tool_call_id
        if self.error:
            result["error"] = self.error
        else:
            result["result"] = self.output
        if self.compilation:
            result["compilation"] = self.compilation
        return result


class ToolParamsModel(BaseModel):
    """Strict parameter model for public tool-call arguments."""

    model_config = ConfigDict(extra="forbid")


class BaseToolInvocation(ABC, Generic[TParams, TResult]):
    """Base class for tool invocations"""

    def __init__(self, params: TParams):
        self.params = params

    @abstractmethod
    def get_description(self) -> str:
        """Human-readable description of what this invocation will do"""
        pass

    @abstractmethod
    async def execute(self) -> ToolResult:
        """Execute the tool and return result"""
        pass


class BoundFileToolInvocation(BaseToolInvocation[TParams, TResult], ABC):
    """Invocation that operates on the harness-bound target file."""

    def __init__(self, params: TParams):
        super().__init__(params)
        self.file_path: str | None = None

    def bind_file_path(self, file_path: str) -> None:
        self.file_path = file_path


class BaseDeclarativeTool(ABC):
    """Base class for declarative tools that can be called by the LLM"""

    def __init__(self, name: str, schema: ToolSchema):
        self.name = name
        self.schema = schema

    @abstractmethod
    async def build(self, params: dict[str, Any]) -> BaseToolInvocation:
        """
        Validate parameters and build a tool invocation.

        Args:
            params: Raw parameters from LLM

        Returns:
            ToolInvocation ready to execute
        """
        pass

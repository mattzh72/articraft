from __future__ import annotations

from pathlib import Path
from typing import Any

from agent.examples import search_example_documents
from agent.tools.base import (
    BaseDeclarativeTool,
    BaseToolInvocation,
    ToolParamsModel,
    ToolResult,
    make_tool_schema,
    validate_tool_params,
)


class FindExamplesParams(ToolParamsModel):
    query: str
    limit: int = 3


class FindExamplesInvocation(BaseToolInvocation[FindExamplesParams, list[dict[str, object]]]):
    def __init__(
        self,
        params: FindExamplesParams,
        *,
        sdk_package: str,
        include_paths: bool,
    ) -> None:
        super().__init__(params)
        self.sdk_package = sdk_package
        self.include_paths = include_paths

    def get_description(self) -> str:
        return (
            f"Run lexical example search over curated {self.sdk_package} examples for "
            f"query={self.params.query!r} (limit={self.params.limit})"
        )

    async def execute(self) -> ToolResult:
        query = self.params.query.strip()
        if not query:
            return ToolResult(error="query must not be empty")
        if self.params.limit < 1:
            return ToolResult(error="limit must be >= 1")

        matches = search_example_documents(
            query,
            sdk_package=self.sdk_package,
            limit=self.params.limit,
        )
        return ToolResult(
            output=[
                self._serialize_match(doc)
                for doc in matches
            ]
        )

    def _serialize_match(self, doc: Any) -> dict[str, object]:
        relative_path = doc.path.relative_to(Path(__file__).resolve().parents[2]).as_posix()
        result: dict[str, object] = {
            "example_id": relative_path,
            "title": doc.title,
            "description": doc.description,
            "tags": list(doc.tags),
            "content": doc.content,
            "match_quality": doc.match_quality,
            "matched_tokens": list(doc.matched_tokens),
            "matched_fields": list(doc.matched_fields),
        }
        if self.include_paths:
            result["path"] = relative_path
        return result


class FindExamplesTool(BaseDeclarativeTool):
    def __init__(self, *, sdk_package: str, include_paths: bool = True) -> None:
        self.sdk_package = sdk_package
        self.include_paths = include_paths
        schema = make_tool_schema(
            name="find_examples",
            description=(
                "Run lexical search over curated example documents for the active SDK and "
                "return sufficiently relevant full markdown matches.\n\n"
                "Search checks file names, titles, descriptions, tags, prose, and code "
                "identifiers. It works best with short concrete queries such as object names, "
                "feature names, geometry operations, CadQuery API names, or exact example "
                "titles.\n\n"
                "When strong matches do not exist, the tool may return `[weakly relevant]` "
                "results as inspiration-only hints. Treat those cautiously.\n\n"
                "This does not search SDK docs, tests, test helper APIs, or arbitrary "
                "repository code. It is not a general API search tool.\n\n"
                "If relevance is weak, the search may return fewer than limit results."
            ),
            parameters={
                "query": {
                    "type": "string",
                    "description": (
                        "Short lexical query. Prefer concrete nouns, feature names, API names, "
                        "or example titles over long natural-language descriptions."
                    ),
                },
                "limit": {
                    "type": "integer",
                    "description": (
                        "Maximum number of full example documents to return. Search may return "
                        "fewer if relevance is weak."
                    ),
                },
            },
            required=["query"],
        )
        super().__init__("find_examples", schema)

    async def build(self, params: dict) -> FindExamplesInvocation:
        validated = validate_tool_params(FindExamplesParams, params)
        return FindExamplesInvocation(
            validated,
            sdk_package=self.sdk_package,
            include_paths=self.include_paths,
        )

from __future__ import annotations

from pathlib import Path

from pydantic import BaseModel

from agent.examples import search_example_documents
from agent.tools.base import BaseDeclarativeTool, BaseToolInvocation, ToolResult, make_tool_schema


class FindExamplesParams(BaseModel):
    file_path: str | None = None
    query: str
    limit: int = 3


class FindExamplesInvocation(BaseToolInvocation[FindExamplesParams, list[dict[str, object]]]):
    def __init__(self, params: FindExamplesParams, *, sdk_package: str) -> None:
        super().__init__(params)
        self.sdk_package = sdk_package

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
                {
                    "title": doc.title,
                    "description": doc.description,
                    "tags": list(doc.tags),
                    "path": doc.path.relative_to(Path(__file__).resolve().parents[2]).as_posix(),
                    "content": doc.content,
                }
                for doc in matches
            ]
        )


class FindExamplesTool(BaseDeclarativeTool):
    def __init__(self, *, sdk_package: str) -> None:
        self.sdk_package = sdk_package
        schema = make_tool_schema(
            name="find_examples",
            description=(
                "Run lexical search over curated example documents for the active SDK and "
                "return sufficiently relevant full markdown matches.\n\n"
                "Search checks file names, titles, descriptions, tags, prose, and code "
                "identifiers. It works best with short concrete queries such as object names, "
                "feature names, geometry operations, CadQuery API names, or exact example "
                "titles.\n\n"
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
        validated = FindExamplesParams(**params)
        return FindExamplesInvocation(validated, sdk_package=self.sdk_package)

from __future__ import annotations

import re
import unicodedata
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import bm25s

_FRONTMATTER_RE = re.compile(r"\A---\n(?P<frontmatter>.*?)\n---\n?(?P<body>.*)\Z", re.DOTALL)
_TOKEN_PATTERN = re.compile(r"[a-z0-9]+")
_CODE_FENCE_RE = re.compile(r"```[^\n]*\n(?P<code>.*?)```", re.DOTALL)
_IDENTIFIER_PATTERN = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
_CAMEL_CASE_BOUNDARY_RE = re.compile(r"(?<=[a-z0-9])(?=[A-Z])")
_NON_DISTINCTIVE_TAG_TOKENS = frozenset({"cadquery", "example", "examples"})
_NON_DISTINCTIVE_CODE_TOKENS = frozenset({"cq", "false", "none", "result", "self", "true"})
_STOPWORD_TOKENS = frozenset(
    {
        "a",
        "an",
        "and",
        "as",
        "at",
        "by",
        "for",
        "from",
        "in",
        "into",
        "of",
        "on",
        "or",
        "the",
        "to",
        "with",
        "without",
    }
)
_FIELD_REPETITIONS = {
    "slug": 6,
    "title": 5,
    "tags": 4,
    "description": 3,
    "code": 2,
    "prose": 1,
}
_FIELD_BONUSES = {
    "slug": 4.0,
    "title": 3.5,
    "tags": 3.0,
    "description": 1.75,
    "code": 1.25,
    "prose": 0.5,
}
_FIELD_PREFIX_BONUSES = {
    "slug": 1.5,
    "title": 1.25,
    "tags": 1.0,
    "description": 0.6,
    "code": 0.5,
    "prose": 0.2,
}
_FIELD_PHRASE_BONUSES = {
    "slug": 4.0,
    "title": 3.5,
    "tags": 2.5,
    "description": 1.5,
    "code": 1.0,
    "prose": 0.75,
}


@dataclass(slots=True, frozen=True)
class ExampleDocument:
    path: Path
    title: str
    description: str
    tags: tuple[str, ...]
    body: str
    content: str


@dataclass(slots=True, frozen=True)
class SearchField:
    tokens: tuple[str, ...]
    token_set: frozenset[str]
    phrase_text: str


@dataclass(slots=True, frozen=True)
class IndexedExampleDocument:
    doc: ExampleDocument
    slug: SearchField
    title: SearchField
    description: SearchField
    tags: SearchField
    prose: SearchField
    code: SearchField
    weighted_tokens: tuple[str, ...]


@dataclass(slots=True, frozen=True)
class ExampleSearchIndex:
    documents: tuple[IndexedExampleDocument, ...]
    retriever: bm25s.BM25


@dataclass(slots=True, frozen=True)
class ExampleSearchMatch:
    doc: ExampleDocument
    match_quality: str
    matched_tokens: tuple[str, ...]
    matched_fields: tuple[str, ...]
    score: float
    structured_match_count: int
    total_match_count: int
    coverage: float
    has_non_prose_phrase_hit: bool
    has_code_only_signal: bool

    @property
    def path(self) -> Path:
        return self.doc.path

    @property
    def title(self) -> str:
        return self.doc.title

    @property
    def description(self) -> str:
        return self.doc.description

    @property
    def tags(self) -> tuple[str, ...]:
        return self.doc.tags

    @property
    def body(self) -> str:
        return self.doc.body

    @property
    def content(self) -> str:
        return self.doc.content


@dataclass(slots=True, frozen=True)
class _CandidateMatch:
    doc: ExampleDocument
    score: float
    bm25_score: float
    matched_tokens: tuple[str, ...]
    matched_fields: tuple[str, ...]
    structured_match_count: int
    total_match_count: int
    coverage: float
    has_non_prose_phrase_hit: bool
    has_metadata_phrase_hit: bool
    has_code_only_signal: bool
    has_structured_signal: bool
    has_slug_or_title_prefix: bool
    prose_matches: tuple[str, ...]


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def examples_root() -> Path:
    return _repo_root() / "sdk" / "_examples"


def example_dirs_for_sdk(sdk_package: str) -> tuple[Path, ...]:
    if sdk_package == "sdk":
        return (
            examples_root() / "base",
            examples_root() / "cadquery",
        )
    raise ValueError(f"Unsupported SDK package for examples: {sdk_package!r}")


def _normalize_text(value: str) -> str:
    normalized = unicodedata.normalize("NFKD", value)
    ascii_text = normalized.encode("ascii", "ignore").decode("ascii")
    return ascii_text.lower()


def _tokenize(value: str) -> list[str]:
    return _TOKEN_PATTERN.findall(_normalize_text(value))


def _meaningful_tokens(value: str) -> list[str]:
    return [
        token
        for token in _tokenize(value)
        if token not in _NON_DISTINCTIVE_TAG_TOKENS and token not in _STOPWORD_TOKENS
    ]


def _dedupe_preserving_order(tokens: list[str]) -> list[str]:
    return list(dict.fromkeys(tokens))


def _token_variants(token: str) -> list[str]:
    variants = [token]
    candidate: str | None = None
    if len(token) > 5 and token.endswith("ies"):
        candidate = f"{token[:-3]}y"
    elif len(token) > 6 and token.endswith("ing"):
        candidate = token[:-3]
    elif len(token) > 5 and token.endswith("ed"):
        candidate = token[:-2]
    elif len(token) > 4 and token.endswith("es"):
        candidate = token[:-2]
    elif len(token) > 4 and token.endswith("s"):
        candidate = token[:-1]

    if candidate and len(candidate) >= 4 and candidate not in variants:
        variants.append(candidate)

    if len(token) > 6 and token.endswith("e"):
        candidate = token[:-1]
        if len(candidate) >= 4 and candidate not in variants:
            variants.append(candidate)

    return variants


def _split_identifier(identifier: str) -> list[str]:
    normalized = _normalize_text(identifier)
    if not normalized:
        return []

    expanded = _CAMEL_CASE_BOUNDARY_RE.sub(" ", identifier.replace("_", " ").replace("-", " "))
    split_tokens = _tokenize(expanded)

    ordered: list[str] = []
    seen: set[str] = set()
    for token in [normalized, *split_tokens]:
        if (
            token
            and token not in seen
            and token not in _STOPWORD_TOKENS
            and token not in _NON_DISTINCTIVE_TAG_TOKENS
            and token not in _NON_DISTINCTIVE_CODE_TOKENS
        ):
            ordered.append(token)
            seen.add(token)
    return ordered


def _query_alias_tokens(query: str, query_tokens: list[str]) -> list[str]:
    aliases: list[str] = []
    for token in query_tokens:
        for variant in _token_variants(token)[1:]:
            if variant not in query_tokens:
                aliases.append(variant)

    for identifier in _IDENTIFIER_PATTERN.findall(query):
        parts = _split_identifier(identifier)
        if len(parts) <= 1:
            continue
        for token in parts[1:]:
            if token not in query_tokens and len(token) >= 4:
                aliases.append(token)
    return _dedupe_preserving_order(aliases)


def _extract_code_tokens(body: str) -> list[str]:
    tokens: list[str] = []
    for match in _CODE_FENCE_RE.finditer(body):
        code = match.group("code")
        for identifier in _IDENTIFIER_PATTERN.findall(code):
            tokens.extend(_split_identifier(identifier))
    return tokens


def _strip_code_fences(body: str) -> str:
    return _CODE_FENCE_RE.sub("\n", body)


def _build_search_field(tokens: list[str]) -> SearchField:
    stable_tokens = tuple(
        _dedupe_preserving_order(
            [variant for token in tokens for variant in _token_variants(token)]
        )
    )
    phrase_tokens = tuple(_dedupe_preserving_order(tokens))
    return SearchField(
        tokens=stable_tokens,
        token_set=frozenset(stable_tokens),
        phrase_text=" ".join(phrase_tokens),
    )


def _build_weighted_tokens(
    *,
    slug: SearchField,
    title: SearchField,
    description: SearchField,
    tags: SearchField,
    prose: SearchField,
    code: SearchField,
) -> tuple[str, ...]:
    weighted: list[str] = []
    for field_name, field in (
        ("slug", slug),
        ("title", title),
        ("tags", tags),
        ("description", description),
        ("code", code),
        ("prose", prose),
    ):
        weighted.extend(list(field.tokens) * _FIELD_REPETITIONS[field_name])
    return tuple(weighted)


def _index_document(doc: ExampleDocument) -> IndexedExampleDocument:
    slug = _build_search_field(_meaningful_tokens(doc.path.stem.replace("_", " ")))
    title = _build_search_field(_meaningful_tokens(doc.title))
    description = _build_search_field(_meaningful_tokens(doc.description))
    tags = _build_search_field(_meaningful_tokens(" ".join(doc.tags)))
    prose = _build_search_field(_meaningful_tokens(_strip_code_fences(doc.body)))
    code = _build_search_field(_extract_code_tokens(doc.body))
    return IndexedExampleDocument(
        doc=doc,
        slug=slug,
        title=title,
        description=description,
        tags=tags,
        prose=prose,
        code=code,
        weighted_tokens=_build_weighted_tokens(
            slug=slug,
            title=title,
            description=description,
            tags=tags,
            prose=prose,
            code=code,
        ),
    )


def _field_has_prefix(field: SearchField, query_token: str) -> bool:
    if len(query_token) < 4:
        return False
    return any(
        token != query_token
        and min(len(token), len(query_token)) >= 4
        and (token.startswith(query_token) or query_token.startswith(token))
        for token in field.token_set
    )


def _parse_frontmatter_value(raw: str) -> str:
    value = raw.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
        inner = value[1:-1]
        if value[0] == "'":
            return inner.replace("''", "'")
        return inner.replace('\\"', '"')
    return value


def _parse_frontmatter(text: str, *, source: Path) -> dict[str, object]:
    values: dict[str, object] = {}
    current_key: str | None = None
    list_values: list[str] = []

    def _flush_list() -> None:
        nonlocal current_key, list_values
        if current_key is not None:
            values[current_key] = list(list_values)
        current_key = None
        list_values = []

    for raw_line in text.splitlines():
        line = raw_line.rstrip()
        stripped = line.strip()
        if not stripped:
            continue
        if line.startswith("  - ") or line.startswith("- "):
            item = line.split("-", 1)[1].strip()
            if current_key is None:
                raise ValueError(f"{source}: list item found before list key")
            list_values.append(_parse_frontmatter_value(item))
            continue

        _flush_list()
        if ":" not in line:
            raise ValueError(f"{source}: invalid frontmatter line: {raw_line!r}")
        key, raw_value = line.split(":", 1)
        key = key.strip()
        value = raw_value.strip()
        if not key:
            raise ValueError(f"{source}: missing frontmatter key")
        if not value:
            current_key = key
            list_values = []
            continue
        if value.startswith("[") and value.endswith("]"):
            inner = value[1:-1].strip()
            if not inner:
                values[key] = []
            else:
                values[key] = [_parse_frontmatter_value(part) for part in inner.split(",")]
            continue
        values[key] = _parse_frontmatter_value(value)

    _flush_list()
    return values


def parse_example_document(path: Path) -> ExampleDocument:
    text = path.read_text(encoding="utf-8")
    match = _FRONTMATTER_RE.match(text)
    if not match:
        raise ValueError(f"{path}: expected YAML frontmatter delimited by ---")

    frontmatter = _parse_frontmatter(match.group("frontmatter"), source=path)
    title = str(frontmatter.get("title") or "").strip()
    description = str(frontmatter.get("description") or "").strip()
    raw_tags = frontmatter.get("tags")
    if not title:
        raise ValueError(f"{path}: missing required frontmatter field 'title'")
    if not description:
        raise ValueError(f"{path}: missing required frontmatter field 'description'")
    if not isinstance(raw_tags, list) or not raw_tags:
        raise ValueError(f"{path}: missing required frontmatter field 'tags'")

    tags = tuple(str(tag).strip() for tag in raw_tags if str(tag).strip())
    if not tags:
        raise ValueError(f"{path}: tags must contain at least one non-empty value")

    return ExampleDocument(
        path=path,
        title=title,
        description=description,
        tags=tags,
        body=match.group("body"),
        content=text,
    )


@lru_cache(maxsize=8)
def load_example_documents(sdk_package: str) -> tuple[ExampleDocument, ...]:
    docs: list[ExampleDocument] = []
    for root in example_dirs_for_sdk(sdk_package):
        if not root.exists():
            continue
        docs.extend(parse_example_document(path) for path in sorted(root.glob("*.md")))
    return tuple(docs)


@lru_cache(maxsize=8)
def load_example_search_index(sdk_package: str) -> ExampleSearchIndex:
    documents = tuple(_index_document(doc) for doc in load_example_documents(sdk_package))
    retriever = bm25s.BM25()
    if documents:
        retriever.index(
            [list(document.weighted_tokens) for document in documents],
            show_progress=False,
        )
    return ExampleSearchIndex(documents=documents, retriever=retriever)


def _collect_candidate_match(
    indexed_doc: IndexedExampleDocument,
    *,
    bm25_score: float,
    query_tokens: list[str],
    query_alias_tokens: list[str],
    phrase_query: str,
) -> _CandidateMatch | None:
    metadata_field_names = {"slug", "title", "description", "tags"}
    fields = {
        "slug": indexed_doc.slug,
        "title": indexed_doc.title,
        "description": indexed_doc.description,
        "tags": indexed_doc.tags,
        "code": indexed_doc.code,
        "prose": indexed_doc.prose,
    }

    score = bm25_score
    matched_tokens: set[str] = set()
    matched_fields: set[str] = set()
    structured_matches: set[str] = set()
    structured_fields: set[str] = set()
    code_matches: set[str] = set()
    prose_matches: set[str] = set()
    phrase_fields: set[str] = set()
    slug_or_title_prefix = False

    for token in query_tokens:
        for field_name, field in fields.items():
            if token in field.token_set:
                score += _FIELD_BONUSES[field_name]
                matched_tokens.add(token)
                matched_fields.add(field_name)
                if field_name == "prose":
                    prose_matches.add(token)
                elif field_name == "code":
                    code_matches.add(token)
                else:
                    if field_name in metadata_field_names:
                        structured_matches.add(token)
                        structured_fields.add(field_name)
                continue

            if _field_has_prefix(field, token):
                score += _FIELD_PREFIX_BONUSES[field_name]
                matched_tokens.add(token)
                matched_fields.add(field_name)
                if field_name in {"slug", "title"}:
                    slug_or_title_prefix = True
                if field_name == "prose":
                    prose_matches.add(token)
                elif field_name == "code":
                    code_matches.add(token)
                else:
                    if field_name in metadata_field_names:
                        structured_matches.add(token)
                        structured_fields.add(field_name)

    for token in query_alias_tokens:
        for field_name in ("slug", "title", "tags", "description"):
            field = fields[field_name]
            if token in field.token_set:
                score += _FIELD_BONUSES[field_name] * 0.3
            elif _field_has_prefix(field, token):
                score += _FIELD_PREFIX_BONUSES[field_name] * 0.3

    if len(query_tokens) >= 2:
        for field_name, field in fields.items():
            if phrase_query in field.phrase_text:
                score += _FIELD_PHRASE_BONUSES[field_name]
                phrase_fields.add(field_name)
                matched_fields.add(field_name)

    if not matched_tokens and not phrase_fields:
        return None

    coverage = len(matched_tokens) / len(query_tokens)
    score += coverage * coverage * 3.0
    if len(structured_matches) >= 2:
        score += 2.0
    elif structured_matches:
        score += 1.0

    metadata_phrase_fields = phrase_fields & metadata_field_names
    has_structured_signal = bool(structured_matches or metadata_phrase_fields)
    has_code_only_signal = bool(code_matches) and not has_structured_signal

    return _CandidateMatch(
        doc=indexed_doc.doc,
        score=score,
        bm25_score=bm25_score,
        matched_tokens=tuple(sorted(matched_tokens)),
        matched_fields=tuple(sorted(matched_fields)),
        structured_match_count=len(structured_matches),
        total_match_count=len(matched_tokens),
        coverage=coverage,
        has_non_prose_phrase_hit=bool(phrase_fields - {"prose"}),
        has_metadata_phrase_hit=bool(metadata_phrase_fields),
        has_code_only_signal=has_code_only_signal,
        has_structured_signal=has_structured_signal,
        has_slug_or_title_prefix=slug_or_title_prefix,
        prose_matches=tuple(sorted(prose_matches)),
    )


def _make_search_match(candidate: _CandidateMatch, *, match_quality: str) -> ExampleSearchMatch:
    return ExampleSearchMatch(
        doc=candidate.doc,
        match_quality=match_quality,
        matched_tokens=candidate.matched_tokens,
        matched_fields=candidate.matched_fields,
        score=candidate.score,
        structured_match_count=candidate.structured_match_count,
        total_match_count=candidate.total_match_count,
        coverage=candidate.coverage,
        has_non_prose_phrase_hit=candidate.has_non_prose_phrase_hit,
        has_code_only_signal=candidate.has_code_only_signal,
    )


def _single_token_matches(
    candidates: list[_CandidateMatch],
    *,
    token: str,
    limit: int,
) -> list[ExampleSearchMatch]:
    ranked: list[_CandidateMatch] = []
    for candidate in candidates:
        if candidate.has_code_only_signal:
            if len(token) < 5 or candidate.bm25_score < 0.5:
                continue
        elif not candidate.has_structured_signal:
            if not (
                token in candidate.prose_matches and len(token) >= 6 and candidate.score >= 2.0
            ):
                continue
        ranked.append(candidate)

    ranked.sort(key=lambda item: (-item.score, item.doc.title, item.doc.path.as_posix()))
    if not ranked:
        return []

    best_score = ranked[0].score
    retained = [item for item in ranked if item.score >= max(1.0, best_score * 0.5)]
    return [_make_search_match(item, match_quality="strong") for item in retained[:limit]]


def _is_strong_multi_token_match(candidate: _CandidateMatch) -> bool:
    if candidate.has_code_only_signal:
        return False
    if candidate.has_metadata_phrase_hit:
        return True
    if candidate.structured_match_count >= 2:
        return True
    return candidate.total_match_count >= 3 and candidate.structured_match_count >= 1


def _is_weak_multi_token_match(candidate: _CandidateMatch, *, query_len: int) -> bool:
    if not (candidate.structured_match_count >= 1 or candidate.has_slug_or_title_prefix):
        return False
    if query_len >= 3 and candidate.total_match_count < 2:
        return False
    return not candidate.has_code_only_signal


def _strong_multi_token_matches(
    candidates: list[_CandidateMatch],
    *,
    limit: int,
) -> list[ExampleSearchMatch]:
    ranked = [candidate for candidate in candidates if _is_strong_multi_token_match(candidate)]
    ranked.sort(
        key=lambda item: (
            -item.coverage,
            -item.structured_match_count,
            -int(item.has_non_prose_phrase_hit),
            -item.score,
            item.doc.title,
            item.doc.path.as_posix(),
        )
    )
    if not ranked:
        return []

    best_score = max(item.score for item in ranked)
    retained = [item for item in ranked if item.score >= max(1.0, best_score * 0.5)]
    return [_make_search_match(item, match_quality="strong") for item in retained[:limit]]


def _weak_multi_token_matches(
    candidates: list[_CandidateMatch],
    *,
    query_len: int,
    limit: int,
) -> list[ExampleSearchMatch]:
    ranked = [
        candidate
        for candidate in candidates
        if not _is_strong_multi_token_match(candidate)
        and _is_weak_multi_token_match(candidate, query_len=query_len)
    ]
    ranked.sort(
        key=lambda item: (
            -item.structured_match_count,
            -item.coverage,
            -int(item.has_slug_or_title_prefix),
            -item.score,
            item.doc.title,
            item.doc.path.as_posix(),
        )
    )
    if not ranked:
        return []

    best_score = ranked[0].score
    retained = [item for item in ranked if item.score >= max(1.0, best_score * 0.7)]
    capped_limit = min(limit, 2)
    return [
        _make_search_match(item, match_quality="weakly_relevant")
        for item in retained[:capped_limit]
    ]


def search_example_documents(
    query: str,
    *,
    sdk_package: str,
    limit: int = 3,
) -> list[ExampleSearchMatch]:
    query_tokens = _dedupe_preserving_order(_meaningful_tokens(query))
    if not query_tokens:
        return []

    index = load_example_search_index(sdk_package)
    if not index.documents:
        return []

    query_alias_tokens = _query_alias_tokens(query, query_tokens)
    query_for_bm25 = _dedupe_preserving_order([*query_tokens, *query_alias_tokens])
    candidate_k = min(len(index.documents), max(limit * 6, 12))
    retrieval = index.retriever.retrieve(
        [query_for_bm25],
        k=candidate_k,
        corpus=list(range(len(index.documents))),
        show_progress=False,
    )

    phrase_query = " ".join(query_tokens)
    candidates: list[_CandidateMatch] = []
    for raw_doc_index, raw_score in zip(retrieval.documents[0], retrieval.scores[0], strict=True):
        bm25_score = float(raw_score)
        if bm25_score <= 0:
            continue

        indexed_doc = index.documents[int(raw_doc_index)]
        candidate = _collect_candidate_match(
            indexed_doc,
            bm25_score=bm25_score,
            query_tokens=query_tokens,
            query_alias_tokens=query_alias_tokens,
            phrase_query=phrase_query,
        )
        if candidate is not None:
            candidates.append(candidate)

    if not candidates:
        return []

    if len(query_tokens) == 1:
        return _single_token_matches(candidates, token=query_tokens[0], limit=limit)

    strong_matches = _strong_multi_token_matches(candidates, limit=limit)
    if strong_matches:
        return strong_matches
    return _weak_multi_token_matches(candidates, query_len=len(query_tokens), limit=limit)

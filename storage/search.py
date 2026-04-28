from __future__ import annotations

import json
import os
import re
import tempfile
import unicodedata
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from storage.repo import StorageRepo

_INDEX_SCHEMA_VERSION = 1
_TOKEN_PATTERN = re.compile(r"[a-z0-9]+")


@dataclass(slots=True, frozen=True)
class SearchIndexStats:
    record_count: int
    category_count: int
    workbench_entry_count: int
    path: Path


def _normalize_text(value: str) -> str:
    normalized = unicodedata.normalize("NFKD", value)
    ascii_text = normalized.encode("ascii", "ignore").decode("ascii")
    return ascii_text.lower()


def _tokenize(value: str) -> list[str]:
    return _TOKEN_PATTERN.findall(_normalize_text(value))


def _phrase_text(value: str) -> str:
    return " ".join(_tokenize(value))


def _unique_tokens(*groups: list[str]) -> list[str]:
    seen: set[str] = set()
    unique: list[str] = []
    for group in groups:
        for token in group:
            if token in seen:
                continue
            seen.add(token)
            unique.append(token)
    return unique


def _field_document(value: str) -> dict[str, Any]:
    return {
        "text": _phrase_text(value),
        "tokens": _tokenize(value),
    }


def _field_token_score(
    token: str,
    field: dict[str, Any],
    *,
    exact_score: float,
    prefix_score: float,
) -> float:
    tokens = field.get("tokens")
    if not isinstance(tokens, list):
        return 0.0
    normalized_tokens = [str(item) for item in tokens]
    if token in normalized_tokens:
        return exact_score
    if any(field_token.startswith(token) for field_token in normalized_tokens):
        return prefix_score
    return 0.0


def _field_has_phrase(phrase_query: str, field: dict[str, Any]) -> bool:
    text = field.get("text")
    if not isinstance(text, str) or not phrase_query:
        return False
    return phrase_query in text


@dataclass(slots=True)
class SearchIndex:
    repo: StorageRepo
    _cached_path_mtime_ns: int | None = field(default=None, init=False, repr=False)
    _cached_documents: list[dict[str, Any]] | None = field(default=None, init=False, repr=False)

    def _path_mtime_ns(self, path: Path) -> int | None:
        try:
            return path.stat().st_mtime_ns
        except OSError:
            return None

    def _max_mtime_ns(self, current: int | None, candidate: int | None) -> int | None:
        if candidate is None:
            return current
        if current is None or candidate > current:
            return candidate
        return current

    def _source_mtime_ns(self) -> int | None:
        latest_mtime_ns = self._max_mtime_ns(
            None,
            self._path_mtime_ns(self.repo.layout.records_root),
        )
        latest_mtime_ns = self._max_mtime_ns(
            latest_mtime_ns,
            self._path_mtime_ns(self.repo.layout.categories_root),
        )
        latest_mtime_ns = self._max_mtime_ns(
            latest_mtime_ns,
            self._path_mtime_ns(self.repo.layout.local_workbench_path()),
        )

        for category_path in self.repo.layout.categories_root.glob("*/category.json"):
            latest_mtime_ns = self._max_mtime_ns(
                latest_mtime_ns,
                self._path_mtime_ns(category_path),
            )

        if self.repo.layout.records_root.exists():
            for record_dir in self.repo.layout.records_root.iterdir():
                if not record_dir.is_dir():
                    continue
                latest_mtime_ns = self._max_mtime_ns(
                    latest_mtime_ns,
                    self._path_mtime_ns(record_dir),
                )
                latest_mtime_ns = self._max_mtime_ns(
                    latest_mtime_ns,
                    self._path_mtime_ns(record_dir / "record.json"),
                )
                latest_mtime_ns = self._max_mtime_ns(
                    latest_mtime_ns,
                    self._path_mtime_ns(record_dir / "prompt.txt"),
                )
                latest_mtime_ns = self._max_mtime_ns(
                    latest_mtime_ns,
                    self._path_mtime_ns(record_dir / "dataset_entry.json"),
                )

        return latest_mtime_ns

    def _load_workbench_entries(self) -> dict[str, dict[str, str]]:
        raw = self.repo.read_json(self.repo.layout.local_workbench_path(), default={}) or {}
        entries: dict[str, dict[str, str]] = {}
        for item in raw.get("entries", []):
            record_id = str(item.get("record_id", ""))
            if not record_id:
                continue
            entries[record_id] = {
                "label": str(item.get("label") or ""),
                "tags": " ".join(str(tag) for tag in item.get("tags", [])),
            }
        return entries

    def _load_categories(self) -> dict[str, dict[str, str]]:
        categories: dict[str, dict[str, str]] = {}
        for category_path in self.repo.layout.categories_root.glob("*/category.json"):
            category = self.repo.read_json(category_path)
            if not isinstance(category, dict):
                continue
            slug = str(category.get("slug", ""))
            if not slug:
                continue
            categories[slug] = {
                "title": str(category.get("title") or ""),
                "description": str(category.get("description") or ""),
            }
        return categories

    def _build_documents(self) -> tuple[list[dict[str, Any]], int, int]:
        workbench_entries = self._load_workbench_entries()
        categories = self._load_categories()
        documents: list[dict[str, Any]] = []

        records_root = self.repo.layout.records_root
        if records_root.exists():
            record_dirs = sorted(path for path in records_root.iterdir() if path.is_dir())
        else:
            record_dirs = []

        for record_dir in record_dirs:
            record = self.repo.read_json(record_dir / "record.json")
            if not isinstance(record, dict):
                continue

            record_id = str(record.get("record_id") or record_dir.name)
            display = record.get("display") if isinstance(record.get("display"), dict) else {}
            dataset_entry = self.repo.read_json(record_dir / "dataset_entry.json", default={}) or {}
            prompt_text = ""
            prompt_path = record_dir / "prompt.txt"
            if prompt_path.exists():
                prompt_text = prompt_path.read_text(encoding="utf-8")

            category_slug = str(record.get("category_slug") or "")
            category = categories.get(category_slug, {})
            workbench = workbench_entries.get(record_id, {})
            source = record.get("source") if isinstance(record.get("source"), dict) else {}
            dataset_id = ""
            if isinstance(dataset_entry, dict):
                dataset_id = str(dataset_entry.get("dataset_id") or "")

            title = str(display.get("title") or "")
            label = str(workbench.get("label") or "")
            tags = str(workbench.get("tags") or "")
            category_title = str(category.get("title") or "")

            title_field = _field_document(title)
            label_field = _field_document(label)
            tags_field = _field_document(tags)
            category_slug_field = _field_document(category_slug)
            category_title_field = _field_document(category_title)
            record_id_field = _field_document(record_id)
            dataset_id_field = _field_document(dataset_id)
            prompt_field = _field_document(prompt_text)

            documents.append(
                {
                    "record_id": record_id,
                    "run_id": str(source.get("run_id") or ""),
                    "created_at": str(record.get("created_at") or ""),
                    "in_workbench": record_id in workbench_entries,
                    "in_dataset": bool(dataset_id),
                    "title": title_field,
                    "label": label_field,
                    "tags": tags_field,
                    "category_slug": category_slug_field,
                    "category_title": category_title_field,
                    "record_id_field": record_id_field,
                    "dataset_id": dataset_id_field,
                    "prompt_full": prompt_field,
                    "all_tokens": _unique_tokens(
                        title_field["tokens"],
                        label_field["tokens"],
                        tags_field["tokens"],
                        category_slug_field["tokens"],
                        category_title_field["tokens"],
                        record_id_field["tokens"],
                        dataset_id_field["tokens"],
                        prompt_field["tokens"],
                    ),
                }
            )

        return documents, len(categories), len(workbench_entries)

    def _write_cache(
        self,
        *,
        documents: list[dict[str, Any]],
        category_count: int,
        workbench_entry_count: int,
    ) -> None:
        path = self.repo.layout.search_index_path()
        path.parent.mkdir(parents=True, exist_ok=True)

        payload = {
            "schema_version": _INDEX_SCHEMA_VERSION,
            "record_count": len(documents),
            "category_count": category_count,
            "workbench_entry_count": workbench_entry_count,
            "documents": documents,
        }
        with tempfile.NamedTemporaryFile(
            "w",
            dir=path.parent,
            encoding="utf-8",
            prefix=f"{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            temporary_path = Path(temporary_file.name)
            temporary_file.write(json.dumps(payload, indent=2) + "\n")
        try:
            os.replace(temporary_path, path)
        except OSError:
            temporary_path.unlink(missing_ok=True)
            raise

    def _cache_stats(
        self,
        *,
        record_count: int,
        category_count: int,
        workbench_entry_count: int,
    ) -> SearchIndexStats:
        return SearchIndexStats(
            record_count=record_count,
            category_count=category_count,
            workbench_entry_count=workbench_entry_count,
            path=self.repo.layout.search_index_path(),
        )

    def _prime_cache(
        self,
        *,
        documents: list[dict[str, Any]],
        path_mtime_ns: int | None,
    ) -> None:
        self._cached_path_mtime_ns = path_mtime_ns
        self._cached_documents = documents

    def rebuild(self) -> SearchIndexStats:
        documents, category_count, workbench_entry_count = self._build_documents()
        self._write_cache(
            documents=documents,
            category_count=category_count,
            workbench_entry_count=workbench_entry_count,
        )
        stats = self._cache_stats(
            record_count=len(documents),
            category_count=category_count,
            workbench_entry_count=workbench_entry_count,
        )
        try:
            path_mtime_ns = self.repo.layout.search_index_path().stat().st_mtime_ns
        except OSError:
            path_mtime_ns = None
        self._prime_cache(documents=documents, path_mtime_ns=path_mtime_ns)
        return stats

    def _load_cached_payload(self) -> tuple[dict[str, Any], int | None] | None:
        path = self.repo.layout.search_index_path()
        if not path.exists():
            return None
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            path_mtime_ns = path.stat().st_mtime_ns
        except (OSError, json.JSONDecodeError):
            return None
        if not isinstance(payload, dict):
            return None
        if payload.get("schema_version") != _INDEX_SCHEMA_VERSION:
            return None
        documents = payload.get("documents")
        if not isinstance(documents, list):
            return None
        return payload, path_mtime_ns

    def ensure_current(self) -> SearchIndexStats | None:
        path = self.repo.layout.search_index_path()
        try:
            current_mtime_ns = path.stat().st_mtime_ns
        except OSError:
            current_mtime_ns = None
        source_mtime_ns = self._source_mtime_ns()
        if (
            current_mtime_ns is not None
            and source_mtime_ns is not None
            and source_mtime_ns > current_mtime_ns
        ):
            return self.rebuild()
        if (
            current_mtime_ns is not None
            and self._cached_path_mtime_ns == current_mtime_ns
            and self._cached_documents is not None
        ):
            return None
        cached = self._load_cached_payload()
        if cached is None:
            return self.rebuild()
        payload, path_mtime_ns = cached
        self._prime_cache(
            documents=[doc for doc in payload["documents"] if isinstance(doc, dict)],
            path_mtime_ns=path_mtime_ns,
        )
        return None

    def _score_document(
        self, document: dict[str, Any], query_tokens: list[str], phrase_query: str
    ) -> float:
        all_tokens = document.get("all_tokens")
        if not isinstance(all_tokens, list):
            return 0.0
        searchable_tokens = [str(token) for token in all_tokens]
        if not all(
            token in searchable_tokens
            or any(candidate.startswith(token) for candidate in searchable_tokens)
            for token in query_tokens
        ):
            return 0.0

        score = 0.0
        for token in query_tokens:
            score += max(
                _field_token_score(
                    token, document.get("title", {}), exact_score=140.0, prefix_score=100.0
                ),
                _field_token_score(
                    token, document.get("label", {}), exact_score=130.0, prefix_score=90.0
                ),
                _field_token_score(
                    token, document.get("tags", {}), exact_score=110.0, prefix_score=75.0
                ),
                _field_token_score(
                    token,
                    document.get("category_title", {}),
                    exact_score=110.0,
                    prefix_score=75.0,
                ),
                _field_token_score(
                    token,
                    document.get("category_slug", {}),
                    exact_score=100.0,
                    prefix_score=70.0,
                ),
            )
            score += max(
                _field_token_score(
                    token,
                    document.get("record_id_field", {}),
                    exact_score=85.0,
                    prefix_score=60.0,
                ),
                _field_token_score(
                    token,
                    document.get("dataset_id", {}),
                    exact_score=85.0,
                    prefix_score=60.0,
                ),
            )
            if len(token) >= 3:
                score += _field_token_score(
                    token,
                    document.get("prompt_full", {}),
                    exact_score=15.0,
                    prefix_score=8.0,
                )

        if phrase_query:
            if _field_has_phrase(phrase_query, document.get("title", {})):
                score += 90.0
            elif _field_has_phrase(phrase_query, document.get("label", {})):
                score += 80.0
            elif _field_has_phrase(phrase_query, document.get("category_title", {})):
                score += 70.0
            elif _field_has_phrase(phrase_query, document.get("category_slug", {})):
                score += 60.0
            elif _field_has_phrase(phrase_query, document.get("record_id_field", {})):
                score += 50.0
            elif _field_has_phrase(phrase_query, document.get("dataset_id", {})):
                score += 50.0
            elif len(phrase_query) >= 5 and _field_has_phrase(
                phrase_query, document.get("prompt_full", {})
            ):
                score += 20.0

        return score

    def search_record_ids(
        self,
        query: str,
        *,
        source_filter: str | None = None,
        run_id: str | None = None,
        limit: int = 200,
    ) -> list[str]:
        self.ensure_current()
        query_tokens = _tokenize(query)
        if not query_tokens:
            return []
        phrase_query = " ".join(query_tokens)
        documents = self._cached_documents or []
        ranked_rows: list[tuple[float, str, str]] = []

        for document in documents:
            if source_filter == "workbench" and not document.get("in_workbench"):
                continue
            if source_filter == "dataset" and not document.get("in_dataset"):
                continue
            if run_id and str(document.get("run_id") or "") != run_id:
                continue
            score = self._score_document(document, query_tokens, phrase_query)
            if score <= 0:
                continue
            ranked_rows.append(
                (
                    score,
                    str(document.get("created_at") or ""),
                    str(document.get("record_id") or ""),
                )
            )

        ranked_rows.sort(key=lambda item: item[2])
        ranked_rows.sort(key=lambda item: item[1], reverse=True)
        ranked_rows.sort(key=lambda item: item[0], reverse=True)
        return [record_id for _, _, record_id in ranked_rows[:limit]]

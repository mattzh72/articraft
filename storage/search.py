from __future__ import annotations

import hashlib
import os
import re
import sqlite3
import unicodedata
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from storage.repo import StorageRepo

_INDEX_SCHEMA_VERSION = 1
_TOKEN_PATTERN = re.compile(r"[a-z0-9]+")
_SCORE_WEIGHTS = (4.5, 5.5, 4.5, 4.5, 4.5, 1.5, 3.0, 1.0)


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


def _escape_token(token: str) -> str:
    return token.replace('"', '""')


def _query_expression(query: str) -> str | None:
    tokens = _tokenize(query)
    if not tokens:
        return None
    return " AND ".join(f"{_escape_token(token)}*" for token in tokens)


def _signature_for_paths(paths: list[Path], repo_root: Path) -> str:
    digest = hashlib.sha1()
    for path in sorted(paths):
        stat = path.stat()
        digest.update(path.resolve().relative_to(repo_root.resolve()).as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(stat.st_mtime_ns).encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(stat.st_size).encode("utf-8"))
        digest.update(b"\n")
    return digest.hexdigest()


@dataclass(slots=True)
class SearchIndex:
    repo: StorageRepo

    def _source_paths(self) -> list[Path]:
        paths: list[Path] = []
        workbench_path = self.repo.layout.local_workbench_path()
        if workbench_path.exists():
            paths.append(workbench_path)
        if self.repo.layout.categories_root.exists():
            for category_path in self.repo.layout.categories_root.glob("*/category.json"):
                if category_path.exists():
                    paths.append(category_path)
        if self.repo.layout.records_root.exists():
            for record_dir in self.repo.layout.records_root.iterdir():
                if not record_dir.is_dir():
                    continue
                for name in ("record.json", "dataset_entry.json", "prompt.txt"):
                    candidate = record_dir / name
                    if candidate.exists():
                        paths.append(candidate)
        return paths

    def _current_signature(self) -> str:
        return _signature_for_paths(self._source_paths(), self.repo.root)

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

    def _build_documents(self) -> tuple[list[tuple[Any, ...]], int, int]:
        workbench_entries = self._load_workbench_entries()
        categories = self._load_categories()
        documents: list[tuple[Any, ...]] = []

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

            documents.append(
                (
                    record_id,
                    str(display.get("title") or ""),
                    str(workbench.get("label") or ""),
                    str(workbench.get("tags") or ""),
                    category_slug,
                    str(category.get("title") or ""),
                    dataset_id,
                    prompt_text,
                    str(source.get("run_id") or ""),
                    str(record.get("created_at") or ""),
                    1 if record_id in workbench_entries else 0,
                    1 if dataset_id else 0,
                )
            )

        return documents, len(categories), len(workbench_entries)

    def rebuild(self) -> SearchIndexStats:
        documents, category_count, workbench_entry_count = self._build_documents()
        signature = self._current_signature()
        path = self.repo.layout.search_index_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = path.with_suffix(f"{path.suffix}.tmp")
        if temporary_path.exists():
            temporary_path.unlink()

        with sqlite3.connect(temporary_path) as connection:
            connection.execute("PRAGMA synchronous=NORMAL")
            connection.execute(
                """
                CREATE TABLE search_meta (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL
                )
                """
            )
            connection.execute(
                """
                CREATE TABLE record_lookup (
                    record_id TEXT PRIMARY KEY,
                    run_id TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    in_workbench INTEGER NOT NULL,
                    in_dataset INTEGER NOT NULL
                )
                """
            )
            connection.execute(
                """
                CREATE VIRTUAL TABLE records_fts USING fts5(
                    record_id,
                    title,
                    label,
                    tags,
                    category_slug,
                    category_title,
                    dataset_id,
                    prompt_full,
                    tokenize = 'unicode61 remove_diacritics 2'
                )
                """
            )
            connection.executemany(
                "INSERT INTO search_meta (key, value) VALUES (?, ?)",
                (
                    ("schema_version", str(_INDEX_SCHEMA_VERSION)),
                    ("signature", signature),
                ),
            )
            connection.executemany(
                """
                INSERT INTO record_lookup (
                    record_id,
                    run_id,
                    created_at,
                    in_workbench,
                    in_dataset
                ) VALUES (?, ?, ?, ?, ?)
                """,
                [(doc[0], doc[8], doc[9], doc[10], doc[11]) for doc in documents],
            )
            connection.executemany(
                """
                INSERT INTO records_fts (
                    record_id,
                    title,
                    label,
                    tags,
                    category_slug,
                    category_title,
                    dataset_id,
                    prompt_full
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """,
                [doc[:8] for doc in documents],
            )
            connection.commit()

        os.replace(temporary_path, path)

        return SearchIndexStats(
            record_count=len(documents),
            category_count=category_count,
            workbench_entry_count=workbench_entry_count,
            path=path,
        )

    def _stored_meta(self) -> tuple[str | None, str | None]:
        path = self.repo.layout.search_index_path()
        if not path.exists():
            return None, None
        try:
            with sqlite3.connect(path) as connection:
                rows = connection.execute(
                    "SELECT key, value FROM search_meta WHERE key IN ('schema_version', 'signature')"
                ).fetchall()
        except sqlite3.Error:
            return None, None
        values = {str(key): str(value) for key, value in rows}
        return values.get("schema_version"), values.get("signature")

    def ensure_current(self) -> SearchIndexStats | None:
        path = self.repo.layout.search_index_path()
        signature = self._current_signature()
        if not path.exists():
            return self.rebuild()
        stored_schema_version, stored_signature = self._stored_meta()
        if stored_schema_version != str(_INDEX_SCHEMA_VERSION) or stored_signature != signature:
            return self.rebuild()
        return None

    def search_record_ids(
        self,
        query: str,
        *,
        source_filter: str | None = None,
        run_id: str | None = None,
        limit: int = 200,
    ) -> list[str]:
        self.ensure_current()
        expression = _query_expression(query)
        if expression is None:
            return []

        sql_parts = [
            """
            SELECT
                records_fts.record_id,
                records_fts.title,
                records_fts.label,
                records_fts.tags,
                records_fts.category_slug,
                records_fts.category_title,
                records_fts.dataset_id,
                records_fts.prompt_full,
                record_lookup.created_at,
                bm25(records_fts, ?, ?, ?, ?, ?, ?, ?, ?) AS rank_score
            FROM records_fts
            JOIN record_lookup USING (record_id)
            WHERE records_fts MATCH ?
            """
        ]
        parameters: list[Any] = [*_SCORE_WEIGHTS, expression]

        if source_filter == "workbench":
            sql_parts.append("AND record_lookup.in_workbench = 1")
        elif source_filter == "dataset":
            sql_parts.append("AND record_lookup.in_dataset = 1")

        if run_id:
            sql_parts.append("AND record_lookup.run_id = ?")
            parameters.append(run_id)

        sql_parts.append("LIMIT ?")
        parameters.append(limit)

        path = self.repo.layout.search_index_path()
        with sqlite3.connect(path) as connection:
            rows = connection.execute("\n".join(sql_parts), parameters).fetchall()

        query_tokens = _tokenize(query)
        phrase_query = " ".join(query_tokens)
        ranked_rows: list[tuple[float, str, str]] = []
        for row in rows:
            (
                record_id,
                title,
                label,
                tags,
                category_slug,
                category_title,
                dataset_id,
                prompt_full,
                created_at,
                rank_score,
            ) = row

            title_tokens = _tokenize(title)
            label_tokens = _tokenize(label)
            tag_tokens = _tokenize(tags)
            category_slug_tokens = _tokenize(category_slug)
            category_title_tokens = _tokenize(category_title)
            record_id_tokens = _tokenize(str(record_id))
            dataset_id_tokens = _tokenize(dataset_id)
            prompt_tokens = _tokenize(prompt_full)

            bonus = 0.0
            for token in query_tokens:
                high_signal_groups = (
                    title_tokens,
                    label_tokens,
                    tag_tokens,
                    category_slug_tokens,
                    category_title_tokens,
                )
                identifier_groups = (record_id_tokens, dataset_id_tokens)

                if any(token in group for group in high_signal_groups):
                    bonus += 24.0
                elif any(
                    group_token.startswith(token)
                    for group in high_signal_groups
                    for group_token in group
                ):
                    bonus += 14.0

                if any(token in group for group in identifier_groups):
                    bonus += 12.0
                elif any(
                    group_token.startswith(token)
                    for group in identifier_groups
                    for group_token in group
                ):
                    bonus += 8.0

                if len(token) >= 3:
                    if token in prompt_tokens:
                        bonus += 3.0
                    elif any(group_token.startswith(token) for group_token in prompt_tokens):
                        bonus += 1.0

            if phrase_query:
                if phrase_query in _phrase_text(title):
                    bonus += 10.0
                elif phrase_query in _phrase_text(label):
                    bonus += 8.0
                elif phrase_query in _phrase_text(category_title):
                    bonus += 8.0
                elif len(phrase_query) >= 5 and phrase_query in _phrase_text(prompt_full):
                    bonus += 2.0

            final_score = float(rank_score) - bonus
            ranked_rows.append((final_score, str(created_at or ""), str(record_id)))

        ranked_rows.sort(key=lambda item: (item[0], item[1]), reverse=False)
        ranked_rows.sort(key=lambda item: item[1], reverse=True)
        ranked_rows.sort(key=lambda item: item[0])
        return [record_id for _, _, record_id in ranked_rows]

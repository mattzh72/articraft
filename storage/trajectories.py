from __future__ import annotations

import hashlib
import shutil
from pathlib import Path

import zstandard as zstd

from storage.repo import StorageRepo

LEGACY_CONVERSATION_FILENAME = "conversation.jsonl"
TRAJECTORY_FILENAME = "trajectory.jsonl"
COMPRESSED_TRAJECTORY_FILENAME = "trajectory.jsonl.zst"
SYSTEM_PROMPT_FILENAMES = {
    "designer_system_prompt.txt",
    "designer_system_prompt_openai.txt",
    "designer_system_prompt_gemini.txt",
    "system_prompt.txt",
}
_ZSTD_LEVEL = 19


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compress_file_zstd(source: Path, destination: Path) -> Path:
    destination.parent.mkdir(parents=True, exist_ok=True)
    cctx = zstd.ZstdCompressor(level=_ZSTD_LEVEL)
    with source.open("rb") as src_handle, destination.open("wb") as dst_handle:
        with cctx.stream_writer(dst_handle) as compressor:
            shutil.copyfileobj(src_handle, compressor)
    return destination


def decompress_file_zstd(source: Path, destination: Path) -> Path:
    destination.parent.mkdir(parents=True, exist_ok=True)
    dctx = zstd.ZstdDecompressor()
    with source.open("rb") as src_handle, destination.open("wb") as dst_handle:
        with dctx.stream_reader(src_handle) as reader:
            shutil.copyfileobj(reader, dst_handle)
    return destination


def ensure_shared_system_prompt_text(
    repo: StorageRepo,
    prompt_text: str,
    *,
    prompt_sha256: str | None = None,
) -> Path:
    digest = prompt_sha256 or sha256_text(prompt_text)
    destination = repo.layout.system_prompt_path(digest)
    if destination.exists():
        existing = destination.read_text(encoding="utf-8")
        if existing != prompt_text:
            raise ValueError(f"Shared system prompt hash collision at {destination}")
        return destination
    repo.write_text(destination, prompt_text)
    return destination


def ensure_shared_system_prompt_file(
    repo: StorageRepo,
    source: Path,
    *,
    prompt_sha256: str | None = None,
) -> Path:
    return ensure_shared_system_prompt_text(
        repo,
        source.read_text(encoding="utf-8"),
        prompt_sha256=prompt_sha256,
    )


def legacy_system_prompt_paths(trace_dir: Path) -> list[Path]:
    return sorted(
        path
        for path in trace_dir.iterdir()
        if path.is_file() and path.name in SYSTEM_PROMPT_FILENAMES
    )


def compress_trajectory_file(source: Path, destination: Path | None = None) -> Path:
    target = destination or source.with_name(COMPRESSED_TRAJECTORY_FILENAME)
    return compress_file_zstd(source, target)


def find_plain_trajectory_path(trace_dir: Path) -> Path | None:
    for name in (TRAJECTORY_FILENAME, LEGACY_CONVERSATION_FILENAME):
        candidate = trace_dir / name
        if candidate.exists() and candidate.is_file():
            return candidate
    return None


def find_compressed_trajectory_path(trace_dir: Path) -> Path | None:
    candidate = trace_dir / COMPRESSED_TRAJECTORY_FILENAME
    if candidate.exists() and candidate.is_file():
        return candidate
    return None


def canonicalize_record_trace_dir(repo: StorageRepo, record_id: str) -> Path | None:
    trace_dir = repo.layout.record_traces_dir(record_id)
    if not trace_dir.exists():
        return None
    compressed_path = trace_dir / COMPRESSED_TRAJECTORY_FILENAME
    plain_path = find_plain_trajectory_path(trace_dir)
    if plain_path is not None and not compressed_path.exists():
        compress_trajectory_file(plain_path, compressed_path)
    for name in (TRAJECTORY_FILENAME, LEGACY_CONVERSATION_FILENAME):
        candidate = trace_dir / name
        if candidate.exists():
            candidate.unlink()
    for prompt_path in legacy_system_prompt_paths(trace_dir):
        prompt_path.unlink()
    return compressed_path if compressed_path.exists() else None


def unroll_record_trajectory(
    repo: StorageRepo,
    record_id: str,
    *,
    force: bool = False,
) -> Path:
    trace_dir = repo.layout.record_traces_dir(record_id)
    source = trace_dir / COMPRESSED_TRAJECTORY_FILENAME
    destination = repo.layout.record_trajectory_unroll_path(record_id)
    if source.exists():
        if (
            not force
            and destination.exists()
            and destination.stat().st_mtime >= source.stat().st_mtime
        ):
            return destination
        return decompress_file_zstd(source, destination)

    plain_source = find_plain_trajectory_path(trace_dir)
    if plain_source is None:
        raise FileNotFoundError(f"Trajectory not found for {record_id}: {source}")
    if (
        not force
        and destination.exists()
        and destination.stat().st_mtime >= plain_source.stat().st_mtime
    ):
        return destination
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(plain_source, destination)
    return destination

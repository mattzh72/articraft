from __future__ import annotations

from pathlib import Path

TEXT_MEDIA_TYPES = {
    ".urdf": "application/xml",
    ".xml": "application/xml",
    ".py": "text/x-python",
    ".txt": "text/plain",
    ".json": "application/json",
    ".md": "text/markdown",
    ".yaml": "application/yaml",
    ".yml": "application/yaml",
}

MEDIA_TYPE_MAP = {
    **TEXT_MEDIA_TYPES,
    ".glb": "model/gltf-binary",
    ".gltf": "model/gltf+json",
    ".stl": "model/stl",
    ".obj": "model/obj",
}

TRACE_MEDIA_TYPE_MAP = {
    ".jsonl": "application/x-ndjson",
    ".txt": "text/plain",
    ".json": "application/json",
    ".zst": "application/zstd",
}


def file_media_type(target: Path) -> str:
    return MEDIA_TYPE_MAP.get(target.suffix.lower(), "application/octet-stream")


def trace_media_type(target: Path) -> str:
    return TRACE_MEDIA_TYPE_MAP.get(target.suffix.lower(), file_media_type(target))


def file_cache_control(*, immutable: bool) -> str:
    return "public, max-age=31536000, immutable" if immutable else "no-store"


def read_text_file_payload(
    target: Path, *, preview_bytes: int, full: bool
) -> tuple[str, bool, int]:
    byte_count = target.stat().st_size
    truncated = False
    if full or byte_count <= preview_bytes:
        raw = target.read_bytes()
    else:
        with target.open("rb") as handle:
            raw = handle.read(preview_bytes + 1)
        truncated = byte_count > preview_bytes
        raw = raw[:preview_bytes]
    return raw.decode("utf-8", errors="replace"), truncated, byte_count

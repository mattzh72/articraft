from __future__ import annotations

from pathlib import Path


def test_active_repo_surfaces_do_not_reference_removed_collision_apis() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    search_roots = [
        repo_root / "agent",
        repo_root / "sdk",
        repo_root / "tests",
        repo_root / "viewer",
        repo_root / "scaffold.py",
    ]
    banned_fragments = (
        "generated_collisions",
        "CollisionGenerationSettings",
        "collision_generation_settings_from_env",
        'warn_if_part_contains_disconnected_geometry_islands(use="visual")',
        'warn_if_coplanar_surfaces(use="visual"',
        'geometry_source="collision"',
        "geometry_source='collision'",
        "include_generated_collisions",
        "source_collisions",
        "positive_use=",
        "negative_use=",
        "visual/collision geometry appear misaligned",
        "geometry=physical",
        "exact physical geometry",
        "CoACD",
        "coacd",
        "convex decomposition",
    )

    hits: list[str] = []
    for root in search_roots:
        paths = (
            [root] if root.is_file() else sorted(path for path in root.rglob("*") if path.is_file())
        )
        for path in paths:
            if path == Path(__file__).resolve():
                continue
            if path.suffix in {".pyc", ".png", ".jpg", ".jpeg", ".gif", ".obj", ".stl"}:
                continue
            try:
                text = path.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                continue
            for fragment in banned_fragments:
                if fragment in text:
                    hits.append(f"{path.relative_to(repo_root)}: {fragment}")

    assert hits == []

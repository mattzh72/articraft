# Geometry QC

Import:

```python
from sdk import (
    GeometryOverlap,
    UnsupportedPartFinding,
    default_contact_tol_from_env,
    default_overlap_tol_from_env,
    default_overlap_volume_tol_from_env,
    find_geometry_overlaps,
    find_unsupported_parts,
    validate_no_geometry_overlaps,
)
```

These helpers run conservative overlap QC across sampled articulation poses.

## Mesh support

For overlap QC, `Mesh(...)` must reference an `.obj` file.

## `find_geometry_overlaps(...)`

```python
find_geometry_overlaps(
    model,
    asset_root=None,
    geometry_source="collision",
    max_pose_samples=256,
    overlap_tol=1e-3,
    overlap_volume_tol=0.0,
    allowed_pairs=None,
    seed=0,
) -> list[GeometryOverlap]
```

Use `geometry_source="collision"` for physical overlap checks. Use `"visual"` when you explicitly want visible-envelope overlap checks.
Treat this as a safety backstop, not as the primary proof that parts look attached.

## `find_unsupported_parts(...)`

```python
find_unsupported_parts(
    model,
    asset_root=None,
    geometry_source="collision",
    max_pose_samples=1,
    contact_tol=1e-6,
    seed=0,
) -> list[UnsupportedPartFinding]
```

Find parts in a multi-part object that are not contacting any other part in the checked pose.

- Use `geometry_source="visual"` for visible-envelope support checks based on world AABBs.
- Use `geometry_source="collision"` for physical support checks on generated collision geometry.
- Collision-mode support queries require FCL. Missing or broken FCL support is an error; the SDK does not silently fall back.
- This function is used by the harness automatically at compile time to emit non-blocking isolated-part warnings.

## `validate_no_geometry_overlaps(...)`

```python
validate_no_geometry_overlaps(...)
```

Raises `ValidationError` with the worst overlapping pair and a coarse movement hint.

## Important limitations

This QC is conservative. It relies on generated collision geometry plus transformed bounding volumes, not exact visual-surface contact.

- It can report false positives for nested mounts, forks, axle supports, and other tightly packed assemblies.
- It should not be used to force visible air gaps between parts that are supposed to look attached.
- Use intent checks such as `expect_aabb_gap(...)`, `expect_aabb_overlap(..., axes="xy", ...)`, `expect_aabb_contact(...)`, and pose-specific checks to prove attachment quality.
- When slight intended interpenetration is acceptable, allow the specific pair narrowly rather than disabling QC entirely.

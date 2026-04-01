# Section Lofts (`sdk.mesh`)

## Purpose

Use `section_loft(...)` when a shell or exterior form can be described by a
small number of ordered cross-sections, with an optional path and symmetry.

## Import

```python
from sdk import LoftTessellation, LoftSection, SectionLoftSpec, section_loft, repair_loft
```

## Recommended APIs

- `section_loft(...)`
- `SectionLoftSpec`
- `repair_loft(...)`

## Core Entry Point

### `section_loft(...)`

```python
section_loft(
    spec: SectionLoftSpec | Sequence[LoftSection | Sequence[Sequence[float]]],
    **overrides,
) -> MeshGeometry
```

Accepted input forms:

- raw sequence of section loops
- `SectionLoftSpec(...)`

Each section loop must be an ordered closed loop of 3D points. At least two
sections are required.

## Spec Types

### `LoftTessellation`

```python
LoftTessellation(
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
)
```

- `tolerance`: linear tessellation tolerance.
- `angular_tolerance`: angular tessellation tolerance.

### `LoftSection`

```python
LoftSection(points: tuple[tuple[float, float, float], ...])
```

- `points`: one ordered section loop.

### `SectionLoftSpec`

```python
SectionLoftSpec(
    sections: tuple[LoftSection, ...],
    path: tuple[tuple[float, float, float], ...] | None = None,
    guide_curves: Mapping[str, tuple[tuple[float, float, float], ...]] | None = None,
    cap: bool = True,
    solid: bool = True,
    symmetry: str | None = None,
    ruled: bool = False,
    continuity: str = "C2",
    parametrization: str = "uniform",
    degree: int = 3,
    compat: bool = True,
    smoothing: bool = False,
    weights: tuple[float, float, float] = (1.0, 1.0, 1.0),
    repair: str = "auto",
    tessellation: LoftTessellation = LoftTessellation(),
)
```

Common fields:

- `sections`: required ordered cross-sections
- `path`: optional path for a path-driven multisection sweep
- `cap`: request end caps
- `solid`: request a solid result when supported
- `symmetry`: optional symmetry mode. Supported value: `mirror_yz`
- `repair`: one of `auto`, `mesh`, `kernel`, or `off`
- `tessellation`: tessellation settings for the mesh result

Advanced backend tuning fields:

- `guide_curves`
- `ruled`
- `continuity`
- `parametrization`
- `degree`
- `compat`
- `smoothing`
- `weights`

These fields are available, but the normal authoring path is to leave them at
their defaults unless a difficult loft specifically requires tuning.

## Repair

### `repair_loft(...)`

```python
repair_loft(
    geometry_or_spec: MeshGeometry | SectionLoftSpec | Sequence[LoftSection | Sequence[Sequence[float]]],
    *,
    repair: str = "auto",
) -> MeshGeometry
```

- If the input is already `MeshGeometry`, performs mesh-side repair.
- If the input is a loft spec or raw sections, rebuilds the loft with the
  requested repair mode.

## Advice

### Section correspondence

- Keep section order consistent from start to end.
- Keep each section as one clean loop.
- Use roughly corresponding perimeter points where possible.

### When to add `path`

- Omit `path` for a regular loft through the sections.
- Add `path` only when the overall form needs to bend or follow a centerline.

### When to use `repair_loft(...)`

- Use it when sparse or rough section input produces a visibly broken mesh.
- Prefer it over immediately reaching for advanced backend controls.

## Examples

Raw sections:

```python
geom = section_loft(
    [
        [(-0.05, -0.03, 0.00), (0.05, -0.03, 0.00), (0.05, 0.03, 0.00), (-0.05, 0.03, 0.00)],
        [(-0.03, -0.02, 0.08), (0.03, -0.02, 0.08), (0.03, 0.02, 0.08), (-0.03, 0.02, 0.08)],
    ]
)
```

Structured spec:

```python
geom = section_loft(
    SectionLoftSpec(
        sections=(section0, section1, section2),
        path=((0.0, 0.0, 0.0), (0.02, 0.01, 0.04), (0.04, 0.0, 0.08)),
        symmetry="mirror_yz",
    )
)
```

Repair:

```python
clean = repair_loft(spec, repair="mesh")
```

## See Also

- `40_mesh_geometry.md` for lower-level mesh loft helpers

## Clarifications for agent usage

- `guide_curves` is intentionally narrow: the supported keys are `spine`, `aux_spine`, and `binormal`.
- If `path` is omitted and `guide_curves.spine` is present, the spine becomes the sweep path.
- When the implementation takes the path-driven sweep branch, several advanced tuning fields (`continuity`, `parametrization`, `degree`, `compat`, `smoothing`, `weights`) are not active knobs. Treat them as advanced/conditional controls rather than guaranteed universal behavior.

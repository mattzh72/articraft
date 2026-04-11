from __future__ import annotations

import sdk


def test_section_loft_builds_simple_tapered_solid() -> None:
    geom = sdk.section_loft(
        [
            [(-0.05, -0.03, 0.0), (0.05, -0.03, 0.0), (0.05, 0.03, 0.0), (-0.05, 0.03, 0.0)],
            [(-0.03, -0.02, 0.08), (0.03, -0.02, 0.08), (0.03, 0.02, 0.08), (-0.03, 0.02, 0.08)],
        ]
    )

    assert len(geom.vertices) > 0
    assert len(geom.faces) > 0

    xs = [v[0] for v in geom.vertices]
    ys = [v[1] for v in geom.vertices]
    zs = [v[2] for v in geom.vertices]
    assert min(xs) <= -0.049
    assert max(xs) >= 0.049
    assert min(ys) <= -0.029
    assert max(ys) >= 0.029
    assert min(zs) <= 0.001
    assert max(zs) >= 0.079


def test_repair_loft_normalizes_dirty_meshgeometry() -> None:
    base = sdk.section_loft(
        [
            [(-0.04, -0.025, 0.0), (0.04, -0.025, 0.0), (0.04, 0.025, 0.0), (-0.04, 0.025, 0.0)],
            [
                (-0.02, -0.015, 0.05),
                (0.02, -0.015, 0.05),
                (0.02, 0.015, 0.05),
                (-0.02, 0.015, 0.05),
            ],
        ]
    )
    dirty = sdk.MeshGeometry(
        vertices=list(base.vertices) + [base.vertices[0]],
        faces=list(base.faces) + [base.faces[0]],
    )

    repaired = sdk.repair_loft(dirty)

    assert len(repaired.vertices) > 0
    assert len(repaired.faces) > 0
    assert all(len(face) == 3 for face in repaired.faces)
    assert all(max(face) < len(repaired.vertices) for face in repaired.faces)
    assert len(repaired.vertices) < len(dirty.vertices)
    assert repaired.faces != dirty.faces

    untouched = sdk.repair_loft(dirty, repair="off")
    assert untouched.vertices == dirty.vertices
    assert untouched.faces == dirty.faces


def test_section_loft_supports_path_and_aux_spine() -> None:
    spec = sdk.SectionLoftSpec(
        sections=(
            sdk.LoftSection(
                points=(
                    (-0.02, -0.01, 0.00),
                    (0.02, -0.01, 0.00),
                    (0.02, 0.01, 0.00),
                    (-0.02, 0.01, 0.00),
                )
            ),
            sdk.LoftSection(
                points=(
                    (-0.015, -0.008, 0.03),
                    (0.015, -0.008, 0.03),
                    (0.015, 0.008, 0.03),
                    (-0.015, 0.008, 0.03),
                )
            ),
            sdk.LoftSection(
                points=(
                    (-0.01, -0.006, 0.06),
                    (0.01, -0.006, 0.06),
                    (0.01, 0.006, 0.06),
                    (-0.01, 0.006, 0.06),
                )
            ),
        ),
        path=((0.0, 0.0, 0.0), (0.015, 0.01, 0.03), (0.03, 0.0, 0.06)),
        guide_curves={
            "aux_spine": (
                (0.0, 0.02, 0.0),
                (0.015, 0.025, 0.03),
                (0.03, 0.02, 0.06),
            )
        },
    )

    geom = sdk.section_loft(spec)

    assert len(geom.vertices) > 0
    assert len(geom.faces) > 0
    xs = [v[0] for v in geom.vertices]
    ys = [v[1] for v in geom.vertices]
    zs = [v[2] for v in geom.vertices]
    assert max(xs) > 0.019
    assert max(ys) > 0.009
    assert max(zs) >= 0.059


def test_section_loft_supports_spine_alias_and_symmetry() -> None:
    geom = sdk.section_loft(
        sdk.SectionLoftSpec(
            sections=(
                sdk.LoftSection(
                    points=(
                        (0.0, -0.02, 0.00),
                        (0.03, -0.02, 0.00),
                        (0.03, 0.02, 0.00),
                        (0.0, 0.02, 0.00),
                    )
                ),
                sdk.LoftSection(
                    points=(
                        (0.0, -0.015, 0.05),
                        (0.02, -0.015, 0.05),
                        (0.02, 0.015, 0.05),
                        (0.0, 0.015, 0.05),
                    )
                ),
            ),
            guide_curves={
                "spine": (
                    (0.0, 0.0, 0.00),
                    (0.0, 0.01, 0.025),
                    (0.0, 0.0, 0.05),
                )
            },
            symmetry="mirror_yz",
        )
    )

    xs = [v[0] for v in geom.vertices]
    assert min(xs) < -0.019
    assert max(xs) > 0.019

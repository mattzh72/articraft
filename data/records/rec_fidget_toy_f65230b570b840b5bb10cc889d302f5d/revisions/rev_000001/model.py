from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cubic_bezier_points(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    *,
    segments: int,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(1, segments + 1):
        t = index / segments
        u = 1.0 - t
        r = (
            (u**3) * p0[0]
            + 3.0 * (u**2) * t * p1[0]
            + 3.0 * u * (t**2) * p2[0]
            + (t**3) * p3[0]
        )
        z = (
            (u**3) * p0[1]
            + 3.0 * (u**2) * t * p1[1]
            + 3.0 * u * (t**2) * p2[1]
            + (t**3) * p3[1]
        )
        points.append((r, z))
    return points


def _top_profile() -> list[tuple[float, float]]:
    """Radius/z cross-section for a small one-piece spinning top."""

    profile: list[tuple[float, float]] = [
        (0.0, 0.0),
        (0.0038, 0.0),  # blunt flat contact tip
    ]
    profile.extend(
        _cubic_bezier_points(
            (0.0038, 0.0),
            (0.0080, 0.0045),
            (0.0280, 0.0270),
            (0.0480, 0.0310),
            segments=14,
        )
    )
    profile.extend(
        [
            (0.0500, 0.0345),
            (0.0470, 0.0387),
            (0.0400, 0.0398),
            (0.0340, 0.0398),
            (0.0320, 0.0392),  # shallow concentric top groove
            (0.0300, 0.0392),
            (0.0280, 0.0398),
            (0.0215, 0.0398),
            (0.0170, 0.0418),
            (0.0140, 0.0437),
            (0.0110, 0.0443),
        ]
    )

    for rib_index in range(5):
        base_z = 0.0452 + rib_index * 0.00365
        profile.extend(
            [
                (0.0110, base_z),
                (0.0136, base_z + 0.00055),
                (0.0136, base_z + 0.00215),
                (0.0110, base_z + 0.00280),
            ]
        )

    profile.extend(
        [
            (0.0105, 0.0644),
            (0.0072, 0.0668),
            (0.0032, 0.0684),
            (0.0, 0.0690),
        ]
    )
    return profile


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spinning_top_fidget_desk_toy")

    warm_wood = model.material("warm_wood", rgba=(0.62, 0.43, 0.26, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    anodized_teal = model.material("anodized_teal", rgba=(0.05, 0.42, 0.52, 1.0))

    desk = model.part("desk")
    desk.visual(
        Box((0.140, 0.140, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.0040)),
        material=warm_wood,
        name="desktop_slab",
    )
    desk.visual(
        Cylinder(radius=0.018, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, -0.0005)),
        material=brushed_steel,
        name="spin_pad",
    )
    desk.inertial = Inertial.from_geometry(
        Box((0.140, 0.140, 0.007)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.0035)),
    )

    top = model.part("top")
    top.visual(
        mesh_from_geometry(LatheGeometry(_top_profile(), segments=96), "spinning_top_body"),
        material=anodized_teal,
        name="top_body",
    )
    top.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.069),
        mass=0.080,
        origin=Origin(xyz=(0.0, 0.0, 0.0345)),
    )

    model.articulation(
        "spin_axis",
        ArticulationType.CONTINUOUS,
        parent=desk,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.02, velocity=80.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desk = object_model.get_part("desk")
    top = object_model.get_part("top")
    spin = object_model.get_articulation("spin_axis")

    ctx.check(
        "top has continuous vertical spin",
        spin.articulation_type == ArticulationType.CONTINUOUS and spin.axis == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_gap(
        top,
        desk,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="top_body",
        negative_elem="spin_pad",
        name="blunt tip sits on the desk spin pad",
    )
    ctx.expect_contact(
        top,
        desk,
        elem_a="top_body",
        elem_b="spin_pad",
        contact_tol=0.0005,
        name="tip contacts the support point",
    )
    ctx.expect_overlap(
        top,
        desk,
        axes="xy",
        elem_a="top_body",
        elem_b="spin_pad",
        min_overlap=0.006,
        name="tip footprint remains centered on the spin pad",
    )

    rest_position = ctx.part_world_position(top)
    with ctx.pose({spin: math.tau * 1.75}):
        spun_position = ctx.part_world_position(top)
        ctx.expect_gap(
            top,
            desk,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem="top_body",
            negative_elem="spin_pad",
            name="spinning pose keeps the tip on the pad",
        )
    ctx.check(
        "spin is about the fixed tip contact point",
        rest_position is not None
        and spun_position is not None
        and all(abs(a - b) < 1.0e-6 for a, b in zip(rest_position, spun_position)),
        details=f"rest={rest_position}, spun={spun_position}",
    )

    return ctx.report()


object_model = build_object_model()

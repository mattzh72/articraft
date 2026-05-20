from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_cover_hinge")

    zinc = model.material("brushed_zinc", rgba=(0.64, 0.66, 0.66, 1.0))
    darker_zinc = model.material("shadowed_zinc", rgba=(0.43, 0.45, 0.46, 1.0))
    cover_paint = model.material("painted_cover", rgba=(0.18, 0.31, 0.42, 1.0))
    screw_dark = model.material("dark_screw_heads", rgba=(0.11, 0.12, 0.13, 1.0))

    fixed_bracket = model.part("fixed_bracket")
    fixed_bracket.visual(
        Box((0.006, 0.210, 0.082)),
        origin=Origin(xyz=(-0.014, 0.0, -0.042)),
        material=zinc,
        name="bracket_leaf",
    )
    fixed_bracket.visual(
        Box((0.024, 0.216, 0.004)),
        origin=Origin(xyz=(-0.011, 0.0, -0.083)),
        material=darker_zinc,
        name="box_rim_shadow",
    )

    fixed_segments = (
        ("fixed_knuckle_0", -0.0805, 0.039),
        ("fixed_knuckle_1", 0.0, 0.039),
        ("fixed_knuckle_2", 0.0805, 0.039),
    )
    for name, y, length in fixed_segments:
        fixed_bracket.visual(
            Cylinder(radius=0.007, length=length),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=name,
        )
        fixed_bracket.visual(
            Box((0.013, length, 0.004)),
            origin=Origin(xyz=(-0.0065, y, -0.0065)),
            material=zinc,
            name=f"{name}_strap",
        )

    for y in (-0.103, 0.103):
        fixed_bracket.visual(
            Cylinder(radius=0.0048, length=0.006),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=darker_zinc,
            name=f"pin_head_{'neg' if y < 0 else 'pos'}",
        )
    fixed_bracket.visual(
        Cylinder(radius=0.0035, length=0.216),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=darker_zinc,
        name="hinge_pin",
    )

    for y in (-0.055, 0.055):
        for z in (-0.062, -0.025):
            fixed_bracket.visual(
                Cylinder(radius=0.0055, length=0.002),
                origin=Origin(xyz=(-0.0105, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=screw_dark,
                name=f"bracket_screw_{y:+.3f}_{z:+.3f}",
            )

    cover_leaf = model.part("cover_leaf")
    cover_leaf.visual(
        Box((0.163, 0.190, 0.005)),
        origin=Origin(xyz=(0.0885, 0.0, -0.0105)),
        material=cover_paint,
        name="cover_panel",
    )
    cover_leaf.visual(
        Box((0.006, 0.190, 0.042)),
        origin=Origin(xyz=(0.170, 0.0, -0.029)),
        material=cover_paint,
        name="edge_flange",
    )

    moving_segments = (
        ("cover_knuckle_0", -0.040, 0.038),
        ("cover_knuckle_1", 0.040, 0.038),
    )
    for name, y, length in moving_segments:
        cover_leaf.visual(
            Cylinder(radius=0.007, length=length),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=cover_paint,
            name=name,
        )
        cover_leaf.visual(
            Box((0.013, length, 0.004)),
            origin=Origin(xyz=(0.0065, y, -0.0065)),
            material=cover_paint,
            name=f"{name}_strap",
        )

    for x in (0.056, 0.130):
        for y in (-0.052, 0.052):
            cover_leaf.visual(
                Cylinder(radius=0.0046, length=0.0016),
                origin=Origin(xyz=(x, y, -0.0073)),
                material=screw_dark,
                name=f"cover_screw_{x:.3f}_{y:+.3f}",
            )

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed_bracket,
        child=cover_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_bracket = object_model.get_part("fixed_bracket")
    cover_leaf = object_model.get_part("cover_leaf")
    hinge = object_model.get_articulation("cover_hinge")

    for knuckle_name in ("cover_knuckle_0", "cover_knuckle_1"):
        ctx.allow_overlap(
            fixed_bracket,
            cover_leaf,
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            reason="The moving cover knuckle is intentionally captured around the fixed hinge pin.",
        )
        ctx.expect_within(
            fixed_bracket,
            cover_leaf,
            axes="xz",
            inner_elem="hinge_pin",
            outer_elem=knuckle_name,
            margin=0.0005,
            name=f"{knuckle_name} surrounds the hinge pin",
        )
        ctx.expect_overlap(
            fixed_bracket,
            cover_leaf,
            axes="y",
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            min_overlap=0.030,
            name=f"{knuckle_name} is retained along the pin",
        )

    ctx.expect_overlap(
        cover_leaf,
        fixed_bracket,
        axes="xz",
        elem_a="cover_knuckle_0",
        elem_b="fixed_knuckle_0",
        min_overlap=0.010,
        name="interleaved knuckles share the barrel axis",
    )
    ctx.expect_gap(
        cover_leaf,
        fixed_bracket,
        axis="y",
        positive_elem="cover_knuckle_0",
        negative_elem="fixed_knuckle_0",
        min_gap=0.001,
        max_gap=0.004,
        name="outer fixed knuckle clears moving knuckle",
    )
    ctx.expect_gap(
        fixed_bracket,
        cover_leaf,
        axis="y",
        positive_elem="fixed_knuckle_1",
        negative_elem="cover_knuckle_0",
        min_gap=0.001,
        max_gap=0.004,
        name="center fixed knuckle clears moving knuckle",
    )

    closed_aabb = ctx.part_world_aabb(cover_leaf)
    with ctx.pose({hinge: 1.2}):
        opened_aabb = ctx.part_world_aabb(cover_leaf)
    ctx.check(
        "cover leaf opens upward about supported barrel",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.10,
        details=f"closed_aabb={closed_aabb}, opened_aabb={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

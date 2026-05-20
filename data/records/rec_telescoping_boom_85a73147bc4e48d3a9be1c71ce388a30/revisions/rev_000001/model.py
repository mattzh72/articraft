from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_rectangular_tube_visuals(
    part,
    prefix: str,
    length: float,
    width_y: float,
    height_z: float,
    wall: float,
    origin_xyz: tuple[float, float, float],
    material: Material,
) -> None:
    """Build a true rectangular tube from four interlocked wall solids."""

    ox, oy, oz = origin_xyz
    part.visual(
        Box((length, width_y, wall)),
        origin=Origin(xyz=(ox, oy, oz + height_z / 2.0 - wall / 2.0)),
        material=material,
        name=f"{prefix}_top_wall",
    )
    part.visual(
        Box((length, width_y, wall)),
        origin=Origin(xyz=(ox, oy, oz - height_z / 2.0 + wall / 2.0)),
        material=material,
        name=f"{prefix}_bottom_wall",
    )
    part.visual(
        Box((length, wall, height_z)),
        origin=Origin(xyz=(ox, oy + width_y / 2.0 - wall / 2.0, oz)),
        material=material,
        name=f"{prefix}_pos_wall",
    )
    part.visual(
        Box((length, wall, height_z)),
        origin=Origin(xyz=(ox, oy - width_y / 2.0 + wall / 2.0, oz)),
        material=material,
        name=f"{prefix}_neg_wall",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_telescoping_boom")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.28, 0.31, 0.33, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.96, 0.68, 0.05, 1.0))
    orange = model.material("industrial_orange", rgba=(0.90, 0.32, 0.06, 1.0))
    black_rubber = model.material("black_wear_pad", rgba=(0.015, 0.014, 0.013, 1.0))
    zinc = model.material("zinc_bolt", rgba=(0.62, 0.64, 0.62, 1.0))

    base = model.part("base_bracket")
    base.visual(
        Box((0.70, 0.62, 0.055)),
        origin=Origin(xyz=(0.18, 0.0, 0.0275)),
        material=dark_steel,
        name="floor_plate",
    )
    base.visual(
        Box((0.055, 0.52, 0.50)),
        origin=Origin(xyz=(-0.035, 0.0, 0.315)),
        material=dark_steel,
        name="rear_web",
    )
    for y in (-0.245, 0.245):
        base.visual(
            Box((0.58, 0.045, 0.62)),
            origin=Origin(xyz=(0.26, y, 0.36)),
            material=dark_steel,
            name=f"side_cheek_{'neg' if y < 0 else 'pos'}",
        )
        base.visual(
            Box((0.36, 0.040, 0.055)),
            origin=Origin(
                xyz=(0.10, y, 0.205),
                rpy=(0.0, -0.63, 0.0),
            ),
            material=dark_steel,
            name=f"lower_gusset_{'neg' if y < 0 else 'pos'}",
        )
    base.visual(
        Box((0.58, 0.42, 0.045)),
        origin=Origin(xyz=(0.30, 0.0, 0.4075)),
        material=dark_steel,
        name="bottom_cradle",
    )
    base.visual(
        Box((0.58, 0.42, 0.040)),
        origin=Origin(xyz=(0.30, 0.0, 0.690)),
        material=dark_steel,
        name="top_clamp",
    )
    base.visual(
        Box((0.07, 0.44, 0.31)),
        origin=Origin(xyz=(0.02, 0.0, 0.55)),
        material=dark_steel,
        name="rear_collar",
    )
    _add_rectangular_tube_visuals(
        base,
        "outer",
        length=1.20,
        width_y=0.32,
        height_z=0.24,
        wall=0.020,
        origin_xyz=(0.60, 0.0, 0.55),
        material=gunmetal,
    )
    base.visual(
        Box((0.065, 0.39, 0.030)),
        origin=Origin(xyz=(1.215, 0.0, 0.685)),
        material=black_rubber,
        name="outer_top_wiper",
    )
    base.visual(
        Box((0.065, 0.39, 0.030)),
        origin=Origin(xyz=(1.215, 0.0, 0.415)),
        material=black_rubber,
        name="outer_bottom_wiper",
    )
    for y in (-0.185, 0.185):
        base.visual(
            Box((0.065, 0.030, 0.26)),
            origin=Origin(xyz=(1.215, y, 0.55)),
            material=black_rubber,
            name=f"outer_side_wiper_{'neg' if y < 0 else 'pos'}",
        )
    for x in (-0.08, 0.43):
        for y in (-0.235, 0.235):
            base.visual(
                Cylinder(radius=0.030, length=0.018),
                origin=Origin(xyz=(x, y, 0.064)),
                material=zinc,
                name=f"anchor_bolt_{x:.2f}_{y:.2f}",
            )

    middle = model.part("middle_tube")
    _add_rectangular_tube_visuals(
        middle,
        "middle",
        length=1.15,
        width_y=0.25,
        height_z=0.18,
        wall=0.016,
        origin_xyz=(-0.275, 0.0, 0.0),
        material=safety_yellow,
    )
    middle.visual(
        Box((0.055, 0.300, 0.024)),
        origin=Origin(xyz=(0.315, 0.0, 0.102)),
        material=black_rubber,
        name="middle_top_wiper",
    )
    middle.visual(
        Box((0.055, 0.300, 0.024)),
        origin=Origin(xyz=(0.315, 0.0, -0.102)),
        material=black_rubber,
        name="middle_bottom_wiper",
    )
    for y in (-0.145, 0.145):
        middle.visual(
            Box((0.055, 0.024, 0.205)),
            origin=Origin(xyz=(0.315, y, 0.0)),
            material=black_rubber,
            name=f"middle_side_wiper_{'neg' if y < 0 else 'pos'}",
        )
    middle.visual(
        Box((0.42, 0.10, 0.010)),
        origin=Origin(xyz=(-0.55, 0.0, 0.095)),
        material=black_rubber,
        name="middle_top_shoe",
    )
    middle.visual(
        Box((0.42, 0.10, 0.010)),
        origin=Origin(xyz=(-0.55, 0.0, -0.095)),
        material=black_rubber,
        name="middle_bottom_shoe",
    )
    for y in (-0.1325, 0.1325):
        middle.visual(
            Box((0.42, 0.015, 0.080)),
            origin=Origin(xyz=(-0.55, y, 0.0)),
            material=black_rubber,
            name=f"middle_side_shoe_{'neg' if y < 0 else 'pos'}",
        )

    inner = model.part("inner_tube")
    _add_rectangular_tube_visuals(
        inner,
        "inner",
        length=1.00,
        width_y=0.18,
        height_z=0.12,
        wall=0.012,
        origin_xyz=(-0.25, 0.0, 0.0),
        material=orange,
    )
    inner.visual(
        Box((0.060, 0.235, 0.170)),
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        material=dark_steel,
        name="tip_plate",
    )
    inner.visual(
        Cylinder(radius=0.040, length=0.026),
        origin=Origin(xyz=(0.322, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="tip_boss",
    )
    inner.visual(
        Box((0.36, 0.080, 0.014)),
        origin=Origin(xyz=(-0.45, 0.0, 0.067)),
        material=black_rubber,
        name="inner_top_shoe",
    )
    inner.visual(
        Box((0.36, 0.080, 0.014)),
        origin=Origin(xyz=(-0.45, 0.0, -0.067)),
        material=black_rubber,
        name="inner_bottom_shoe",
    )
    for y in (-0.0995, 0.0995):
        inner.visual(
            Box((0.36, 0.019, 0.060)),
            origin=Origin(xyz=(-0.45, y, 0.0)),
            material=black_rubber,
            name=f"inner_side_shoe_{'neg' if y < 0 else 'pos'}",
        )

    model.articulation(
        "middle_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=middle,
        origin=Origin(xyz=(1.20, 0.0, 0.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.20, lower=0.0, upper=0.55),
    )
    model.articulation(
        "inner_slide",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.30, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.18, lower=0.0, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_bracket")
    middle = object_model.get_part("middle_tube")
    inner = object_model.get_part("inner_tube")
    middle_slide = object_model.get_articulation("middle_slide")
    inner_slide = object_model.get_articulation("inner_slide")

    ctx.expect_within(
        middle,
        base,
        axes="yz",
        margin=0.0,
        name="middle tube nests inside outer sleeve cross section",
    )
    ctx.expect_overlap(
        middle,
        base,
        axes="x",
        min_overlap=0.80,
        name="middle tube has deep retained insertion when retracted",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.0,
        name="inner tube nests inside middle sleeve cross section",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.70,
        name="inner tube has deep retained insertion when retracted",
    )

    middle_rest = ctx.part_world_position(middle)
    inner_rest = ctx.part_world_position(inner)
    with ctx.pose({middle_slide: 0.55, inner_slide: 0.45}):
        ctx.expect_within(
            middle,
            base,
            axes="yz",
            margin=0.0,
            name="extended middle tube remains centered in outer sleeve",
        )
        ctx.expect_overlap(
            middle,
            base,
            axes="x",
            min_overlap=0.25,
            name="extended middle tube remains captured by outer sleeve",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.0,
            name="extended inner tube remains centered in middle sleeve",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.25,
            name="extended inner tube remains captured by middle sleeve",
        )
        middle_extended = ctx.part_world_position(middle)
        inner_extended = ctx.part_world_position(inner)

    ctx.check(
        "telescoping stages extend along positive x",
        middle_rest is not None
        and inner_rest is not None
        and middle_extended is not None
        and inner_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.50
        and inner_extended[0] > inner_rest[0] + 0.95,
        details=f"middle {middle_rest}->{middle_extended}, inner {inner_rest}->{inner_extended}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

from math import pi

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


STEEL_BLUE = Material("painted_steel_blue", rgba=(0.06, 0.13, 0.20, 1.0))
DARK_STEEL = Material("dark_machined_steel", rgba=(0.025, 0.028, 0.030, 1.0))
PIN_STEEL = Material("brushed_pin_steel", rgba=(0.62, 0.64, 0.62, 1.0))
SAFETY_YELLOW = Material("safety_yellow_caps", rgba=(0.95, 0.66, 0.08, 1.0))
RUBBER_BLACK = Material("black_rubber_pads", rgba=(0.01, 0.01, 0.012, 1.0))


def _hinge_cylinder(part, *, x: float, y: float, z: float, radius: float, length: float, material, name: str) -> None:
    """Add a cylinder whose axis is the arm's transverse hinge axis (local Y)."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_boxed_link(
    part,
    *,
    length: float,
    beam_height: float,
    beam_width: float,
    name_prefix: str,
    proximal_boss_length: float,
) -> None:
    """Deep welded box link with a center proximal boss and a forked distal end."""
    beam_len = length - 0.105
    beam_x = length / 2.0 - 0.0075
    part.visual(
        Box((beam_len, beam_width, beam_height)),
        origin=Origin(xyz=(beam_x, 0.0, 0.0)),
        material=STEEL_BLUE,
        name=f"{name_prefix}_box_beam",
    )
    part.visual(
        Box((beam_len - 0.090, 0.006, beam_height * 0.50)),
        origin=Origin(xyz=(beam_x, beam_width / 2.0 + 0.002, 0.0)),
        material=DARK_STEEL,
        name=f"{name_prefix}_side_recess_0",
    )
    part.visual(
        Box((beam_len - 0.090, 0.006, beam_height * 0.50)),
        origin=Origin(xyz=(beam_x, -beam_width / 2.0 - 0.002, 0.0)),
        material=DARK_STEEL,
        name=f"{name_prefix}_side_recess_1",
    )
    part.visual(
        Box((beam_len - 0.120, beam_width * 0.45, 0.018)),
        origin=Origin(xyz=(beam_x, 0.0, beam_height / 2.0 + 0.006)),
        material=SAFETY_YELLOW,
        name=f"{name_prefix}_top_rib",
    )

    _hinge_cylinder(
        part,
        x=0.0,
        y=0.0,
        z=0.0,
        radius=0.060,
        length=proximal_boss_length,
        material=PIN_STEEL,
        name=f"{name_prefix}_proximal_boss",
    )

    # Fork cheeks that receive the next link's central boss.  The transverse
    # gap leaves a visible mechanical clearance instead of an overlap.
    for side, y in enumerate((0.077, -0.077)):
        part.visual(
            Box((0.120, 0.030, beam_height + 0.050)),
            origin=Origin(xyz=(length, y, 0.0)),
            material=STEEL_BLUE,
            name=f"{name_prefix}_fork_cheek_{side}",
        )
        _hinge_cylinder(
            part,
            x=length,
            y=y * 1.16,
            z=0.0,
            radius=0.064,
            length=0.026,
            material=PIN_STEEL,
            name=f"{name_prefix}_outer_boss_{side}",
        )
        _hinge_cylinder(
            part,
            x=length,
            y=y * 1.37,
            z=0.0,
            radius=0.043,
            length=0.008,
            material=SAFETY_YELLOW,
            name=f"{name_prefix}_pin_cap_{side}",
        )

    for z in (beam_height / 2.0 + 0.014, -beam_height / 2.0 - 0.014):
        part.visual(
            Box((0.052, 0.184, 0.016)),
            origin=Origin(xyz=(length - 0.064, 0.0, z)),
            material=STEEL_BLUE,
            name=f"{name_prefix}_fork_bridge_{'top' if z > 0.0 else 'bottom'}",
        )
    part.visual(
        Box((0.026, beam_width * 0.55, 0.018)),
        origin=Origin(xyz=(length - 0.074, 0.0, beam_height / 2.0 + 0.003)),
        material=STEEL_BLUE,
        name=f"{name_prefix}_distal_web",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_fold_out_arm")

    base = model.part("base_cheek")
    base.visual(
        Box((0.42, 0.34, 0.050)),
        origin=Origin(xyz=(-0.040, 0.0, 0.025)),
        material=DARK_STEEL,
        name="floor_plate",
    )
    base.visual(
        Box((0.205, 0.205, 0.180)),
        origin=Origin(xyz=(-0.025, 0.0, 0.135)),
        material=STEEL_BLUE,
        name="pedestal_block",
    )
    base.visual(
        Box((0.055, 0.245, 0.315)),
        origin=Origin(xyz=(-0.105, 0.0, 0.220)),
        material=STEEL_BLUE,
        name="rear_web",
    )
    for side, y in enumerate((0.093, -0.093)):
        base.visual(
            Box((0.170, 0.032, 0.375)),
            origin=Origin(xyz=(0.0, y, 0.230)),
            material=STEEL_BLUE,
            name=f"base_cheek_plate_{side}",
        )
        base.visual(
            Box((0.210, 0.028, 0.040)),
            origin=Origin(xyz=(-0.045, y, 0.180), rpy=(0.0, 0.55, 0.0)),
            material=SAFETY_YELLOW,
            name=f"cheek_gusset_{side}",
        )
        _hinge_cylinder(
            base,
            x=0.0,
            y=y * 1.20,
            z=0.380,
            radius=0.076,
            length=0.034,
            material=PIN_STEEL,
            name=f"base_outer_boss_{side}",
        )

    for i, (x, y) in enumerate(((-0.180, 0.120), (-0.180, -0.120), (0.105, 0.120), (0.105, -0.120))):
        base.visual(
            Cylinder(radius=0.015, length=0.014),
            origin=Origin(xyz=(x, y, 0.053)),
            material=PIN_STEEL,
            name=f"anchor_bolt_{i}",
        )

    link_0 = model.part("link_0")
    _add_boxed_link(
        link_0,
        length=0.580,
        beam_height=0.125,
        beam_width=0.082,
        name_prefix="first",
        proximal_boss_length=0.154,
    )

    link_1 = model.part("link_1")
    _add_boxed_link(
        link_1,
        length=0.500,
        beam_height=0.118,
        beam_width=0.078,
        name_prefix="second",
        proximal_boss_length=0.124,
    )

    link_2 = model.part("link_2")
    _add_boxed_link(
        link_2,
        length=0.420,
        beam_height=0.108,
        beam_width=0.074,
        name_prefix="third",
        proximal_boss_length=0.124,
    )

    platform = model.part("platform_bracket")
    _hinge_cylinder(
        platform,
        x=0.0,
        y=0.0,
        z=0.0,
        radius=0.055,
        length=0.124,
        material=PIN_STEEL,
        name="wrist_boss",
    )
    platform.visual(
        Box((0.185, 0.072, 0.060)),
        origin=Origin(xyz=(0.095, 0.0, -0.020)),
        material=STEEL_BLUE,
        name="bracket_neck",
    )
    platform.visual(
        Box((0.190, 0.190, 0.026)),
        origin=Origin(xyz=(0.205, 0.0, -0.063)),
        material=STEEL_BLUE,
        name="mounting_plate",
    )
    platform.visual(
        Box((0.030, 0.170, 0.070)),
        origin=Origin(xyz=(0.295, 0.0, -0.040)),
        material=STEEL_BLUE,
        name="front_lip",
    )
    for i, (x, y) in enumerate(((0.150, 0.066), (0.150, -0.066), (0.260, 0.066), (0.260, -0.066))):
        platform.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(xyz=(x, y, -0.056)),
            material=RUBBER_BLACK,
            name=f"platform_pad_{i}",
        )

    hinge_limits = MotionLimits(effort=160.0, velocity=1.2, lower=-2.20, upper=2.20)
    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.580, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.500, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "link_2_to_platform",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=platform,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.4, lower=-1.70, upper=1.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [
        object_model.get_articulation("base_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
        object_model.get_articulation("link_2_to_platform"),
    ]
    ctx.check(
        "serial chain has four revolute hinges",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details="The fold-out arm should be a four-joint revolute serial chain.",
    )
    ctx.check(
        "hinge axes are parallel",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    platform = object_model.get_part("platform_bracket")

    ctx.expect_overlap(
        link_1,
        link_0,
        axes="z",
        elem_a="second_proximal_boss",
        elem_b="first_fork_cheek_0",
        min_overlap=0.035,
        name="second link boss sits inside first fork height",
    )
    ctx.expect_overlap(
        link_2,
        link_1,
        axes="z",
        elem_a="third_proximal_boss",
        elem_b="second_fork_cheek_0",
        min_overlap=0.030,
        name="third link boss sits inside second fork height",
    )
    ctx.expect_overlap(
        platform,
        link_2,
        axes="z",
        elem_a="wrist_boss",
        elem_b="third_fork_cheek_0",
        min_overlap=0.030,
        name="platform wrist boss sits inside final fork height",
    )

    rest = ctx.part_world_position(platform)
    with ctx.pose({"base_to_link_0": -0.65, "link_0_to_link_1": 1.10, "link_1_to_link_2": -0.85, "link_2_to_platform": 0.45}):
        folded = ctx.part_world_position(platform)
    ctx.check(
        "folded pose moves terminal bracket",
        rest is not None
        and folded is not None
        and abs(folded[0] - rest[0]) > 0.10
        and abs(folded[2] - rest[2]) > 0.08,
        details=f"rest={rest}, folded={folded}",
    )

    return ctx.report()


object_model = build_object_model()

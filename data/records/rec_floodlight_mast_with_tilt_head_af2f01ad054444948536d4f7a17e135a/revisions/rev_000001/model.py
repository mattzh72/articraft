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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_work_light_tower")

    black = Material("powder_coated_black", rgba=(0.015, 0.015, 0.018, 1.0))
    rubber = Material("matte_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    aluminum = Material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    lens = Material("warm_led_lens", rgba=(1.0, 0.82, 0.26, 0.92))
    glow = Material("led_emitters", rgba=(1.0, 0.96, 0.48, 1.0))
    orange = Material("safety_orange_trim", rgba=(1.0, 0.34, 0.03, 1.0))

    # Root: a wide floor cross-bar with protruding receiver sleeves for the two
    # telescoping stabilizer legs and an upright receiver sleeve for the mast.
    base = model.part("base")
    base.visual(
        Box((1.35, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=black,
        name="cross_bar",
    )
    for x in (-0.55, 0.55):
        base.visual(
            Box((0.22, 0.11, 0.026)),
            origin=Origin(xyz=(x, 0.0, -0.010)),
            material=rubber,
            name=f"rubber_foot_{0 if x < 0 else 1}",
        )
    base.visual(
        Box((0.16, 0.23, 0.075)),
        origin=Origin(xyz=(0.0, 0.170, 0.040)),
        material=black,
        name="leg_socket_0",
    )
    base.visual(
        Box((0.16, 0.23, 0.075)),
        origin=Origin(xyz=(0.0, -0.170, 0.040)),
        material=black,
        name="leg_socket_1",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.545),
        origin=Origin(xyz=(0.0, 0.0, 0.3475)),
        material=black,
        name="mast_sleeve",
    )
    base.visual(
        Box((0.16, 0.16, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=black,
        name="sleeve_collar",
    )

    # Two opposed stabilizer legs slide outward from the base sockets.  Their
    # long rectangular tubes include hidden insertion length so the upper-limit
    # pose is still mechanically retained inside the sleeves.
    leg_0 = model.part("leg_0")
    leg_0.visual(
        Box((0.090, 0.900, 0.045)),
        origin=Origin(xyz=(0.0, 0.270, 0.0)),
        material=aluminum,
        name="leg_tube",
    )
    leg_0.visual(
        Box((0.26, 0.13, 0.030)),
        origin=Origin(xyz=(0.0, 0.735, -0.038)),
        material=rubber,
        name="outer_pad",
    )
    leg_0.visual(
        Box((0.11, 0.08, 0.060)),
        origin=Origin(xyz=(0.0, 0.685, -0.010)),
        material=black,
        name="pad_bracket",
    )

    leg_1 = model.part("leg_1")
    leg_1.visual(
        Box((0.090, 0.900, 0.045)),
        origin=Origin(xyz=(0.0, -0.270, 0.0)),
        material=aluminum,
        name="leg_tube",
    )
    leg_1.visual(
        Box((0.26, 0.13, 0.030)),
        origin=Origin(xyz=(0.0, -0.735, -0.038)),
        material=rubber,
        name="outer_pad",
    )
    leg_1.visual(
        Box((0.11, 0.08, 0.060)),
        origin=Origin(xyz=(0.0, -0.685, -0.010)),
        material=black,
        name="pad_bracket",
    )

    # The mast is the moving telescoping column.  Its local frame starts at the
    # top lip of the fixed sleeve; the visible T-bar and clevis blocks ride with
    # the mast and carry the two tilting flood heads.
    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.030, length=1.750),
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
        material=aluminum,
        name="inner_column",
    )
    mast.visual(
        Cylinder(radius=0.026, length=0.860),
        origin=Origin(xyz=(0.0, -0.065, 1.250), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="t_bar",
    )
    mast.visual(
        Box((0.13, 0.150, 0.105)),
        origin=Origin(xyz=(0.0, -0.032, 1.225)),
        material=black,
        name="t_collar",
    )
    mast.visual(
        Box((0.100, 0.085, 0.090)),
        origin=Origin(xyz=(0.48, 0.0, 1.250)),
        material=black,
        name="end_clevis_0",
    )
    mast.visual(
        Box((0.100, 0.085, 0.090)),
        origin=Origin(xyz=(-0.48, 0.0, 1.250)),
        material=black,
        name="end_clevis_1",
    )

    def add_flood_head(name: str) -> object:
        head = model.part(name)
        head.visual(
            Cylinder(radius=0.032, length=0.280),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name="pivot_barrel",
        )
        for idx, x in enumerate((-0.115, 0.115)):
            head.visual(
                Box((0.028, 0.115, 0.052)),
                origin=Origin(xyz=(x, 0.048, 0.0)),
                material=black,
                name=f"yoke_arm_{idx}",
            )
        head.visual(
            Box((0.310, 0.105, 0.220)),
            origin=Origin(xyz=(0.0, 0.135, 0.0)),
            material=black,
            name="lamp_housing",
        )
        head.visual(
            Box((0.292, 0.024, 0.202)),
            origin=Origin(xyz=(0.0, 0.198, 0.0)),
            material=orange,
            name="front_bezel",
        )
        head.visual(
            Box((0.248, 0.014, 0.154)),
            origin=Origin(xyz=(0.0, 0.216, 0.0)),
            material=lens,
            name="led_lens",
        )
        for idx, z in enumerate((-0.052, 0.0, 0.052)):
            head.visual(
                Box((0.210, 0.006, 0.018)),
                origin=Origin(xyz=(0.0, 0.222, z)),
                material=glow,
                name=f"led_row_{idx}",
            )
        for idx, z in enumerate((-0.080, -0.040, 0.0, 0.040, 0.080)):
            head.visual(
                Box((0.255, 0.030, 0.009)),
                origin=Origin(xyz=(0.0, 0.071, z)),
                material=black,
                name=f"heat_fin_{idx}",
            )
        return head

    head_0 = add_flood_head("head_0")
    head_1 = add_flood_head("head_1")

    model.articulation(
        "leg_slide_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=leg_0,
        origin=Origin(xyz=(0.0, 0.285, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=0.0, upper=0.16),
    )
    model.articulation(
        "leg_slide_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=leg_1,
        origin=Origin(xyz=(0.0, -0.285, 0.040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=0.0, upper=0.16),
    )
    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.18, lower=0.0, upper=0.45),
    )
    model.articulation(
        "head_tilt_0",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=head_0,
        origin=Origin(xyz=(0.48, 0.0, 1.250)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.70, upper=0.85),
    )
    model.articulation(
        "head_tilt_1",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=head_1,
        origin=Origin(xyz=(-0.48, 0.0, 1.250)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.70, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    leg_0 = object_model.get_part("leg_0")
    leg_1 = object_model.get_part("leg_1")
    mast = object_model.get_part("mast")
    head_0 = object_model.get_part("head_0")
    head_1 = object_model.get_part("head_1")

    leg_slide_0 = object_model.get_articulation("leg_slide_0")
    leg_slide_1 = object_model.get_articulation("leg_slide_1")
    mast_slide = object_model.get_articulation("mast_slide")
    head_tilt_0 = object_model.get_articulation("head_tilt_0")
    head_tilt_1 = object_model.get_articulation("head_tilt_1")

    ctx.allow_overlap(
        base,
        leg_0,
        elem_a="leg_socket_0",
        elem_b="leg_tube",
        reason="The front stabilizer leg tube is intentionally retained inside its telescoping receiver sleeve.",
    )
    ctx.allow_overlap(
        base,
        leg_1,
        elem_a="leg_socket_1",
        elem_b="leg_tube",
        reason="The rear stabilizer leg tube is intentionally retained inside its telescoping receiver sleeve.",
    )
    ctx.allow_overlap(
        base,
        mast,
        elem_a="mast_sleeve",
        elem_b="inner_column",
        reason="The mast column is intentionally nested inside the fixed vertical sleeve.",
    )
    ctx.allow_overlap(
        mast,
        head_0,
        elem_a="end_clevis_0",
        elem_b="pivot_barrel",
        reason="The flood-head pivot barrel is captured inside the bar-end clevis.",
    )
    ctx.allow_overlap(
        mast,
        head_1,
        elem_a="end_clevis_1",
        elem_b="pivot_barrel",
        reason="The flood-head pivot barrel is captured inside the bar-end clevis.",
    )

    ctx.expect_within(
        leg_0,
        base,
        axes="xz",
        inner_elem="leg_tube",
        outer_elem="leg_socket_0",
        margin=0.002,
        name="leg_0 stays centered in its sleeve",
    )
    ctx.expect_overlap(
        leg_0,
        base,
        axes="y",
        elem_a="leg_tube",
        elem_b="leg_socket_0",
        min_overlap=0.16,
        name="leg_0 retained at rest",
    )
    ctx.expect_within(
        leg_1,
        base,
        axes="xz",
        inner_elem="leg_tube",
        outer_elem="leg_socket_1",
        margin=0.002,
        name="leg_1 stays centered in its sleeve",
    )
    ctx.expect_overlap(
        leg_1,
        base,
        axes="y",
        elem_a="leg_tube",
        elem_b="leg_socket_1",
        min_overlap=0.16,
        name="leg_1 retained at rest",
    )
    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_column",
        outer_elem="mast_sleeve",
        margin=0.002,
        name="mast stays centered in sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_column",
        elem_b="mast_sleeve",
        min_overlap=0.48,
        name="mast retained at rest",
    )
    for head, elem in ((head_0, "end_clevis_0"), (head_1, "end_clevis_1")):
        ctx.expect_within(
            head,
            mast,
            axes="yz",
            inner_elem="pivot_barrel",
            outer_elem=elem,
            margin=0.003,
            name=f"{head.name} pivot centered in clevis",
        )
        ctx.expect_overlap(
            head,
            mast,
            axes="x",
            elem_a="pivot_barrel",
            elem_b=elem,
            min_overlap=0.09,
            name=f"{head.name} pivot retained along hinge",
        )

    rest_leg_0 = ctx.part_world_position(leg_0)
    rest_leg_1 = ctx.part_world_position(leg_1)
    rest_mast = ctx.part_world_position(mast)
    rest_lens = ctx.part_element_world_aabb(head_0, elem="led_lens")

    with ctx.pose({leg_slide_0: 0.16}):
        ctx.expect_overlap(
            leg_0,
            base,
            axes="y",
            elem_a="leg_tube",
            elem_b="leg_socket_0",
            min_overlap=0.018,
            name="leg_0 retained when extended",
        )
        extended_leg_0 = ctx.part_world_position(leg_0)
    with ctx.pose({leg_slide_1: 0.16}):
        ctx.expect_overlap(
            leg_1,
            base,
            axes="y",
            elem_a="leg_tube",
            elem_b="leg_socket_1",
            min_overlap=0.018,
            name="leg_1 retained when extended",
        )
        extended_leg_1 = ctx.part_world_position(leg_1)
    with ctx.pose({mast_slide: 0.45}):
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_column",
            elem_b="mast_sleeve",
            min_overlap=0.045,
            name="mast retained when raised",
        )
        extended_mast = ctx.part_world_position(mast)
    with ctx.pose({head_tilt_0: 0.60, head_tilt_1: 0.60}):
        tilted_lens = ctx.part_element_world_aabb(head_0, elem="led_lens")

    ctx.check(
        "leg_0 slides outward",
        rest_leg_0 is not None
        and extended_leg_0 is not None
        and extended_leg_0[1] > rest_leg_0[1] + 0.12,
        details=f"rest={rest_leg_0}, extended={extended_leg_0}",
    )
    ctx.check(
        "leg_1 slides outward",
        rest_leg_1 is not None
        and extended_leg_1 is not None
        and extended_leg_1[1] < rest_leg_1[1] - 0.12,
        details=f"rest={rest_leg_1}, extended={extended_leg_1}",
    )
    ctx.check(
        "mast raises vertically",
        rest_mast is not None
        and extended_mast is not None
        and extended_mast[2] > rest_mast[2] + 0.40,
        details=f"rest={rest_mast}, extended={extended_mast}",
    )
    if rest_lens is not None and tilted_lens is not None:
        rest_z = 0.5 * (rest_lens[0][2] + rest_lens[1][2])
        tilted_z = 0.5 * (tilted_lens[0][2] + tilted_lens[1][2])
    else:
        rest_z = tilted_z = None
    ctx.check(
        "head tilt raises beam face",
        rest_z is not None and tilted_z is not None and tilted_z > rest_z + 0.08,
        details=f"rest_lens_z={rest_z}, tilted_lens_z={tilted_z}",
    )

    return ctx.report()


object_model = build_object_model()

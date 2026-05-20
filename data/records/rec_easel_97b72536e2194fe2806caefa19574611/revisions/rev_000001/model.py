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
    model = ArticulatedObject(name="h_frame_easel")

    wood = model.material("varnished_maple", rgba=(0.72, 0.48, 0.25, 1.0))
    dark_wood = model.material("end_grain", rgba=(0.48, 0.30, 0.14, 1.0))
    metal = model.material("dark_burnished_steel", rgba=(0.06, 0.06, 0.055, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))

    frame = model.part("frame")
    # Broad floor base: a wide front rail, rear stabilizer rail, and a center spine.
    frame.visual(
        Box((1.30, 0.16, 0.07)),
        origin=Origin(xyz=(0.0, -0.10, 0.035)),
        material=wood,
        name="front_base_rail",
    )
    frame.visual(
        Box((1.05, 0.14, 0.07)),
        origin=Origin(xyz=(0.0, 0.32, 0.035)),
        material=wood,
        name="rear_base_rail",
    )
    frame.visual(
        Box((0.12, 0.52, 0.06)),
        origin=Origin(xyz=(0.0, 0.10, 0.035)),
        material=dark_wood,
        name="base_spine",
    )

    # Uprights and split H crossbars leave a real central channel for the mast.
    for index, x in enumerate((-0.46, 0.46)):
        frame.visual(
            Box((0.07, 0.075, 1.50)),
            origin=Origin(xyz=(x, 0.0, 0.82)),
            material=wood,
            name=f"upright_{index}",
        )

    for z, height, label, width, x_abs in (
        (0.28, 0.08, "lower_bar", 0.43, 0.285),
        (0.82, 0.07, "middle_bar", 0.43, 0.285),
        (1.45, 0.08, "top_bar", 0.39, 0.305),
    ):
        for index, x in enumerate((-x_abs, x_abs)):
            frame.visual(
                Box((width, 0.07, height)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=wood,
                name=f"{label}_{index}",
            )

    for name, x in (("guide_rail_0", -0.0575), ("guide_rail_1", 0.0575)):
        frame.visual(
            Box((0.025, 0.09, 1.00)),
            origin=Origin(xyz=(x, 0.0, 0.92)),
            material=dark_wood,
            name=name,
        )

    # Fixed lower canvas tray, connected back into the H-frame by short brackets.
    frame.visual(
        Box((0.90, 0.10, 0.045)),
        origin=Origin(xyz=(0.0, -0.12, 0.42)),
        material=wood,
        name="canvas_tray",
    )
    frame.visual(
        Box((0.90, 0.035, 0.12)),
        origin=Origin(xyz=(0.0, -0.065, 0.475)),
        material=dark_wood,
        name="tray_back_lip",
    )
    for index, x in enumerate((-0.32, 0.32)):
        frame.visual(
            Box((0.06, 0.17, 0.055)),
            origin=Origin(xyz=(x, -0.035, 0.405)),
            material=wood,
            name=f"tray_bracket_{index}",
        )
        frame.visual(
            Box((0.05, 0.06, 0.08)),
            origin=Origin(xyz=(x, 0.0, 0.36)),
            material=wood,
            name=f"tray_stanchion_{index}",
        )

    mast = model.part("mast")
    mast.visual(
        Box((0.060, 0.048, 1.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material=wood,
        name="mast_bar",
    )
    for name, x in (("slide_pad_0", -0.0375), ("slide_pad_1", 0.0375)):
        mast.visual(
            Box((0.015, 0.048, 0.70)),
            origin=Origin(xyz=(x, 0.0, 0.55)),
            material=dark_wood,
            name=name,
        )
    mast.visual(
        Box((0.18, 0.075, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.13)),
        material=wood,
        name="mast_head",
    )
    for index, x in enumerate((-0.075, 0.075)):
        mast.visual(
            Box((0.035, 0.10, 0.12)),
            origin=Origin(xyz=(x, -0.005, 1.23)),
            material=wood,
            name=f"hinge_lug_{index}",
        )
    mast.visual(
        Cylinder(radius=0.012, length=0.19),
        origin=Origin(xyz=(0.0, -0.005, 1.23), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_pin",
    )

    holder_arm = model.part("holder_arm")
    holder_arm.visual(
        Cylinder(radius=0.025, length=0.09),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_barrel",
    )
    holder_arm.visual(
        Box((0.08, 0.34, 0.04)),
        origin=Origin(xyz=(0.0, -0.20, -0.04)),
        material=wood,
        name="holder_arm",
    )
    for index, x in enumerate((-0.043, 0.043)):
        holder_arm.visual(
            Box((0.022, 0.055, 0.025)),
            origin=Origin(xyz=(x, -0.015, -0.0375)),
            material=wood,
            name=f"barrel_cheek_{index}",
        )
    holder_arm.visual(
        Box((0.58, 0.05, 0.045)),
        origin=Origin(xyz=(0.0, -0.335, -0.005)),
        material=wood,
        name="top_clamp_crossbar",
    )
    holder_arm.visual(
        Box((0.58, 0.025, 0.08)),
        origin=Origin(xyz=(0.0, -0.365, -0.035)),
        material=rubber,
        name="front_grip_pad",
    )

    model.articulation(
        "frame_to_mast",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=0.0, upper=0.35),
    )
    model.articulation(
        "mast_to_holder_arm",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=holder_arm,
        origin=Origin(xyz=(0.0, -0.005, 1.23)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.75, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    mast = object_model.get_part("mast")
    holder_arm = object_model.get_part("holder_arm")
    mast_slide = object_model.get_articulation("frame_to_mast")
    holder_hinge = object_model.get_articulation("mast_to_holder_arm")

    ctx.allow_overlap(
        mast,
        holder_arm,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The dark metal hinge pin is intentionally modeled as a solid captured rod through the arm barrel.",
    )
    ctx.expect_overlap(
        mast,
        holder_arm,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.08,
        name="hinge pin passes through arm barrel",
    )

    # The telescoping mast rides in a real open channel: side pads touch the guide
    # rails while the central crossbars are split to avoid a solid proxy overlap.
    with ctx.pose({mast_slide: 0.0}):
        ctx.expect_gap(
            frame,
            mast,
            axis="x",
            positive_elem="guide_rail_1",
            negative_elem="slide_pad_1",
            max_gap=0.001,
            max_penetration=0.0,
            name="right slide pad bears on guide rail",
        )
        ctx.expect_gap(
            mast,
            frame,
            axis="x",
            positive_elem="slide_pad_0",
            negative_elem="guide_rail_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="left slide pad bears on guide rail",
        )
        ctx.expect_overlap(
            mast,
            frame,
            axes="z",
            elem_a="slide_pad_0",
            elem_b="guide_rail_0",
            min_overlap=0.60,
            name="collapsed mast remains deeply engaged",
        )

    rest_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.35}):
        ctx.expect_gap(
            frame,
            mast,
            axis="x",
            positive_elem="guide_rail_1",
            negative_elem="slide_pad_1",
            max_gap=0.001,
            max_penetration=0.0,
            name="extended right slide pad stays guided",
        )
        ctx.expect_overlap(
            mast,
            frame,
            axes="z",
            elem_a="slide_pad_1",
            elem_b="guide_rail_1",
            min_overlap=0.45,
            name="extended mast retains insertion",
        )
        extended_pos = ctx.part_world_position(mast)

    ctx.check(
        "mast slides upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.30,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    rest_pad_aabb = ctx.part_element_world_aabb(holder_arm, elem="front_grip_pad")
    with ctx.pose({holder_hinge: -0.60}):
        lifted_pad_aabb = ctx.part_element_world_aabb(holder_arm, elem="front_grip_pad")

    def aabb_center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_pad_z = aabb_center_z(rest_pad_aabb)
    lifted_pad_z = aabb_center_z(lifted_pad_aabb)
    ctx.check(
        "top holder arm pivots upward",
        rest_pad_z is not None and lifted_pad_z is not None and lifted_pad_z > rest_pad_z + 0.15,
        details=f"rest_z={rest_pad_z}, lifted_z={lifted_pad_z}",
    )

    return ctx.report()


object_model = build_object_model()

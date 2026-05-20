from __future__ import annotations

from math import pi

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


def add_box(
    part,
    size,
    xyz,
    *,
    rpy=(0.0, 0.0, 0.0),
    material=None,
    name=None,
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def add_cylinder(
    part,
    radius,
    length,
    xyz,
    *,
    rpy=(0.0, 0.0, 0.0),
    material=None,
    name=None,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_easel")

    wood = model.material("wood", rgba=(0.64, 0.49, 0.31, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.54, 0.39, 0.23, 1.0))
    steel = model.material("steel", rgba=(0.27, 0.29, 0.32, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.11, 0.12, 0.13, 1.0))

    frame = model.part("frame")
    add_box(frame, (0.74, 0.07, 0.07), (0.10, -0.28, 0.035), material=wood, name="foot_0")
    add_box(frame, (0.74, 0.07, 0.07), (0.10, 0.28, 0.035), material=wood, name="foot_1")
    add_box(frame, (0.09, 0.54, 0.07), (0.32, 0.0, 0.08), material=dark_wood, name="front_stretcher")
    add_box(frame, (0.09, 0.54, 0.07), (-0.20, 0.0, 0.08), material=dark_wood, name="rear_stretcher")
    add_box(frame, (0.055, 0.055, 1.72), (0.0, -0.28, 0.86), material=wood, name="side_post_0")
    add_box(frame, (0.055, 0.055, 1.72), (0.0, 0.28, 0.86), material=wood, name="side_post_1")
    add_box(frame, (0.055, 0.62, 0.09), (0.0, 0.0, 0.18), material=dark_wood, name="lower_crossbar")
    add_box(frame, (0.055, 0.62, 0.09), (0.0, 0.0, 1.60), material=dark_wood, name="upper_crossbar")
    add_box(frame, (0.09, 0.08, 1.78), (0.0, 0.0, 0.89), material=dark_wood, name="mast")

    upper_clamp = model.part("upper_clamp")
    add_box(upper_clamp, (0.08, 0.03, 0.18), (0.086, -0.0565, 0.0), material=steel, name="slider_side_0")
    add_box(upper_clamp, (0.08, 0.03, 0.18), (0.086, 0.0565, 0.0), material=steel, name="slider_side_1")
    add_box(upper_clamp, (0.091, 0.03, 0.14), (0.0005, -0.0565, 0.0), material=steel, name="guide_link_0")
    add_box(upper_clamp, (0.091, 0.03, 0.14), (0.0005, 0.0565, 0.0), material=steel, name="guide_link_1")
    add_box(upper_clamp, (0.045, 0.143, 0.03), (0.104, 0.0, 0.075), material=steel, name="slider_top")
    add_box(upper_clamp, (0.045, 0.143, 0.03), (0.104, 0.0, -0.075), material=steel, name="slider_bottom")
    add_box(upper_clamp, (0.032, 0.143, 0.14), (-0.061, 0.0, 0.0), material=steel, name="guide_back")
    add_box(upper_clamp, (0.05, 0.42, 0.05), (0.150, 0.0, 0.0), material=blackened_steel, name="clamp_pad")
    add_cylinder(
        upper_clamp,
        radius=0.010,
        length=0.08,
        xyz=(0.190, 0.0, 0.0),
        rpy=(0.0, pi / 2.0, 0.0),
        material=blackened_steel,
        name="clamp_shaft",
    )
    add_cylinder(
        upper_clamp,
        radius=0.028,
        length=0.018,
        xyz=(0.239, 0.0, 0.0),
        rpy=(0.0, pi / 2.0, 0.0),
        material=blackened_steel,
        name="clamp_knob",
    )

    lower_carriage = model.part("lower_carriage")
    add_box(lower_carriage, (0.08, 0.03, 0.14), (0.086, -0.0565, 0.0), material=steel, name="carriage_side_0")
    add_box(lower_carriage, (0.08, 0.03, 0.14), (0.086, 0.0565, 0.0), material=steel, name="carriage_side_1")
    add_box(lower_carriage, (0.091, 0.03, 0.10), (0.0005, -0.0565, 0.0), material=steel, name="guide_link_0")
    add_box(lower_carriage, (0.091, 0.03, 0.10), (0.0005, 0.0565, 0.0), material=steel, name="guide_link_1")
    add_box(lower_carriage, (0.04, 0.143, 0.03), (0.104, 0.0, 0.055), material=steel, name="carriage_top")
    add_box(lower_carriage, (0.04, 0.143, 0.03), (0.104, 0.0, -0.055), material=steel, name="carriage_bottom")
    add_box(lower_carriage, (0.032, 0.143, 0.12), (-0.061, 0.0, 0.0), material=steel, name="guide_back")
    add_box(lower_carriage, (0.046, 0.34, 0.05), (0.123, 0.0, -0.01), material=blackened_steel, name="carriage_rail")
    add_box(lower_carriage, (0.05, 0.12, 0.04), (0.125, -0.15, -0.015), material=blackened_steel, name="wing_0")
    add_box(lower_carriage, (0.05, 0.12, 0.04), (0.125, 0.15, -0.015), material=blackened_steel, name="wing_1")
    add_box(lower_carriage, (0.036, 0.09, 0.06), (0.135, -0.105, 0.02), material=steel, name="hinge_cheek_0")
    add_box(lower_carriage, (0.036, 0.09, 0.06), (0.135, 0.105, 0.02), material=steel, name="hinge_cheek_1")

    tray = model.part("tray")
    add_cylinder(
        tray,
        radius=0.012,
        length=0.34,
        xyz=(0.0, 0.0, 0.0),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=steel,
        name="hinge_barrel",
    )
    add_box(tray, (0.34, 0.56, 0.022), (0.17, 0.0, -0.023), material=wood, name="tray_board")
    add_box(tray, (0.02, 0.56, 0.06), (0.33, 0.0, 0.018), material=dark_wood, name="front_lip")
    add_box(tray, (0.24, 0.02, 0.035), (0.20, -0.27, 0.0055), material=dark_wood, name="side_lip_0")
    add_box(tray, (0.24, 0.02, 0.035), (0.20, 0.27, 0.0055), material=dark_wood, name="side_lip_1")

    model.articulation(
        "mast_to_upper_clamp",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=upper_clamp,
        origin=Origin(xyz=(0.0, 0.0, 1.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.30,
            lower=-0.34,
            upper=0.34,
        ),
    )
    model.articulation(
        "frame_to_lower_carriage",
        ArticulationType.FIXED,
        parent=frame,
        child=lower_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
    )
    model.articulation(
        "carriage_to_tray",
        ArticulationType.REVOLUTE,
        parent=lower_carriage,
        child=tray,
        origin=Origin(xyz=(0.165, 0.0, 0.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.4,
            lower=-0.12,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    upper_clamp = object_model.get_part("upper_clamp")
    lower_carriage = object_model.get_part("lower_carriage")
    tray = object_model.get_part("tray")

    upper_slide = object_model.get_articulation("mast_to_upper_clamp")
    tray_hinge = object_model.get_articulation("carriage_to_tray")

    ctx.expect_origin_distance(
        upper_clamp,
        frame,
        axes="xy",
        max_dist=0.001,
        name="upper clamp is centered on the mast",
    )
    ctx.expect_gap(
        tray,
        lower_carriage,
        axis="x",
        positive_elem="tray_board",
        negative_elem="carriage_rail",
        min_gap=0.012,
        max_gap=0.030,
        name="tray board sits just ahead of the lower carriage",
    )
    ctx.expect_overlap(
        tray,
        lower_carriage,
        axes="y",
        elem_a="tray_board",
        elem_b="carriage_rail",
        min_overlap=0.30,
        name="tray stays centered on the lower carriage",
    )

    upper_limit = upper_slide.motion_limits.upper if upper_slide.motion_limits is not None else None
    rest_clamp_pos = ctx.part_world_position(upper_clamp)
    with ctx.pose({upper_slide: upper_limit if upper_limit is not None else 0.0}):
        raised_clamp_pos = ctx.part_world_position(upper_clamp)

    ctx.check(
        "upper clamp slides upward along the mast",
        rest_clamp_pos is not None
        and raised_clamp_pos is not None
        and raised_clamp_pos[2] > rest_clamp_pos[2] + 0.25
        and abs(raised_clamp_pos[0] - rest_clamp_pos[0]) < 1e-5
        and abs(raised_clamp_pos[1] - rest_clamp_pos[1]) < 1e-5,
        details=f"rest={rest_clamp_pos}, raised={raised_clamp_pos}",
    )

    tray_limit = tray_hinge.motion_limits.upper if tray_hinge.motion_limits is not None else None
    rest_lip_aabb = ctx.part_element_world_aabb(tray, elem="front_lip")
    with ctx.pose({tray_hinge: tray_limit if tray_limit is not None else 0.0}):
        stored_lip_aabb = ctx.part_element_world_aabb(tray, elem="front_lip")
        ctx.expect_gap(
            tray,
            frame,
            axis="x",
            positive_elem="tray_board",
            negative_elem="mast",
            min_gap=0.025,
            name="stored tray stays in front of the mast",
        )

    ctx.check(
        "tray folds down for storage",
        rest_lip_aabb is not None
        and stored_lip_aabb is not None
        and stored_lip_aabb[0][2] < rest_lip_aabb[0][2] - 0.20,
        details=f"rest={rest_lip_aabb}, stored={stored_lip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

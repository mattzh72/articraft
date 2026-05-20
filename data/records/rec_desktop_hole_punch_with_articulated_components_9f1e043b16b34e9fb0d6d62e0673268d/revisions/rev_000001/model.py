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
    model = ArticulatedObject(name="desk_hole_punch")

    steel = model.material("steel", rgba=(0.25, 0.27, 0.30, 1.0))
    charcoal = model.material("charcoal", rgba=(0.11, 0.12, 0.13, 1.0))
    black = model.material("black", rgba=(0.06, 0.06, 0.07, 1.0))
    red = model.material("red", rgba=(0.70, 0.16, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.205, 0.118, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=steel,
        name="deck",
    )
    base.visual(
        Box((0.194, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, -0.053, 0.011)),
        material=charcoal,
        name="left_skirt",
    )
    base.visual(
        Box((0.014, 0.118, 0.022)),
        origin=Origin(xyz=(-0.0955, 0.0, 0.011)),
        material=charcoal,
        name="rear_skirt",
    )
    base.visual(
        Box((0.014, 0.098, 0.007)),
        origin=Origin(xyz=(0.0955, 0.0, 0.0185)),
        material=charcoal,
        name="front_slot_upper",
    )
    base.visual(
        Box((0.014, 0.098, 0.006)),
        origin=Origin(xyz=(0.0955, 0.0, 0.004)),
        material=charcoal,
        name="front_slot_lower",
    )
    base.visual(
        Box((0.014, 0.010, 0.020)),
        origin=Origin(xyz=(0.0955, -0.054, 0.011)),
        material=charcoal,
        name="front_cheek_0",
    )
    base.visual(
        Box((0.014, 0.010, 0.020)),
        origin=Origin(xyz=(0.0955, 0.054, 0.011)),
        material=charcoal,
        name="front_cheek_1",
    )
    base.visual(
        Box((0.050, 0.008, 0.004)),
        origin=Origin(xyz=(-0.010, 0.055, 0.020)),
        material=charcoal,
        name="button_track_upper",
    )
    base.visual(
        Box((0.050, 0.008, 0.004)),
        origin=Origin(xyz=(-0.010, 0.055, 0.010)),
        material=charcoal,
        name="button_track_lower",
    )
    base.visual(
        Box((0.010, 0.008, 0.014)),
        origin=Origin(xyz=(-0.030, 0.055, 0.015)),
        material=charcoal,
        name="button_track_post",
    )
    base.visual(
        Box((0.040, 0.010, 0.010)),
        origin=Origin(xyz=(-0.078, 0.054, 0.005)),
        material=charcoal,
        name="rear_foot",
    )
    base.visual(
        Box((0.016, 0.012, 0.024)),
        origin=Origin(xyz=(-0.094, -0.041, 0.012)),
        material=charcoal,
        name="hinge_pillar_0",
    )
    base.visual(
        Box((0.016, 0.012, 0.024)),
        origin=Origin(xyz=(-0.094, 0.041, 0.012)),
        material=charcoal,
        name="hinge_pillar_1",
    )
    base.visual(
        Box((0.012, 0.012, 0.010)),
        origin=Origin(xyz=(-0.094, -0.041, 0.024)),
        material=steel,
        name="hinge_pad_0",
    )
    base.visual(
        Box((0.012, 0.012, 0.010)),
        origin=Origin(xyz=(-0.094, 0.041, 0.024)),
        material=steel,
        name="hinge_pad_1",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.005, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="hinge_barrel",
    )
    handle.visual(
        Box((0.192, 0.108, 0.010)),
        origin=Origin(xyz=(0.096, 0.0, 0.004)),
        material=black,
        name="handle_plate",
    )
    handle.visual(
        Box((0.016, 0.012, 0.016)),
        origin=Origin(xyz=(0.010, -0.034, -0.002)),
        material=black,
        name="hinge_cheek_0",
    )
    handle.visual(
        Box((0.016, 0.012, 0.016)),
        origin=Origin(xyz=(0.010, 0.034, -0.002)),
        material=black,
        name="hinge_cheek_1",
    )
    handle.visual(
        Box((0.018, 0.020, 0.006)),
        origin=Origin(xyz=(0.118, -0.024, 0.001)),
        material=black,
        name="punch_head_0",
    )
    handle.visual(
        Box((0.018, 0.020, 0.006)),
        origin=Origin(xyz=(0.118, 0.024, 0.001)),
        material=black,
        name="punch_head_1",
    )

    button = model.part("button")
    button.visual(
        Box((0.014, 0.007, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=red,
        name="button_cap",
    )
    button.visual(
        Box((0.008, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, -0.0115, 0.0)),
        material=black,
        name="button_plunger",
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.120, 0.076, 0.002)),
        origin=Origin(xyz=(0.0, -0.038, 0.001)),
        material=black,
        name="tray_floor",
    )
    tray.visual(
        Box((0.004, 0.076, 0.008)),
        origin=Origin(xyz=(-0.058, -0.038, 0.005)),
        material=charcoal,
        name="tray_back",
    )
    tray.visual(
        Box((0.004, 0.076, 0.008)),
        origin=Origin(xyz=(0.058, -0.038, 0.005)),
        material=charcoal,
        name="tray_front",
    )
    tray.visual(
        Box((0.120, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.074, 0.005)),
        material=charcoal,
        name="tray_inner_wall",
    )
    tray.visual(
        Box((0.040, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.002, 0.005)),
        material=red,
        name="tray_pull",
    )

    guide = model.part("guide")
    guide.visual(
        Box((0.002, 0.022, 0.008)),
        origin=Origin(xyz=(-0.001, 0.011, 0.0)),
        material=steel,
        name="guide_pad",
    )
    guide.visual(
        Box((0.010, 0.014, 0.012)),
        origin=Origin(xyz=(0.005, 0.011, 0.0)),
        material=steel,
        name="guide_tab",
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(-0.094, 0.0, 0.034)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=3.0, lower=0.0, upper=0.95),
    )
    model.articulation(
        "base_to_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=button,
        origin=Origin(xyz=(-0.012, 0.0625, 0.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.006),
    )
    model.articulation(
        "base_to_tray",
        ArticulationType.PRISMATIC,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.012, 0.059, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.10, lower=0.0, upper=0.030),
    )
    model.articulation(
        "base_to_guide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=guide,
        origin=Origin(xyz=(0.1025, -0.038, 0.011)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.06, lower=0.0, upper=0.056),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    button = object_model.get_part("button")
    tray = object_model.get_part("tray")
    guide = object_model.get_part("guide")

    handle_hinge = object_model.get_articulation("base_to_handle")
    button_slide = object_model.get_articulation("base_to_button")
    tray_slide = object_model.get_articulation("base_to_tray")
    guide_slide = object_model.get_articulation("base_to_guide")

    ctx.expect_gap(
        handle,
        base,
        axis="z",
        positive_elem="handle_plate",
        max_gap=0.020,
        max_penetration=0.0,
        name="handle rests just above the base deck",
    )
    ctx.expect_overlap(
        handle,
        base,
        axes="xy",
        elem_a="handle_plate",
        elem_b="deck",
        min_overlap=0.080,
        name="handle covers the punch body footprint",
    )
    ctx.expect_within(
        tray,
        base,
        axes="xz",
        margin=0.004,
        name="tray stays guided under the punch body at rest",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_plate")
    closed_button_pos = ctx.part_world_position(button)
    closed_tray_pos = ctx.part_world_position(tray)
    closed_guide_pos = ctx.part_world_position(guide)

    with ctx.pose({handle_hinge: 0.95}):
        open_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_plate")
        ctx.check(
            "handle lifts upward when opened",
            closed_handle_aabb is not None
            and open_handle_aabb is not None
            and open_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.045,
            details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
        )

    with ctx.pose({button_slide: 0.006}):
        pressed_button_pos = ctx.part_world_position(button)
        ctx.check(
            "button presses inward",
            closed_button_pos is not None
            and pressed_button_pos is not None
            and pressed_button_pos[1] < closed_button_pos[1] - 0.004,
            details=f"closed={closed_button_pos}, pressed={pressed_button_pos}",
        )

    with ctx.pose({tray_slide: 0.030}):
        open_tray_pos = ctx.part_world_position(tray)
        ctx.expect_within(
            tray,
            base,
            axes="xz",
            margin=0.004,
            name="tray stays captured under the punch body when extended",
        )
        ctx.check(
            "tray slides out to the side",
            closed_tray_pos is not None
            and open_tray_pos is not None
            and open_tray_pos[1] > closed_tray_pos[1] + 0.025,
            details=f"closed={closed_tray_pos}, open={open_tray_pos}",
        )

    with ctx.pose({guide_slide: 0.056}):
        shifted_guide_pos = ctx.part_world_position(guide)
        ctx.check(
            "paper guide slides along the front edge",
            closed_guide_pos is not None
            and shifted_guide_pos is not None
            and shifted_guide_pos[1] > closed_guide_pos[1] + 0.045,
            details=f"closed={closed_guide_pos}, shifted={shifted_guide_pos}",
        )

    return ctx.report()


object_model = build_object_model()

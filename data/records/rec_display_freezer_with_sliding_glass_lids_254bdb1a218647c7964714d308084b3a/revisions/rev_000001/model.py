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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_display_freezer")

    enamel_blue = model.material("blue_enamel", rgba=(0.05, 0.18, 0.42, 1.0))
    white_liner = model.material("white_liner", rgba=(0.92, 0.96, 0.98, 1.0))
    gray_plastic = model.material("gray_plastic", rgba=(0.42, 0.45, 0.47, 1.0))
    dark_rubber = model.material("black_rubber", rgba=(0.02, 0.025, 0.03, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.68, 0.70, 0.70, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.50, 0.78, 0.92, 0.38))

    body = model.part("body")

    # Convenience-store scale chest: about 1.45 m wide, 0.78 m deep, 0.86 m tall.
    body.visual(
        Box((1.45, 0.78, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=enamel_blue,
        name="bottom_plinth",
    )
    body.visual(
        Box((1.45, 0.06, 0.72)),
        origin=Origin(xyz=(0.0, -0.36, 0.44)),
        material=enamel_blue,
        name="front_wall",
    )
    body.visual(
        Box((1.45, 0.06, 0.72)),
        origin=Origin(xyz=(0.0, 0.36, 0.44)),
        material=enamel_blue,
        name="rear_wall",
    )
    body.visual(
        Box((0.06, 0.78, 0.72)),
        origin=Origin(xyz=(-0.695, 0.0, 0.44)),
        material=enamel_blue,
        name="side_wall_neg",
    )
    body.visual(
        Box((0.06, 0.78, 0.72)),
        origin=Origin(xyz=(0.695, 0.0, 0.44)),
        material=enamel_blue,
        name="side_wall_pos",
    )

    # White liner panels make the tub read as a deep open insulated cavity.
    body.visual(
        Box((1.32, 0.66, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0925)),
        material=white_liner,
        name="inner_floor",
    )
    body.visual(
        Box((1.32, 0.010, 0.62)),
        origin=Origin(xyz=(0.0, -0.326, 0.415)),
        material=white_liner,
        name="front_liner",
    )
    body.visual(
        Box((1.32, 0.010, 0.62)),
        origin=Origin(xyz=(0.0, 0.326, 0.415)),
        material=white_liner,
        name="rear_liner",
    )
    body.visual(
        Box((0.010, 0.60, 0.62)),
        origin=Origin(xyz=(-0.656, 0.0, 0.415)),
        material=white_liner,
        name="side_liner_neg",
    )
    body.visual(
        Box((0.010, 0.60, 0.62)),
        origin=Origin(xyz=(0.656, 0.0, 0.415)),
        material=white_liner,
        name="side_liner_pos",
    )

    # Heavy top rim and sliding tracks; the lid panels ride just above these rails.
    body.visual(
        Box((1.53, 0.085, 0.06)),
        origin=Origin(xyz=(0.0, -0.3775, 0.83)),
        material=gray_plastic,
        name="front_top_rim",
    )
    body.visual(
        Box((1.53, 0.085, 0.06)),
        origin=Origin(xyz=(0.0, 0.3775, 0.83)),
        material=gray_plastic,
        name="rear_top_rim",
    )
    body.visual(
        Box((0.085, 0.78, 0.06)),
        origin=Origin(xyz=(-0.7225, 0.0, 0.83)),
        material=gray_plastic,
        name="side_top_rim_neg",
    )
    body.visual(
        Box((0.085, 0.78, 0.06)),
        origin=Origin(xyz=(0.7225, 0.0, 0.83)),
        material=gray_plastic,
        name="side_top_rim_pos",
    )
    body.visual(
        Box((1.42, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, -0.325, 0.872)),
        material=gray_plastic,
        name="front_outer_track",
    )
    body.visual(
        Box((1.42, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, -0.035, 0.872)),
        material=gray_plastic,
        name="front_inner_track",
    )
    body.visual(
        Box((1.42, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, 0.035, 0.872)),
        material=gray_plastic,
        name="rear_inner_track",
    )
    body.visual(
        Box((1.42, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, 0.325, 0.872)),
        material=gray_plastic,
        name="rear_outer_track",
    )

    # Small service fittings in the shell: lower drain and side key cylinder.
    body.visual(
        Cylinder(radius=0.036, length=0.008),
        origin=Origin(xyz=(0.49, -0.394, 0.25), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="drain_port",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.010),
        origin=Origin(xyz=(0.730, -0.13, 0.55), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_metal,
        name="key_cylinder",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.736, -0.13, 0.55), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_rubber,
        name="key_slot",
    )

    front_slider = model.part("front_slider")
    _add_glass_slider(
        front_slider,
        frame_material=gray_plastic,
        glass_material=glass,
        handle_material=dark_rubber,
        handle_x=0.23,
        handle_y=-0.160,
    )

    rear_slider = model.part("rear_slider")
    _add_glass_slider(
        rear_slider,
        frame_material=gray_plastic,
        glass_material=glass,
        handle_material=dark_rubber,
        handle_x=-0.23,
        handle_y=0.160,
    )

    drain_flap = model.part("drain_flap")
    drain_flap.visual(
        Box((0.120, 0.012, 0.090)),
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        material=gray_plastic,
        name="drain_plate",
    )
    drain_flap.visual(
        Cylinder(radius=0.010, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_metal,
        name="drain_hinge",
    )
    drain_flap.visual(
        Box((0.038, 0.006, 0.014)),
        origin=Origin(xyz=(-0.060, -0.008, 0.030)),
        material=dark_rubber,
        name="drain_pull_lip",
    )

    lock_flap = model.part("lock_flap")
    lock_flap.visual(
        Box((0.014, 0.130, 0.110)),
        origin=Origin(xyz=(0.008, 0.065, 0.0)),
        material=gray_plastic,
        name="lock_plate",
    )
    lock_flap.visual(
        Cylinder(radius=0.009, length=0.122),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_metal,
        name="lock_hinge",
    )
    lock_flap.visual(
        Box((0.005, 0.030, 0.018)),
        origin=Origin(xyz=(0.0175, 0.098, -0.035)),
        material=dark_rubber,
        name="lock_pull_tab",
    )

    model.articulation(
        "body_to_front_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_slider,
        origin=Origin(xyz=(-0.22, -0.18, 0.899)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.42),
    )
    model.articulation(
        "body_to_rear_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rear_slider,
        origin=Origin(xyz=(0.22, 0.18, 0.899)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.42),
    )
    model.articulation(
        "body_to_drain_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=drain_flap,
        origin=Origin(xyz=(0.55, -0.400, 0.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "body_to_lock_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lock_flap,
        origin=Origin(xyz=(0.742, -0.20, 0.55)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.55),
    )
    return model


def _add_glass_slider(part, *, frame_material, glass_material, handle_material, handle_x, handle_y) -> None:
    """Add a framed, transparent top slider centered on the part frame."""
    # The glass sheet tucks slightly under the frame bars so the part reads as one
    # assembled lid rather than four loose strips around a pane.
    part.visual(
        Box((0.810, 0.300, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=glass_material,
        name="glass_pane",
    )
    part.visual(
        Box((0.860, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, -0.160, 0.000)),
        material=frame_material,
        name="front_frame_bar",
    )
    part.visual(
        Box((0.860, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.160, 0.000)),
        material=frame_material,
        name="rear_frame_bar",
    )
    part.visual(
        Box((0.030, 0.350, 0.030)),
        origin=Origin(xyz=(-0.415, 0.0, 0.000)),
        material=frame_material,
        name="end_frame_bar_neg",
    )
    part.visual(
        Box((0.030, 0.350, 0.030)),
        origin=Origin(xyz=(0.415, 0.0, 0.000)),
        material=frame_material,
        name="end_frame_bar_pos",
    )
    part.visual(
        Box((0.110, 0.022, 0.018)),
        origin=Origin(xyz=(handle_x, handle_y, 0.024)),
        material=handle_material,
        name="finger_pull",
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_slider = object_model.get_part("front_slider")
    rear_slider = object_model.get_part("rear_slider")
    drain_flap = object_model.get_part("drain_flap")
    lock_flap = object_model.get_part("lock_flap")

    front_slide = object_model.get_articulation("body_to_front_slider")
    rear_slide = object_model.get_articulation("body_to_rear_slider")
    drain_hinge = object_model.get_articulation("body_to_drain_flap")
    lock_hinge = object_model.get_articulation("body_to_lock_flap")

    ctx.expect_overlap(
        front_slider,
        body,
        axes="x",
        elem_a="front_frame_bar",
        elem_b="front_outer_track",
        min_overlap=0.55,
        name="front glass slider rides on its front rail",
    )
    ctx.expect_overlap(
        rear_slider,
        body,
        axes="x",
        elem_a="rear_frame_bar",
        elem_b="rear_outer_track",
        min_overlap=0.55,
        name="rear glass slider rides on its rear rail",
    )
    ctx.expect_overlap(
        drain_flap,
        body,
        axes="xz",
        elem_a="drain_plate",
        elem_b="drain_port",
        min_overlap=0.030,
        name="drain cover actually covers the drain port",
    )
    ctx.expect_overlap(
        lock_flap,
        body,
        axes="yz",
        elem_a="lock_plate",
        elem_b="key_cylinder",
        min_overlap=0.030,
        name="side lock flap covers the key cylinder",
    )
    ctx.expect_gap(
        body,
        drain_flap,
        axis="y",
        positive_elem="front_wall",
        negative_elem="drain_plate",
        min_gap=0.004,
        max_gap=0.020,
        name="drain flap sits proud of the lower wall",
    )
    ctx.expect_gap(
        lock_flap,
        body,
        axis="x",
        positive_elem="lock_plate",
        negative_elem="side_wall_pos",
        min_gap=0.006,
        max_gap=0.030,
        name="lock flap sits proud of the side wall",
    )

    front_rest = ctx.part_world_position(front_slider)
    rear_rest = ctx.part_world_position(rear_slider)
    with ctx.pose({front_slide: 0.32, rear_slide: 0.32}):
        front_moved = ctx.part_world_position(front_slider)
        rear_moved = ctx.part_world_position(rear_slider)
    ctx.check(
        "glass sliders translate in opposite tracks",
        front_rest is not None
        and rear_rest is not None
        and front_moved is not None
        and rear_moved is not None
        and front_moved[0] > front_rest[0] + 0.25
        and rear_moved[0] < rear_rest[0] - 0.25,
        details=f"front_rest={front_rest}, front_moved={front_moved}, rear_rest={rear_rest}, rear_moved={rear_moved}",
    )

    drain_closed = ctx.part_world_aabb(drain_flap)
    with ctx.pose({drain_hinge: 1.1}):
        drain_open = ctx.part_world_aabb(drain_flap)
    ctx.check(
        "drain cover flap swings outward from the front wall",
        drain_closed is not None
        and drain_open is not None
        and drain_open[0][1] < drain_closed[0][1] - 0.04,
        details=f"closed={drain_closed}, open={drain_open}",
    )

    lock_closed = ctx.part_world_aabb(lock_flap)
    with ctx.pose({lock_hinge: 1.1}):
        lock_open = ctx.part_world_aabb(lock_flap)
    ctx.check(
        "side lock flap swings outward from the side wall",
        lock_closed is not None
        and lock_open is not None
        and lock_open[1][0] > lock_closed[1][0] + 0.04,
        details=f"closed={lock_closed}, open={lock_open}",
    )

    return ctx.report()


object_model = build_object_model()

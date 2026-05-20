from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_h_frame_easel")

    aluminium = model.material("brushed_aluminium", rgba=(0.76, 0.78, 0.76, 1.0))
    dark_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    dark_plastic = model.material("matte_black_plastic", rgba=(0.015, 0.015, 0.017, 1.0))
    tray_grey = model.material("anodized_grey", rgba=(0.30, 0.32, 0.34, 1.0))

    frame = model.part("frame")
    # Tall twin H-frame uprights and rigid aluminium cross members.
    frame.visual(
        Box((0.05, 0.05, 2.155)),
        origin=Origin(xyz=(-0.36, 0.0, 1.1225)),
        material=aluminium,
        name="left_upright",
    )
    frame.visual(
        Box((0.16, 0.68, 0.045)),
        origin=Origin(xyz=(-0.36, -0.02, 0.0225)),
        material=aluminium,
        name="left_upright_foot",
    )
    frame.visual(
        Box((0.05, 0.05, 2.155)),
        origin=Origin(xyz=(0.36, 0.0, 1.1225)),
        material=aluminium,
        name="right_upright",
    )
    frame.visual(
        Box((0.16, 0.68, 0.045)),
        origin=Origin(xyz=(0.36, -0.02, 0.0225)),
        material=aluminium,
        name="right_upright_foot",
    )

    frame.visual(
        Box((0.77, 0.05, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 2.18)),
        material=aluminium,
        name="top_crossbar",
    )
    frame.visual(
        Box((0.77, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=aluminium,
        name="lower_crossbar",
    )
    frame.visual(
        Box((0.77, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, 0.18, 0.0225)),
        material=aluminium,
        name="rear_floor_tie",
    )

    # Forked central hinge support for the tilt post, attached back to the uprights
    # with side arms so the rotating barrel has a clear center span.
    frame.visual(
        Box((0.04, 0.06, 0.12)),
        origin=Origin(xyz=(-0.08, 0.045, 1.12)),
        material=aluminium,
        name="left_yoke",
    )
    frame.visual(
        Box((0.04, 0.06, 0.12)),
        origin=Origin(xyz=(0.08, 0.045, 1.12)),
        material=aluminium,
        name="right_yoke",
    )
    frame.visual(
        Box((0.235, 0.035, 0.028)),
        origin=Origin(xyz=(-0.2175, 0.03, 1.12)),
        material=aluminium,
        name="left_yoke_arm",
    )
    frame.visual(
        Box((0.235, 0.035, 0.028)),
        origin=Origin(xyz=(0.2175, 0.03, 1.12)),
        material=aluminium,
        name="right_yoke_arm",
    )

    top_grip = model.part("top_grip_rail")
    top_grip.visual(
        Box((0.67, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, -0.055, 0.0)),
        material=aluminium,
        name="top_rail",
    )
    top_grip.visual(
        Box((0.085, 0.04, 0.12)),
        origin=Origin(xyz=(-0.36, -0.045, 0.0)),
        material=aluminium,
        name="left_slider",
    )
    top_grip.visual(
        Cylinder(radius=0.009, length=0.04),
        origin=Origin(xyz=(-0.36, -0.085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="left_slider_screw",
    )
    top_grip.visual(
        Cylinder(radius=0.028, length=0.025),
        origin=Origin(xyz=(-0.36, -0.1125, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="left_slider_knob",
    )
    top_grip.visual(
        Box((0.085, 0.04, 0.12)),
        origin=Origin(xyz=(0.36, -0.045, 0.0)),
        material=aluminium,
        name="right_slider",
    )
    top_grip.visual(
        Cylinder(radius=0.009, length=0.04),
        origin=Origin(xyz=(0.36, -0.085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="right_slider_screw",
    )
    top_grip.visual(
        Cylinder(radius=0.028, length=0.025),
        origin=Origin(xyz=(0.36, -0.1125, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="right_slider_knob",
    )
    top_grip.visual(
        Box((0.58, 0.018, 0.075)),
        origin=Origin(xyz=(0.0, -0.083, 0.02)),
        material=aluminium,
        name="grip_face",
    )
    top_grip.visual(
        Box((0.50, 0.012, 0.025)),
        origin=Origin(xyz=(0.0, -0.098, -0.01)),
        material=dark_rubber,
        name="rubber_grip_pad",
    )

    ledge = model.part("ledge_rail")
    ledge.visual(
        Box((0.67, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, -0.055, 0.0)),
        material=aluminium,
        name="ledge_back_rail",
    )
    ledge.visual(
        Box((0.62, 0.12, 0.025)),
        origin=Origin(xyz=(0.0, -0.105, -0.0375)),
        material=tray_grey,
        name="canvas_shelf",
    )
    ledge.visual(
        Box((0.62, 0.025, 0.075)),
        origin=Origin(xyz=(0.0, -0.165, 0.0)),
        material=tray_grey,
        name="front_lip",
    )
    ledge.visual(
        Box((0.58, 0.08, 0.006)),
        origin=Origin(xyz=(0.0, -0.105, -0.022)),
        material=dark_rubber,
        name="shelf_pad",
    )
    ledge.visual(
        Box((0.085, 0.04, 0.13)),
        origin=Origin(xyz=(-0.36, -0.045, 0.0)),
        material=aluminium,
        name="left_slider",
    )
    ledge.visual(
        Cylinder(radius=0.009, length=0.04),
        origin=Origin(xyz=(-0.36, -0.085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="left_slider_screw",
    )
    ledge.visual(
        Cylinder(radius=0.028, length=0.025),
        origin=Origin(xyz=(-0.36, -0.1125, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="left_slider_knob",
    )
    ledge.visual(
        Box((0.085, 0.04, 0.13)),
        origin=Origin(xyz=(0.36, -0.045, 0.0)),
        material=aluminium,
        name="right_slider",
    )
    ledge.visual(
        Cylinder(radius=0.009, length=0.04),
        origin=Origin(xyz=(0.36, -0.085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="right_slider_screw",
    )
    ledge.visual(
        Cylinder(radius=0.028, length=0.025),
        origin=Origin(xyz=(0.36, -0.1125, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="right_slider_knob",
    )

    tilt_post = model.part("tilt_post")
    tilt_post.visual(
        Cylinder(radius=0.026, length=0.12),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminium,
        name="tilt_barrel",
    )
    tilt_post.visual(
        Box((0.045, 0.035, 1.60)),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=aluminium,
        name="tilt_post_bar",
    )
    tilt_post.visual(
        Box((0.16, 0.025, 0.05)),
        origin=Origin(xyz=(0.0, -0.03, 1.12)),
        material=dark_rubber,
        name="top_canvas_stop",
    )

    model.articulation(
        "frame_to_top_grip",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=top_grip,
        origin=Origin(xyz=(0.0, 0.0, 1.64)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=-0.30, upper=0.44),
        motion_properties=MotionProperties(damping=1.0, friction=0.5),
    )
    model.articulation(
        "frame_to_ledge",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=ledge,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=110.0, velocity=0.18, lower=-0.27, upper=0.38),
        motion_properties=MotionProperties(damping=1.0, friction=0.6),
    )
    model.articulation(
        "frame_to_tilt_post",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=tilt_post,
        origin=Origin(xyz=(0.0, 0.045, 1.12)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.7, lower=-0.45, upper=0.45),
        motion_properties=MotionProperties(damping=0.4, friction=0.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    frame = object_model.get_part("frame")
    top_grip = object_model.get_part("top_grip_rail")
    ledge = object_model.get_part("ledge_rail")
    tilt_post = object_model.get_part("tilt_post")
    top_slide = object_model.get_articulation("frame_to_top_grip")
    ledge_slide = object_model.get_articulation("frame_to_ledge")
    tilt_hinge = object_model.get_articulation("frame_to_tilt_post")

    ctx.check(
        "primary mechanisms present",
        len(object_model.parts) == 4 and len(object_model.articulations) == 3,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )

    # Sliding clamp blocks sit against the front faces of the two uprights.
    for part_obj, prefix in [(top_grip, "top"), (ledge, "ledge")]:
        ctx.expect_gap(
            frame,
            part_obj,
            axis="y",
            positive_elem="left_upright",
            negative_elem="left_slider",
            max_gap=0.001,
            max_penetration=0.00001,
            name=f"{prefix} left slider rides on upright",
        )
        ctx.expect_gap(
            frame,
            part_obj,
            axis="y",
            positive_elem="right_upright",
            negative_elem="right_slider",
            max_gap=0.001,
            max_penetration=0.00001,
            name=f"{prefix} right slider rides on upright",
        )
        ctx.expect_overlap(
            frame,
            part_obj,
            axes="xz",
            elem_a="left_upright",
            elem_b="left_slider",
            min_overlap=0.04,
            name=f"{prefix} left slider wraps upright face",
        )
        ctx.expect_overlap(
            frame,
            part_obj,
            axes="xz",
            elem_a="right_upright",
            elem_b="right_slider",
            min_overlap=0.04,
            name=f"{prefix} right slider wraps upright face",
        )

    top_rest = ctx.part_world_position(top_grip)
    with ctx.pose({top_slide: top_slide.motion_limits.upper}):
        top_high = ctx.part_world_position(top_grip)
    ctx.check(
        "top grip rail slides upward",
        top_rest is not None and top_high is not None and top_high[2] > top_rest[2] + 0.35,
        details=f"rest={top_rest}, high={top_high}",
    )

    ledge_rest = ctx.part_world_position(ledge)
    with ctx.pose({ledge_slide: ledge_slide.motion_limits.lower}):
        ledge_low = ctx.part_world_position(ledge)
    ctx.check(
        "lower ledge rail slides downward",
        ledge_rest is not None and ledge_low is not None and ledge_low[2] < ledge_rest[2] - 0.20,
        details=f"rest={ledge_rest}, low={ledge_low}",
    )

    # The tilt barrel is captured between the forked yoke plates.
    ctx.expect_gap(
        frame,
        tilt_post,
        axis="x",
        positive_elem="right_yoke",
        negative_elem="tilt_barrel",
        max_gap=0.001,
        max_penetration=0.0,
        name="right yoke captures tilt barrel",
    )
    ctx.expect_gap(
        tilt_post,
        frame,
        axis="x",
        positive_elem="tilt_barrel",
        negative_elem="left_yoke",
        max_gap=0.001,
        max_penetration=0.0,
        name="left yoke captures tilt barrel",
    )
    rest_bar = ctx.part_element_world_aabb(tilt_post, elem="tilt_post_bar")
    with ctx.pose({tilt_hinge: tilt_hinge.motion_limits.upper}):
        tilted_bar = ctx.part_element_world_aabb(tilt_post, elem="tilt_post_bar")
    ctx.check(
        "central tilt post tips forward",
        rest_bar is not None and tilted_bar is not None and tilted_bar[0][1] < rest_bar[0][1] - 0.12,
        details=f"rest_aabb={rest_bar}, tilted_aabb={tilted_bar}",
    )

    return ctx.report()


object_model = build_object_model()

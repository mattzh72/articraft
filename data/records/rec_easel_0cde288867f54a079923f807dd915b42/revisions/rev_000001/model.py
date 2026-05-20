from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="presentation_a_frame_display_easel")

    beech = model.material("beech", rgba=(0.82, 0.72, 0.56, 1.0))
    warm_oak = model.material("warm_oak", rgba=(0.70, 0.58, 0.41, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))

    front_frame = model.part("front_frame")
    front_frame.visual(
        Box((0.048, 0.024, 1.68)),
        origin=Origin(xyz=(-0.156, 0.0, 0.84), rpy=(0.0, 0.185, 0.0)),
        material=beech,
        name="left_leg",
    )
    front_frame.visual(
        Box((0.048, 0.024, 1.68)),
        origin=Origin(xyz=(0.156, 0.0, 0.84), rpy=(0.0, -0.185, 0.0)),
        material=beech,
        name="right_leg",
    )
    front_frame.visual(
        Box((0.190, 0.050, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 1.645)),
        material=warm_oak,
        name="top_header",
    )
    front_frame.visual(
        Cylinder(radius=0.010, length=0.170),
        origin=Origin(xyz=(0.0, -0.022, 1.675), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    front_frame.visual(
        Box((0.520, 0.030, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        material=warm_oak,
        name="tray_support_bar",
    )
    front_frame.visual(
        Box((0.480, 0.082, 0.018)),
        origin=Origin(xyz=(0.0, 0.054, 0.600)),
        material=beech,
        name="display_tray",
    )
    front_frame.visual(
        Box((0.480, 0.012, 0.038)),
        origin=Origin(xyz=(0.0, 0.089, 0.618)),
        material=warm_oak,
        name="tray_lip",
    )
    front_frame.visual(
        Box((0.050, 0.028, 0.350)),
        origin=Origin(xyz=(0.0, -0.014, 0.805)),
        material=warm_oak,
        name="center_spine_lower",
    )
    front_frame.visual(
        Box((0.132, 0.032, 0.040)),
        origin=Origin(xyz=(0.0, -0.014, 0.980)),
        material=warm_oak,
        name="slot_bottom_bridge",
    )
    front_frame.visual(
        Box((0.024, 0.026, 0.360)),
        origin=Origin(xyz=(-0.060, -0.014, 1.180)),
        material=warm_oak,
        name="slot_left_rail",
    )
    front_frame.visual(
        Box((0.024, 0.026, 0.360)),
        origin=Origin(xyz=(0.060, -0.014, 1.180)),
        material=warm_oak,
        name="slot_right_rail",
    )
    front_frame.visual(
        Box((0.050, 0.028, 0.215)),
        origin=Origin(xyz=(0.0, -0.014, 1.470)),
        material=warm_oak,
        name="center_spine_upper",
    )
    front_frame.visual(
        Box((0.152, 0.030, 0.036)),
        origin=Origin(xyz=(0.0, -0.014, 1.375)),
        material=warm_oak,
        name="slot_top_bridge",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.700, 0.160, 1.720)),
        mass=4.4,
        origin=Origin(xyz=(0.0, 0.018, 0.860)),
    )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Box((0.054, 0.024, 1.820)),
        origin=Origin(xyz=(0.0, -0.377, -0.844), rpy=(-0.410, 0.0, 0.0)),
        material=beech,
        name="rear_stile",
    )
    rear_leg.visual(
        Box((0.100, 0.036, 0.080)),
        origin=Origin(xyz=(0.0, -0.028, -0.040)),
        material=graphite,
        name="top_bracket",
    )
    rear_leg.visual(
        Box((0.020, 0.018, 0.483)),
        origin=Origin(xyz=(0.0, -0.040, -0.279)),
        material=graphite,
        name="lock_strut",
    )
    rear_leg.visual(
        Box((0.076, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, -0.744, -1.668), rpy=(-0.410, 0.0, 0.0)),
        material=black_plastic,
        name="rear_foot",
    )
    rear_leg.inertial = Inertial.from_geometry(
        Box((0.120, 0.820, 1.860)),
        mass=1.5,
        origin=Origin(xyz=(0.0, -0.380, -0.850)),
    )

    slider_block = model.part("slider_block")
    slider_block.visual(
        Box((0.096, 0.024, 0.100)),
        origin=Origin(),
        material=graphite,
        name="carriage",
    )
    slider_block.visual(
        Box((0.032, 0.040, 0.042)),
        origin=Origin(xyz=(0.0, -0.028, 0.0)),
        material=graphite,
        name="rear_clevis",
    )
    slider_block.visual(
        Cylinder(radius=0.013, length=0.032),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="lock_knob",
    )
    slider_block.inertial = Inertial.from_geometry(
        Box((0.120, 0.080, 0.110)),
        mass=0.35,
        origin=Origin(),
    )

    model.articulation(
        "front_frame_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_leg,
        origin=Origin(xyz=(0.0, -0.022, 1.675)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=0.28,
        ),
    )
    model.articulation(
        "front_frame_to_slider_block",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=slider_block,
        origin=Origin(xyz=(0.0, -0.014, 1.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.12,
            lower=-0.08,
            upper=0.16,
        ),
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
    front_frame = object_model.get_part("front_frame")
    rear_leg = object_model.get_part("rear_leg")
    slider_block = object_model.get_part("slider_block")
    rear_hinge = object_model.get_articulation("front_frame_to_rear_leg")
    slider_joint = object_model.get_articulation("front_frame_to_slider_block")

    ctx.expect_contact(
        rear_leg,
        slider_block,
        elem_a="lock_strut",
        elem_b="rear_clevis",
        contact_tol=0.012,
        name="rear locking strut meets the sliding clevis",
    )

    with ctx.pose({slider_joint: slider_joint.motion_limits.lower}):
        ctx.expect_gap(
            slider_block,
            front_frame,
            axis="z",
            positive_elem="carriage",
            negative_elem="slot_bottom_bridge",
            max_penetration=0.0005,
            max_gap=0.030,
            name="slider clears the bottom slot stop",
        )

    with ctx.pose({slider_joint: slider_joint.motion_limits.upper}):
        ctx.expect_gap(
            front_frame,
            slider_block,
            axis="z",
            positive_elem="slot_top_bridge",
            negative_elem="carriage",
            min_gap=0.015,
            max_gap=0.060,
            name="slider clears the top slot stop",
        )

    slider_rest = ctx.part_world_position(slider_block)
    with ctx.pose({slider_joint: slider_joint.motion_limits.upper}):
        slider_extended = ctx.part_world_position(slider_block)
    ctx.check(
        "slider travels upward in the frame slot",
        slider_rest is not None
        and slider_extended is not None
        and slider_extended[2] > slider_rest[2] + 0.10,
        details=f"rest={slider_rest}, extended={slider_extended}",
    )

    rear_rest_aabb = ctx.part_world_aabb(rear_leg)
    with ctx.pose({rear_hinge: rear_hinge.motion_limits.upper}):
        rear_open_aabb = ctx.part_world_aabb(rear_leg)
    ctx.check(
        "rear leg swings farther behind the front frame when opened",
        rear_rest_aabb is not None
        and rear_open_aabb is not None
        and rear_open_aabb[0][1] < rear_rest_aabb[0][1] - 0.08,
        details=f"rest={rear_rest_aabb}, open={rear_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

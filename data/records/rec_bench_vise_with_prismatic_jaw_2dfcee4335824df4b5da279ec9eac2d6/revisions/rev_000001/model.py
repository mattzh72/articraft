from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="precision_milling_sine_vise")

    steel = model.material("ground_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_steel = model.material("jaw_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    base_steel = model.material("base_steel", rgba=(0.60, 0.63, 0.67, 1.0))

    body_length = 0.220
    body_width = 0.090
    body_thickness = 0.028
    front_bar_radius = 0.011
    bar_length = 0.105
    rear_bar_x = 0.165

    fixed_jaw_thickness = 0.016
    jaw_height = 0.048
    jaw_plate_thickness = 0.002

    moving_shoe_length = 0.032
    moving_shoe_width = 0.086
    moving_shoe_height = 0.014
    moving_jaw_thickness = 0.012

    base = model.part("sine_base")
    base.visual(
        Cylinder(radius=front_bar_radius, length=bar_length),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=base_steel,
        name="front_bar",
    )
    base.visual(
        Cylinder(radius=front_bar_radius, length=bar_length),
        origin=Origin(xyz=(rear_bar_x, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=base_steel,
        name="rear_bar",
    )
    base.visual(
        Box((0.176, 0.016, 0.014)),
        origin=Origin(xyz=(0.082, 0.036, -0.004)),
        material=base_steel,
        name="left_side_rail",
    )
    base.visual(
        Box((0.176, 0.016, 0.014)),
        origin=Origin(xyz=(0.082, -0.036, -0.004)),
        material=base_steel,
        name="right_side_rail",
    )
    base.visual(
        Box((0.148, 0.040, 0.006)),
        origin=Origin(xyz=(0.082, 0.0, -0.010)),
        material=base_steel,
        name="base_spine",
    )

    body = model.part("body_frame")
    body.visual(
        Box((body_length, body_width, body_thickness)),
        origin=Origin(xyz=(body_length / 2.0, 0.0, front_bar_radius + body_thickness / 2.0)),
        material=steel,
        name="bed",
    )
    body.visual(
        Box((0.022, body_width, 0.010)),
        origin=Origin(xyz=(0.011, 0.0, front_bar_radius + 0.005)),
        material=steel,
        name="front_lip",
    )
    body.visual(
        Box((fixed_jaw_thickness, body_width, jaw_height)),
        origin=Origin(
            xyz=(
                0.196,
                0.0,
                front_bar_radius + body_thickness + jaw_height / 2.0,
            )
        ),
        material=steel,
        name="fixed_jaw",
    )
    body.visual(
        Box((jaw_plate_thickness, 0.076, 0.028)),
        origin=Origin(
            xyz=(
                0.187,
                0.0,
                front_bar_radius + body_thickness + 0.018,
            )
        ),
        material=dark_steel,
        name="fixed_jaw_plate",
    )
    body.visual(
        Box((0.022, 0.068, 0.006)),
        origin=Origin(xyz=(rear_bar_x, 0.0, front_bar_radius + 0.003)),
        material=steel,
        name="rear_pad",
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((moving_shoe_length, moving_shoe_width, moving_shoe_height)),
        origin=Origin(xyz=(0.0, 0.0, moving_shoe_height / 2.0)),
        material=steel,
        name="jaw_slide",
    )
    moving_jaw.visual(
        Box((moving_jaw_thickness, 0.088, jaw_height)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                moving_shoe_height + jaw_height / 2.0,
            )
        ),
        material=steel,
        name="jaw_block",
    )
    moving_jaw.visual(
        Box((jaw_plate_thickness, 0.076, 0.028)),
        origin=Origin(
            xyz=(
                moving_jaw_thickness / 2.0 + jaw_plate_thickness / 2.0,
                0.0,
                moving_shoe_height + 0.018,
            )
        ),
        material=dark_steel,
        name="moving_jaw_plate",
    )

    model.articulation(
        "base_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.6,
            lower=0.0,
            upper=0.70,
        ),
    )
    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=moving_jaw,
        origin=Origin(xyz=(0.166, 0.0, front_bar_radius + body_thickness)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.08,
            lower=0.0,
            upper=0.080,
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

    base = object_model.get_part("sine_base")
    body = object_model.get_part("body_frame")
    moving_jaw = object_model.get_part("moving_jaw")
    tilt = object_model.get_articulation("base_tilt")
    slide = object_model.get_articulation("jaw_slide")

    ctx.check(
        "vise parts exist",
        base is not None and body is not None and moving_jaw is not None,
        details=f"parts={[part.name for part in object_model.parts]}",
    )

    with ctx.pose({tilt: 0.0, slide: 0.0}):
        ctx.expect_contact(
            body,
            base,
            elem_a="bed",
            elem_b="front_bar",
            name="body bears on front sine bar hinge line",
        )
        ctx.expect_contact(
            body,
            base,
            elem_a="rear_pad",
            elem_b="rear_bar",
            name="body rests on rear sine bar when level",
        )
        ctx.expect_contact(
            moving_jaw,
            body,
            elem_a="jaw_slide",
            elem_b="bed",
            name="moving jaw slide rides on body bed",
        )
        ctx.expect_gap(
            body,
            moving_jaw,
            axis="x",
            positive_elem="fixed_jaw_plate",
            negative_elem="moving_jaw_plate",
            min_gap=0.011,
            max_gap=0.016,
            name="closed vise leaves a narrow clamping gap",
        )
        ctx.expect_overlap(
            moving_jaw,
            body,
            axes="y",
            elem_a="jaw_slide",
            elem_b="bed",
            min_overlap=0.080,
            name="moving jaw stays captured across vise width",
        )

    with ctx.pose({tilt: 0.55, slide: 0.0}):
        ctx.expect_contact(
            body,
            base,
            elem_a="bed",
            elem_b="front_bar",
            name="tilted body still pivots on the front hinge bar",
        )
        ctx.expect_gap(
            body,
            base,
            axis="z",
            positive_elem="rear_pad",
            negative_elem="rear_bar",
            min_gap=0.060,
            name="rear support lifts clear when the sine base tilts",
        )

    rest_position = None
    open_position = None
    with ctx.pose({tilt: 0.0, slide: 0.0}):
        rest_position = ctx.part_world_position(moving_jaw)
    with ctx.pose({tilt: 0.0, slide: 0.080}):
        open_position = ctx.part_world_position(moving_jaw)
        ctx.expect_overlap(
            moving_jaw,
            body,
            axes="xy",
            elem_a="jaw_slide",
            elem_b="bed",
            min_overlap=0.030,
            name="extended jaw remains supported on the bed",
        )
        ctx.expect_gap(
            body,
            moving_jaw,
            axis="x",
            positive_elem="fixed_jaw_plate",
            negative_elem="moving_jaw_plate",
            min_gap=0.090,
            max_gap=0.105,
            name="extended jaw opens the vise throat",
        )
    ctx.check(
        "jaw opens toward the front of the vise",
        rest_position is not None
        and open_position is not None
        and open_position[0] < rest_position[0] - 0.050,
        details=f"rest={rest_position}, open={open_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

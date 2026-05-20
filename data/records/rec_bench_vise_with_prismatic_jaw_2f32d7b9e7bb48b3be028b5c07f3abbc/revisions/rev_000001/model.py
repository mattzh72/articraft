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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drill_press_vise")

    cast_blue = model.material("cast_blue", rgba=(0.18, 0.34, 0.62, 1.0))
    jaw_steel = model.material("jaw_steel", rgba=(0.42, 0.44, 0.46, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.28, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.260, 0.120, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=cast_blue,
        name="base_plate",
    )
    base.visual(
        Box((0.022, 0.182, 0.010)),
        origin=Origin(xyz=(-0.034, -0.015, 0.021)),
        material=dark_steel,
        name="left_rail",
    )
    base.visual(
        Box((0.022, 0.182, 0.010)),
        origin=Origin(xyz=(0.034, -0.015, 0.021)),
        material=dark_steel,
        name="right_rail",
    )
    base.visual(
        Box((0.022, 0.026, 0.016)),
        origin=Origin(xyz=(-0.034, 0.086, 0.024)),
        material=cast_blue,
        name="left_fixed_support",
    )
    base.visual(
        Box((0.022, 0.026, 0.016)),
        origin=Origin(xyz=(0.034, 0.086, 0.024)),
        material=cast_blue,
        name="right_fixed_support",
    )
    base.visual(
        Box((0.096, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, 0.086, 0.057)),
        material=cast_blue,
        name="fixed_jaw_body",
    )
    base.visual(
        Box((0.086, 0.004, 0.044)),
        origin=Origin(xyz=(0.0, 0.079, 0.057)),
        material=jaw_steel,
        name="fixed_jaw_face",
    )
    base.visual(
        Box((0.014, 0.032, 0.020)),
        origin=Origin(xyz=(-0.018, 0.111, 0.026)),
        material=cast_blue,
        name="left_bearing_ear",
    )
    base.visual(
        Box((0.014, 0.032, 0.020)),
        origin=Origin(xyz=(0.018, 0.111, 0.026)),
        material=cast_blue,
        name="right_bearing_ear",
    )
    base.visual(
        Box((0.050, 0.032, 0.006)),
        origin=Origin(xyz=(0.0, 0.111, 0.039)),
        material=cast_blue,
        name="bearing_cap",
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((0.018, 0.028, 0.010)),
        origin=Origin(xyz=(-0.034, -0.002, 0.015)),
        material=dark_steel,
        name="moving_left_runner",
    )
    moving_jaw.visual(
        Box((0.018, 0.028, 0.010)),
        origin=Origin(xyz=(0.034, -0.002, 0.015)),
        material=dark_steel,
        name="moving_right_runner",
    )
    moving_jaw.visual(
        Box((0.014, 0.034, 0.022)),
        origin=Origin(xyz=(-0.016, -0.002, 0.015)),
        material=cast_blue,
        name="left_carriage_block",
    )
    moving_jaw.visual(
        Box((0.014, 0.034, 0.022)),
        origin=Origin(xyz=(0.016, -0.002, 0.015)),
        material=cast_blue,
        name="right_carriage_block",
    )
    moving_jaw.visual(
        Box((0.046, 0.026, 0.008)),
        origin=Origin(xyz=(0.0, -0.002, 0.030)),
        material=cast_blue,
        name="top_bridge",
    )
    moving_jaw.visual(
        Box((0.094, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, 0.016, 0.041)),
        material=cast_blue,
        name="moving_jaw_body",
    )
    moving_jaw.visual(
        Box((0.086, 0.004, 0.044)),
        origin=Origin(xyz=(0.0, 0.023, 0.041)),
        material=jaw_steel,
        name="moving_jaw_face",
    )

    lead_screw_handle = model.part("lead_screw_handle")
    lead_screw_handle.visual(
        Cylinder(radius=0.006, length=0.160),
        origin=Origin(xyz=(0.0, 0.105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
        name="lead_screw",
    )
    lead_screw_handle.visual(
        Cylinder(radius=0.008, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handle_hub",
    )
    lead_screw_handle.visual(
        Cylinder(radius=0.004, length=0.132),
        origin=Origin(xyz=(0.0, -0.032, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_steel,
        name="t_handle_bar",
    )
    lead_screw_handle.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=(-0.066, -0.032, 0.0)),
        material=dark_steel,
        name="left_handle_knob",
    )
    lead_screw_handle.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=(0.066, -0.032, 0.0)),
        material=dark_steel,
        name="right_handle_knob",
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=moving_jaw,
        origin=Origin(xyz=(0.0, -0.060, 0.016)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.040,
            lower=0.0,
            upper=0.090,
        ),
    )
    model.articulation(
        "screw_spin",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=lead_screw_handle,
        origin=Origin(xyz=(0.0, -0.020, 0.009)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=10.0),
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

    base = object_model.get_part("base")
    moving_jaw = object_model.get_part("moving_jaw")
    screw = object_model.get_part("lead_screw_handle")
    jaw_slide = object_model.get_articulation("jaw_slide")
    screw_spin = object_model.get_articulation("screw_spin")

    ctx.check("base exists", base is not None)
    ctx.check("moving jaw exists", moving_jaw is not None)
    ctx.check("lead screw handle exists", screw is not None)

    ctx.check(
        "jaw slide uses prismatic motion",
        jaw_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={jaw_slide.articulation_type}",
    )
    ctx.check(
        "screw handle uses continuous rotation",
        screw_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={screw_spin.articulation_type}",
    )
    ctx.check(
        "screw spin axis follows vise length",
        screw_spin.axis == (0.0, 1.0, 0.0),
        details=f"axis={screw_spin.axis}",
    )
    limits = screw_spin.motion_limits
    ctx.check(
        "screw spin is unbounded",
        limits is not None and limits.lower is None and limits.upper is None,
        details=f"limits={limits}",
    )

    ctx.expect_overlap(
        moving_jaw,
        base,
        axes="xz",
        elem_a="moving_jaw_face",
        elem_b="fixed_jaw_face",
        min_overlap=0.040,
        name="jaw faces stay aligned",
    )
    ctx.expect_gap(
        base,
        moving_jaw,
        axis="y",
        positive_elem="fixed_jaw_face",
        negative_elem="moving_jaw_face",
        min_gap=0.100,
        max_gap=0.120,
        name="resting jaw opening is wide enough",
    )
    ctx.expect_within(
        moving_jaw,
        base,
        axes="xy",
        inner_elem="moving_left_runner",
        outer_elem="left_rail",
        margin=0.001,
        name="left runner stays on left rail at rest",
    )
    ctx.expect_within(
        moving_jaw,
        base,
        axes="xy",
        inner_elem="moving_right_runner",
        outer_elem="right_rail",
        margin=0.001,
        name="right runner stays on right rail at rest",
    )
    ctx.expect_gap(
        moving_jaw,
        base,
        axis="z",
        positive_elem="moving_left_runner",
        negative_elem="left_rail",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="left runner sits on left rail",
    )
    ctx.expect_gap(
        moving_jaw,
        base,
        axis="z",
        positive_elem="moving_right_runner",
        negative_elem="right_rail",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="right runner sits on right rail",
    )

    rest_pos = ctx.part_world_position(moving_jaw)
    with ctx.pose({jaw_slide: 0.090}):
        ctx.expect_gap(
            base,
            moving_jaw,
            axis="y",
            positive_elem="fixed_jaw_face",
            negative_elem="moving_jaw_face",
            min_gap=0.018,
            max_gap=0.026,
            name="closed pose leaves a usable clamping gap",
        )
        ctx.expect_within(
            moving_jaw,
            base,
            axes="xy",
            inner_elem="moving_left_runner",
            outer_elem="left_rail",
            margin=0.001,
            name="left runner retains rail support when closed",
        )
        ctx.expect_within(
            moving_jaw,
            base,
            axes="xy",
            inner_elem="moving_right_runner",
            outer_elem="right_rail",
            margin=0.001,
            name="right runner retains rail support when closed",
        )
        closed_pos = ctx.part_world_position(moving_jaw)

    ctx.check(
        "moving jaw advances toward the fixed jaw",
        rest_pos is not None and closed_pos is not None and closed_pos[1] > rest_pos[1] + 0.080,
        details=f"rest={rest_pos}, closed={closed_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

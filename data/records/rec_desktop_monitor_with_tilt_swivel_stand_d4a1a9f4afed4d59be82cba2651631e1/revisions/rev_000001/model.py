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
    model = ArticulatedObject(name="compact_office_monitor")

    matte_black = Material("matte_black", rgba=(0.010, 0.011, 0.012, 1.0))
    graphite = Material("graphite", rgba=(0.075, 0.080, 0.085, 1.0))
    dark_plastic = Material("dark_plastic", rgba=(0.030, 0.033, 0.036, 1.0))
    screen_glass = Material("screen_glass", rgba=(0.005, 0.012, 0.020, 1.0))
    rubber = Material("rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.300, 0.220, 0.024)),
        origin=Origin(xyz=(0.0, 0.020, 0.012)),
        material=graphite,
        name="foot",
    )
    base.visual(
        Box((0.190, 0.110, 0.012)),
        origin=Origin(xyz=(0.0, 0.045, 0.030)),
        material=dark_plastic,
        name="foot_riser",
    )

    sleeve_z = 0.1745
    sleeve_h = 0.301
    base.visual(
        Box((0.008, 0.047, sleeve_h)),
        origin=Origin(xyz=(0.0235, 0.050, sleeve_z)),
        material=graphite,
        name="sleeve_side_0",
    )
    base.visual(
        Box((0.008, 0.047, sleeve_h)),
        origin=Origin(xyz=(-0.0235, 0.050, sleeve_z)),
        material=graphite,
        name="sleeve_side_1",
    )
    base.visual(
        Box((0.039, 0.006, sleeve_h)),
        origin=Origin(xyz=(0.0, 0.0705, sleeve_z)),
        material=graphite,
        name="sleeve_wall_0",
    )
    base.visual(
        Box((0.039, 0.006, sleeve_h)),
        origin=Origin(xyz=(0.0, 0.0295, sleeve_z)),
        material=graphite,
        name="sleeve_wall_1",
    )
    base.visual(
        Box((0.014, 0.064, 0.018)),
        origin=Origin(xyz=(0.0265, 0.050, 0.334)),
        material=dark_plastic,
        name="collar_side_0",
    )
    base.visual(
        Box((0.014, 0.064, 0.018)),
        origin=Origin(xyz=(-0.0265, 0.050, 0.334)),
        material=dark_plastic,
        name="collar_side_1",
    )
    base.visual(
        Box((0.039, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.077, 0.334)),
        material=dark_plastic,
        name="collar_wall_0",
    )
    base.visual(
        Box((0.039, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.023, 0.334)),
        material=dark_plastic,
        name="collar_wall_1",
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.039, 0.035, 0.420)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=matte_black,
        name="inner_post",
    )
    mast.visual(
        Box((0.104, 0.090, 0.030)),
        origin=Origin(xyz=(0.0, -0.045, 0.165)),
        material=graphite,
        name="tilt_arm",
    )
    mast.visual(
        Box((0.016, 0.032, 0.070)),
        origin=Origin(xyz=(0.045, -0.096, 0.200)),
        material=graphite,
        name="tilt_ear_0",
    )
    mast.visual(
        Box((0.016, 0.032, 0.070)),
        origin=Origin(xyz=(-0.045, -0.096, 0.200)),
        material=graphite,
        name="tilt_ear_1",
    )

    panel = model.part("panel")
    panel.visual(
        Cylinder(radius=0.016, length=0.074),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="hinge_lug",
    )
    panel.visual(
        Box((0.060, 0.040, 0.068)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=graphite,
        name="rear_boss",
    )
    panel.visual(
        Box((0.560, 0.038, 0.340)),
        origin=Origin(xyz=(0.0, -0.059, 0.0)),
        material=dark_plastic,
        name="rear_shell",
    )
    panel.visual(
        Box((0.560, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, -0.081, 0.159)),
        material=matte_black,
        name="top_bezel",
    )
    panel.visual(
        Box((0.560, 0.006, 0.032)),
        origin=Origin(xyz=(0.0, -0.081, -0.154)),
        material=matte_black,
        name="bottom_bezel",
    )
    panel.visual(
        Box((0.022, 0.006, 0.340)),
        origin=Origin(xyz=(0.269, -0.081, 0.0)),
        material=matte_black,
        name="side_bezel_0",
    )
    panel.visual(
        Box((0.022, 0.006, 0.340)),
        origin=Origin(xyz=(-0.269, -0.081, 0.0)),
        material=matte_black,
        name="side_bezel_1",
    )
    panel.visual(
        Box((0.505, 0.003, 0.276)),
        origin=Origin(xyz=(0.0, -0.0855, 0.000)),
        material=screen_glass,
        name="screen",
    )
    panel.visual(
        Box((0.032, 0.014, 0.026)),
        origin=Origin(xyz=(0.270, -0.083, -0.183)),
        material=matte_black,
        name="control_tab",
    )
    panel.visual(
        Cylinder(radius=0.006, length=0.056),
        origin=Origin(xyz=(0.300, -0.083, -0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="control_shaft",
    )

    control_wheel = model.part("control_wheel")
    control_wheel.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="wheel_disc",
    )
    control_wheel.visual(
        Cylinder(radius=0.011, length=0.028),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="wheel_hub",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.050, 0.325)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.18, lower=0.0, upper=0.120),
    )
    model.articulation(
        "mast_to_panel",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=panel,
        origin=Origin(xyz=(0.0, -0.100, 0.200)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.30, upper=0.38),
    )
    model.articulation(
        "panel_to_control_wheel",
        ArticulationType.CONTINUOUS,
        parent=panel,
        child=control_wheel,
        origin=Origin(xyz=(0.318, -0.083, -0.190)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    panel = object_model.get_part("panel")
    control_wheel = object_model.get_part("control_wheel")
    height_slide = object_model.get_articulation("base_to_mast")
    control_spin = object_model.get_articulation("panel_to_control_wheel")

    ctx.allow_overlap(
        panel,
        control_wheel,
        elem_a="control_shaft",
        elem_b="wheel_disc",
        reason="The side control wheel is intentionally captured on a short shaft through its hub.",
    )
    ctx.allow_overlap(
        panel,
        control_wheel,
        elem_a="control_shaft",
        elem_b="wheel_hub",
        reason="The modeled hub surrounds the same short shaft as a retained rotary fit.",
    )

    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_post",
        elem_b="sleeve_side_0",
        min_overlap=0.180,
        name="collapsed mast remains deeply inserted",
    )
    ctx.expect_contact(
        mast,
        base,
        elem_a="inner_post",
        elem_b="sleeve_side_0",
        contact_tol=0.0005,
        name="mast bears on sleeve side 0",
    )
    ctx.expect_contact(
        mast,
        base,
        elem_a="inner_post",
        elem_b="sleeve_side_1",
        contact_tol=0.0005,
        name="mast bears on sleeve side 1",
    )
    ctx.expect_within(
        panel,
        mast,
        axes="x",
        inner_elem="hinge_lug",
        outer_elem="tilt_arm",
        margin=0.010,
        name="hinge lug stays between bracket cheeks",
    )
    ctx.expect_overlap(
        control_wheel,
        panel,
        axes="x",
        elem_a="wheel_disc",
        elem_b="control_shaft",
        min_overlap=0.010,
        name="control wheel is retained on its shaft",
    )
    ctx.expect_within(
        panel,
        control_wheel,
        axes="yz",
        inner_elem="control_shaft",
        outer_elem="wheel_disc",
        margin=0.001,
        name="control shaft is centered in wheel",
    )

    rest_panel_pos = ctx.part_world_position(panel)
    with ctx.pose({height_slide: 0.120}):
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="sleeve_side_0",
            min_overlap=0.080,
            name="raised mast keeps retained insertion",
        )
        raised_panel_pos = ctx.part_world_position(panel)

    ctx.check(
        "height slide raises the panel",
        rest_panel_pos is not None
        and raised_panel_pos is not None
        and raised_panel_pos[2] > rest_panel_pos[2] + 0.10,
        details=f"rest={rest_panel_pos}, raised={raised_panel_pos}",
    )
    with ctx.pose({control_spin: math.pi / 2.0}):
        ctx.expect_overlap(
            control_wheel,
            panel,
            axes="x",
            elem_a="wheel_disc",
            elem_b="control_shaft",
            min_overlap=0.010,
            name="rotated wheel stays captured",
        )

    return ctx.report()


object_model = build_object_model()

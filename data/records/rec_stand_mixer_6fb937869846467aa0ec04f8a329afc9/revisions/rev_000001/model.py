from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    z_center: float,
    corner: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_val, z_center + z_val)
        for z_val, y_val in rounded_rect_profile(height, width, corner)
    ]


def _build_column_shell():
    return section_loft(
        [
            _yz_section(-0.080, width=0.112, height=0.210, z_center=0.105, corner=0.028),
            _yz_section(-0.040, width=0.108, height=0.245, z_center=0.142, corner=0.030),
            _yz_section(0.000, width=0.098, height=0.300, z_center=0.168, corner=0.032),
            _yz_section(0.030, width=0.090, height=0.235, z_center=0.182, corner=0.028),
        ]
    )


def _build_head_shell():
    return section_loft(
        [
            _yz_section(-0.005, width=0.112, height=0.145, z_center=0.295, corner=0.034),
            _yz_section(0.090, width=0.145, height=0.150, z_center=0.316, corner=0.044),
            _yz_section(0.185, width=0.150, height=0.145, z_center=0.310, corner=0.044),
            _yz_section(0.275, width=0.095, height=0.092, z_center=0.282, corner=0.030),
        ]
    )


def _build_bowl_shell():
    outer_profile = [
        (0.018, 0.000),
        (0.060, 0.014),
        (0.098, 0.070),
        (0.106, 0.132),
        (0.108, 0.168),
    ]
    inner_profile = [
        (0.000, 0.005),
        (0.051, 0.015),
        (0.094, 0.070),
        (0.100, 0.156),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        end_cap="flat",
    )


def _build_dough_hook():
    return tube_from_spline_points(
        [
            (0.000, 0.000, -0.028),
            (0.004, 0.002, -0.048),
            (0.010, 0.006, -0.072),
            (0.014, 0.007, -0.096),
            (0.010, -0.002, -0.118),
            (-0.002, -0.009, -0.128),
            (-0.012, -0.004, -0.116),
        ],
        radius=0.0062,
        samples_per_segment=16,
        radial_segments=22,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixed_head_bowl_lift_mixer")

    enamel = model.material("enamel", rgba=(0.79, 0.11, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.83, 0.85, 0.88, 1.0))
    dark = model.material("dark", rgba=(0.12, 0.12, 0.13, 1.0))
    trim = model.material("trim", rgba=(0.70, 0.72, 0.75, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.340, 0.250, 0.055), 0.045),
            "base_plinth",
        ),
        origin=Origin(xyz=(0.015, 0.0, 0.0225)),
        material=enamel,
        name="base_plinth",
    )
    body.visual(
        mesh_from_geometry(_build_column_shell(), "column_shell"),
        material=enamel,
        name="column_shell",
    )
    body.visual(
        mesh_from_geometry(_build_head_shell(), "head_shell"),
        material=enamel,
        name="head_shell",
    )
    body.visual(
        Cylinder(radius=0.039, length=0.032),
        origin=Origin(xyz=(0.205, 0.0, 0.226)),
        material=enamel,
        name="planetary_housing",
    )
    body.visual(
        Box((0.016, 0.024, 0.155)),
        origin=Origin(xyz=(0.038, 0.083, 0.1175)),
        material=enamel,
        name="guide_rail_0",
    )
    body.visual(
        Box((0.016, 0.024, 0.155)),
        origin=Origin(xyz=(0.038, -0.083, 0.1175)),
        material=enamel,
        name="guide_rail_1",
    )
    body.visual(
        Box((0.045, 0.030, 0.072)),
        origin=Origin(xyz=(0.015, 0.060, 0.156)),
        material=enamel,
        name="lift_wheel_pad",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.028),
        origin=Origin(xyz=(0.010, 0.084, 0.156), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=enamel,
        name="lift_wheel_boss",
    )
    body.visual(
        Box((0.034, 0.026, 0.042)),
        origin=Origin(xyz=(0.030, 0.060, 0.226)),
        material=enamel,
        name="speed_knob_pad",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.044, 0.072, 0.226), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=enamel,
        name="speed_knob_boss",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.205, 0.0, 0.205)),
        material=trim,
        name="attachment_socket",
    )

    bowl_lift = model.part("bowl_lift")
    bowl_lift.visual(
        Box((0.030, 0.180, 0.028)),
        origin=Origin(xyz=(0.068, 0.0, 0.020)),
        material=dark,
        name="rear_crossbar",
    )
    bowl_lift.visual(
        Box((0.024, 0.024, 0.110)),
        origin=Origin(xyz=(0.055, 0.083, 0.072)),
        material=dark,
        name="slider_0",
    )
    bowl_lift.visual(
        Box((0.024, 0.024, 0.110)),
        origin=Origin(xyz=(0.055, -0.083, 0.072)),
        material=dark,
        name="slider_1",
    )
    bowl_lift.visual(
        Box((0.120, 0.070, 0.022)),
        origin=Origin(xyz=(0.110, 0.110, 0.029)),
        material=dark,
        name="arm_0",
    )
    bowl_lift.visual(
        Box((0.120, 0.070, 0.022)),
        origin=Origin(xyz=(0.110, -0.110, 0.029)),
        material=dark,
        name="arm_1",
    )
    bowl_lift.visual(
        Box((0.028, 0.022, 0.120)),
        origin=Origin(xyz=(0.155, 0.138, 0.100)),
        material=dark,
        name="saddle_0",
    )
    bowl_lift.visual(
        Box((0.028, 0.022, 0.120)),
        origin=Origin(xyz=(0.155, -0.138, 0.100)),
        material=dark,
        name="saddle_1",
    )
    model.articulation(
        "body_to_bowl_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bowl_lift,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=0.0, upper=0.078),
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(_build_bowl_shell(), "bowl_shell"),
        material=steel,
        name="bowl_shell",
    )
    bowl.visual(
        Box((0.018, 0.020, 0.020)),
        origin=Origin(xyz=(0.000, 0.109, 0.108)),
        material=steel,
        name="lug_0_block",
    )
    bowl.visual(
        Box((0.018, 0.020, 0.020)),
        origin=Origin(xyz=(0.000, -0.109, 0.108)),
        material=steel,
        name="lug_1_block",
    )
    bowl.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(0.0, 0.118, 0.108), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lug_0_pin",
    )
    bowl.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(0.0, -0.118, 0.108), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lug_1_pin",
    )

    model.articulation(
        "bowl_lift_to_bowl",
        ArticulationType.FIXED,
        parent=bowl_lift,
        child=bowl,
        origin=Origin(xyz=(0.176, 0.0, 0.018)),
    )

    lift_wheel = model.part("lift_wheel")
    lift_wheel.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.074,
                0.020,
                body_style="skirted",
                top_diameter=0.060,
                skirt=KnobSkirt(0.080, 0.004, flare=0.05),
                grip=KnobGrip(style="fluted", count=18, depth=0.0016),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "lift_wheel",
        ),
        origin=Origin(xyz=(0.0, 0.0248, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="wheel_shell",
    )
    model.articulation(
        "body_to_lift_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lift_wheel,
        origin=Origin(xyz=(0.010, 0.098, 0.156)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.018,
                body_style="skirted",
                top_diameter=0.028,
                skirt=KnobSkirt(0.042, 0.0035, flare=0.06),
                grip=KnobGrip(style="fluted", count=12, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006, angle_deg=90.0),
                center=False,
            ),
            "speed_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0215, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="knob_shell",
    )
    model.articulation(
        "body_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=speed_knob,
        origin=Origin(xyz=(0.044, 0.083, 0.226)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    dough_hook = model.part("dough_hook")
    dough_hook.visual(
        Cylinder(radius=0.0065, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=steel,
        name="hook_shaft",
    )
    dough_hook.visual(
        Cylinder(radius=0.0105, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        material=trim,
        name="hook_ferrule",
    )
    dough_hook.visual(
        mesh_from_geometry(_build_dough_hook(), "dough_hook_curve"),
        material=steel,
        name="hook_curve",
    )
    model.articulation(
        "body_to_dough_hook",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dough_hook,
        origin=Origin(xyz=(0.188, 0.0, 0.199)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=14.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bowl_lift = object_model.get_part("bowl_lift")
    bowl = object_model.get_part("bowl")
    dough_hook = object_model.get_part("dough_hook")
    speed_knob = object_model.get_part("speed_knob")
    lift_wheel = object_model.get_part("lift_wheel")
    lift_joint = object_model.get_articulation("body_to_bowl_lift")
    wheel_joint = object_model.get_articulation("body_to_lift_wheel")
    knob_joint = object_model.get_articulation("body_to_speed_knob")
    hook_joint = object_model.get_articulation("body_to_dough_hook")

    ctx.expect_overlap(
        bowl,
        bowl_lift,
        axes="xy",
        min_overlap=0.08,
        name="bowl stays supported within lift footprint",
    )

    limits = lift_joint.motion_limits
    rest_pos = ctx.part_world_position(bowl)
    raised_pos = None
    if limits is not None and limits.upper is not None:
        with ctx.pose({lift_joint: limits.upper}):
            ctx.expect_overlap(
                bowl,
                bowl_lift,
                axes="xy",
                min_overlap=0.08,
                name="raised bowl stays supported within lift footprint",
            )
            ctx.expect_overlap(
                dough_hook,
                bowl,
                axes="xy",
                min_overlap=0.028,
                name="raised bowl stays under dough hook",
            )
            raised_pos = ctx.part_world_position(bowl)

    ctx.check(
        "bowl lift raises vertically",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.06,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )
    ctx.expect_overlap(
        dough_hook,
        bowl,
        axes="xy",
        min_overlap=0.028,
        name="dough hook sits over bowl footprint",
    )

    speed_pos = ctx.part_world_position(speed_knob)
    wheel_pos = ctx.part_world_position(lift_wheel)
    ctx.check(
        "speed knob sits above lift wheel",
        speed_pos is not None
        and wheel_pos is not None
        and speed_pos[2] > wheel_pos[2] + 0.05
        and abs(speed_pos[1] - wheel_pos[1]) < 0.02,
        details=f"speed={speed_pos}, wheel={wheel_pos}",
    )
    ctx.check(
        "primary articulations have requested motion types",
        lift_joint.articulation_type == ArticulationType.PRISMATIC
        and wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and hook_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"lift={lift_joint.articulation_type}, wheel={wheel_joint.articulation_type}, "
            f"knob={knob_joint.articulation_type}, hook={hook_joint.articulation_type}"
        ),
    )
    ctx.check(
        "continuous axes match mixer controls",
        tuple(round(v, 3) for v in wheel_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in knob_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in hook_joint.axis) == (0.0, 0.0, 1.0),
        details=f"wheel={wheel_joint.axis}, knob={knob_joint.axis}, hook={hook_joint.axis}",
    )

    return ctx.report()


object_model = build_object_model()

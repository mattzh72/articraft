from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
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


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    *,
    z: float,
    x_center: float = 0.0,
    y_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_center, y + y_center, z)
        for x, y in rounded_rect_profile(width, depth, radius)
    ]


def _yz_section(
    x: float,
    width: float,
    height: float,
    radius: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for y, z in rounded_rect_profile(width, height, radius)
    ]


def _build_whisk_geometry() -> CylinderGeometry:
    z_drop = -0.010
    whisk_geom = CylinderGeometry(radius=0.0045, height=0.033).translate(0.0, 0.0, -0.0195)
    whisk_geom.merge(CylinderGeometry(radius=0.0095, height=0.020).translate(0.0, 0.0, -0.034 + z_drop))
    whisk_geom.merge(CylinderGeometry(radius=0.0060, height=0.012).translate(0.0, 0.0, -0.048 + z_drop))

    loop_count = 6
    for index in range(loop_count):
        angle = index * math.pi / loop_count
        c = math.cos(angle)
        s = math.sin(angle)
        loop_points = [
            (0.004 * c, 0.004 * s, -0.026 + z_drop),
            (0.016 * c, 0.016 * s, -0.040 + z_drop),
            (0.030 * c, 0.030 * s, -0.061 + z_drop),
            (0.041 * c, 0.041 * s, -0.085 + z_drop),
            (0.000, 0.000, -0.110 + z_drop),
            (-0.041 * c, -0.041 * s, -0.085 + z_drop),
            (-0.030 * c, -0.030 * s, -0.061 + z_drop),
            (-0.016 * c, -0.016 * s, -0.040 + z_drop),
            (-0.004 * c, -0.004 * s, -0.026 + z_drop),
        ]
        whisk_geom.merge(
            tube_from_spline_points(
                loop_points,
                radius=0.0016,
                samples_per_segment=16,
                radial_segments=16,
            )
        )
    return whisk_geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head_mixer")

    body_paint = model.material("body_paint", rgba=(0.84, 0.14, 0.12, 1.0))
    stainless = model.material("stainless", rgba=(0.82, 0.84, 0.88, 1.0))
    control_dark = model.material("control_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    foot_dark = model.material("foot_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    base = model.part("base")
    bowl = model.part("bowl")
    head = model.part("head")
    slider = model.part("slider")
    whisk = model.part("whisk")

    base_shell = section_loft(
        [
            _xy_section(0.340, 0.222, 0.052, z=0.000),
            _xy_section(0.312, 0.206, 0.050, z=0.024),
            _xy_section(0.255, 0.168, 0.040, z=0.058),
        ]
    )
    base.visual(
        mesh_from_geometry(base_shell, "base_shell"),
        material=body_paint,
        name="base_shell",
    )

    column_shell = section_loft(
        [
            _xy_section(0.136, 0.100, 0.028, z=0.052, x_center=-0.066),
            _xy_section(0.116, 0.092, 0.025, z=0.148, x_center=-0.082),
            _xy_section(0.094, 0.084, 0.022, z=0.224, x_center=-0.094),
            _xy_section(0.082, 0.078, 0.020, z=0.272, x_center=-0.098),
        ]
    )
    base.visual(
        mesh_from_geometry(column_shell, "column_shell"),
        material=body_paint,
        name="column_shell",
    )

    base.visual(
        Cylinder(radius=0.073, length=0.012),
        origin=Origin(xyz=(0.125, 0.0, 0.064)),
        material=body_paint,
        name="bowl_pad",
    )
    base.visual(
        Box((0.026, 0.102, 0.020)),
        origin=Origin(xyz=(-0.102, 0.0, 0.281)),
        material=body_paint,
        name="hinge_bridge",
    )
    for y_pos, name in ((-0.044, "hinge_cheek_0"), (0.044, "hinge_cheek_1")):
        base.visual(
            Box((0.026, 0.018, 0.055)),
            origin=Origin(xyz=(-0.092, y_pos, 0.3025)),
            material=body_paint,
            name=name,
        )
    for x_pos, y_pos, name in (
        (-0.110, -0.070, "foot_0"),
        (0.125, -0.070, "foot_1"),
        (-0.110, 0.070, "foot_2"),
        (0.125, 0.070, "foot_3"),
    ):
        base.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(x_pos, y_pos, 0.004)),
            material=foot_dark,
            name=name,
        )

    bowl_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.040, 0.000),
            (0.054, 0.006),
            (0.080, 0.022),
            (0.103, 0.065),
            (0.109, 0.124),
            (0.114, 0.136),
        ],
        inner_profile=[
            (0.000, 0.010),
            (0.046, 0.015),
            (0.074, 0.030),
            (0.097, 0.065),
            (0.103, 0.130),
        ],
        segments=72,
        end_cap="round",
        lip_samples=10,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "bowl_shell"),
        material=stainless,
        name="bowl_shell",
    )

    head_shell = section_loft(
        [
            _yz_section(0.000, 0.082, 0.102, 0.026, z_center=-0.006),
            _yz_section(0.105, 0.128, 0.156, 0.041, z_center=-0.020),
            _yz_section(0.215, 0.116, 0.140, 0.036, z_center=-0.032),
            _yz_section(0.302, 0.086, 0.100, 0.026, z_center=-0.045),
        ]
    )
    head.visual(
        mesh_from_geometry(head_shell, "head_shell"),
        origin=Origin(xyz=(0.028, 0.0, 0.010)),
        material=body_paint,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.023, length=0.070),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_paint,
        name="hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(xyz=(0.214, 0.0, -0.083)),
        material=stainless,
        name="drive_collar",
    )
    head.visual(
        Box((0.064, 0.012, 0.013)),
        origin=Origin(xyz=(0.060, 0.063, 0.010)),
        material=control_dark,
        name="slider_track",
    )

    slider.visual(
        Box((0.034, 0.008, 0.010)),
        origin=Origin(xyz=(0.017, 0.000, 0.002)),
        material=control_dark,
        name="slider_shoe",
    )
    slider.visual(
        Box((0.020, 0.010, 0.006)),
        origin=Origin(xyz=(0.012, -0.005, 0.003)),
        material=control_dark,
        name="slider_stem",
    )
    slider.visual(
        Box((0.018, 0.020, 0.014)),
        origin=Origin(xyz=(0.016, 0.012, 0.006)),
        material=control_dark,
        name="slider_tab",
    )

    whisk.visual(
        mesh_from_geometry(_build_whisk_geometry(), "whisk_shell"),
        material=stainless,
        name="whisk_shell",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.125, 0.0, 0.070)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.092, 0.0, 0.326)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "head_to_slider",
        ArticulationType.PRISMATIC,
        parent=head,
        child=slider,
        origin=Origin(xyz=(0.026, 0.079, 0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=0.08,
            lower=0.0,
            upper=0.020,
        ),
    )
    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.214, 0.0, -0.094)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=20.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    slider = object_model.get_part("slider")
    whisk = object_model.get_part("whisk")

    head_joint = object_model.get_articulation("base_to_head")
    slider_joint = object_model.get_articulation("head_to_slider")
    whisk_joint = object_model.get_articulation("head_to_whisk")

    head_upper = (
        head_joint.motion_limits.upper
        if head_joint.motion_limits is not None and head_joint.motion_limits.upper is not None
        else math.radians(62.0)
    )
    slider_upper = (
        slider_joint.motion_limits.upper
        if slider_joint.motion_limits is not None and slider_joint.motion_limits.upper is not None
        else 0.020
    )

    with ctx.pose({head_joint: 0.0, slider_joint: 0.0, whisk_joint: 0.0}):
        ctx.expect_overlap(
            whisk,
            bowl,
            axes="xy",
            elem_a="whisk_shell",
            elem_b="bowl_shell",
            min_overlap=0.070,
            name="whisk sits over the bowl opening",
        )
        ctx.expect_gap(
            head,
            bowl,
            axis="z",
            elem_a="head_shell",
            elem_b="bowl_shell",
            min_gap=0.014,
            name="head shell clears the bowl rim",
        )
        ctx.expect_overlap(
            slider,
            head,
            axes="xz",
            elem_a="slider_shoe",
            elem_b="slider_track",
            min_overlap=0.006,
            name="slider remains seated on the guide track at rest",
        )

    whisk_rest = None
    whisk_raised = None
    with ctx.pose({head_joint: 0.0}):
        whisk_rest = ctx.part_world_position(whisk)
    with ctx.pose({head_joint: head_upper}):
        ctx.expect_gap(
            whisk,
            bowl,
            axis="z",
            elem_a="whisk_shell",
            elem_b="bowl_shell",
            min_gap=0.035,
            name="raised whisk clears above the bowl",
        )
        whisk_raised = ctx.part_world_position(whisk)
    ctx.check(
        "head tilts the whisk upward and rearward",
        whisk_rest is not None
        and whisk_raised is not None
        and whisk_raised[2] > whisk_rest[2] + 0.080
        and whisk_raised[0] < whisk_rest[0] - 0.030,
        details=f"rest={whisk_rest}, raised={whisk_raised}",
    )

    slider_rest = None
    slider_fast = None
    with ctx.pose({slider_joint: 0.0}):
        slider_rest = ctx.part_world_position(slider)
    with ctx.pose({slider_joint: slider_upper}):
        ctx.expect_overlap(
            slider,
            head,
            axes="xz",
            elem_a="slider_shoe",
            elem_b="slider_track",
            min_overlap=0.006,
            name="slider remains seated on the guide track at full travel",
        )
        slider_fast = ctx.part_world_position(slider)
    ctx.check(
        "speed slider moves forward along the side guide",
        slider_rest is not None
        and slider_fast is not None
        and slider_fast[0] > slider_rest[0] + 0.015
        and abs(slider_fast[1] - slider_rest[1]) < 0.003
        and abs(slider_fast[2] - slider_rest[2]) < 0.003,
        details=f"rest={slider_rest}, fast={slider_fast}",
    )

    whisk_spin_rest = None
    whisk_spin_turn = None
    with ctx.pose({whisk_joint: 0.0}):
        whisk_spin_rest = ctx.part_world_position(whisk)
    with ctx.pose({whisk_joint: math.pi / 2.0}):
        whisk_spin_turn = ctx.part_world_position(whisk)
    ctx.check(
        "whisk spins in place on the drive axis",
        whisk_spin_rest is not None
        and whisk_spin_turn is not None
        and max(abs(a - b) for a, b in zip(whisk_spin_rest, whisk_spin_turn)) < 1e-6,
        details=f"q0={whisk_spin_rest}, q90={whisk_spin_turn}",
    )

    return ctx.report()


object_model = build_object_model()

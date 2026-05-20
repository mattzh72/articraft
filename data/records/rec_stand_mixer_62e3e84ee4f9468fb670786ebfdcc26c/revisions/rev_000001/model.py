from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    place_on_surface,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


HINGE_X = -0.102
HINGE_Z = 0.326
BOWL_CENTER_X = 0.104
BOWL_SEAT_Z = 0.04367
DRIVE_X = 0.206
DRIVE_Z = -0.055


def _yz_section(
    *,
    x: float,
    width: float,
    height: float,
    z_center: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=10)
    ]


def _build_base_solid() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.340, 0.230, 0.026)
        .edges("|Z")
        .fillet(0.020)
        .translate((0.030, 0.000, 0.013))
    )

    pedestal = (
        cq.Workplane("XZ")
        .moveTo(-0.128, 0.026)
        .lineTo(-0.052, 0.026)
        .lineTo(-0.020, 0.148)
        .lineTo(-0.058, 0.318)
        .lineTo(-0.122, 0.300)
        .close()
        .extrude(0.104)
        .translate((0.000, 0.052, 0.000))
    )

    bowl_dais = (
        cq.Workplane("XY")
        .circle(0.020)
        .extrude(0.01767)
        .translate((BOWL_CENTER_X, 0.000, 0.026))
    )

    lug_radius = 0.015
    lug_length = 0.030
    left_lug = (
        cq.Workplane("XZ")
        .center(HINGE_X, HINGE_Z)
        .circle(lug_radius)
        .extrude(lug_length)
        .translate((0.000, -0.011, 0.000))
    )
    right_lug = (
        cq.Workplane("XZ")
        .center(HINGE_X, HINGE_Z)
        .circle(lug_radius)
        .extrude(lug_length)
        .translate((0.000, 0.041, 0.000))
    )

    left_post = (
        cq.Workplane("XY")
        .box(0.026, 0.022, 0.040)
        .translate((HINGE_X, -0.026, 0.291))
    )
    right_post = (
        cq.Workplane("XY")
        .box(0.026, 0.022, 0.040)
        .translate((HINGE_X, 0.026, 0.291))
    )

    return (
        plate.union(pedestal)
        .union(bowl_dais)
        .union(left_post)
        .union(right_post)
        .union(left_lug)
        .union(right_lug)
    )


def _build_bowl_geometry() -> LatheGeometry:
    outer = [
        (0.018, 0.000),
        (0.034, 0.000),
        (0.046, 0.010),
        (0.074, 0.030),
        (0.102, 0.078),
        (0.110, 0.132),
        (0.114, 0.160),
    ]
    inner = [
        (0.000, 0.004),
        (0.018, 0.004),
        (0.032, 0.012),
        (0.067, 0.031),
        (0.096, 0.078),
        (0.102, 0.150),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=64,
        end_cap="round",
        lip_samples=10,
    )


def _build_head_shell() -> object:
    sections = [
        _yz_section(x=0.015, width=0.092, height=0.118, z_center=0.008, radius=0.018),
        _yz_section(x=0.070, width=0.146, height=0.178, z_center=0.040, radius=0.030),
        _yz_section(x=0.150, width=0.160, height=0.170, z_center=0.044, radius=0.030),
        _yz_section(x=0.232, width=0.146, height=0.148, z_center=0.028, radius=0.025),
        _yz_section(x=0.292, width=0.090, height=0.090, z_center=0.000, radius=0.018),
    ]
    return section_loft(sections)


def _build_whisk_geometry() -> CylinderGeometry:
    whisk = CylinderGeometry(radius=0.0048, height=0.046).translate(0.000, 0.000, -0.023)
    whisk.merge(CylinderGeometry(radius=0.0120, height=0.010).translate(0.000, 0.000, -0.050))
    whisk.merge(CylinderGeometry(radius=0.0080, height=0.020).translate(0.000, 0.000, -0.056))
    whisk.merge(CylinderGeometry(radius=0.0110, height=0.016).translate(0.000, 0.000, -0.074))
    whisk.merge(CylinderGeometry(radius=0.0045, height=0.012).translate(0.000, 0.000, -0.158))

    loop_count = 6
    for index in range(loop_count):
        angle = index * math.tau / loop_count
        c = math.cos(angle)
        s = math.sin(angle)
        loop = tube_from_spline_points(
            [
                (0.010 * c, 0.010 * s, -0.050),
                (0.026 * c, 0.026 * s, -0.072),
                (0.042 * c, 0.042 * s, -0.106),
                (0.049 * c, 0.049 * s, -0.136),
                (0.000, 0.000, -0.158),
                (-0.049 * c, -0.049 * s, -0.136),
                (-0.042 * c, -0.042 * s, -0.106),
                (-0.026 * c, -0.026 * s, -0.072),
                (-0.010 * c, -0.010 * s, -0.050),
            ],
            radius=0.0015,
            samples_per_segment=10,
            radial_segments=14,
            cap_ends=True,
        )
        whisk.merge(loop)

    return whisk


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head_stand_mixer")

    body_finish = model.material("body_finish", rgba=(0.86, 0.84, 0.78, 1.0))
    metal = model.material("metal", rgba=(0.86, 0.87, 0.89, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.18, 0.18, 0.20, 1.0))
    button_finish = model.material("button_finish", rgba=(0.92, 0.93, 0.94, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_solid(), "mixer_base"),
        material=body_finish,
        name="base_shell",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(_build_bowl_geometry(), "mixer_bowl"),
        material=metal,
        name="bowl_shell",
    )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(_build_head_shell(), "mixer_head_shell"),
        origin=Origin(xyz=(0.034, 0.000, 0.040)),
        material=body_finish,
        name="head_shell",
    )
    head.visual(
        Box((0.045, 0.018, 0.040)),
        origin=Origin(xyz=(0.027, 0.000, 0.008)),
        material=body_finish,
        name="hinge_bridge",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.000, 0.000)),
        material=trim_dark,
        name="hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.025, length=0.070),
        origin=Origin(xyz=(DRIVE_X, 0.000, -0.020)),
        material=body_finish,
        name="drive_socket",
    )
    head.visual(
        Box((0.078, 0.004, 0.092)),
        origin=Origin(xyz=(0.132, 0.079, 0.024)),
        material=body_finish,
        name="control_panel",
    )

    side_dial = model.part("side_dial")
    side_dial.visual(
        Cylinder(radius=0.0045, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
        material=trim_dark,
        name="dial_shaft",
    )
    side_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.018,
                body_style="skirted",
                top_diameter=0.028,
                skirt=KnobSkirt(0.038, 0.004, flare=0.05),
                grip=KnobGrip(style="fluted", count=16, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006, angle_deg=0.0),
                center=False,
            ),
            "side_dial_knob",
        ),
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=trim_dark,
        name="dial_knob",
    )

    for index in range(2):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.0052, length=0.007),
            origin=Origin(xyz=(0.000, 0.000, 0.0035)),
            material=button_finish,
            name="button_cap",
        )

    whisk = model.part("whisk")
    whisk.visual(
        mesh_from_geometry(_build_whisk_geometry(), "mixer_whisk"),
        material=metal,
        name="whisk_shell",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(BOWL_CENTER_X, 0.000, BOWL_SEAT_Z)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(HINGE_X, 0.000, HINGE_Z)),
        axis=(0.000, -1.000, 0.000),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(65.0),
        ),
    )

    model.articulation(
        "head_to_dial",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=side_dial,
        origin=Origin(
            xyz=(0.134, 0.081, 0.060),
            rpy=(-math.pi / 2.0, 0.000, 0.000),
        ),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(effort=0.5, velocity=12.0),
    )

    for index, z_center in enumerate((0.028, 0.000)):
        model.articulation(
            f"head_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=head,
            child=model.get_part(f"button_{index}"),
            origin=Origin(
                xyz=(0.130, 0.081, z_center),
                rpy=(-math.pi / 2.0, 0.000, 0.000),
            ),
            axis=(0.000, 0.000, -1.000),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.040,
                lower=0.0,
                upper=0.0032,
            ),
        )

    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(DRIVE_X, 0.000, DRIVE_Z)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(effort=20.0, velocity=28.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    side_dial = object_model.get_part("side_dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    head_joint = object_model.get_articulation("base_to_head")
    button_0_joint = object_model.get_articulation("head_to_button_0")
    button_1_joint = object_model.get_articulation("head_to_button_1")

    ctx.expect_within(
        whisk,
        bowl,
        axes="xy",
        margin=0.010,
        name="whisk stays over the bowl opening at rest",
    )
    ctx.allow_overlap(
        "base",
        "bowl",
        reason="The removable bowl is intentionally simplified as seating into a compact pedestal socket on the low base.",
    )
    ctx.expect_origin_gap(
        side_dial,
        button_0,
        axis="z",
        min_gap=0.020,
        name="dial sits above the upper push button",
    )
    ctx.expect_origin_gap(
        button_0,
        button_1,
        axis="z",
        min_gap=0.016,
        name="push buttons are vertically separated",
    )

    rest_whisk_pos = ctx.part_world_position(whisk)
    head_upper = 0.0
    if head_joint.motion_limits is not None and head_joint.motion_limits.upper is not None:
        head_upper = head_joint.motion_limits.upper
    with ctx.pose({head_joint: head_upper}):
        opened_whisk_pos = ctx.part_world_position(whisk)
        ctx.expect_gap(
            whisk,
            bowl,
            axis="z",
            min_gap=0.120,
            name="opened whisk clears above the bowl",
        )

    ctx.check(
        "tilt head raises the whisk",
        rest_whisk_pos is not None
        and opened_whisk_pos is not None
        and opened_whisk_pos[2] > rest_whisk_pos[2] + 0.180,
        details=f"rest={rest_whisk_pos}, opened={opened_whisk_pos}",
    )

    rest_button_0 = ctx.part_world_position(button_0)
    with ctx.pose({button_0_joint: 0.0032}):
        pressed_button_0 = ctx.part_world_position(button_0)
    ctx.check(
        "upper button depresses inward",
        rest_button_0 is not None
        and pressed_button_0 is not None
        and pressed_button_0[1] < rest_button_0[1] - 0.0020,
        details=f"rest={rest_button_0}, pressed={pressed_button_0}",
    )

    rest_button_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_1_joint: 0.0032}):
        pressed_button_1 = ctx.part_world_position(button_1)
    ctx.check(
        "lower button depresses inward",
        rest_button_1 is not None
        and pressed_button_1 is not None
        and pressed_button_1[1] < rest_button_1[1] - 0.0020,
        details=f"rest={rest_button_1}, pressed={pressed_button_1}",
    )

    return ctx.report()


object_model = build_object_model()

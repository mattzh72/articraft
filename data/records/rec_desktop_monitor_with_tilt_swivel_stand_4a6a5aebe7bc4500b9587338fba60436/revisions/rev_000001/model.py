from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _curved_planform(
    *,
    width: float,
    front_center: float,
    front_side: float,
    back_center: float,
    back_side: float,
) -> cq.Workplane:
    half_width = width * 0.5
    return (
        cq.Workplane("XY")
        .moveTo(-half_width, front_side)
        .threePointArc((0.0, front_center), (half_width, front_side))
        .lineTo(half_width, back_side)
        .threePointArc((0.0, back_center), (-half_width, back_side))
        .close()
    )


def _display_shell_solid() -> cq.Workplane:
    outer = _curved_planform(
        width=0.860,
        front_center=0.019,
        front_side=0.008,
        back_center=-0.034,
        back_side=-0.028,
    ).extrude(0.390)
    outer = outer.translate((0.0, 0.0, -0.195))
    outer = outer.edges("|Z").fillet(0.012)

    inner = (
        _curved_planform(
            width=0.812,
            front_center=0.014,
            front_side=0.005,
            back_center=-0.026,
            back_side=-0.021,
        )
        .extrude(0.348)
        .translate((0.0, -0.002, -0.174))
    )
    return outer.cut(inner)


def _screen_solid() -> cq.Workplane:
    return (
        _curved_planform(
            width=0.818,
            front_center=0.0135,
            front_side=0.0065,
            back_center=0.0095,
            back_side=0.0045,
        )
        .extrude(0.352)
        .translate((0.0, 0.001, -0.176))
    )


def _outer_sleeve_solid() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.028).circle(0.0215).extrude(0.180)


def _yoke_collar_solid() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.024).circle(0.0205).extrude(0.048)


def _tripod_leg(angle: float) -> MeshGeometry:
    c = math.cos(angle)
    s = math.sin(angle)
    return tube_from_spline_points(
        [
            (0.040 * c, 0.040 * s, 0.052),
            (0.140 * c, 0.140 * s, 0.032),
            (0.295 * c, 0.295 * s, 0.010),
        ],
        radius=0.011,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def _yoke_frame_mesh() -> MeshGeometry:
    frame = MeshGeometry()
    frame.merge(
        tube_from_spline_points(
            [
                (0.0, -0.021, 0.048),
                (0.0, -0.032, 0.058),
                (0.0, -0.036, 0.066),
            ],
            radius=0.010,
            samples_per_segment=12,
            radial_segments=16,
            cap_ends=True,
        )
    )
    frame.merge(
        tube_from_spline_points(
            [
                (0.0, -0.036, 0.060),
                (0.0, -0.034, 0.100),
                (0.0, -0.020, 0.140),
            ],
            radius=0.014,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        )
    )
    left_arm = tube_from_spline_points(
        [
            (0.0, -0.020, 0.140),
            (-0.140, -0.018, 0.165),
            (-0.275, -0.010, 0.176),
            (-0.390, -0.004, 0.180),
        ],
        radius=0.013,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    right_arm = left_arm.copy().scale(-1.0, 1.0, 1.0)
    frame.merge(left_arm)
    frame.merge(right_arm)
    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_gaming_monitor")

    plastic_black = model.material("plastic_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.17, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.08, 0.13, 0.16, 0.72))
    joystick_black = model.material("joystick_black", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.052, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=dark_metal,
        name="hub",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=dark_metal,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=graphite,
        name="turntable",
    )
    for index, angle in enumerate((math.pi / 2.0, -math.pi / 6.0, 7.0 * math.pi / 6.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        base.visual(
            mesh_from_geometry(_tripod_leg(angle), f"tripod_leg_{index}"),
            material=dark_metal,
            name=f"leg_{index}",
        )
        base.visual(
            Sphere(radius=0.013),
            origin=Origin(xyz=(0.295 * c, 0.295 * s, 0.010)),
            material=plastic_black,
            name=f"foot_{index}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Cylinder(radius=0.034, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=graphite,
        name="swivel_collar",
    )
    carriage.visual(
        mesh_from_cadquery(_outer_sleeve_solid(), "outer_sleeve"),
        material=satin_black,
        name="outer_sleeve",
    )
    carriage.visual(
        Cylinder(radius=0.004, length=0.022),
        origin=Origin(xyz=(0.030, 0.0, 0.122), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="clamp_stem",
    )
    carriage.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.044, 0.0, 0.122)),
        material=plastic_black,
        name="clamp_knob",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.0185, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=dark_metal,
        name="inner_mast",
    )
    column.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=graphite,
        name="head_collar",
    )

    yoke = model.part("yoke")
    yoke.visual(
        mesh_from_cadquery(_yoke_collar_solid(), "yoke_base_collar"),
        origin=Origin(),
        material=graphite,
        name="base_collar",
    )
    yoke.visual(
        mesh_from_geometry(_yoke_frame_mesh(), "rear_yoke_frame"),
        material=satin_black,
        name="frame",
    )
    yoke.visual(
        Box((0.032, 0.022, 0.050)),
        origin=Origin(xyz=(-0.398, -0.008, 0.180)),
        material=satin_black,
        name="arm_block_0",
    )
    yoke.visual(
        Box((0.032, 0.022, 0.050)),
        origin=Origin(xyz=(0.398, -0.008, 0.180)),
        material=satin_black,
        name="arm_block_1",
    )

    display = model.part("display")
    display.visual(
        mesh_from_cadquery(_display_shell_solid(), "display_shell"),
        origin=Origin(xyz=(0.0, 0.060, 0.0)),
        material=plastic_black,
        name="shell",
    )
    display.visual(
        mesh_from_cadquery(_screen_solid(), "screen_panel"),
        origin=Origin(xyz=(0.0, 0.061, 0.0)),
        material=screen_glass,
        name="screen",
    )
    display.visual(
        Box((0.180, 0.032, 0.120)),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=graphite,
        name="rear_hub",
    )
    display.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(xyz=(-0.426, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="trunnion_0",
    )
    display.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(xyz=(0.426, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="trunnion_1",
    )
    display.visual(
        Box((0.336, 0.012, 0.040)),
        origin=Origin(xyz=(-0.258, 0.018, 0.0)),
        material=graphite,
        name="trunnion_bridge_0",
    )
    display.visual(
        Box((0.336, 0.012, 0.040)),
        origin=Origin(xyz=(0.258, 0.018, 0.0)),
        material=graphite,
        name="trunnion_bridge_1",
    )
    display.visual(
        Cylinder(radius=0.0085, length=0.008),
        origin=Origin(xyz=(0.0, 0.060, -0.194)),
        material=graphite,
        name="joystick_socket",
    )

    joystick = model.part("joystick")
    joystick.visual(
        Cylinder(radius=0.0034, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=joystick_black,
        name="stem",
    )
    joystick.visual(
        Cylinder(radius=0.0048, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=joystick_black,
        name="pivot_hub",
    )
    joystick.visual(
        Sphere(radius=0.0065),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=joystick_black,
        name="cap",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.133)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=3.0),
    )
    model.articulation(
        "carriage_to_column",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.16, lower=0.0, upper=0.080),
    )
    model.articulation(
        "column_to_yoke",
        ArticulationType.FIXED,
        parent=column,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
    )
    model.articulation(
        "yoke_to_display",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=math.radians(-7.0),
            upper=math.radians(20.0),
        ),
    )
    model.articulation(
        "display_to_joystick",
        ArticulationType.REVOLUTE,
        parent=display,
        child=joystick,
        origin=Origin(xyz=(0.0, 0.060, -0.194)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=5.0,
            lower=math.radians(-16.0),
            upper=math.radians(16.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carriage = object_model.get_part("carriage")
    column = object_model.get_part("column")
    display = object_model.get_part("display")
    joystick = object_model.get_part("joystick")

    swivel = object_model.get_articulation("base_to_carriage")
    lift = object_model.get_articulation("carriage_to_column")
    tilt = object_model.get_articulation("yoke_to_display")
    nub = object_model.get_articulation("display_to_joystick")

    def _aabb_size(aabb):
        return (
            aabb[1][0] - aabb[0][0],
            aabb[1][1] - aabb[0][1],
            aabb[1][2] - aabb[0][2],
        )

    def _aabb_center(aabb):
        return (
            0.5 * (aabb[0][0] + aabb[1][0]),
            0.5 * (aabb[0][1] + aabb[1][1]),
            0.5 * (aabb[0][2] + aabb[1][2]),
        )

    ctx.expect_within(
        column,
        carriage,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="column stays centered in the sleeve at rest",
    )
    ctx.expect_overlap(
        column,
        carriage,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve",
        min_overlap=0.125,
        name="column remains deeply inserted at rest",
    )

    rest_column_pos = ctx.part_world_position(column)
    lift_limits = lift.motion_limits
    if lift_limits is not None and lift_limits.upper is not None:
        with ctx.pose({lift: lift_limits.upper}):
            ctx.expect_within(
                column,
                carriage,
                axes="xy",
                inner_elem="inner_mast",
                outer_elem="outer_sleeve",
                margin=0.002,
                name="column stays centered at max height",
            )
            ctx.expect_overlap(
                column,
                carriage,
                axes="z",
                elem_a="inner_mast",
                elem_b="outer_sleeve",
                min_overlap=0.045,
                name="column keeps retained insertion at max height",
            )
            high_column_pos = ctx.part_world_position(column)
        ctx.check(
            "column extends upward",
            rest_column_pos is not None
            and high_column_pos is not None
            and high_column_pos[2] > rest_column_pos[2] + 0.070,
            details=f"rest={rest_column_pos}, high={high_column_pos}",
        )

    with ctx.pose({swivel: 0.0}):
        rest_display_bb = ctx.part_world_aabb(display)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_display_bb = ctx.part_world_aabb(display)
    ctx.check(
        "stand swivels the wide display footprint",
        rest_display_bb is not None
        and turned_display_bb is not None
        and _aabb_size(rest_display_bb)[0] > _aabb_size(rest_display_bb)[1] * 5.0
        and _aabb_size(turned_display_bb)[1] > _aabb_size(turned_display_bb)[0] * 5.0,
        details=f"rest={rest_display_bb}, turned={turned_display_bb}",
    )

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt: tilt_limits.lower}):
            forward_screen_bb = ctx.part_element_world_aabb(display, elem="screen")
        with ctx.pose({tilt: tilt_limits.upper}):
            back_screen_bb = ctx.part_element_world_aabb(display, elem="screen")
        ctx.check(
            "display tilts back across its range",
            forward_screen_bb is not None
            and back_screen_bb is not None
            and back_screen_bb[0][1] < forward_screen_bb[0][1] - 0.030,
            details=f"forward={forward_screen_bb}, back={back_screen_bb}",
        )

    nub_limits = nub.motion_limits
    if nub_limits is not None and nub_limits.lower is not None and nub_limits.upper is not None:
        with ctx.pose({nub: nub_limits.lower}):
            low_cap_bb = ctx.part_element_world_aabb(joystick, elem="cap")
        with ctx.pose({nub: nub_limits.upper}):
            high_cap_bb = ctx.part_element_world_aabb(joystick, elem="cap")
        ctx.check(
            "joystick pivots under the bezel",
            low_cap_bb is not None
            and high_cap_bb is not None
            and _aabb_center(high_cap_bb)[1] > _aabb_center(low_cap_bb)[1] + 0.004,
            details=f"low={low_cap_bb}, high={high_cap_bb}",
        )

    return ctx.report()


object_model = build_object_model()

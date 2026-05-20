from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_side_panel_mesh(name: str, *, thickness: float = 0.004):
    outer_profile = [
        (-0.145, 0.035),
        (-0.132, 0.055),
        (-0.100, 0.082),
        (-0.050, 0.092),
        (-0.018, 0.124),
        (0.030, 0.132),
        (0.090, 0.113),
        (0.135, 0.072),
        (0.140, 0.055),
        (0.126, 0.055),
        (0.121, 0.073),
        (0.104, 0.090),
        (0.085, 0.097),
        (0.066, 0.090),
        (0.050, 0.073),
        (0.046, 0.055),
        (-0.060, 0.055),
        (-0.064, 0.073),
        (-0.080, 0.091),
        (-0.100, 0.098),
        (-0.120, 0.091),
        (-0.136, 0.073),
        (-0.140, 0.055),
    ]
    door_opening = [
        (-0.034, 0.055),
        (-0.034, 0.093),
        (-0.012, 0.121),
        (0.015, 0.122),
        (0.044, 0.108),
        (0.052, 0.055),
    ]
    geom = ExtrudeWithHolesGeometry(
        outer_profile,
        [door_opening],
        height=thickness,
        center=True,
    ).rotate_x(pi / 2.0)
    return _save_mesh(name, geom)


def _build_half_door_mesh(name: str, *, thickness: float = 0.004):
    profile = [
        (0.000, -0.013),
        (0.000, 0.018),
        (0.020, 0.019),
        (0.050, 0.015),
        (0.078, 0.008),
        (0.078, -0.013),
    ]
    geom = ExtrudeGeometry(profile, thickness, center=True).rotate_x(pi / 2.0)
    return _save_mesh(name, geom)


def _add_wheel_visuals(
    part,
    mesh_prefix: str,
    *,
    tire_radius: float,
    tire_width: float,
    rubber,
    rim_dark,
    rim_light,
) -> None:
    half_width = tire_width * 0.5
    tire_profile = [
        (tire_radius * 0.58, -half_width * 0.95),
        (tire_radius * 0.78, -half_width),
        (tire_radius * 0.92, -half_width * 0.78),
        (tire_radius, -half_width * 0.28),
        (tire_radius, half_width * 0.28),
        (tire_radius * 0.92, half_width * 0.78),
        (tire_radius * 0.78, half_width),
        (tire_radius * 0.58, half_width * 0.95),
        (tire_radius * 0.46, half_width * 0.38),
        (tire_radius * 0.42, 0.0),
        (tire_radius * 0.46, -half_width * 0.38),
        (tire_radius * 0.58, -half_width * 0.95),
    ]
    tire_mesh = _save_mesh(
        f"{mesh_prefix}_tire",
        LatheGeometry(tire_profile, segments=56).rotate_x(pi / 2.0),
    )
    part.visual(tire_mesh, material=rubber, name="tire")

    tread_radius = tire_radius * 0.90
    for lug_index in range(10):
        angle = 2.0 * pi * lug_index / 10.0
        part.visual(
            Box((0.011, tire_width * 0.92, 0.007)),
            origin=Origin(
                xyz=(tread_radius * cos(angle), 0.0, tread_radius * sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=rubber,
        )

    spin_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius * 0.72, length=tire_width * 0.54),
        origin=spin_origin,
        material=rim_dark,
        name="rim_body",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.56, length=tire_width * 0.42),
        origin=spin_origin,
        material=rim_light,
        name="rim_face",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.20, length=tire_width * 0.94),
        origin=spin_origin,
        material=rim_dark,
        name="hub",
    )
    for side_sign in (-1.0, 1.0):
        part.visual(
            Cylinder(radius=tire_radius * 0.30, length=0.006),
            origin=Origin(
                xyz=(0.0, side_sign * tire_width * 0.18, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=rim_light,
        )

    spoke_length = tire_radius * 0.48
    for spoke_index in range(6):
        angle = 2.0 * pi * spoke_index / 6.0
        radial_x = 0.5 * spoke_length
        part.visual(
            Box((spoke_length, tire_width * 0.14, 0.008)),
            origin=Origin(
                xyz=(radial_x * cos(angle), 0.0, radial_x * sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=rim_light,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_offroad_buggy")

    shell_orange = model.material("shell_orange", rgba=(0.87, 0.39, 0.11, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.19, 0.20, 0.22, 1.0))
    bar_black = model.material("bar_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    rim_dark = model.material("rim_dark", rgba=(0.23, 0.24, 0.26, 1.0))
    rim_light = model.material("rim_light", rgba=(0.71, 0.72, 0.75, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.30, 0.18, 0.14)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    side_panel_mesh = _build_side_panel_mesh("buggy_side_panel")
    door_mesh = _build_half_door_mesh("buggy_half_door")

    body.visual(
        Box((0.215, 0.108, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=frame_gray,
        name="floor_pan",
    )
    body.visual(
        side_panel_mesh,
        origin=Origin(xyz=(0.0, 0.062, 0.0)),
        material=shell_orange,
        name="left_side_panel",
    )
    body.visual(
        side_panel_mesh,
        origin=Origin(xyz=(0.0, -0.062, 0.0)),
        material=shell_orange,
        name="right_side_panel",
    )
    body.visual(
        Box((0.188, 0.014, 0.022)),
        origin=Origin(xyz=(-0.002, 0.058, 0.046)),
        material=frame_gray,
        name="left_sill",
    )
    body.visual(
        Box((0.188, 0.014, 0.022)),
        origin=Origin(xyz=(-0.002, -0.058, 0.046)),
        material=frame_gray,
        name="right_sill",
    )
    body.visual(
        Box((0.090, 0.128, 0.014)),
        origin=Origin(xyz=(-0.096, 0.0, 0.074), rpy=(0.0, -0.22, 0.0)),
        material=shell_orange,
        name="hood_panel",
    )
    body.visual(
        Box((0.026, 0.126, 0.018)),
        origin=Origin(xyz=(-0.042, 0.0, 0.091)),
        material=shell_orange,
        name="cowl_panel",
    )
    body.visual(
        Box((0.036, 0.122, 0.024)),
        origin=Origin(xyz=(-0.138, 0.0, 0.049)),
        material=frame_gray,
        name="nose_block",
    )
    body.visual(
        Box((0.104, 0.126, 0.016)),
        origin=Origin(xyz=(0.089, 0.0, 0.101), rpy=(0.0, 0.10, 0.0)),
        material=shell_orange,
        name="rear_deck",
    )
    body.visual(
        Box((0.026, 0.120, 0.048)),
        origin=Origin(xyz=(0.048, 0.0, 0.082)),
        material=frame_gray,
        name="rear_bulkhead",
    )
    body.visual(
        Box((0.018, 0.112, 0.014)),
        origin=Origin(xyz=(-0.100, 0.0, 0.045)),
        material=frame_gray,
        name="front_axle_beam",
    )
    body.visual(
        Box((0.018, 0.112, 0.014)),
        origin=Origin(xyz=(0.100, 0.0, 0.045)),
        material=frame_gray,
        name="rear_axle_beam",
    )
    for name, xyz in (
        ("front_left_axle_stub", (-0.100, 0.062, 0.045)),
        ("front_right_axle_stub", (-0.100, -0.062, 0.045)),
        ("rear_left_axle_stub", (0.100, 0.062, 0.045)),
        ("rear_right_axle_stub", (0.100, -0.062, 0.045)),
    ):
        body.visual(
            Cylinder(radius=0.010, length=0.01228),
            origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
            material=frame_gray,
            name=name,
        )

    for name, x_pos in (
        ("front_left_fender_cap", -0.100),
        ("rear_left_fender_cap", 0.100),
    ):
        body.visual(
            Box((0.078, 0.028, 0.012)),
            origin=Origin(xyz=(x_pos, 0.076, 0.099)),
            material=shell_orange,
            name=name,
        )
    for name, x_pos in (
        ("front_right_fender_cap", -0.100),
        ("rear_right_fender_cap", 0.100),
    ):
        body.visual(
            Box((0.078, 0.028, 0.012)),
            origin=Origin(xyz=(x_pos, -0.076, 0.099)),
            material=shell_orange,
            name=name,
        )

    body.visual(
        Box((0.008, 0.004, 0.052)),
        origin=Origin(xyz=(-0.034, 0.058, 0.079)),
        material=frame_gray,
        name="left_hinge_jamb",
    )
    body.visual(
        Box((0.008, 0.004, 0.052)),
        origin=Origin(xyz=(-0.034, -0.058, 0.079)),
        material=frame_gray,
        name="right_hinge_jamb",
    )
    body.visual(
        Cylinder(radius=0.0016, length=0.034),
        origin=Origin(xyz=(-0.034, 0.0638, 0.070)),
        material=bar_black,
        name="left_hinge_pin",
    )
    body.visual(
        Cylinder(radius=0.0016, length=0.034),
        origin=Origin(xyz=(-0.034, -0.0638, 0.070)),
        material=bar_black,
        name="right_hinge_pin",
    )

    roof_bar = tube_from_spline_points(
        [(0.000, -0.062, 0.129), (0.004, 0.0, 0.132), (0.000, 0.062, 0.129)],
        radius=0.0055,
        samples_per_segment=12,
        radial_segments=18,
    )
    left_front_pillar = tube_from_spline_points(
        [(-0.030, 0.062, 0.094), (-0.018, 0.062, 0.108), (-0.002, 0.062, 0.129)],
        radius=0.0045,
        samples_per_segment=8,
        radial_segments=16,
    )
    right_front_pillar = tube_from_spline_points(
        [(-0.030, -0.062, 0.094), (-0.018, -0.062, 0.108), (-0.002, -0.062, 0.129)],
        radius=0.0045,
        samples_per_segment=8,
        radial_segments=16,
    )
    left_rear_support = tube_from_spline_points(
        [(0.050, 0.062, 0.105), (0.035, 0.062, 0.118), (0.005, 0.062, 0.129)],
        radius=0.0045,
        samples_per_segment=8,
        radial_segments=16,
    )
    right_rear_support = tube_from_spline_points(
        [(0.050, -0.062, 0.105), (0.035, -0.062, 0.118), (0.005, -0.062, 0.129)],
        radius=0.0045,
        samples_per_segment=8,
        radial_segments=16,
    )
    body.visual(_save_mesh("buggy_roof_bar", roof_bar), material=bar_black, name="roof_bar")
    body.visual(_save_mesh("buggy_left_front_pillar", left_front_pillar), material=bar_black)
    body.visual(_save_mesh("buggy_right_front_pillar", right_front_pillar), material=bar_black)
    body.visual(_save_mesh("buggy_left_rear_support", left_rear_support), material=bar_black)
    body.visual(_save_mesh("buggy_right_rear_support", right_rear_support), material=bar_black)

    left_door = model.part("left_door")
    left_door.visual(door_mesh, material=shell_orange, name="door_panel")
    left_door.visual(
        Cylinder(radius=0.0036, length=0.034),
        origin=Origin(),
        material=bar_black,
        name="hinge_barrel",
    )
    left_door.visual(
        Box((0.016, 0.006, 0.008)),
        origin=Origin(xyz=(0.056, 0.0, -0.002)),
        material=frame_gray,
        name="door_handle",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((0.080, 0.010, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )

    right_door = model.part("right_door")
    right_door.visual(door_mesh, material=shell_orange, name="door_panel")
    right_door.visual(
        Cylinder(radius=0.0036, length=0.034),
        origin=Origin(),
        material=bar_black,
        name="hinge_barrel",
    )
    right_door.visual(
        Box((0.016, 0.006, 0.008)),
        origin=Origin(xyz=(0.056, 0.0, -0.002)),
        material=frame_gray,
        name="door_handle",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((0.080, 0.010, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )

    wheel_specs = [
        ("front_left_wheel", (-0.100, 0.086, 0.045)),
        ("front_right_wheel", (-0.100, -0.086, 0.045)),
        ("rear_left_wheel", (0.100, 0.086, 0.045)),
        ("rear_right_wheel", (0.100, -0.086, 0.045)),
    ]
    for part_name, _ in wheel_specs:
        wheel = model.part(part_name)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.045, length=0.038),
            mass=0.18,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )
        _add_wheel_visuals(
            wheel,
            f"buggy_{part_name}",
            tire_radius=0.045,
            tire_width=0.038,
            rubber=rubber,
            rim_dark=rim_dark,
            rim_light=rim_light,
        )

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(-0.034, 0.069, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.5, lower=0.0, upper=1.10),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(-0.034, -0.069, 0.070)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.5, lower=0.0, upper=1.10),
    )

    for joint_name, part_name, wheel_origin in (
        ("front_left_wheel_spin", "front_left_wheel", (-0.100, 0.086, 0.045)),
        ("front_right_wheel_spin", "front_right_wheel", (-0.100, -0.086, 0.045)),
        ("rear_left_wheel_spin", "rear_left_wheel", (0.100, 0.086, 0.045)),
        ("rear_right_wheel_spin", "rear_right_wheel", (0.100, -0.086, 0.045)),
    ):
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=part_name,
            origin=Origin(xyz=wheel_origin),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    left_door_hinge = object_model.get_articulation("left_door_hinge")
    right_door_hinge = object_model.get_articulation("right_door_hinge")
    wheel_joints = [
        object_model.get_articulation("front_left_wheel_spin"),
        object_model.get_articulation("front_right_wheel_spin"),
        object_model.get_articulation("rear_left_wheel_spin"),
        object_model.get_articulation("rear_right_wheel_spin"),
    ]

    ctx.expect_origin_distance(
        front_left_wheel,
        rear_left_wheel,
        axes="x",
        min_dist=0.18,
        max_dist=0.22,
        name="left wheelbase reads as short off-road buggy",
    )
    ctx.expect_origin_distance(
        front_left_wheel,
        front_right_wheel,
        axes="y",
        min_dist=0.16,
        max_dist=0.18,
        name="front track width matches buggy stance",
    )
    ctx.expect_origin_distance(
        rear_left_wheel,
        rear_right_wheel,
        axes="y",
        min_dist=0.16,
        max_dist=0.18,
        name="rear track width matches buggy stance",
    )

    ctx.expect_gap(
        left_door,
        body,
        axis="y",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem="door_panel",
        negative_elem="left_side_panel",
        name="left half-door sits flush with left opening",
    )
    ctx.expect_gap(
        body,
        right_door,
        axis="y",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem="right_side_panel",
        negative_elem="door_panel",
        name="right half-door sits flush with right opening",
    )

    ctx.check(
        "left door hinge is a front-mounted vertical revolute",
        left_door_hinge.articulation_type == ArticulationType.REVOLUTE
        and left_door_hinge.axis == (0.0, 0.0, 1.0)
        and left_door_hinge.motion_limits is not None
        and left_door_hinge.motion_limits.lower == 0.0
        and left_door_hinge.motion_limits.upper is not None
        and left_door_hinge.motion_limits.upper >= 1.0,
        details=f"axis={left_door_hinge.axis}, limits={left_door_hinge.motion_limits}",
    )
    ctx.check(
        "right door hinge is a mirrored front-mounted vertical revolute",
        right_door_hinge.articulation_type == ArticulationType.REVOLUTE
        and right_door_hinge.axis == (0.0, 0.0, -1.0)
        and right_door_hinge.motion_limits is not None
        and right_door_hinge.motion_limits.lower == 0.0
        and right_door_hinge.motion_limits.upper is not None
        and right_door_hinge.motion_limits.upper >= 1.0,
        details=f"axis={right_door_hinge.axis}, limits={right_door_hinge.motion_limits}",
    )

    for wheel_joint in wheel_joints:
        limits = wheel_joint.motion_limits
        ctx.check(
            f"{wheel_joint.name} is continuous wheel spin",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS
            and wheel_joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}, limits={limits}",
        )

    left_closed = ctx.part_element_world_aabb(left_door, elem="door_panel")
    right_closed = ctx.part_element_world_aabb(right_door, elem="door_panel")
    with ctx.pose({left_door_hinge: 1.0, right_door_hinge: 1.0}):
        left_open = ctx.part_element_world_aabb(left_door, elem="door_panel")
        right_open = ctx.part_element_world_aabb(right_door, elem="door_panel")

    ctx.check(
        "left door swings outward",
        left_closed is not None
        and left_open is not None
        and left_open[1][1] > left_closed[1][1] + 0.020,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right door swings outward",
        right_closed is not None
        and right_open is not None
        and right_open[0][1] < right_closed[0][1] - 0.020,
        details=f"closed={right_closed}, open={right_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

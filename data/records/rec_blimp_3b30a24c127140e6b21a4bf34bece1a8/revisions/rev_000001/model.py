from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _revolved_x(profile: list[tuple[float, float]]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .moveTo(profile[0][0], profile[0][1])
        .spline(profile[1:])
        .close()
        .revolve(360, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    )


def _horizontal_surface(
    *,
    span: float,
    root_chord: float,
    tip_chord: float,
    thickness: float,
    side_sign: float,
) -> cq.Workplane:
    y_mid = side_sign * span * 0.58
    y_tip = side_sign * span
    outline = [
        (0.0, 0.0),
        (-0.16 * root_chord, y_mid),
        (-0.26 * tip_chord, y_tip),
        (-tip_chord, y_tip),
        (-root_chord, 0.0),
    ]
    return (
        cq.Workplane("XY")
        .polyline(outline)
        .close()
        .extrude(thickness, both=True)
        .edges("|Z")
        .fillet(min(thickness * 0.45, 0.02))
    )


def _vertical_root_surface(
    *,
    span: float,
    root_chord: float,
    tip_chord: float,
    thickness: float,
    z_sign: float,
) -> cq.Workplane:
    z_mid = z_sign * span * 0.56
    z_tip = z_sign * span
    outline = [
        (0.0, 0.0),
        (-0.16 * root_chord, z_mid),
        (-0.28 * tip_chord, z_tip),
        (-tip_chord, z_tip),
        (-root_chord, 0.0),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(outline)
        .close()
        .extrude(thickness, both=True)
        .edges("|Y")
        .fillet(min(thickness * 0.45, 0.02))
    )


def _rudder_surface(*, thickness: float) -> cq.Workplane:
    outline = [
        (0.0, 0.92),
        (-0.22, 0.74),
        (-0.96, 0.18),
        (-1.02, 0.0),
        (-0.96, -0.18),
        (-0.22, -0.74),
        (0.0, -0.92),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(outline)
        .close()
        .extrude(thickness, both=True)
        .edges("|Y")
        .fillet(min(thickness * 0.45, 0.025))
    )


def _propeller_shape() -> cq.Workplane:
    hub = cq.Workplane("YZ").circle(0.055).extrude(0.08, both=True)
    blade = (
        cq.Workplane("XY")
        .box(0.018, 0.34, 0.058, centered=(True, False, True))
        .translate((0.0, 0.035, 0.0))
        .edges("|X")
        .fillet(0.012)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 18.0)
    )
    assembly = hub
    for angle in (0.0, 120.0, 240.0):
        assembly = assembly.union(blade.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle))
    return assembly


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sightseeing_blimp")

    envelope_white = model.material("envelope_white", rgba=(0.92, 0.93, 0.90, 1.0))
    cabin_white = model.material("cabin_white", rgba=(0.86, 0.87, 0.85, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.18, 0.32, 0.54, 1.0))
    strut_grey = model.material("strut_grey", rgba=(0.43, 0.46, 0.49, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.30, 0.42, 0.50, 0.72))
    prop_black = model.material("prop_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    hub_silver = model.material("hub_silver", rgba=(0.63, 0.65, 0.68, 1.0))

    hull_profile = [
        (-6.65, 0.0),
        (-6.48, 0.08),
        (-5.92, 0.34),
        (-4.40, 0.94),
        (-1.40, 1.48),
        (1.70, 1.60),
        (4.55, 1.33),
        (6.10, 0.82),
        (6.88, 0.19),
        (7.05, 0.0),
    ]
    hull_shape = _revolved_x(hull_profile)

    cabin_shape = (
        cq.Workplane("XY")
        .box(4.55, 1.34, 1.26)
        .edges("|Z")
        .fillet(0.16)
        .edges(">Z")
        .fillet(0.08)
    )
    cabin_nose = (
        cq.Workplane("XY")
        .box(0.95, 1.18, 0.92)
        .translate((2.20, 0.0, 0.02))
        .edges("|Z")
        .fillet(0.10)
    )
    cabin_tail = (
        cq.Workplane("XY")
        .box(0.70, 1.06, 0.84)
        .translate((-2.25, 0.0, -0.08))
        .edges("|Z")
        .fillet(0.08)
    )
    gondola_shape = cabin_shape.union(cabin_nose).union(cabin_tail)

    left_root_shape = _horizontal_surface(
        span=0.24,
        root_chord=0.42,
        tip_chord=0.16,
        thickness=0.08,
        side_sign=1.0,
    )
    right_root_shape = _horizontal_surface(
        span=0.24,
        root_chord=0.42,
        tip_chord=0.16,
        thickness=0.08,
        side_sign=-1.0,
    )
    dorsal_root_shape = _vertical_root_surface(
        span=0.34,
        root_chord=0.50,
        tip_chord=0.16,
        thickness=0.08,
        z_sign=1.0,
    )
    ventral_root_shape = _vertical_root_surface(
        span=0.34,
        root_chord=0.50,
        tip_chord=0.16,
        thickness=0.08,
        z_sign=-1.0,
    )

    airframe = model.part("airframe")
    airframe.visual(
        mesh_from_cadquery(hull_shape, "blimp_hull"),
        material=envelope_white,
        name="hull_shell",
    )
    airframe.visual(
        mesh_from_cadquery(gondola_shape, "blimp_gondola"),
        origin=Origin(xyz=(0.15, 0.0, -2.22)),
        material=cabin_white,
        name="cabin_shell",
    )
    airframe.visual(
        Box((3.35, 0.14, 0.38)),
        origin=Origin(xyz=(0.20, 0.64, -2.07)),
        material=glass_tint,
        name="left_windows",
    )
    airframe.visual(
        Box((3.35, 0.14, 0.38)),
        origin=Origin(xyz=(0.20, -0.64, -2.07)),
        material=glass_tint,
        name="right_windows",
    )
    airframe.visual(
        Cylinder(radius=0.17, length=0.62),
        origin=Origin(xyz=(0.71, 1.30, -2.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cabin_white,
        name="left_pod",
    )
    airframe.visual(
        Cylinder(radius=0.17, length=0.62),
        origin=Origin(xyz=(0.71, -1.30, -2.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cabin_white,
        name="right_pod",
    )
    airframe.visual(
        Box((0.24, 0.34, 0.34)),
        origin=Origin(xyz=(1.00, 1.30, -2.00)),
        material=cabin_white,
        name="left_pod_nose",
    )
    airframe.visual(
        Box((0.24, 0.34, 0.34)),
        origin=Origin(xyz=(1.00, -1.30, -2.00)),
        material=cabin_white,
        name="right_pod_nose",
    )
    airframe.visual(
        Box((0.18, 0.24, 0.24)),
        origin=Origin(xyz=(0.40, 1.30, -2.00)),
        material=accent_blue,
        name="left_pod_tail",
    )
    airframe.visual(
        Box((0.18, 0.24, 0.24)),
        origin=Origin(xyz=(0.40, -1.30, -2.00)),
        material=accent_blue,
        name="right_pod_tail",
    )
    airframe.visual(
        mesh_from_cadquery(dorsal_root_shape, "dorsal_root"),
        origin=Origin(xyz=(-5.94, 0.0, 0.46)),
        material=envelope_white,
        name="dorsal_root",
    )
    airframe.visual(
        mesh_from_cadquery(ventral_root_shape, "ventral_root"),
        origin=Origin(xyz=(-5.94, 0.0, -0.46)),
        material=envelope_white,
        name="ventral_root",
    )
    airframe.visual(
        mesh_from_cadquery(left_root_shape, "left_tail_root"),
        origin=Origin(xyz=(-5.94, 0.42, 0.0)),
        material=envelope_white,
        name="left_tail_root",
    )
    airframe.visual(
        mesh_from_cadquery(right_root_shape, "right_tail_root"),
        origin=Origin(xyz=(-5.94, -0.42, 0.0)),
        material=envelope_white,
        name="right_tail_root",
    )

    for x_pos in (1.65, 0.35, -1.25):
        _add_member(
            airframe,
            (x_pos + 0.10, 0.26, -1.46 if x_pos > 0.0 else -1.40),
            (x_pos, 0.26, -1.61),
            radius=0.05,
            material=strut_grey,
        )
        _add_member(
            airframe,
            (x_pos + 0.10, -0.26, -1.46 if x_pos > 0.0 else -1.40),
            (x_pos, -0.26, -1.61),
            radius=0.05,
            material=strut_grey,
        )

    _add_member(
        airframe,
        (0.42, 0.62, -1.95),
        (0.56, 1.14, -2.00),
        radius=0.085,
        material=strut_grey,
        name="left_pod_boom",
    )
    _add_member(
        airframe,
        (0.42, -0.62, -1.95),
        (0.56, -1.14, -2.00),
        radius=0.085,
        material=strut_grey,
        name="right_pod_boom",
    )
    _add_member(
        airframe,
        (0.04, 0.54, -1.74),
        (0.28, 1.03, -1.92),
        radius=0.032,
        material=strut_grey,
    )
    _add_member(
        airframe,
        (0.04, -0.54, -1.74),
        (0.28, -1.03, -1.92),
        radius=0.032,
        material=strut_grey,
    )
    _add_member(
        airframe,
        (-6.10, 0.22, 0.00),
        (-5.96, 0.42, 0.00),
        radius=0.07,
        material=strut_grey,
        name="left_tail_mount",
    )
    _add_member(
        airframe,
        (-6.10, -0.22, 0.00),
        (-5.96, -0.42, 0.00),
        radius=0.07,
        material=strut_grey,
        name="right_tail_mount",
    )
    _add_member(
        airframe,
        (-6.10, 0.00, 0.20),
        (-5.98, 0.00, 0.48),
        radius=0.07,
        material=strut_grey,
        name="dorsal_mount",
    )
    _add_member(
        airframe,
        (-6.10, 0.00, -0.20),
        (-5.98, 0.00, -0.48),
        radius=0.07,
        material=strut_grey,
        name="ventral_mount",
    )

    airframe.visual(
        Box((0.16, 0.14, 0.12)),
        origin=Origin(xyz=(-2.22, 0.0, -3.02)),
        material=strut_grey,
        name="fork_crown",
    )
    _add_member(
        airframe,
        (-2.05, 0.0, -2.78),
        (-2.20, 0.0, -2.98),
        radius=0.05,
        material=strut_grey,
        name="wheel_post",
    )
    _add_member(
        airframe,
        (-2.22, 0.06, -3.02),
        (-2.46, 0.07, -3.18),
        radius=0.022,
        material=strut_grey,
        name="fork_arm_0",
    )
    _add_member(
        airframe,
        (-2.22, -0.06, -3.02),
        (-2.46, -0.07, -3.18),
        radius=0.022,
        material=strut_grey,
        name="fork_arm_1",
    )
    _add_member(
        airframe,
        (-2.46, 0.07, -3.18),
        (-2.46, 0.035, -3.18),
        radius=0.006,
        material=strut_grey,
        name="fork_stub_0",
    )
    _add_member(
        airframe,
        (-2.46, -0.07, -3.18),
        (-2.46, -0.035, -3.18),
        radius=0.006,
        material=strut_grey,
        name="fork_stub_1",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((14.20, 3.30, 6.40)),
        mass=2800.0,
        origin=Origin(xyz=(0.0, 0.0, -0.55)),
    )

    propeller_mesh = mesh_from_cadquery(_propeller_shape(), "propeller")

    left_propeller = model.part("left_propeller")
    left_propeller.visual(
        propeller_mesh,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        material=prop_black,
        name="propeller_assembly",
    )
    left_propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.18),
        mass=10.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_propeller = model.part("right_propeller")
    right_propeller.visual(
        propeller_mesh,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        material=prop_black,
        name="propeller_assembly",
    )
    right_propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.18),
        mass=10.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    rudder = model.part("rudder")
    rudder.visual(
        mesh_from_cadquery(_rudder_surface(thickness=0.11), "rudder_surface"),
        material=envelope_white,
        name="rudder_surface",
    )
    rudder.inertial = Inertial.from_geometry(
        Box((1.05, 0.12, 1.90)),
        mass=24.0,
        origin=Origin(xyz=(-0.50, 0.0, 0.0)),
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        mesh_from_cadquery(
            _horizontal_surface(
                span=1.18,
                root_chord=0.96,
                tip_chord=0.72,
                thickness=0.10,
                side_sign=1.0,
            ),
            "left_elevator_surface",
        ),
        material=envelope_white,
        name="left_elevator_surface",
    )
    left_elevator.inertial = Inertial.from_geometry(
        Box((0.96, 1.18, 0.10)),
        mass=18.0,
        origin=Origin(xyz=(-0.48, 0.59, 0.0)),
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        mesh_from_cadquery(
            _horizontal_surface(
                span=1.18,
                root_chord=0.96,
                tip_chord=0.72,
                thickness=0.10,
                side_sign=-1.0,
            ),
            "right_elevator_surface",
        ),
        material=envelope_white,
        name="right_elevator_surface",
    )
    right_elevator.inertial = Inertial.from_geometry(
        Box((0.96, 1.18, 0.10)),
        mass=18.0,
        origin=Origin(xyz=(-0.48, -0.59, 0.0)),
    )

    mooring_wheel = model.part("mooring_wheel")
    mooring_wheel.visual(
        Cylinder(radius=0.18, length=0.07),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="wheel_tire",
    )
    mooring_wheel.visual(
        Cylinder(radius=0.075, length=0.060),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_silver,
        name="wheel_hub",
    )
    mooring_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.07),
        mass=8.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "airframe_to_left_propeller",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=left_propeller,
        origin=Origin(xyz=(1.02, 1.30, -2.00)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=30.0),
    )
    model.articulation(
        "airframe_to_right_propeller",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=right_propeller,
        origin=Origin(xyz=(1.02, -1.30, -2.00)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=30.0),
    )
    model.articulation(
        "airframe_to_rudder",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=rudder,
        origin=Origin(xyz=(-6.48, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.2,
            lower=-math.radians(35.0),
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "airframe_to_left_elevator",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=left_elevator,
        origin=Origin(xyz=(-6.323, 0.42, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.2,
            lower=-math.radians(22.0),
            upper=math.radians(22.0),
        ),
    )
    model.articulation(
        "airframe_to_right_elevator",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=right_elevator,
        origin=Origin(xyz=(-6.323, -0.42, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.2,
            lower=-math.radians(22.0),
            upper=math.radians(22.0),
        ),
    )
    model.articulation(
        "airframe_to_mooring_wheel",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=mooring_wheel,
        origin=Origin(xyz=(-2.46, 0.0, -3.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=15.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    airframe = object_model.get_part("airframe")
    left_propeller = object_model.get_part("left_propeller")
    right_propeller = object_model.get_part("right_propeller")
    rudder = object_model.get_part("rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")
    mooring_wheel = object_model.get_part("mooring_wheel")

    rudder_joint = object_model.get_articulation("airframe_to_rudder")
    left_elevator_joint = object_model.get_articulation("airframe_to_left_elevator")
    right_elevator_joint = object_model.get_articulation("airframe_to_right_elevator")

    ctx.expect_gap(
        left_propeller,
        airframe,
        axis="x",
        positive_elem="propeller_assembly",
        negative_elem="left_pod_nose",
        max_gap=0.04,
        max_penetration=1e-5,
        name="left propeller sits just ahead of left pod",
    )
    ctx.expect_gap(
        right_propeller,
        airframe,
        axis="x",
        positive_elem="propeller_assembly",
        negative_elem="right_pod_nose",
        max_gap=0.04,
        max_penetration=1e-5,
        name="right propeller sits just ahead of right pod",
    )
    ctx.expect_gap(
        airframe,
        mooring_wheel,
        axis="z",
        positive_elem="cabin_shell",
        negative_elem="wheel_tire",
        min_gap=0.05,
        max_gap=0.45,
        name="mooring wheel hangs below the gondola cabin",
    )

    rudder_limits = rudder_joint.motion_limits
    if rudder_limits is not None and rudder_limits.upper is not None:
        rest_aabb = ctx.part_element_world_aabb(rudder, elem="rudder_surface")
        with ctx.pose({rudder_joint: rudder_limits.upper}):
            deflected_aabb = ctx.part_element_world_aabb(rudder, elem="rudder_surface")
        rest_center_y = (
            (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5 if rest_aabb is not None else None
        )
        deflected_center_y = (
            (deflected_aabb[0][1] + deflected_aabb[1][1]) * 0.5
            if deflected_aabb is not None
            else None
        )
        ctx.check(
            "rudder deflects sideways",
            rest_center_y is not None
            and deflected_center_y is not None
            and abs(deflected_center_y - rest_center_y) > 0.08,
            details=f"rest_center_y={rest_center_y}, deflected_center_y={deflected_center_y}",
        )

    left_limits = left_elevator_joint.motion_limits
    if left_limits is not None and left_limits.upper is not None:
        rest_aabb = ctx.part_element_world_aabb(left_elevator, elem="left_elevator_surface")
        with ctx.pose({left_elevator_joint: left_limits.upper}):
            raised_aabb = ctx.part_element_world_aabb(left_elevator, elem="left_elevator_surface")
        rest_center_z = (
            (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5 if rest_aabb is not None else None
        )
        raised_center_z = (
            (raised_aabb[0][2] + raised_aabb[1][2]) * 0.5 if raised_aabb is not None else None
        )
        ctx.check(
            "left elevator pitches upward",
            rest_center_z is not None
            and raised_center_z is not None
            and raised_center_z > rest_center_z + 0.04,
            details=f"rest_center_z={rest_center_z}, raised_center_z={raised_center_z}",
        )

    right_limits = right_elevator_joint.motion_limits
    if right_limits is not None and right_limits.upper is not None:
        rest_aabb = ctx.part_element_world_aabb(right_elevator, elem="right_elevator_surface")
        with ctx.pose({right_elevator_joint: right_limits.upper}):
            raised_aabb = ctx.part_element_world_aabb(right_elevator, elem="right_elevator_surface")
        rest_center_z = (
            (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5 if rest_aabb is not None else None
        )
        raised_center_z = (
            (raised_aabb[0][2] + raised_aabb[1][2]) * 0.5 if raised_aabb is not None else None
        )
        ctx.check(
            "right elevator pitches upward",
            rest_center_z is not None
            and raised_center_z is not None
            and raised_center_z > rest_center_z + 0.04,
            details=f"rest_center_z={rest_center_z}, raised_center_z={raised_center_z}",
        )

    return ctx.report()


object_model = build_object_model()

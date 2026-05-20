from __future__ import annotations

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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


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


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_cockpit_door(part, *, shell, glass) -> None:
    part.visual(
        Box((1.20, 0.045, 1.26)),
        origin=Origin(xyz=(-0.60, 0.0, 0.63)),
        material=shell,
        name="door_panel",
    )
    part.visual(
        Box((0.70, 0.016, 0.50)),
        origin=Origin(xyz=(-0.58, 0.016, 0.87)),
        material=glass,
        name="window",
    )
    for index, z_center in enumerate((0.16, 0.63, 1.10)):
        part.visual(
            Cylinder(radius=0.017, length=0.12),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=shell,
            name=f"hinge_barrel_{index}",
        )
    part.visual(
        Box((0.06, 0.02, 0.10)),
        origin=Origin(xyz=(-0.96, -0.02, 0.60)),
        material=shell,
        name="handle",
    )


def _add_cabin_door(part, *, shell, glass, track_material) -> None:
    part.visual(
        Box((1.58, 0.040, 1.30)),
        origin=Origin(xyz=(-0.79, 0.0, 0.65)),
        material=shell,
        name="door_panel",
    )
    part.visual(
        Box((0.86, 0.016, 0.56)),
        origin=Origin(xyz=(-0.82, 0.014, 0.92)),
        material=glass,
        name="window",
    )
    part.visual(
        Box((1.66, 0.060, 0.06)),
        origin=Origin(xyz=(-0.83, 0.0, 1.35)),
        material=track_material,
        name="upper_hanger",
    )
    part.visual(
        Box((0.12, 0.05, 0.10)),
        origin=Origin(xyz=(-0.18, 0.0, 1.28)),
        material=track_material,
        name="front_roller",
    )
    part.visual(
        Box((0.12, 0.05, 0.10)),
        origin=Origin(xyz=(-1.38, 0.0, 1.28)),
        material=track_material,
        name="rear_roller",
    )
    part.visual(
        Box((0.08, 0.04, 0.16)),
        origin=Origin(xyz=(-0.18, 0.0, 0.18)),
        material=track_material,
        name="lower_guide",
    )


def _add_main_blade(part, *, angle: float, body, tip_color, index: int) -> None:
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    root_span = 0.65
    blade_span = 3.55

    part.visual(
        Box((root_span, 0.28, 0.06)),
        origin=Origin(
            xyz=(cos_a * 0.46, sin_a * 0.46, 0.24),
            rpy=(0.0, 0.0, angle),
        ),
        material=body,
        name=f"root_{index}",
    )
    part.visual(
        Box((blade_span, 0.18, 0.045)),
        origin=Origin(
            xyz=(cos_a * 2.55, sin_a * 2.55, 0.24),
            rpy=(0.0, 0.0, angle),
        ),
        material=body,
        name=f"blade_{index}",
    )
    part.visual(
        Box((0.38, 0.14, 0.040)),
        origin=Origin(
            xyz=(cos_a * 4.50, sin_a * 4.50, 0.24),
            rpy=(0.0, 0.0, angle),
        ),
        material=tip_color,
        name=f"tip_{index}",
    )


def _add_tail_blade(part, *, angle: float, body, index: int) -> None:
    radius = 0.43
    part.visual(
        Box((0.04, 0.78, 0.10)),
        origin=Origin(
            xyz=(0.0, math.cos(angle) * radius, math.sin(angle) * radius),
            rpy=(angle, 0.0, 0.0),
        ),
        material=body,
        name=f"blade_{index}",
    )
    part.visual(
        Box((0.08, 0.20, 0.12)),
        origin=Origin(
            xyz=(0.0, math.cos(angle) * 0.13, math.sin(angle) * 0.13),
            rpy=(angle, 0.0, 0.0),
        ),
        material=body,
        name=f"cuff_{index}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offshore_transport_helicopter")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.93, 0.42, 0.11, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.22, 0.25, 1.0))
    metal = model.material("metal", rgba=(0.61, 0.63, 0.66, 1.0))
    tire_black = model.material("tire_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.48, 0.69, 0.78, 0.42))
    rotor_gray = model.material("rotor_gray", rgba=(0.24, 0.25, 0.27, 1.0))

    airframe = model.part("airframe")
    airframe.visual(
        Box((4.00, 0.60, 1.25)),
        origin=Origin(xyz=(0.10, 0.0, 0.82)),
        material=dark_gray,
        name="cabin_core",
    )
    airframe.visual(
        Box((4.20, 2.10, 0.24)),
        origin=Origin(xyz=(0.10, 0.0, 0.12)),
        material=body_white,
        name="cabin_floor",
    )
    airframe.visual(
        Box((4.00, 2.00, 0.26)),
        origin=Origin(xyz=(0.15, 0.0, 1.73)),
        material=body_white,
        name="cabin_roof",
    )
    airframe.visual(
        Box((1.55, 1.70, 0.46)),
        origin=Origin(xyz=(-0.25, 0.0, 1.98)),
        material=body_white,
        name="engine_cowling",
    )
    airframe.visual(
        Box((1.20, 1.05, 0.55)),
        origin=Origin(xyz=(-0.15, 0.0, 2.23)),
        material=dark_gray,
        name="mast_fairing",
    )
    airframe.visual(
        Cylinder(radius=0.10, length=0.70),
        origin=Origin(xyz=(-0.10, 0.0, 2.20)),
        material=metal,
        name="mast",
    )

    airframe.visual(
        Box((2.10, 1.65, 0.70)),
        origin=Origin(xyz=(3.05, 0.0, 0.56)),
        material=body_white,
        name="nose_lower",
    )
    airframe.visual(
        Box((1.65, 1.92, 0.54)),
        origin=Origin(xyz=(3.15, 0.0, 1.18)),
        material=body_white,
        name="nose_upper",
    )
    airframe.visual(
        Sphere(radius=0.57),
        origin=Origin(xyz=(4.20, 0.0, 0.95)),
        material=safety_orange,
        name="nose_radome",
    )
    airframe.visual(
        Box((1.10, 1.02, 0.30)),
        origin=Origin(xyz=(3.42, 0.0, 0.28)),
        material=safety_orange,
        name="nose_chin",
    )
    airframe.visual(
        Box((0.72, 1.20, 0.52)),
        origin=Origin(xyz=(3.84, 0.0, 1.28)),
        material=glass,
        name="windscreen",
    )

    for side_name, y in (("port", 1.03), ("starboard", -1.03)):
        airframe.visual(
            Box((0.40, 0.14, 1.10)),
            origin=Origin(xyz=(3.52, y, 0.87)),
            material=body_white,
            name=f"{side_name}_nose_post",
        )
        airframe.visual(
            Box((0.14, 0.14, 1.18)),
            origin=Origin(xyz=(1.92, y, 0.88)),
            material=body_white,
            name=f"{side_name}_cockpit_pillar",
        )
        airframe.visual(
            Box((1.32, 0.16, 0.30)),
            origin=Origin(xyz=(2.61, y, 0.15)),
            material=body_white,
            name=f"{side_name}_cockpit_sill",
        )
        airframe.visual(
            Box((1.32, 0.16, 0.28)),
            origin=Origin(xyz=(2.61, y, 1.60)),
            material=body_white,
            name=f"{side_name}_cockpit_header",
        )

    airframe.visual(
        Box((3.00, 0.14, 1.50)),
        origin=Origin(xyz=(0.10, 1.03, 0.88)),
        material=body_white,
        name="port_cabin_wall",
    )
    airframe.visual(
        Box((1.58, 0.16, 0.32)),
        origin=Origin(xyz=(0.93, -1.04, 0.16)),
        material=body_white,
        name="starboard_cabin_sill",
    )
    airframe.visual(
        Box((1.58, 0.16, 0.28)),
        origin=Origin(xyz=(0.93, -1.03, 1.60)),
        material=body_white,
        name="starboard_cabin_header",
    )
    airframe.visual(
        Box((0.18, 0.16, 1.10)),
        origin=Origin(xyz=(0.02, -1.04, 0.86)),
        material=body_white,
        name="starboard_cabin_rear_pillar",
    )
    airframe.visual(
        Box((2.20, 0.14, 1.05)),
        origin=Origin(xyz=(-1.05, -1.03, 0.85)),
        material=body_white,
        name="starboard_aft_wall",
    )
    airframe.visual(
        Box((2.30, 0.08, 0.08)),
        origin=Origin(xyz=(0.60, -1.135, 1.66)),
        material=metal,
        name="starboard_cabin_track",
    )
    airframe.visual(
        Box((1.92, 0.03, 0.04)),
        origin=Origin(xyz=(0.72, -1.08, 0.18)),
        material=metal,
        name="starboard_cabin_guide",
    )

    airframe.visual(
        Box((1.25, 0.04, 0.50)),
        origin=Origin(xyz=(0.85, 1.09, 1.00)),
        material=glass,
        name="port_cabin_window",
    )
    airframe.visual(
        Box((0.88, 0.02, 0.44)),
        origin=Origin(xyz=(-1.18, -1.11, 0.98)),
        material=glass,
        name="starboard_aft_window",
    )

    airframe.visual(
        Cylinder(radius=0.32, length=4.80),
        origin=Origin(xyz=(-4.00, 0.0, 1.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_white,
        name="tail_boom",
    )
    airframe.visual(
        Box((2.80, 0.60, 0.28)),
        origin=Origin(xyz=(-2.80, 0.0, 1.44)),
        material=body_white,
        name="boom_spine",
    )
    airframe.visual(
        Cylinder(radius=0.20, length=1.20),
        origin=Origin(xyz=(-6.25, 0.0, 1.16), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=safety_orange,
        name="tail_tip",
    )
    airframe.visual(
        Box((1.10, 0.22, 1.50)),
        origin=Origin(xyz=(-5.90, 0.0, 1.95)),
        material=body_white,
        name="fin",
    )
    airframe.visual(
        Box((0.70, 0.18, 0.55)),
        origin=Origin(xyz=(-6.15, 0.0, 2.68)),
        material=safety_orange,
        name="fin_cap",
    )
    airframe.visual(
        Box((0.42, 0.30, 0.32)),
        origin=Origin(xyz=(-6.28, 0.0, 2.30)),
        material=dark_gray,
        name="tail_gearbox",
    )
    airframe.visual(
        Box((0.95, 0.10, 0.24)),
        origin=Origin(xyz=(-5.60, 0.44, 1.22)),
        material=body_white,
        name="port_stabilizer",
    )
    airframe.visual(
        Box((0.95, 0.10, 0.24)),
        origin=Origin(xyz=(-5.60, -0.44, 1.22)),
        material=body_white,
        name="starboard_stabilizer",
    )
    _add_member(
        airframe,
        (-5.86, 0.06, 1.33),
        (-5.74, 0.39, 1.23),
        0.05,
        body_white,
    )
    _add_member(
        airframe,
        (-5.86, -0.06, 1.33),
        (-5.74, -0.39, 1.23),
        0.05,
        body_white,
    )

    airframe.visual(
        Box((1.40, 0.40, 0.55)),
        origin=Origin(xyz=(-0.95, 1.25, 0.28)),
        material=dark_gray,
        name="port_sponson",
    )
    airframe.visual(
        Box((1.40, 0.40, 0.55)),
        origin=Origin(xyz=(-0.95, -1.25, 0.28)),
        material=dark_gray,
        name="starboard_sponson",
    )
    airframe.visual(
        Box((0.80, 0.55, 0.42)),
        origin=Origin(xyz=(2.85, 0.0, 0.18)),
        material=dark_gray,
        name="nose_gear_bay",
    )

    for side_name, y in (("port", 1.33), ("starboard", -1.33)):
        _add_member(
            airframe,
            (-0.95, y, 0.02),
            (-0.95, y, -0.32),
            0.05,
            metal,
        )
        airframe.visual(
            Cylinder(radius=0.30, length=0.14),
            origin=Origin(xyz=(-0.95, y, -0.62), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=tire_black,
            name=f"{side_name}_main_wheel",
        )

    _add_member(airframe, (3.00, 0.0, 0.05), (3.00, 0.0, -0.45), 0.05, metal)
    airframe.visual(
        Cylinder(radius=0.040, length=0.52),
        origin=Origin(xyz=(3.00, 0.0, -0.48), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="nose_axle",
    )
    airframe.visual(
        Box((0.03, 0.025, 1.52)),
        origin=Origin(xyz=(3.21, 1.10, 0.76)),
        material=metal,
        name="port_hinge_post",
    )
    airframe.visual(
        Box((0.03, 0.025, 1.52)),
        origin=Origin(xyz=(3.21, -1.10, 0.76)),
        material=metal,
        name="starboard_hinge_post",
    )
    for index, y in enumerate((-0.21, 0.21)):
        airframe.visual(
            Cylinder(radius=0.22, length=0.10),
            origin=Origin(xyz=(3.00, y, -0.68), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=tire_black,
            name=f"nose_wheel_{index}",
        )

    port_cockpit_door = model.part("port_cockpit_door")
    _add_cockpit_door(port_cockpit_door, shell=body_white, glass=glass)

    starboard_cockpit_door = model.part("starboard_cockpit_door")
    _add_cockpit_door(starboard_cockpit_door, shell=body_white, glass=glass)

    starboard_cabin_door = model.part("starboard_cabin_door")
    _add_cabin_door(
        starboard_cabin_door,
        shell=body_white,
        glass=glass,
        track_material=metal,
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.12, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=metal,
        name="mast_collar",
    )
    main_rotor.visual(
        Cylinder(radius=0.24, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=rotor_gray,
        name="hub",
    )
    main_rotor.visual(
        Box((0.34, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=rotor_gray,
        name="hub_cap",
    )
    for blade_index in range(5):
        _add_main_blade(
            main_rotor,
            angle=blade_index * (2.0 * math.pi / 5.0),
            body=rotor_gray,
            tip_color=safety_orange,
            index=blade_index,
        )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.09, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hub",
    )
    tail_rotor.visual(
        Cylinder(radius=0.03, length=0.11),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="shaft",
    )
    tail_rotor.visual(
        Box((0.12, 0.14, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rotor_gray,
        name="hub_block",
    )
    for blade_index in range(4):
        _add_tail_blade(
            tail_rotor,
            angle=blade_index * (math.pi / 2.0),
            body=rotor_gray,
            index=blade_index,
        )

    model.articulation(
        "port_cockpit_door_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=port_cockpit_door,
        origin=Origin(xyz=(3.21, 1.135, 0.28)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.3,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "starboard_cockpit_door_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=starboard_cockpit_door,
        origin=Origin(xyz=(3.21, -1.135, 0.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.3,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "starboard_cabin_door_slide",
        ArticulationType.PRISMATIC,
        parent=airframe,
        child=starboard_cabin_door,
        origin=Origin(xyz=(1.72, -1.145, 0.24)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.9,
            lower=0.0,
            upper=1.28,
        ),
    )
    model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=main_rotor,
        origin=Origin(xyz=(-0.10, 0.0, 2.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=400.0, velocity=35.0),
    )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=tail_rotor,
        origin=Origin(xyz=(-6.60, 0.0, 2.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=55.0),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    airframe = object_model.get_part("airframe")
    port_cockpit_door = object_model.get_part("port_cockpit_door")
    starboard_cockpit_door = object_model.get_part("starboard_cockpit_door")
    starboard_cabin_door = object_model.get_part("starboard_cabin_door")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")

    port_hinge = object_model.get_articulation("port_cockpit_door_hinge")
    starboard_hinge = object_model.get_articulation("starboard_cockpit_door_hinge")
    cabin_slide = object_model.get_articulation("starboard_cabin_door_slide")
    main_spin = object_model.get_articulation("main_rotor_spin")
    tail_spin = object_model.get_articulation("tail_rotor_spin")

    ctx.expect_gap(
        main_rotor,
        airframe,
        axis="z",
        positive_elem="mast_collar",
        negative_elem="mast",
        max_gap=0.002,
        max_penetration=1e-5,
        name="main rotor collar seats on mast top",
    )

    port_closed = _aabb_center(
        ctx.part_element_world_aabb(port_cockpit_door, elem="door_panel")
    )
    starboard_closed = _aabb_center(
        ctx.part_element_world_aabb(starboard_cockpit_door, elem="door_panel")
    )
    cabin_closed = _aabb_center(
        ctx.part_element_world_aabb(starboard_cabin_door, elem="door_panel")
    )
    main_blade_closed = _aabb_center(
        ctx.part_element_world_aabb(main_rotor, elem="blade_0")
    )
    tail_blade_closed = _aabb_center(
        ctx.part_element_world_aabb(tail_rotor, elem="blade_0")
    )

    with ctx.pose({port_hinge: 0.95}):
        port_open = _aabb_center(
            ctx.part_element_world_aabb(port_cockpit_door, elem="door_panel")
        )
    ctx.check(
        "port cockpit door opens outward",
        port_closed is not None
        and port_open is not None
        and port_open[1] > port_closed[1] + 0.20,
        details=f"closed={port_closed}, open={port_open}",
    )

    with ctx.pose({starboard_hinge: 0.95}):
        starboard_open = _aabb_center(
            ctx.part_element_world_aabb(starboard_cockpit_door, elem="door_panel")
        )
    ctx.check(
        "starboard cockpit door opens outward",
        starboard_closed is not None
        and starboard_open is not None
        and starboard_open[1] < starboard_closed[1] - 0.20,
        details=f"closed={starboard_closed}, open={starboard_open}",
    )

    with ctx.pose({cabin_slide: 1.20}):
        cabin_open = _aabb_center(
            ctx.part_element_world_aabb(starboard_cabin_door, elem="door_panel")
        )
    ctx.check(
        "cabin door slides aft along the rail",
        cabin_closed is not None
        and cabin_open is not None
        and cabin_open[0] < cabin_closed[0] - 1.0,
        details=f"closed={cabin_closed}, open={cabin_open}",
    )

    with ctx.pose({main_spin: 0.70}):
        main_blade_spun = _aabb_center(
            ctx.part_element_world_aabb(main_rotor, elem="blade_0")
        )
    ctx.check(
        "main rotor spins about mast axis",
        main_blade_closed is not None
        and main_blade_spun is not None
        and abs(main_blade_spun[1] - main_blade_closed[1]) > 1.5
        and abs(main_blade_spun[2] - main_blade_closed[2]) < 0.05,
        details=f"closed={main_blade_closed}, spun={main_blade_spun}",
    )

    with ctx.pose({tail_spin: 0.80}):
        tail_blade_spun = _aabb_center(
            ctx.part_element_world_aabb(tail_rotor, elem="blade_0")
        )
    ctx.check(
        "tail rotor spins on the tail axis",
        tail_blade_closed is not None
        and tail_blade_spun is not None
        and abs(tail_blade_spun[2] - tail_blade_closed[2]) > 0.20
        and abs(tail_blade_spun[0] - tail_blade_closed[0]) < 0.05,
        details=f"closed={tail_blade_closed}, spun={tail_blade_spun}",
    )

    return ctx.report()


object_model = build_object_model()

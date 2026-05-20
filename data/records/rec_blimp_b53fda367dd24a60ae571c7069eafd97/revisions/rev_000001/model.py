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
    mesh_from_geometry,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ellipse_section(x: float, ry: float, rz: float, *, segments: int = 32) -> list[tuple[float, float, float]]:
    return [
        (x, ry * math.cos(angle), rz * math.sin(angle))
        for angle in [2.0 * math.pi * i / segments for i in range(segments)]
    ]


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


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_propeller(model: ArticulatedObject, name: str, blade_material, hub_material):
    propeller = model.part(name)
    propeller.visual(
        Cylinder(radius=0.024, length=0.09),
        origin=Origin(xyz=(-0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_material,
        name="shaft",
    )
    propeller.visual(
        Cylinder(radius=0.06, length=0.12),
        origin=Origin(xyz=(0.03, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    propeller.visual(
        Sphere(radius=0.05),
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
        material=hub_material,
        name="spinner",
    )
    propeller.visual(
        Box((0.02, 0.82, 0.12)),
        material=blade_material,
        name="blade_y",
    )
    propeller.visual(
        Box((0.02, 0.12, 0.82)),
        material=blade_material,
        name="blade_z",
    )
    return propeller


def _build_wheel(model: ArticulatedObject, name: str, tire_material, hub_material):
    wheel = model.part(name)
    wheel.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=tire_material,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.065, length=0.08),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surveillance_blimp")

    envelope_white = model.material("envelope_white", rgba=(0.93, 0.94, 0.95, 1.0))
    light_gray = model.material("light_gray", rgba=(0.73, 0.76, 0.79, 1.0))
    structure_gray = model.material("structure_gray", rgba=(0.43, 0.46, 0.50, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.40, 0.56, 0.67, 0.50))
    safety_orange = model.material("safety_orange", rgba=(0.82, 0.45, 0.14, 1.0))

    hull = model.part("hull")
    envelope_geom = section_loft(
        [
            _ellipse_section(-8.90, 0.04, 0.04),
            _ellipse_section(-7.70, 0.82, 0.82),
            _ellipse_section(-4.80, 1.65, 1.68),
            _ellipse_section(-0.40, 2.08, 2.10),
            _ellipse_section(3.20, 2.00, 2.02),
            _ellipse_section(6.80, 1.10, 1.10),
            _ellipse_section(8.95, 0.06, 0.06),
        ]
    )
    hull.visual(
        _save_mesh("blimp_envelope", envelope_geom),
        material=envelope_white,
        name="envelope",
    )

    gondola = model.part("gondola")
    gondola.visual(
        Box((0.72, 0.54, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=structure_gray,
        name="roof_saddle",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            _add_member(
                gondola,
                (0.24 * x_sign, 0.18 * y_sign, -0.08),
                (0.54 * x_sign, 0.30 * y_sign, -0.40),
                radius=0.040,
                material=structure_gray,
            )
    gondola.visual(
        Box((2.10, 0.96, 0.72)),
        origin=Origin(xyz=(0.0, 0.0, -0.74)),
        material=light_gray,
        name="cabin",
    )
    gondola.visual(
        Cylinder(radius=0.42, length=0.84),
        origin=Origin(xyz=(1.42, 0.0, -0.74), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_gray,
        name="nose_fairing",
    )
    gondola.visual(
        Cylinder(radius=0.30, length=0.58),
        origin=Origin(xyz=(-1.34, 0.0, -0.74), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_gray,
        name="tail_fairing",
    )
    gondola.visual(
        Box((0.90, 0.04, 0.28)),
        origin=Origin(xyz=(0.36, 0.50, -0.70)),
        material=glass_blue,
        name="port_window",
    )
    gondola.visual(
        Box((0.90, 0.04, 0.28)),
        origin=Origin(xyz=(0.36, -0.50, -0.70)),
        material=glass_blue,
        name="starboard_window",
    )
    gondola.visual(
        Box((0.16, 0.54, 0.30)),
        origin=Origin(xyz=(1.00, 0.0, -0.70)),
        material=glass_blue,
        name="windscreen",
    )
    gondola.visual(
        Cylinder(radius=0.055, length=0.10),
        origin=Origin(xyz=(0.58, 0.0, -1.11)),
        material=dark_metal,
        name="sensor_mast",
    )
    gondola.visual(
        Sphere(radius=0.22),
        origin=Origin(xyz=(0.58, 0.0, -1.27)),
        material=dark_metal,
        name="sensor_dome",
    )
    gondola.visual(
        Cylinder(radius=0.07, length=0.30),
        origin=Origin(xyz=(0.79, 0.0, -1.27), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="camera_barrel",
    )
    model.articulation(
        "hull_to_gondola",
        ArticulationType.FIXED,
        parent=hull,
        child=gondola,
        origin=Origin(xyz=(1.10, 0.0, -2.14)),
    )

    def add_engine(
        engine_name: str,
        prop_name: str,
        joint_name: str,
        attach_xyz: tuple[float, float, float],
        side_sign: float,
    ) -> None:
        engine = model.part(engine_name)
        engine.visual(
            Sphere(radius=0.0447),
            origin=Origin(),
            material=structure_gray,
            name="mount_boss",
        )
        engine.visual(
            Box((0.16, 0.30, 0.12)),
            origin=Origin(xyz=(0.0, 0.145 * side_sign, -0.06)),
            material=structure_gray,
            name="mount_arm",
        )
        engine.visual(
            Box((0.18, 0.42, 0.14)),
            origin=Origin(xyz=(0.0, 0.34 * side_sign, -0.07)),
            material=structure_gray,
            name="mount_bracket",
        )
        _add_member(
            engine,
            (0.02, 0.18 * side_sign, -0.02),
            (0.12, 0.64 * side_sign, -0.08),
            radius=0.042,
            material=structure_gray,
            name="upper_pylon",
        )
        _add_member(
            engine,
            (-0.06, 0.14 * side_sign, -0.06),
            (-0.12, 0.64 * side_sign, -0.18),
            radius=0.040,
            material=structure_gray,
            name="lower_pylon",
        )
        engine.visual(
            Cylinder(radius=0.20, length=1.10),
            origin=Origin(
                xyz=(0.02, 0.84 * side_sign, -0.12),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=structure_gray,
            name="nacelle_body",
        )
        engine.visual(
            Sphere(radius=0.11),
            origin=Origin(xyz=(0.56, 0.84 * side_sign, -0.12)),
            material=structure_gray,
            name="cowling",
        )
        engine.visual(
            Sphere(radius=0.14),
            origin=Origin(xyz=(-0.53, 0.84 * side_sign, -0.12)),
            material=dark_metal,
            name="rear_cap",
        )
        engine.visual(
            Cylinder(radius=0.045, length=0.18),
            origin=Origin(
                xyz=(-0.67, 0.84 * side_sign, -0.12),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_metal,
            name="exhaust",
        )
        model.articulation(
            f"hull_to_{engine_name}",
            ArticulationType.FIXED,
            parent=hull,
            child=engine,
            origin=Origin(xyz=attach_xyz),
        )

        propeller = _build_propeller(model, prop_name, safety_orange, dark_metal)
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=engine,
            child=propeller,
            origin=Origin(xyz=(0.76, 0.84 * side_sign, -0.12)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=40.0, velocity=55.0),
        )

    add_engine("port_engine", "port_propeller", "port_prop_spin", (-0.80, 2.10, 0.0), 1.0)
    add_engine(
        "starboard_engine",
        "starboard_propeller",
        "starboard_prop_spin",
        (-0.80, -2.10, 0.0),
        -1.0,
    )

    gear_frame = model.part("gear_frame")
    gear_frame.visual(
        Box((0.86, 0.28, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=structure_gray,
        name="gear_mount",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            _add_member(
                gear_frame,
                (0.24 * x_sign, 0.12 * y_sign, -0.08),
                (0.02 * x_sign, 0.52 * y_sign, -0.42),
                radius=0.030,
                material=structure_gray,
            )
    gear_frame.visual(
        Cylinder(radius=0.032, length=1.16),
        origin=Origin(xyz=(0.0, 0.0, -0.42), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="axle",
    )
    model.articulation(
        "gondola_to_gear_frame",
        ArticulationType.FIXED,
        parent=gondola,
        child=gear_frame,
        origin=Origin(xyz=(-0.30, 0.0, -1.10)),
    )

    port_wheel = _build_wheel(model, "port_wheel", rubber, dark_metal)
    starboard_wheel = _build_wheel(model, "starboard_wheel", rubber, dark_metal)
    model.articulation(
        "port_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=gear_frame,
        child=port_wheel,
        origin=Origin(xyz=(0.0, 0.62, -0.42)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )
    model.articulation(
        "starboard_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=gear_frame,
        child=starboard_wheel,
        origin=Origin(xyz=(0.0, -0.62, -0.42)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )

    tail_frame = model.part("tail_frame")
    tail_frame.visual(
        Cylinder(radius=0.13, length=0.62),
        origin=Origin(xyz=(-0.31, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=structure_gray,
        name="tail_stinger",
    )
    tail_frame.visual(
        Box((0.48, 0.58, 0.14)),
        origin=Origin(xyz=(-0.34, 0.0, 0.0)),
        material=structure_gray,
        name="tail_center",
    )
    tail_frame.visual(
        Box((0.68, 0.10, 1.10)),
        origin=Origin(xyz=(-0.56, 0.0, 0.64)),
        material=light_gray,
        name="fin",
    )
    tail_frame.visual(
        Box((0.42, 0.08, 0.48)),
        origin=Origin(xyz=(-0.38, 0.0, -0.28)),
        material=light_gray,
        name="ventral_fin",
    )
    tail_frame.visual(
        Box((0.70, 0.58, 0.08)),
        origin=Origin(xyz=(-0.56, 0.39, 0.0)),
        material=light_gray,
        name="port_stabilizer",
    )
    tail_frame.visual(
        Box((0.70, 0.58, 0.08)),
        origin=Origin(xyz=(-0.56, -0.39, 0.0)),
        material=light_gray,
        name="starboard_stabilizer",
    )
    model.articulation(
        "hull_to_tail_frame",
        ArticulationType.FIXED,
        parent=hull,
        child=tail_frame,
        origin=Origin(xyz=(-8.90, 0.0, 0.0)),
    )

    rudder = model.part("rudder")
    rudder.visual(
        Box((0.44, 0.05, 0.72)),
        origin=Origin(xyz=(-0.22, 0.0, 0.0)),
        material=light_gray,
        name="rudder_surface",
    )
    model.articulation(
        "rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=tail_frame,
        child=rudder,
        origin=Origin(xyz=(-0.90, 0.0, 0.60)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.4,
            lower=-0.45,
            upper=0.45,
        ),
    )

    port_elevator = model.part("port_elevator")
    port_elevator.visual(
        Box((0.38, 0.38, 0.05)),
        origin=Origin(xyz=(-0.19, 0.0, 0.0)),
        material=light_gray,
        name="elevator_surface",
    )
    model.articulation(
        "port_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=tail_frame,
        child=port_elevator,
        origin=Origin(xyz=(-0.91, 0.39, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-0.35,
            upper=0.35,
        ),
    )

    starboard_elevator = model.part("starboard_elevator")
    starboard_elevator.visual(
        Box((0.38, 0.38, 0.05)),
        origin=Origin(xyz=(-0.19, 0.0, 0.0)),
        material=light_gray,
        name="elevator_surface",
    )
    model.articulation(
        "starboard_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=tail_frame,
        child=starboard_elevator,
        origin=Origin(xyz=(-0.91, -0.39, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-0.35,
            upper=0.35,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_corner, max_corner) = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    gondola = object_model.get_part("gondola")
    port_engine = object_model.get_part("port_engine")
    gear_frame = object_model.get_part("gear_frame")
    tail_frame = object_model.get_part("tail_frame")
    port_wheel = object_model.get_part("port_wheel")
    starboard_wheel = object_model.get_part("starboard_wheel")
    rudder = object_model.get_part("rudder")
    port_elevator = object_model.get_part("port_elevator")
    starboard_elevator = object_model.get_part("starboard_elevator")

    ctx.expect_origin_gap(hull, gondola, axis="z", min_gap=1.90, name="gondola hangs below hull")
    ctx.expect_origin_gap(
        gondola,
        port_engine,
        axis="x",
        min_gap=1.40,
        name="engine nacelles sit aft of the cabin",
    )
    ctx.expect_origin_gap(
        gondola,
        gear_frame,
        axis="z",
        min_gap=1.00,
        name="landing gear sits below the gondola",
    )
    ctx.expect_origin_gap(
        hull,
        tail_frame,
        axis="x",
        min_gap=8.80,
        name="tail assembly sits aft of the hull center",
    )
    ctx.expect_origin_distance(
        port_wheel,
        starboard_wheel,
        axes="y",
        min_dist=1.20,
        name="landing gear has a real wheel track",
    )
    ctx.allow_overlap(
        hull,
        port_engine,
        elem_a="envelope",
        elem_b="mount_boss",
        reason="The port engine mount boss is intentionally faired into the envelope skin.",
    )
    ctx.allow_overlap(
        hull,
        "starboard_engine",
        elem_a="envelope",
        elem_b="mount_boss",
        reason="The starboard engine mount boss is intentionally faired into the envelope skin.",
    )

    for joint_name, expected_axis in (
        ("port_prop_spin", (1.0, 0.0, 0.0)),
        ("starboard_prop_spin", (1.0, 0.0, 0.0)),
        ("port_wheel_spin", (0.0, 1.0, 0.0)),
        ("starboard_wheel_spin", (0.0, 1.0, 0.0)),
    ):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None
            and tuple(joint.axis) == expected_axis,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    rudder_joint = object_model.get_articulation("rudder_hinge")
    rudder_limits = rudder_joint.motion_limits
    if rudder_limits is not None and rudder_limits.upper is not None:
        rudder_rest = _aabb_center(ctx.part_element_world_aabb(rudder, elem="rudder_surface"))
        with ctx.pose({rudder_joint: rudder_limits.upper}):
            rudder_turned = _aabb_center(ctx.part_element_world_aabb(rudder, elem="rudder_surface"))
        ctx.check(
            "rudder deflects sideways",
            rudder_rest is not None
            and rudder_turned is not None
            and abs(rudder_turned[1] - rudder_rest[1]) > 0.05,
            details=f"rest={rudder_rest}, turned={rudder_turned}",
        )

    for joint_name, elevator in (
        ("port_elevator_hinge", port_elevator),
        ("starboard_elevator_hinge", starboard_elevator),
    ):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        if limits is not None and limits.upper is not None:
            rest_center = _aabb_center(ctx.part_element_world_aabb(elevator, elem="elevator_surface"))
            with ctx.pose({joint: limits.upper}):
                moved_center = _aabb_center(ctx.part_element_world_aabb(elevator, elem="elevator_surface"))
            ctx.check(
                f"{joint_name} pitches elevator",
                rest_center is not None
                and moved_center is not None
                and abs(moved_center[2] - rest_center[2]) > 0.04,
                details=f"rest={rest_center}, moved={moved_center}",
            )

    return ctx.report()


object_model = build_object_model()

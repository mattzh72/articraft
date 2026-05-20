from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _section_loop(
    x_pos: float,
    width: float,
    height: float,
    *,
    center_z: float = 0.0,
    exponent: float = 2.6,
    segments: int = 56,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_pos + center_z)
        for y_pos, z_pos in superellipse_profile(
            width,
            height,
            exponent=exponent,
            segments=segments,
        )
    ]


def _vertical_surface(
    profile_points: list[tuple[float, float]],
    thickness: float,
) -> MeshGeometry:
    return ExtrudeGeometry(profile_points, thickness, center=True).rotate_x(-math.pi / 2.0)


def _build_hull_shell_mesh() -> MeshGeometry:
    envelope = LatheGeometry(
        [
            (0.0, -21.0),
            (0.85, -20.35),
            (2.10, -18.40),
            (3.80, -13.40),
            (4.95, -7.20),
            (5.30, -0.60),
            (5.18, 6.20),
            (4.65, 12.40),
            (3.45, 16.90),
            (1.60, 19.90),
            (0.0, 21.0),
        ],
        segments=96,
    ).rotate_y(math.pi / 2.0)

    dorsal_fin = _vertical_surface(
        [
            (15.60, 0.95),
            (16.60, 2.25),
            (18.55, 3.55),
            (19.20, 3.15),
            (19.00, 1.08),
        ],
        0.30,
    )
    ventral_fin = _vertical_surface(
        [
            (16.00, -0.85),
            (17.05, -1.95),
            (18.85, -3.00),
            (20.05, -2.20),
            (19.55, -0.92),
        ],
        0.24,
    )
    return _merge_geometries(envelope, dorsal_fin, ventral_fin)


def _build_gondola_shell_mesh() -> MeshGeometry:
    shell = section_loft(
        [
            _section_loop(-5.20, 0.45, 0.65, center_z=-1.25, exponent=2.5),
            _section_loop(-4.45, 1.80, 1.70, center_z=-1.28, exponent=2.6),
            _section_loop(-2.40, 3.10, 2.35, center_z=-1.34, exponent=2.7),
            _section_loop(0.80, 3.35, 2.58, center_z=-1.38, exponent=2.8),
            _section_loop(3.55, 3.20, 2.50, center_z=-1.36, exponent=2.8),
            _section_loop(5.20, 2.25, 2.05, center_z=-1.30, exponent=2.7),
            _section_loop(5.85, 0.55, 0.78, center_z=-1.25, exponent=2.5),
        ]
    )
    return shell


def _build_rudder_mesh() -> MeshGeometry:
    return _vertical_surface(
        [
            (0.00, -1.08),
            (0.26, -1.15),
            (1.72, -0.60),
            (1.98, 0.22),
            (1.56, 1.15),
            (0.00, 1.02),
        ],
        0.16,
    )


def _build_elevator_mesh(side_sign: float) -> MeshGeometry:
    return ExtrudeGeometry(
        [
            (0.00, 0.00),
            (0.52, 1.72 * side_sign),
            (2.35, 3.18 * side_sign),
            (3.28, 3.46 * side_sign),
            (3.02, 0.58 * side_sign),
            (0.92, 0.00),
        ],
        0.14,
        center=True,
    )


def _build_pod_mesh(side_sign: float) -> MeshGeometry:
    mount_web = BoxGeometry((1.00, 1.16, 0.18)).translate(0.00, 0.58 * side_sign, 0.24)
    return mount_web


def _build_propeller_mesh() -> MeshGeometry:
    return FanRotorGeometry(
        1.15,
        0.18,
        3,
        thickness=0.18,
        blade_pitch_deg=26.0,
        blade_sweep_deg=14.0,
        blade=FanRotorBlade(shape="broad", tip_pitch_deg=10.0, camber=0.10),
        hub=FanRotorHub(
            style="spinner",
            rear_collar_height=0.10,
            rear_collar_radius=0.16,
            bore_diameter=0.06,
        ),
    )


def _build_wheel_geometry() -> tuple[MeshGeometry, MeshGeometry]:
    wheel = WheelGeometry(
        0.25,
        0.15,
        rim=WheelRim(
            inner_radius=0.16,
            flange_height=0.020,
            flange_thickness=0.010,
            bead_seat_depth=0.008,
        ),
        hub=WheelHub(radius=0.070, width=0.11, cap_style="domed"),
        face=WheelFace(dish_depth=0.016, front_inset=0.010, rear_inset=0.004),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.012, window_radius=0.040),
        bore=WheelBore(style="round", diameter=0.040),
    )
    tire = TireGeometry(
        0.38,
        0.16,
        inner_radius=0.255,
        tread=TireTread(style="circumferential", depth=0.012, count=2),
        sidewall=TireSidewall(style="rounded", bulge=0.06),
    )
    return wheel, tire


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(0.5 * (mins[i] + maxs[i]) for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sightseeing_blimp")

    envelope_paint = model.material("envelope_paint", rgba=(0.93, 0.94, 0.95, 1.0))
    gondola_paint = model.material("gondola_paint", rgba=(0.83, 0.85, 0.88, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.16, 0.22, 0.30, 0.92))
    pod_paint = model.material("pod_paint", rgba=(0.67, 0.71, 0.77, 1.0))
    prop_metal = model.material("prop_metal", rgba=(0.44, 0.46, 0.50, 1.0))
    gear_metal = model.material("gear_metal", rgba=(0.38, 0.40, 0.44, 1.0))
    wheel_finish = model.material("wheel_finish", rgba=(0.72, 0.74, 0.77, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    hull = model.part("hull")
    hull.visual(
        mesh_from_geometry(_build_hull_shell_mesh(), "hull_shell"),
        material=envelope_paint,
        name="hull_shell",
    )
    hull.visual(
        mesh_from_geometry(BoxGeometry((3.80, 0.82, 0.52)).translate(1.60, 0.00, -5.16), "keel_mount"),
        material=envelope_paint,
        name="keel_mount",
    )
    hull.visual(
        mesh_from_geometry(
            BoxGeometry((0.60, 0.24, 2.20)).translate(19.15, 0.00, 4.00),
            "rudder_hinge",
        ),
        material=envelope_paint,
        name="rudder_hinge",
    )
    hull.visual(
        mesh_from_geometry(
            BoxGeometry((0.40, 0.28, 0.22)).translate(19.40, -1.91, 0.00),
            "port_tail_root",
        ),
        material=envelope_paint,
        name="port_tail_root",
    )
    hull.visual(
        mesh_from_geometry(
            BoxGeometry((0.40, 0.28, 0.22)).translate(19.40, 1.91, 0.00),
            "starboard_tail_root",
        ),
        material=envelope_paint,
        name="starboard_tail_root",
    )

    gondola = model.part("gondola")
    gondola.visual(
        mesh_from_geometry(_build_gondola_shell_mesh(), "cabin_shell"),
        material=gondola_paint,
        name="cabin_shell",
    )
    gondola.visual(
        mesh_from_geometry(BoxGeometry((0.72, 0.46, 0.20)).translate(0.00, 0.00, -0.10), "roof_mount"),
        material=gondola_paint,
        name="roof_mount",
    )
    gondola.visual(
        Box((4.30, 0.28, 0.18)),
        origin=Origin(xyz=(0.55, 0.00, -2.34)),
        material=gear_metal,
        name="gear_keel",
    )
    gondola.visual(
        Box((0.22, 0.34, 1.18)),
        origin=Origin(xyz=(-0.90, 0.00, -1.78)),
        material=gear_metal,
        name="gear_post_forward",
    )
    gondola.visual(
        Box((0.22, 0.34, 1.10)),
        origin=Origin(xyz=(2.55, 0.00, -1.82)),
        material=gear_metal,
        name="gear_post_rear",
    )
    gondola.visual(
        mesh_from_geometry(BoxGeometry((1.18, 0.24, 0.34)).translate(1.30, -1.67, -0.95), "port_pod_mount"),
        material=gear_metal,
        name="port_pod_mount",
    )
    gondola.visual(
        mesh_from_geometry(
            BoxGeometry((1.18, 0.24, 0.34)).translate(1.30, 1.67, -0.95),
            "starboard_pod_mount",
        ),
        material=gear_metal,
        name="starboard_pod_mount",
    )
    gondola.visual(
        Box((0.10, 3.18, 0.10)),
        origin=Origin(xyz=(0.80, 0.00, -2.30)),
        material=gear_metal,
        name="axle_beam",
    )
    gondola.visual(
        Box((6.90, 0.18, 0.92)),
        origin=Origin(xyz=(0.45, -1.60, -1.18)),
        material=glass_tint,
        name="port_windows",
    )
    gondola.visual(
        Box((6.90, 0.18, 0.92)),
        origin=Origin(xyz=(0.45, 1.60, -1.18)),
        material=glass_tint,
        name="starboard_windows",
    )
    gondola.visual(
        Box((0.70, 3.30, 0.88)),
        origin=Origin(xyz=(-3.55, 0.00, -1.15)),
        material=glass_tint,
        name="windshield",
    )

    port_pod = model.part("port_pod")
    port_pod.visual(
        mesh_from_geometry(_build_pod_mesh(-1.0), "port_pod_body"),
        material=pod_paint,
        name="mount_web",
    )
    port_pod.visual(
        Cylinder(radius=0.42, length=2.45),
        origin=Origin(xyz=(0.15, -1.55, 0.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pod_paint,
        name="nacelle",
    )
    port_pod.visual(
        Cylinder(radius=0.07, length=0.26),
        origin=Origin(xyz=(-1.04, -1.55, 0.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gear_metal,
        name="shaft_stub",
    )

    starboard_pod = model.part("starboard_pod")
    starboard_pod.visual(
        mesh_from_geometry(_build_pod_mesh(1.0), "starboard_pod_body"),
        material=pod_paint,
        name="mount_web",
    )
    starboard_pod.visual(
        Cylinder(radius=0.42, length=2.45),
        origin=Origin(xyz=(0.15, 1.55, 0.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pod_paint,
        name="nacelle",
    )
    starboard_pod.visual(
        Cylinder(radius=0.07, length=0.26),
        origin=Origin(xyz=(-1.04, 1.55, 0.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gear_metal,
        name="shaft_stub",
    )

    port_propeller = model.part("port_propeller")
    port_propeller.visual(
        mesh_from_geometry(_build_propeller_mesh(), "port_propeller"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=prop_metal,
        name="rotor",
    )

    starboard_propeller = model.part("starboard_propeller")
    starboard_propeller.visual(
        mesh_from_geometry(_build_propeller_mesh(), "starboard_propeller"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=prop_metal,
        name="rotor",
    )

    rudder = model.part("rudder")
    rudder.visual(
        mesh_from_geometry(_build_rudder_mesh(), "rudder_surface"),
        material=envelope_paint,
        name="rudder_surface",
    )

    port_elevator = model.part("port_elevator")
    port_elevator.visual(
        mesh_from_geometry(_build_elevator_mesh(-1.0), "port_elevator_surface"),
        material=envelope_paint,
        name="elevator_surface",
    )

    starboard_elevator = model.part("starboard_elevator")
    starboard_elevator.visual(
        mesh_from_geometry(_build_elevator_mesh(1.0), "starboard_elevator_surface"),
        material=envelope_paint,
        name="elevator_surface",
    )

    wheel_mesh, tire_mesh = _build_wheel_geometry()

    port_wheel = model.part("port_wheel")
    port_wheel.visual(
        mesh_from_geometry(wheel_mesh, "port_wheel_mesh"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=wheel_finish,
        name="wheel",
    )
    port_wheel.visual(
        mesh_from_geometry(tire_mesh, "port_tire_mesh"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=tire_rubber,
        name="tire",
    )
    port_wheel.visual(
        Cylinder(radius=0.05, length=0.26),
        origin=Origin(xyz=(0.0, 0.13, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gear_metal,
        name="axle_shaft",
    )

    starboard_wheel = model.part("starboard_wheel")
    starboard_wheel.visual(
        mesh_from_geometry(wheel_mesh, "starboard_wheel_mesh"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=wheel_finish,
        name="wheel",
    )
    starboard_wheel.visual(
        mesh_from_geometry(tire_mesh, "starboard_tire_mesh"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=tire_rubber,
        name="tire",
    )
    starboard_wheel.visual(
        Cylinder(radius=0.05, length=0.26),
        origin=Origin(xyz=(0.0, -0.13, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gear_metal,
        name="axle_shaft",
    )

    model.articulation(
        "hull_to_gondola",
        ArticulationType.FIXED,
        parent=hull,
        child=gondola,
        origin=Origin(xyz=(1.60, 0.00, -5.42)),
    )
    model.articulation(
        "gondola_to_port_pod",
        ArticulationType.FIXED,
        parent=gondola,
        child=port_pod,
        origin=Origin(xyz=(1.30, -1.79, -0.95)),
    )
    model.articulation(
        "gondola_to_starboard_pod",
        ArticulationType.FIXED,
        parent=gondola,
        child=starboard_pod,
        origin=Origin(xyz=(1.30, 1.79, -0.95)),
    )
    model.articulation(
        "port_pod_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=port_pod,
        child=port_propeller,
        origin=Origin(xyz=(-1.283448, -1.55, 0.00)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=20.0),
    )
    model.articulation(
        "starboard_pod_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=starboard_pod,
        child=starboard_propeller,
        origin=Origin(xyz=(-1.283448, 1.55, 0.00)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=20.0),
    )
    model.articulation(
        "hull_to_rudder",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(19.45, 0.00, 4.00)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-0.50,
            upper=0.50,
        ),
    )
    model.articulation(
        "hull_to_port_elevator",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=port_elevator,
        origin=Origin(xyz=(19.40, -2.05, 0.00)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.28,
        ),
    )
    model.articulation(
        "hull_to_starboard_elevator",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=starboard_elevator,
        origin=Origin(xyz=(19.40, 2.05, 0.00)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.28,
        ),
    )
    model.articulation(
        "gondola_to_port_wheel",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=port_wheel,
        origin=Origin(xyz=(0.80, -1.85, -2.30)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=20.0),
    )
    model.articulation(
        "gondola_to_starboard_wheel",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=starboard_wheel,
        origin=Origin(xyz=(0.80, 1.85, -2.30)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    gondola = object_model.get_part("gondola")
    port_pod = object_model.get_part("port_pod")
    starboard_pod = object_model.get_part("starboard_pod")
    rudder = object_model.get_part("rudder")
    starboard_elevator = object_model.get_part("starboard_elevator")
    port_wheel = object_model.get_part("port_wheel")
    starboard_wheel = object_model.get_part("starboard_wheel")

    rudder_joint = object_model.get_articulation("hull_to_rudder")
    elevator_joint = object_model.get_articulation("hull_to_starboard_elevator")

    ctx.expect_contact(
        hull,
        gondola,
        elem_a="keel_mount",
        elem_b="roof_mount",
        name="gondola mounts to hull keel",
    )
    ctx.expect_contact(
        port_pod,
        gondola,
        elem_a="mount_web",
        elem_b="port_pod_mount",
        name="port pod is mounted to the gondola",
    )
    ctx.expect_contact(
        starboard_pod,
        gondola,
        elem_a="mount_web",
        elem_b="starboard_pod_mount",
        name="starboard pod is mounted to the gondola",
    )
    ctx.expect_contact(
        port_wheel,
        gondola,
        elem_a="axle_shaft",
        elem_b="axle_beam",
        name="port wheel meets the landing axle",
    )
    ctx.expect_contact(
        starboard_wheel,
        gondola,
        elem_a="axle_shaft",
        elem_b="axle_beam",
        name="starboard wheel meets the landing axle",
    )
    ctx.expect_origin_gap(
        hull,
        gondola,
        axis="z",
        min_gap=4.9,
        max_gap=5.8,
        name="gondola hangs below the main envelope",
    )
    ctx.expect_origin_gap(
        starboard_pod,
        port_pod,
        axis="y",
        min_gap=3.2,
        name="propeller pods sit on opposite sides",
    )
    ctx.expect_origin_gap(
        gondola,
        port_wheel,
        axis="z",
        min_gap=2.0,
        name="landing wheels sit below the cabin roofline",
    )

    rudder_limits = rudder_joint.motion_limits
    if rudder_limits is not None and rudder_limits.upper is not None:
        rest_rudder = _aabb_center(ctx.part_world_aabb(rudder))
        with ctx.pose({rudder_joint: rudder_limits.upper}):
            turned_rudder = _aabb_center(ctx.part_world_aabb(rudder))
        ctx.check(
            "positive rudder deflection swings toward starboard",
            rest_rudder is not None
            and turned_rudder is not None
            and turned_rudder[1] > rest_rudder[1] + 0.25,
            details=f"rest={rest_rudder}, turned={turned_rudder}",
        )

    elevator_limits = elevator_joint.motion_limits
    if elevator_limits is not None and elevator_limits.upper is not None:
        rest_elevator = ctx.part_world_aabb(starboard_elevator)
        with ctx.pose({elevator_joint: elevator_limits.upper}):
            raised_elevator = ctx.part_world_aabb(starboard_elevator)
        ctx.check(
            "positive elevator deflection raises the trailing edge",
            rest_elevator is not None
            and raised_elevator is not None
            and raised_elevator[1][2] > rest_elevator[1][2] + 0.18,
            details=f"rest={rest_elevator}, raised={raised_elevator}",
        )

    return ctx.report()


object_model = build_object_model()

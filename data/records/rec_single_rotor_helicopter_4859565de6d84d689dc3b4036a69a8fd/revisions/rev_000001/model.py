from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _main_body_geom() -> MeshGeometry:
    return superellipse_side_loft(
        [
            (2.35, 0.56, 0.12, 0.18),
            (1.95, 0.78, 0.70, 0.62),
            (1.25, 0.94, 1.18, 0.92),
            (0.45, 0.94, 1.56, 1.32),
            (-0.45, 0.91, 1.60, 1.34),
            (-1.05, 0.78, 1.08, 0.96),
        ],
        exponents=2.35,
        segments=72,
    ).rotate_z(-math.pi / 2.0)


def _tail_boom_geom() -> MeshGeometry:
    return superellipse_side_loft(
        [
            (-1.10, 0.78, 0.54, 0.42),
            (-2.15, 0.86, 0.38, 0.28),
            (-3.20, 0.95, 0.28, 0.20),
            (-4.20, 1.03, 0.18, 0.13),
            (-4.55, 1.05, 0.12, 0.10),
        ],
        exponents=2.1,
        segments=52,
    ).rotate_z(-math.pi / 2.0)


def _door_panel_geom(length: float, height: float, thickness: float) -> MeshGeometry:
    half = length * 0.5
    return superellipse_side_loft(
        [
            (-half, 0.0, thickness, height * 0.90),
            (-half * 0.35, 0.0, thickness * 1.04, height),
            (half * 0.45, 0.0, thickness * 1.04, height),
            (half, 0.0, thickness, height * 0.88),
        ],
        exponents=3.0,
        segments=28,
    ).rotate_z(-math.pi / 2.0)


def _rotor_blade_geom(
    *,
    span: float,
    root_chord: float,
    tip_chord: float,
    thickness: float,
    tip_droop: float = 0.0,
) -> MeshGeometry:
    return superellipse_side_loft(
        [
            (0.08, 0.0, root_chord, thickness * 1.7),
            (0.40, 0.0, root_chord * 0.92, thickness * 1.45),
            (span * 0.62, -tip_droop * 0.35, root_chord * 0.56, thickness),
            (span, -tip_droop, tip_chord, thickness * 0.75),
        ],
        exponents=2.0,
        segments=28,
    ).rotate_z(-math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="patrol_helicopter")

    airframe = model.material("airframe", rgba=(0.74, 0.78, 0.82, 1.0))
    stripe = model.material("stripe", rgba=(0.16, 0.24, 0.38, 1.0))
    rotor_gray = model.material("rotor_gray", rgba=(0.24, 0.26, 0.28, 1.0))
    skid_gray = model.material("skid_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    glass = model.material("glass", rgba=(0.16, 0.22, 0.30, 0.92))
    accent = model.material("accent", rgba=(0.56, 0.59, 0.63, 1.0))

    fuselage = model.part("fuselage")
    fuselage.visual(
        _save_mesh("body_shell", _main_body_geom()),
        material=airframe,
        name="body_shell",
    )
    fuselage.visual(
        _save_mesh("tail_boom", _tail_boom_geom()),
        material=airframe,
        name="tail_boom",
    )
    fuselage.visual(
        Box((0.54, 0.05, 0.78)),
        origin=Origin(xyz=(-4.10, 0.0, 1.34)),
        material=airframe,
        name="vertical_fin",
    )
    fuselage.visual(
        Box((0.48, 0.78, 0.05)),
        origin=Origin(xyz=(-4.05, 0.0, 0.92)),
        material=airframe,
        name="tailplane",
    )
    fuselage.visual(
        Cylinder(radius=0.16, length=0.22),
        origin=Origin(xyz=(0.18, 0.0, 1.68)),
        material=accent,
        name="mast_fairing",
    )
    fuselage.visual(
        Cylinder(radius=0.11, length=0.18),
        origin=Origin(xyz=(-4.44, 0.08, 1.04), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="tail_gearbox",
    )
    fuselage.visual(
        Box((1.82, 0.03, 0.06)),
        origin=Origin(xyz=(0.05, 0.663, 1.40)),
        material=accent,
        name="door_rail",
    )
    fuselage.visual(
        Box((0.80, 0.03, 0.26)),
        origin=Origin(xyz=(0.18, -0.665, 0.92)),
        material=glass,
        name="port_cabin_window",
    )
    fuselage.visual(
        Box((0.44, 0.03, 0.22)),
        origin=Origin(xyz=(1.22, 0.665, 1.05), rpy=(0.0, -0.24, 0.0)),
        material=glass,
        name="starboard_cockpit_window",
    )
    fuselage.visual(
        Box((0.44, 0.03, 0.22)),
        origin=Origin(xyz=(1.12, -0.665, 1.05), rpy=(0.0, -0.24, 0.0)),
        material=glass,
        name="port_cockpit_window",
    )
    fuselage.visual(
        Box((0.88, 1.02, 0.38)),
        origin=Origin(xyz=(1.58, 0.0, 1.10), rpy=(0.0, -0.46, 0.0)),
        material=glass,
        name="windshield",
    )
    fuselage.visual(
        Box((1.55, 0.05, 0.14)),
        origin=Origin(xyz=(0.35, 0.00, 0.54)),
        material=stripe,
        name="patrol_band",
    )
    fuselage.visual(
        Box((0.60, 0.12, 0.05)),
        origin=Origin(xyz=(1.63, 0.207, 0.54)),
        material=accent,
        name="avionics_hinge_pad",
    )

    skid_radius = 0.05
    starboard_skid = tube_from_spline_points(
        [(-1.55, 0.95, -0.78), (-0.75, 0.95, -0.84), (0.65, 0.95, -0.84), (1.58, 0.95, -0.76)],
        radius=skid_radius,
        samples_per_segment=16,
        radial_segments=18,
    )
    port_skid = tube_from_spline_points(
        _mirror_y([(-1.55, 0.95, -0.78), (-0.75, 0.95, -0.84), (0.65, 0.95, -0.84), (1.58, 0.95, -0.76)]),
        radius=skid_radius,
        samples_per_segment=16,
        radial_segments=18,
    )
    fuselage.visual(_save_mesh("starboard_skid", starboard_skid), material=skid_gray, name="starboard_skid")
    fuselage.visual(_save_mesh("port_skid", port_skid), material=skid_gray, name="port_skid")

    for name, points in [
        (
            "starboard_front_strut",
            [(0.98, 0.38, 0.06), (1.00, 0.66, -0.26), (0.96, 0.95, -0.78)],
        ),
        (
            "starboard_rear_strut",
            [(-0.58, 0.34, 0.02), (-0.56, 0.65, -0.26), (-0.56, 0.95, -0.82)],
        ),
        (
            "port_front_strut",
            _mirror_y([(0.98, 0.38, 0.06), (1.00, 0.66, -0.26), (0.96, 0.95, -0.78)]),
        ),
        (
            "port_rear_strut",
            _mirror_y([(-0.58, 0.34, 0.02), (-0.56, 0.65, -0.26), (-0.56, 0.95, -0.82)]),
        ),
    ]:
        fuselage.visual(
            _save_mesh(
                name,
                tube_from_spline_points(
                    points,
                    radius=0.033,
                    samples_per_segment=12,
                    radial_segments=16,
                ),
            ),
            material=skid_gray,
            name=name,
        )

    fuselage.visual(
        Cylinder(radius=0.036, length=1.82),
        origin=Origin(xyz=(0.98, 0.0, -0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=skid_gray,
        name="front_cross_tube",
    )
    fuselage.visual(
        Cylinder(radius=0.036, length=1.82),
        origin=Origin(xyz=(-0.56, 0.0, -0.31), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=skid_gray,
        name="rear_cross_tube",
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.17, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=rotor_gray,
        name="hub",
    )
    main_rotor.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=rotor_gray,
        name="mast_shaft",
    )
    base_main_blade = _rotor_blade_geom(
        span=3.65,
        root_chord=0.34,
        tip_chord=0.14,
        thickness=0.032,
        tip_droop=0.06,
    ).rotate_x(math.radians(4.0))
    for blade_index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        main_rotor.visual(
            _save_mesh(
                f"main_blade_{blade_index}",
                base_main_blade.copy().rotate_z(angle),
            ),
            material=rotor_gray,
            name=f"blade_{blade_index}",
        )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.09, length=0.10),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rotor_gray,
        name="hub",
    )
    tail_rotor.visual(
        Cylinder(radius=0.038, length=0.12),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rotor_gray,
        name="shaft",
    )
    base_tail_blade = _rotor_blade_geom(
        span=0.66,
        root_chord=0.15,
        tip_chord=0.09,
        thickness=0.015,
    ).rotate_y(math.pi / 2.0)
    for blade_index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        tail_rotor.visual(
            _save_mesh(
                f"tail_blade_{blade_index}",
                base_tail_blade.copy().rotate((0.0, 1.0, 0.0), angle),
            ),
            material=rotor_gray,
            name=f"blade_{blade_index}",
        )

    observer_door = model.part("observer_door")
    observer_door.visual(
        Box((1.18, 0.045, 1.02)),
        material=airframe,
        name="door_panel",
    )
    observer_door.visual(
        Box((0.70, 0.014, 0.38)),
        origin=Origin(xyz=(0.02, 0.0, 0.16)),
        material=glass,
        name="door_glass",
    )
    observer_door.visual(
        Box((0.96, 0.028, 0.05)),
        origin=Origin(xyz=(0.03, 0.0, 0.555)),
        material=accent,
        name="door_carriage",
    )
    observer_door.visual(
        Box((0.05, 0.028, 0.13)),
        origin=Origin(xyz=(-0.30, 0.0, 0.48)),
        material=accent,
        name="door_post_0",
    )
    observer_door.visual(
        Box((0.05, 0.028, 0.13)),
        origin=Origin(xyz=(0.31, 0.0, 0.48)),
        material=accent,
        name="door_post_1",
    )
    observer_door.visual(
        Box((0.10, 0.025, 0.06)),
        origin=Origin(xyz=(0.22, 0.034, 0.02)),
        material=rotor_gray,
        name="door_handle",
    )

    avionics_door = model.part("avionics_door")
    avionics_door.visual(
        Box((0.52, 0.30, 0.03)),
        origin=Origin(xyz=(0.0, -0.15, -0.078), rpy=(0.28, 0.0, 0.0)),
        material=airframe,
        name="door_panel",
    )
    avionics_door.visual(
        Cylinder(radius=0.013, length=0.58),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent,
        name="hinge_barrel",
    )

    model.articulation(
        "mast_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=main_rotor,
        origin=Origin(xyz=(0.18, 0.0, 1.87)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=35.0),
    )
    model.articulation(
        "tail_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=tail_rotor,
        origin=Origin(xyz=(-4.48, 0.23, 1.04)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=65.0),
    )
    model.articulation(
        "observer_slide",
        ArticulationType.PRISMATIC,
        parent=fuselage,
        child=observer_door,
        origin=Origin(xyz=(0.32, 0.692, 0.82)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.8, lower=0.0, upper=0.95),
    )
    model.articulation(
        "avionics_hinge",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=avionics_door,
        origin=Origin(xyz=(1.63, 0.28, 0.53)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fuselage = object_model.get_part("fuselage")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    observer_door = object_model.get_part("observer_door")
    avionics_door = object_model.get_part("avionics_door")

    mast_spin = object_model.get_articulation("mast_spin")
    tail_spin = object_model.get_articulation("tail_spin")
    observer_slide = object_model.get_articulation("observer_slide")
    avionics_hinge = object_model.get_articulation("avionics_hinge")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    with ctx.pose({observer_slide: 0.0, avionics_hinge: 0.0}):
        ctx.expect_contact(
            observer_door,
            fuselage,
            elem_a="door_carriage",
            elem_b="door_rail",
            contact_tol=0.005,
            name="observer door carriage sits on the side rail when closed",
        )
        ctx.expect_contact(
            avionics_door,
            fuselage,
            elem_a="hinge_barrel",
            elem_b="avionics_hinge_pad",
            contact_tol=0.005,
            name="avionics hinge seats on the hinge pad when closed",
        )

    observer_closed = ctx.part_world_position(observer_door)
    with ctx.pose({observer_slide: 0.95}):
        observer_open = ctx.part_world_position(observer_door)
    ctx.check(
        "observer door slides aft",
        observer_closed is not None
        and observer_open is not None
        and observer_open[0] < observer_closed[0] - 0.70,
        details=f"closed={observer_closed}, open={observer_open}",
    )

    with ctx.pose({avionics_hinge: 0.0}):
        avionics_closed = ctx.part_world_aabb(avionics_door)
    with ctx.pose({avionics_hinge: math.radians(80.0)}):
        avionics_open = ctx.part_world_aabb(avionics_door)
    ctx.check(
        "avionics door swings downward",
        avionics_closed is not None
        and avionics_open is not None
        and avionics_open[0][2] < avionics_closed[0][2] - 0.12,
        details=f"closed={avionics_closed}, open={avionics_open}",
    )

    with ctx.pose({mast_spin: 0.0}):
        main_blade_rest = aabb_center(ctx.part_element_world_aabb(main_rotor, elem="blade_0"))
    with ctx.pose({mast_spin: math.pi / 2.0}):
        main_blade_turn = aabb_center(ctx.part_element_world_aabb(main_rotor, elem="blade_0"))
    ctx.check(
        "main rotor spins about the mast axis",
        main_blade_rest is not None
        and main_blade_turn is not None
        and main_blade_rest[0] > 1.7
        and abs(main_blade_rest[1]) < 0.25
        and main_blade_turn[1] > 1.7
        and abs(main_blade_turn[0]) < 0.25,
        details=f"rest={main_blade_rest}, turned={main_blade_turn}",
    )

    with ctx.pose({tail_spin: 0.0}):
        tail_blade_rest = aabb_center(ctx.part_element_world_aabb(tail_rotor, elem="blade_0"))
    with ctx.pose({tail_spin: math.pi / 2.0}):
        tail_blade_turn = aabb_center(ctx.part_element_world_aabb(tail_rotor, elem="blade_0"))
    ctx.check(
        "tail rotor spins about the transverse tail axis",
        tail_blade_rest is not None
        and tail_blade_turn is not None
        and tail_blade_rest[2] < 0.80
        and tail_blade_turn[0] < tail_blade_rest[0] - 0.25,
        details=f"rest={tail_blade_rest}, turned={tail_blade_turn}",
    )

    return ctx.report()


object_model = build_object_model()

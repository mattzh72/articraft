from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _merged(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _box_mesh(size: tuple[float, float, float], center: tuple[float, float, float]) -> MeshGeometry:
    return BoxGeometry(size).translate(*center)


def _x_tube(radius: float, length: float, center: tuple[float, float, float]) -> MeshGeometry:
    return CylinderGeometry(radius, length).rotate_y(math.pi / 2.0).translate(*center)


def _y_tube(radius: float, length: float, center: tuple[float, float, float]) -> MeshGeometry:
    return CylinderGeometry(radius, length).rotate_x(math.pi / 2.0).translate(*center)


def _z_tube(radius: float, length: float, center: tuple[float, float, float]) -> MeshGeometry:
    return CylinderGeometry(radius, length).translate(*center)


def _add_quad(geometry: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geometry.add_face(a, b, c)
    geometry.add_face(a, c, d)


def _single_tapered_blade_mesh(
    length: float,
    root_chord: float,
    tip_chord: float,
    thickness: float,
    *,
    tip_thickness_ratio: float = 0.45,
) -> MeshGeometry:
    geom = MeshGeometry()
    root_half = root_chord * 0.5
    tip_half = tip_chord * 0.5
    root_thick = thickness * 0.5
    tip_thick = root_thick * tip_thickness_ratio

    vertices = [
        (0.0, -root_half, -root_thick),
        (0.0, root_half, -root_thick),
        (0.0, root_half, root_thick),
        (0.0, -root_half, root_thick),
        (length, -tip_half, -tip_thick),
        (length, tip_half, -tip_thick),
        (length, tip_half, tip_thick),
        (length, -tip_half, tip_thick),
    ]
    ids = [geom.add_vertex(*vertex) for vertex in vertices]

    _add_quad(geom, ids[0], ids[1], ids[2], ids[3])
    _add_quad(geom, ids[4], ids[7], ids[6], ids[5])
    _add_quad(geom, ids[0], ids[4], ids[5], ids[1])
    _add_quad(geom, ids[1], ids[5], ids[6], ids[2])
    _add_quad(geom, ids[2], ids[6], ids[7], ids[3])
    _add_quad(geom, ids[3], ids[7], ids[4], ids[0])
    return geom


def _tapered_blade_pair_mesh(
    total_length: float,
    root_chord: float,
    tip_chord: float,
    thickness: float,
    *,
    axis: str = "x",
    bridge_length: float = 0.22,
) -> MeshGeometry:
    half_span = total_length * 0.5
    blade = _single_tapered_blade_mesh(half_span, root_chord, tip_chord, thickness)
    pair = _merged(
        blade.copy(),
        blade.copy().rotate_z(math.pi),
        _box_mesh((bridge_length, root_chord * 0.92, thickness), (0.0, 0.0, 0.0)),
    )
    if axis == "y":
        pair.rotate_z(math.pi / 2.0)
    elif axis == "z":
        pair.rotate_y(-math.pi / 2.0)
    return pair


def _build_landing_gear_mesh() -> MeshGeometry:
    return _merged(
        _x_tube(0.05, 1.45, (-0.05, -0.68, 0.38)),
        _x_tube(0.05, 1.45, (-0.05, 0.68, 0.38)),
        _y_tube(0.04, 1.68, (0.62, 0.0, 0.38)),
        _y_tube(0.04, 1.68, (-0.68, 0.0, 0.38)),
        _z_tube(0.04, 0.18, (0.45, -0.68, 0.47)),
        _z_tube(0.04, 0.18, (0.45, 0.68, 0.47)),
        _z_tube(0.04, 0.18, (-0.55, -0.68, 0.47)),
        _z_tube(0.04, 0.18, (-0.55, 0.68, 0.47)),
        _y_tube(0.22, 0.16, (0.62, -0.84, 0.22)),
        _y_tube(0.22, 0.16, (0.62, 0.84, 0.22)),
        _y_tube(0.22, 0.16, (-0.68, -0.84, 0.22)),
        _y_tube(0.22, 0.16, (-0.68, 0.84, 0.22)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medevac_helicopter")

    fuselage_white = model.material("fuselage_white", rgba=(0.92, 0.94, 0.95, 1.0))
    rescue_red = model.material("rescue_red", rgba=(0.74, 0.08, 0.11, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.12, 0.22, 0.30, 0.45))
    dark_mechanical = model.material("dark_mechanical", rgba=(0.18, 0.19, 0.21, 1.0))
    medium_grey = model.material("medium_grey", rgba=(0.46, 0.48, 0.50, 1.0))
    rotor_grey = model.material("rotor_grey", rgba=(0.24, 0.25, 0.27, 1.0))

    fuselage = model.part("fuselage")
    fuselage.visual(
        Box((2.05, 1.58, 1.52)),
        origin=Origin(xyz=(0.20, 0.0, 1.32)),
        material=fuselage_white,
        name="cabin_shell",
    )
    fuselage.visual(
        Box((1.10, 0.90, 0.34)),
        origin=Origin(xyz=(-0.05, 0.0, 2.10)),
        material=fuselage_white,
        name="roof_doghouse",
    )
    fuselage.visual(
        Box((0.85, 0.72, 0.28)),
        origin=Origin(xyz=(-0.22, 0.0, 1.86)),
        material=fuselage_white,
        name="upper_cowl",
    )
    fuselage.visual(
        Cylinder(radius=0.48, length=1.10),
        origin=Origin(xyz=(1.25, 0.0, 1.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fuselage_white,
        name="nose_body",
    )
    fuselage.visual(
        Sphere(radius=0.50),
        origin=Origin(xyz=(1.78, 0.0, 1.18)),
        material=fuselage_white,
        name="nose_tip",
    )
    fuselage.visual(
        Box((0.46, 1.34, 0.56)),
        origin=Origin(xyz=(0.98, 0.0, 1.44)),
        material=fuselage_white,
        name="nose_bridge",
    )
    fuselage.visual(
        Box((1.40, 1.08, 0.24)),
        origin=Origin(xyz=(0.00, 0.0, 0.74)),
        material=fuselage_white,
        name="belly_fairing",
    )
    fuselage.visual(
        Cylinder(radius=0.17, length=3.10),
        origin=Origin(xyz=(-2.30, 0.0, 1.14), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fuselage_white,
        name="tail_boom",
    )
    fuselage.visual(
        Box((0.65, 0.60, 0.40)),
        origin=Origin(xyz=(-0.95, 0.0, 1.18)),
        material=fuselage_white,
        name="boom_root",
    )
    fuselage.visual(
        Box((0.18, 0.32, 1.08)),
        origin=Origin(xyz=(-3.55, 0.0, 1.80)),
        material=fuselage_white,
        name="fin",
    )
    fuselage.visual(
        Box((0.55, 0.26, 0.42)),
        origin=Origin(xyz=(-3.30, 0.0, 1.36)),
        material=fuselage_white,
        name="tail_fairing",
    )
    fuselage.visual(
        Box((0.50, 1.10, 0.08)),
        origin=Origin(xyz=(-3.10, 0.0, 1.27)),
        material=fuselage_white,
        name="tailplane",
    )
    fuselage.visual(
        Box((0.48, 0.48, 0.20)),
        origin=Origin(xyz=(0.00, 0.0, 2.18)),
        material=fuselage_white,
        name="mast_base",
    )
    fuselage.visual(
        Cylinder(radius=0.10, length=0.30),
        origin=Origin(xyz=(0.00, 0.0, 2.43)),
        material=medium_grey,
        name="mast",
    )
    fuselage.visual(
        Cylinder(radius=0.05, length=0.24),
        origin=Origin(xyz=(-3.55, -0.21, 1.75), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=medium_grey,
        name="tail_rotor_shaft",
    )
    fuselage.visual(
        Box((0.52, 1.08, 0.48)),
        origin=Origin(xyz=(1.06, 0.0, 1.60)),
        material=glass_tint,
        name="windshield",
    )
    fuselage.visual(
        Box((0.72, 0.02, 0.34)),
        origin=Origin(xyz=(0.86, -0.80, 1.53)),
        material=glass_tint,
        name="cockpit_window_starboard",
    )
    fuselage.visual(
        Box((0.72, 0.02, 0.34)),
        origin=Origin(xyz=(0.86, 0.80, 1.53)),
        material=glass_tint,
        name="cockpit_window_port",
    )
    fuselage.visual(
        Box((2.20, 0.04, 0.05)),
        origin=Origin(xyz=(-0.65, -0.81, 1.97)),
        material=medium_grey,
        name="door_rail",
    )
    fuselage.visual(
        Box((1.55, 0.04, 0.04)),
        origin=Origin(xyz=(-0.40, -0.75, 0.86)),
        material=medium_grey,
        name="door_guide",
    )
    fuselage.visual(
        Box((0.74, 0.02, 0.18)),
        origin=Origin(xyz=(0.80, -0.80, 1.30)),
        material=rescue_red,
        name="stripe_starboard",
    )
    fuselage.visual(
        Box((1.55, 0.02, 0.18)),
        origin=Origin(xyz=(-0.05, 0.80, 1.30)),
        material=rescue_red,
        name="stripe_port",
    )

    landing_gear = model.part("landing_gear")
    landing_gear.visual(
        mesh_from_geometry(_build_landing_gear_mesh(), "landing_gear"),
        material=dark_mechanical,
        name="gear",
    )
    model.articulation(
        "fuselage_to_landing_gear",
        ArticulationType.FIXED,
        parent=fuselage,
        child=landing_gear,
        origin=Origin(),
    )

    patient_door = model.part("patient_door")
    patient_door.visual(
        Box((1.10, 0.04, 1.10)),
        origin=Origin(xyz=(-0.55, -0.02, -0.55)),
        material=fuselage_white,
        name="door_panel",
    )
    patient_door.visual(
        Box((0.22, 0.08, 0.10)),
        origin=Origin(xyz=(-0.14, -0.08, 0.02)),
        material=medium_grey,
        name="door_hanger",
    )
    patient_door.visual(
        Box((0.52, 0.012, 0.42)),
        origin=Origin(xyz=(-0.60, -0.004, -0.25)),
        material=glass_tint,
        name="door_window",
    )
    patient_door.visual(
        Box((0.78, 0.006, 0.16)),
        origin=Origin(xyz=(-0.40, -0.043, -0.18)),
        material=rescue_red,
        name="door_stripe",
    )
    patient_door.visual(
        Box((0.10, 0.006, 0.28)),
        origin=Origin(xyz=(-0.82, -0.041, -0.24)),
        material=rescue_red,
        name="cross_vertical",
    )
    patient_door.visual(
        Box((0.26, 0.006, 0.10)),
        origin=Origin(xyz=(-0.82, -0.041, -0.24)),
        material=rescue_red,
        name="cross_horizontal",
    )
    door_slide = model.articulation(
        "door_slide",
        ArticulationType.PRISMATIC,
        parent=fuselage,
        child=patient_door,
        origin=Origin(xyz=(0.35, -0.79, 1.94)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.75, lower=0.0, upper=0.95),
    )

    service_hatch = model.part("service_hatch")
    service_hatch.visual(
        Box((0.68, 0.04, 0.44)),
        origin=Origin(xyz=(0.0, 0.02, -0.22)),
        material=fuselage_white,
        name="hatch_panel",
    )
    service_hatch.visual(
        Cylinder(radius=0.022, length=0.70),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=medium_grey,
        name="hatch_hinge",
    )
    hatch_hinge = model.articulation(
        "service_hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=service_hatch,
        origin=Origin(xyz=(-0.85, 0.79, 1.98)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=1.15),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.21, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_mechanical,
        name="hub",
    )
    main_rotor.visual(
        mesh_from_geometry(
            _tapered_blade_pair_mesh(4.60, 0.22, 0.11, 0.04, axis="x", bridge_length=0.34),
            "main_blade_longitudinal",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=rotor_grey,
        name="blade_longitudinal",
    )
    main_rotor.visual(
        mesh_from_geometry(
            _tapered_blade_pair_mesh(4.20, 0.22, 0.11, 0.04, axis="y", bridge_length=0.34),
            "main_blade_lateral",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=rotor_grey,
        name="blade_lateral",
    )
    main_rotor_spin = model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=main_rotor,
        origin=Origin(xyz=(0.0, 0.0, 2.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=400.0, velocity=50.0),
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.10, length=0.10),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_mechanical,
        name="tail_hub",
    )
    tail_rotor.visual(
        mesh_from_geometry(
            _tapered_blade_pair_mesh(0.60, 0.06, 0.03, 0.10, axis="x", bridge_length=0.12),
            "tail_blade_transverse",
        ),
        material=rotor_grey,
        name="blade_transverse",
    )
    tail_rotor.visual(
        mesh_from_geometry(
            _tapered_blade_pair_mesh(1.00, 0.08, 0.035, 0.03, axis="z", bridge_length=0.12),
            "tail_blade_vertical",
        ),
        material=rotor_grey,
        name="blade_vertical",
    )
    tail_rotor_spin = model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=tail_rotor,
        origin=Origin(xyz=(-3.55, -0.38, 1.75)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=80.0),
    )

    return model


def _extent(aabb, axis: str) -> float | None:
    if aabb is None:
        return None
    index = {"x": 0, "y": 1, "z": 2}[axis]
    return aabb[1][index] - aabb[0][index]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fuselage = object_model.get_part("fuselage")
    patient_door = object_model.get_part("patient_door")
    service_hatch = object_model.get_part("service_hatch")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")

    door_slide = object_model.get_articulation("door_slide")
    hatch_hinge = object_model.get_articulation("service_hatch_hinge")
    main_rotor_spin = object_model.get_articulation("main_rotor_spin")
    tail_rotor_spin = object_model.get_articulation("tail_rotor_spin")

    with ctx.pose({door_slide: 0.0}):
        ctx.expect_gap(
            fuselage,
            patient_door,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="cabin_shell",
            negative_elem="door_panel",
            name="patient door sits flush on the cabin side",
        )
        closed_door_pos = ctx.part_world_position(patient_door)

    with ctx.pose({door_slide: 0.95}):
        ctx.expect_gap(
            fuselage,
            patient_door,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="cabin_shell",
            negative_elem="door_panel",
            name="patient door stays on the side rail when open",
        )
        open_door_pos = ctx.part_world_position(patient_door)

    ctx.check(
        "patient door slides aft",
        closed_door_pos is not None
        and open_door_pos is not None
        and open_door_pos[0] < closed_door_pos[0] - 0.80,
        details=f"closed={closed_door_pos}, open={open_door_pos}",
    )

    with ctx.pose({hatch_hinge: 0.0}):
        ctx.expect_gap(
            service_hatch,
            fuselage,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="hatch_panel",
            negative_elem="cabin_shell",
            name="service hatch sits closed against the upper fuselage",
        )
        hatch_closed_aabb = ctx.part_element_world_aabb(service_hatch, elem="hatch_panel")

    with ctx.pose({hatch_hinge: 1.0}):
        hatch_open_aabb = ctx.part_element_world_aabb(service_hatch, elem="hatch_panel")

    ctx.check(
        "service hatch swings outward",
        hatch_closed_aabb is not None
        and hatch_open_aabb is not None
        and hatch_open_aabb[1][1] > hatch_closed_aabb[1][1] + 0.12,
        details=f"closed={hatch_closed_aabb}, open={hatch_open_aabb}",
    )
    ctx.check(
        "service hatch lifts its lower edge",
        hatch_closed_aabb is not None
        and hatch_open_aabb is not None
        and hatch_open_aabb[0][2] > hatch_closed_aabb[0][2] + 0.10,
        details=f"closed={hatch_closed_aabb}, open={hatch_open_aabb}",
    )

    with ctx.pose({main_rotor_spin: 0.0}):
        main_rotor_rest = ctx.part_element_world_aabb(main_rotor, elem="blade_longitudinal")
    with ctx.pose({main_rotor_spin: math.pi / 2.0}):
        main_rotor_quarter = ctx.part_element_world_aabb(main_rotor, elem="blade_longitudinal")

    ctx.check(
        "main rotor spins about the vertical mast axis",
        _extent(main_rotor_rest, "x") is not None
        and _extent(main_rotor_rest, "y") is not None
        and _extent(main_rotor_quarter, "x") is not None
        and _extent(main_rotor_quarter, "y") is not None
        and _extent(main_rotor_rest, "x") > 4.0
        and _extent(main_rotor_rest, "y") < 0.4
        and _extent(main_rotor_quarter, "x") < 0.4
        and _extent(main_rotor_quarter, "y") > 4.0,
        details=f"rest={main_rotor_rest}, quarter={main_rotor_quarter}",
    )

    with ctx.pose({tail_rotor_spin: 0.0}):
        tail_rotor_rest = ctx.part_element_world_aabb(tail_rotor, elem="blade_vertical")
    with ctx.pose({tail_rotor_spin: math.pi / 2.0}):
        tail_rotor_quarter = ctx.part_element_world_aabb(tail_rotor, elem="blade_vertical")

    ctx.check(
        "tail rotor spins about the transverse tail axis",
        _extent(tail_rotor_rest, "z") is not None
        and _extent(tail_rotor_rest, "x") is not None
        and _extent(tail_rotor_quarter, "z") is not None
        and _extent(tail_rotor_quarter, "x") is not None
        and _extent(tail_rotor_rest, "z") > 0.9
        and _extent(tail_rotor_rest, "x") < 0.2
        and _extent(tail_rotor_quarter, "z") < 0.2
        and _extent(tail_rotor_quarter, "x") > 0.9,
        details=f"rest={tail_rotor_rest}, quarter={tail_rotor_quarter}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def _mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    x: float,
    width: float,
    height: float,
    *,
    y_center: float = 0.0,
    z_center: float = 0.0,
    exponent: float = 2.6,
    segments: int = 44,
) -> list[tuple[float, float, float]]:
    profile = superellipse_profile(width, height, exponent=exponent, segments=segments)
    return [(x, y_center + y, z_center + z) for y, z in profile]


def _mirror_yz_sections(
    x_stations: list[tuple[float, float, float, float]],
    *,
    y_center: float,
    exponent: float = 2.4,
) -> list[list[tuple[float, float, float]]]:
    return [
        _yz_section(
            x,
            width,
            height,
            y_center=y_center,
            z_center=z_center,
            exponent=exponent,
            segments=36,
        )
        for x, width, height, z_center in x_stations
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="executive_helicopter")

    body_paint = model.material("body_paint", rgba=(0.90, 0.92, 0.95, 1.0))
    stripe_paint = model.material("stripe_paint", rgba=(0.16, 0.24, 0.41, 1.0))
    glass = model.material("glass", rgba=(0.16, 0.20, 0.24, 0.72))
    rotor_paint = model.material("rotor_paint", rgba=(0.15, 0.15, 0.15, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.19, 0.19, 0.20, 1.0))
    metal = model.material("metal", rgba=(0.63, 0.66, 0.70, 1.0))
    tire = model.material("tire", rgba=(0.05, 0.05, 0.05, 1.0))

    fuselage = model.part("fuselage")
    fuselage.inertial = Inertial.from_geometry(
        Box((6.8, 2.3, 1.9)),
        mass=1650.0,
        origin=Origin(xyz=(-1.20, 0.0, 1.15)),
    )

    fuselage_shell = section_loft(
        [
            _yz_section(2.05, 0.10, 0.24, z_center=1.03, exponent=1.8),
            _yz_section(1.58, 0.98, 0.98, z_center=1.05, exponent=2.2),
            _yz_section(0.96, 1.66, 1.40, z_center=1.12, exponent=2.6),
            _yz_section(0.10, 1.82, 1.52, z_center=1.16, exponent=2.8),
            _yz_section(-0.92, 1.54, 1.28, z_center=1.21, exponent=2.7),
            _yz_section(-1.70, 0.82, 0.72, z_center=1.28, exponent=2.3),
            _yz_section(-2.78, 0.40, 0.34, z_center=1.36, exponent=2.1),
            _yz_section(-3.72, 0.27, 0.25, z_center=1.45, exponent=2.0),
            _yz_section(-4.62, 0.14, 0.18, z_center=1.54, exponent=1.8),
        ]
    )
    fuselage.visual(_mesh(fuselage_shell, "fuselage_shell"), material=body_paint, name="shell")

    doghouse = section_loft(
        [
            _yz_section(0.55, 0.76, 0.18, z_center=1.74, exponent=2.6, segments=28),
            _yz_section(0.02, 0.80, 0.20, z_center=1.81, exponent=2.8, segments=28),
            _yz_section(-0.55, 0.54, 0.18, z_center=1.75, exponent=2.4, segments=28),
        ]
    )
    fuselage.visual(_mesh(doghouse, "doghouse"), material=body_paint, name="doghouse")

    left_sponson = section_loft(
        _mirror_yz_sections(
            [
                (-0.92, 0.14, 0.14, 0.66),
                (-0.38, 0.28, 0.24, 0.66),
                (0.18, 0.26, 0.22, 0.65),
                (0.62, 0.10, 0.12, 0.63),
            ],
            y_center=0.92,
            exponent=2.3,
        )
    )
    right_sponson = section_loft(
        _mirror_yz_sections(
            [
                (-0.92, 0.14, 0.14, 0.66),
                (-0.38, 0.28, 0.24, 0.66),
                (0.18, 0.26, 0.22, 0.65),
                (0.62, 0.10, 0.12, 0.63),
            ],
            y_center=-0.92,
            exponent=2.3,
        )
    )
    fuselage.visual(_mesh(left_sponson, "left_sponson"), material=body_paint, name="left_sponson")
    fuselage.visual(_mesh(right_sponson, "right_sponson"), material=body_paint, name="right_sponson")

    fuselage.visual(
        Box((0.82, 0.10, 0.66)),
        origin=Origin(xyz=(-4.05, 0.0, 1.70)),
        material=body_paint,
        name="vertical_fin",
    )
    stabilizer = ExtrudeGeometry(
        rounded_rect_profile(0.98, 0.16, 0.05, corner_segments=6),
        0.04,
        center=True,
    ).rotate_y(pi / 2.0)
    fuselage.visual(
        _mesh(stabilizer.translate(-3.90, 0.0, 1.42), "horizontal_stabilizer"),
        material=body_paint,
        name="horizontal_stabilizer",
    )

    fuselage.visual(
        Cylinder(radius=0.18, length=0.28),
        origin=Origin(xyz=(-0.08, 0.0, 1.92)),
        material=dark_trim,
        name="mast_fairing",
    )

    fuselage.visual(
        Box((0.08, 0.076, 0.98)),
        origin=Origin(xyz=(0.92, -0.918, 1.29)),
        material=body_paint,
        name="door_jamb",
    )
    fuselage.visual(
        Box((0.18, 0.09, 0.24)),
        origin=Origin(xyz=(0.83, -0.875, 1.29)),
        material=body_paint,
        name="door_mount",
    )
    fuselage.visual(
        Box((0.08, 0.13, 0.68)),
        origin=Origin(xyz=(-1.34, -0.635, 1.12)),
        material=body_paint,
        name="hatch_jamb",
    )

    for side, side_name in ((1.0, "left"), (-1.0, "right")):
        fuselage.visual(
            Box((0.10, 0.06, 0.44)),
            origin=Origin(xyz=(-0.06, side * 0.88, 0.38)),
            material=metal,
            name=f"{side_name}_main_leg",
        )
        fuselage.visual(
            Cylinder(radius=0.045, length=0.18),
            origin=Origin(xyz=(-0.08, side * 0.96, 0.22), rpy=(pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"{side_name}_main_axle",
        )
        fuselage.visual(
            Cylinder(radius=0.22, length=0.11),
            origin=Origin(xyz=(-0.08, side * 1.02, 0.22), rpy=(pi / 2.0, 0.0, 0.0)),
            material=tire,
            name=f"{side_name}_main_wheel",
        )
        fuselage.visual(
            Cylinder(radius=0.11, length=0.14),
            origin=Origin(xyz=(-0.08, side * 1.01, 0.22), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_trim,
            name=f"{side_name}_main_hub",
        )

    fuselage.visual(
        Box((0.06, 0.12, 0.48)),
        origin=Origin(xyz=(1.16, 0.0, 0.42)),
        material=metal,
        name="nose_leg",
    )
    fuselage.visual(
        Cylinder(radius=0.040, length=0.38),
        origin=Origin(xyz=(1.18, 0.0, 0.21), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="nose_axle",
    )
    for index, y_pos in enumerate((-0.12, 0.12)):
        fuselage.visual(
            Cylinder(radius=0.16, length=0.07),
            origin=Origin(xyz=(1.18, y_pos, 0.16), rpy=(pi / 2.0, 0.0, 0.0)),
            material=tire,
            name=f"nose_wheel_{index}",
        )
        fuselage.visual(
            Cylinder(radius=0.075, length=0.09),
            origin=Origin(xyz=(1.18, y_pos, 0.16), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_trim,
            name=f"nose_hub_{index}",
        )

    fuselage.visual(
        Box((0.70, 0.03, 0.52)),
        origin=Origin(xyz=(1.18, 0.0, 1.30), rpy=(0.0, -0.34, 0.0)),
        material=glass,
        name="windshield",
    )
    fuselage.visual(
        Box((0.62, 0.02, 0.56)),
        origin=Origin(xyz=(0.70, 0.78, 1.30), rpy=(0.0, -0.04, -0.12)),
        material=glass,
        name="left_cabin_glass",
    )
    fuselage.visual(
        Box((0.36, 0.02, 0.44)),
        origin=Origin(xyz=(1.00, -0.76, 1.33), rpy=(0.0, 0.06, 0.10)),
        material=glass,
        name="right_cockpit_glass",
    )

    fuselage.visual(
        Box((0.18, 0.06, 0.12)),
        origin=Origin(xyz=(-4.46, 0.05, 1.82)),
        material=dark_trim,
        name="tail_gearbox",
    )

    main_rotor = model.part("main_rotor")
    main_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.18),
        mass=68.0,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )
    main_rotor.visual(
        Cylinder(radius=0.055, length=0.23),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=metal,
        name="mast",
    )
    main_rotor.visual(
        Cylinder(radius=0.16, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=dark_trim,
        name="hub",
    )
    main_blade = _mesh(
        ExtrudeGeometry(
            rounded_rect_profile(5.60, 0.18, 0.05, corner_segments=8),
            0.024,
            center=True,
        ),
        "main_blade_span",
    )
    main_rotor.visual(main_blade, origin=Origin(xyz=(0.0, 0.0, 0.28)), material=rotor_paint, name="blade_span_0")
    main_rotor.visual(
        main_blade,
        origin=Origin(xyz=(0.0, 0.0, 0.28), rpy=(0.0, 0.0, pi / 2.0)),
        material=rotor_paint,
        name="blade_span_1",
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.09, length=0.16),
        mass=9.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    tail_rotor.visual(
        Cylinder(radius=0.040, length=0.06),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hub",
    )
    tail_rotor.visual(
        Box((0.82, 0.014, 0.10)),
        material=rotor_paint,
        name="blade_span_0",
    )
    tail_rotor.visual(
        Box((0.10, 0.014, 0.82)),
        material=rotor_paint,
        name="blade_span_1",
    )

    right_door = model.part("right_door")
    right_door.inertial = Inertial.from_geometry(
        Box((1.00, 0.05, 1.00)),
        mass=22.0,
        origin=Origin(xyz=(-0.50, -0.025, 0.50)),
    )
    right_door.visual(
        Box((0.98, 0.025, 0.98)),
        origin=Origin(xyz=(-0.49, -0.0125, 0.49)),
        material=body_paint,
        name="panel",
    )
    right_door.visual(
        Cylinder(radius=0.016, length=0.94),
        origin=Origin(xyz=(-0.016, 0.008, 0.49)),
        material=metal,
        name="hinge_barrel",
    )
    right_door.visual(
        Box((0.44, 0.020, 0.34)),
        origin=Origin(xyz=(-0.36, -0.010, 0.70)),
        material=glass,
        name="window",
    )
    right_door.visual(
        Box((0.54, 0.010, 0.14)),
        origin=Origin(xyz=(-0.38, -0.005, 0.18)),
        material=stripe_paint,
        name="accent",
    )
    right_door.visual(
        Cylinder(radius=0.016, length=0.06),
        origin=Origin(xyz=(-0.18, -0.03, 0.40), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="handle",
    )

    right_hatch = model.part("right_hatch")
    right_hatch.inertial = Inertial.from_geometry(
        Box((0.68, 0.05, 0.70)),
        mass=10.0,
        origin=Origin(xyz=(-0.34, -0.025, 0.35)),
    )
    right_hatch.visual(
        Box((0.66, 0.024, 0.68)),
        origin=Origin(xyz=(-0.33, -0.012, 0.34)),
        material=body_paint,
        name="panel",
    )
    right_hatch.visual(
        Box((0.40, 0.010, 0.12)),
        origin=Origin(xyz=(-0.24, -0.005, 0.16)),
        material=stripe_paint,
        name="accent",
    )
    right_hatch.visual(
        Cylinder(radius=0.014, length=0.05),
        origin=Origin(xyz=(-0.16, -0.026, 0.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="handle",
    )

    model.articulation(
        "mast_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=main_rotor,
        origin=Origin(xyz=(-0.08, 0.0, 2.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=35.0),
    )
    model.articulation(
        "tail_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=tail_rotor,
        origin=Origin(xyz=(-4.46, 0.11, 1.82)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=60.0),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=right_door,
        origin=Origin(xyz=(0.96, -0.98, 0.80)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=right_hatch,
        origin=Origin(xyz=(-1.30, -0.70, 0.78)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.3, lower=0.0, upper=1.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fuselage = object_model.get_part("fuselage")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    right_door = object_model.get_part("right_door")
    right_hatch = object_model.get_part("right_hatch")
    mast_spin = object_model.get_articulation("mast_spin")
    tail_spin = object_model.get_articulation("tail_spin")
    door_hinge = object_model.get_articulation("door_hinge")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    ctx.check(
        "rotor joints are continuous on the correct axes",
        mast_spin.articulation_type == ArticulationType.CONTINUOUS
        and tail_spin.articulation_type == ArticulationType.CONTINUOUS
        and mast_spin.axis == (0.0, 0.0, 1.0)
        and tail_spin.axis == (0.0, 1.0, 0.0),
        details=f"mast={mast_spin.axis}, tail={tail_spin.axis}",
    )

    def _center_y(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    door_rest = _center_y(ctx.part_element_world_aabb(right_door, elem="panel"))
    hatch_rest = _center_y(ctx.part_element_world_aabb(right_hatch, elem="panel"))
    with ctx.pose({door_hinge: 1.10, hatch_hinge: 1.20}):
        door_open = _center_y(ctx.part_element_world_aabb(right_door, elem="panel"))
        hatch_open = _center_y(ctx.part_element_world_aabb(right_hatch, elem="panel"))

    ctx.check(
        "side door swings outward",
        door_rest is not None and door_open is not None and door_open < door_rest - 0.18,
        details=f"rest={door_rest}, open={door_open}",
    )
    ctx.check(
        "luggage hatch swings outward",
        hatch_rest is not None and hatch_open is not None and hatch_open < hatch_rest - 0.12,
        details=f"rest={hatch_rest}, open={hatch_open}",
    )
    ctx.check(
        "main rotor sits above the cabin",
        ctx.part_world_position(main_rotor) is not None and ctx.part_world_position(main_rotor)[2] > 1.90,
        details=f"main_rotor={ctx.part_world_position(main_rotor)}",
    )
    ctx.check(
        "tail rotor sits aft of the cabin",
        ctx.part_world_position(tail_rotor) is not None
        and ctx.part_world_position(fuselage) is not None
        and ctx.part_world_position(tail_rotor)[0] < ctx.part_world_position(fuselage)[0] - 4.0,
        details=f"tail={ctx.part_world_position(tail_rotor)}, fuselage={ctx.part_world_position(fuselage)}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

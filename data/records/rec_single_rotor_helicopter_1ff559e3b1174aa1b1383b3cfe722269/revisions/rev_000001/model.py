from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    z_center: float,
    exponent: float = 2.4,
    segments: int = 42,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for y, z in superellipse_profile(width, height, exponent=exponent, segments=segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="training_helicopter")

    body_red = model.material("body_red", rgba=(0.80, 0.12, 0.10, 1.0))
    white = model.material("white", rgba=(0.96, 0.96, 0.95, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    black = model.material("black", rgba=(0.06, 0.06, 0.07, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.24, 0.30, 0.45))

    airframe = model.part("airframe")
    airframe.inertial = Inertial.from_geometry(
        Box((6.3, 2.3, 2.0)),
        mass=700.0,
        origin=Origin(xyz=(-1.2, 0.0, 1.05)),
    )

    fuselage_geom = section_loft(
        [
            _yz_section(1.38, width=0.20, height=0.26, z_center=1.02, exponent=2.0, segments=48),
            _yz_section(0.96, width=0.98, height=1.14, z_center=1.12, exponent=2.25, segments=48),
            _yz_section(0.22, width=1.34, height=1.52, z_center=1.15, exponent=2.35, segments=48),
            _yz_section(-0.54, width=1.10, height=1.18, z_center=1.05, exponent=2.2, segments=48),
            _yz_section(-1.16, width=0.30, height=0.30, z_center=1.08, exponent=2.0, segments=48),
        ]
    )
    airframe.visual(_mesh("fuselage_shell", fuselage_geom), material=body_red, name="fuselage_shell")

    airframe.visual(
        Box((0.62, 0.22, 0.18)),
        origin=Origin(xyz=(0.10, 0.0, 1.66)),
        material=white,
        name="mast_fairing",
    )
    airframe.visual(
        Cylinder(radius=0.11, length=0.22),
        origin=Origin(xyz=(0.10, 0.0, 1.85)),
        material=dark_gray,
        name="mast_support",
    )
    airframe.visual(
        Box((0.54, 0.10, 0.20)),
        origin=Origin(xyz=(0.66, 0.0, 0.88), rpy=(0.0, math.radians(12.0), 0.0)),
        material=white,
        name="nose_keel",
    )

    boom_origin = Origin(xyz=(-2.86, 0.0, 1.11), rpy=(0.0, math.pi / 2.0, 0.0))
    airframe.visual(
        Cylinder(radius=0.08, length=3.48),
        origin=boom_origin,
        material=white,
        name="tail_boom",
    )
    airframe.visual(
        Box((0.18, 0.12, 0.20)),
        origin=Origin(xyz=(-4.62, 0.0, 1.14)),
        material=dark_gray,
        name="tail_gearbox",
    )
    airframe.visual(
        Box((0.54, 0.05, 0.92)),
        origin=Origin(xyz=(-4.54, 0.0, 1.54)),
        material=white,
        name="vertical_fin",
    )
    airframe.visual(
        Box((0.42, 0.04, 0.34)),
        origin=Origin(xyz=(-4.52, 0.0, 0.86)),
        material=white,
        name="ventral_fin",
    )
    airframe.visual(
        Box((0.62, 1.04, 0.04)),
        origin=Origin(xyz=(-3.40, 0.0, 1.03)),
        material=white,
        name="stabilizer",
    )

    left_skid = tube_from_spline_points(
        [
            (-1.46, 0.88, 0.29),
            (-1.22, 0.88, 0.22),
            (0.10, 0.88, 0.19),
            (0.98, 0.88, 0.22),
            (1.18, 0.88, 0.30),
        ],
        radius=0.038,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    right_skid = tube_from_spline_points(
        [
            (-1.46, -0.88, 0.29),
            (-1.22, -0.88, 0.22),
            (0.10, -0.88, 0.19),
            (0.98, -0.88, 0.22),
            (1.18, -0.88, 0.30),
        ],
        radius=0.038,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    airframe.visual(_mesh("left_skid", left_skid), material=dark_gray, name="left_skid")
    airframe.visual(_mesh("right_skid", right_skid), material=dark_gray, name="right_skid")

    for name, x_pos in (("front_cross_tube", 0.34), ("rear_cross_tube", -0.48)):
        cross_tube = tube_from_spline_points(
            [
                (x_pos, 0.88, 0.24),
                (x_pos, 0.46, 0.37),
                (x_pos, 0.00, 0.45),
                (x_pos, -0.46, 0.37),
                (x_pos, -0.88, 0.24),
            ],
            radius=0.028,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
        )
        airframe.visual(_mesh(name, cross_tube), material=dark_gray, name=name)

    airframe.visual(
        Box((0.18, 0.14, 0.12)),
        origin=Origin(xyz=(0.34, 0.0, 0.49)),
        material=dark_gray,
        name="front_skid_mount",
    )
    airframe.visual(
        Box((0.18, 0.14, 0.12)),
        origin=Origin(xyz=(-0.48, 0.0, 0.49)),
        material=dark_gray,
        name="rear_skid_mount",
    )

    airframe.visual(
        Box((0.62, 1.02, 0.74)),
        origin=Origin(xyz=(0.92, 0.0, 1.18), rpy=(0.0, math.radians(28.0), 0.0)),
        material=glass,
        name="windscreen",
    )
    airframe.visual(
        Box((0.78, 0.006, 0.60)),
        origin=Origin(xyz=(-0.06, 0.57, 1.18), rpy=(0.0, math.radians(6.0), 0.0)),
        material=glass,
        name="left_quarter_window",
    )
    airframe.visual(
        Box((0.78, 0.006, 0.60)),
        origin=Origin(xyz=(-0.06, -0.57, 1.18), rpy=(0.0, math.radians(6.0), 0.0)),
        material=glass,
        name="right_quarter_window",
    )
    airframe.visual(
        Box((0.08, 0.14, 0.12)),
        origin=Origin(xyz=(-4.62, 0.10, 1.14)),
        material=dark_gray,
        name="tail_rotor_bearing",
    )

    door_length = 0.82
    door_height = 0.94
    door_thickness = 0.028
    hinge_barrel_radius = 0.016
    hinge_barrel_length = 0.12

    left_door = model.part("left_door")
    left_door.visual(
        Box((door_length, door_thickness, door_height)),
        origin=Origin(xyz=(-door_length / 2.0, door_thickness / 2.0, 0.0)),
        material=white,
        name="door_shell",
    )
    left_door.visual(
        Box((0.46, 0.008, 0.48)),
        origin=Origin(xyz=(-0.43, 0.020, 0.12)),
        material=glass,
        name="door_window",
    )
    left_door.visual(
        Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
        origin=Origin(xyz=(-0.02, 0.024, 0.28)),
        material=dark_gray,
        name="hinge_upper",
    )
    left_door.visual(
        Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
        origin=Origin(xyz=(-0.02, 0.024, -0.28)),
        material=dark_gray,
        name="hinge_lower",
    )
    left_door.visual(
        Box((0.05, 0.026, 0.11)),
        origin=Origin(xyz=(-0.03, 0.013, 0.28)),
        material=dark_gray,
        name="hinge_upper_leaf",
    )
    left_door.visual(
        Box((0.05, 0.026, 0.11)),
        origin=Origin(xyz=(-0.03, 0.013, -0.28)),
        material=dark_gray,
        name="hinge_lower_leaf",
    )
    left_door.visual(
        Box((0.08, 0.018, 0.05)),
        origin=Origin(xyz=(-0.54, 0.023, -0.04)),
        material=black,
        name="handle",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((door_length, 0.05, door_height)),
        mass=12.0,
        origin=Origin(xyz=(-door_length / 2.0, 0.02, 0.0)),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((door_length, door_thickness, door_height)),
        origin=Origin(xyz=(-door_length / 2.0, -door_thickness / 2.0, 0.0)),
        material=white,
        name="door_shell",
    )
    right_door.visual(
        Box((0.46, 0.008, 0.48)),
        origin=Origin(xyz=(-0.43, -0.020, 0.12)),
        material=glass,
        name="door_window",
    )
    right_door.visual(
        Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
        origin=Origin(xyz=(-0.02, -0.024, 0.28)),
        material=dark_gray,
        name="hinge_upper",
    )
    right_door.visual(
        Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
        origin=Origin(xyz=(-0.02, -0.024, -0.28)),
        material=dark_gray,
        name="hinge_lower",
    )
    right_door.visual(
        Box((0.05, 0.026, 0.11)),
        origin=Origin(xyz=(-0.03, -0.013, 0.28)),
        material=dark_gray,
        name="hinge_upper_leaf",
    )
    right_door.visual(
        Box((0.05, 0.026, 0.11)),
        origin=Origin(xyz=(-0.03, -0.013, -0.28)),
        material=dark_gray,
        name="hinge_lower_leaf",
    )
    right_door.visual(
        Box((0.08, 0.018, 0.05)),
        origin=Origin(xyz=(-0.54, -0.023, -0.04)),
        material=black,
        name="handle",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((door_length, 0.05, door_height)),
        mass=12.0,
        origin=Origin(xyz=(-door_length / 2.0, -0.02, 0.0)),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.036, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=dark_gray,
        name="mast_shaft",
    )
    main_rotor.visual(
        Cylinder(radius=0.13, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=dark_gray,
        name="hub",
    )
    for index, angle_deg in enumerate((0.0, 120.0, 240.0)):
        angle = math.radians(angle_deg)
        main_rotor.visual(
            Box((2.36, 0.115, 0.030)),
            origin=Origin(xyz=(1.23, 0.0, 0.165), rpy=(0.0, 0.0, angle)),
            material=dark_gray,
            name=f"blade_{index}",
        )
        main_rotor.visual(
            Box((0.34, 0.14, 0.05)),
            origin=Origin(xyz=(0.17, 0.0, 0.16), rpy=(0.0, 0.0, angle)),
            material=black,
            name=f"blade_root_{index}",
        )
    main_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=2.5, length=0.24),
        mass=45.0,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.022, length=0.16),
        origin=Origin(xyz=(0.0, 0.30, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="hub",
    )
    tail_rotor.visual(
        Cylinder(radius=0.012, length=0.12),
        origin=Origin(xyz=(0.0, 0.23, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="shaft",
    )
    tail_rotor.visual(
        Box((0.42, 0.028, 0.06)),
        origin=Origin(xyz=(0.0, 0.30, 0.0)),
        material=black,
        name="blade_span",
    )
    tail_rotor.visual(
        Box((0.11, 0.028, 0.36)),
        origin=Origin(xyz=(0.0, 0.30, 0.0)),
        material=black,
        name="blade_chord",
    )
    tail_rotor.inertial = Inertial.from_geometry(
        Box((0.44, 0.20, 0.38)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.30, 0.0)),
    )

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=left_door,
        origin=Origin(xyz=(0.72, 0.671854, 1.11)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=right_door,
        origin=Origin(xyz=(0.72, -0.671854, 1.11)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=main_rotor,
        origin=Origin(xyz=(0.10, 0.0, 1.96)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=45.0),
    )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=tail_rotor,
        origin=Origin(xyz=(-4.62, 0.0, 1.14)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=120.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    airframe = object_model.get_part("airframe")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")

    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")

    ctx.expect_gap(
        left_door,
        airframe,
        axis="y",
        positive_elem="door_shell",
        negative_elem="fuselage_shell",
        max_gap=0.001,
        max_penetration=1e-5,
        name="left door closes flush to fuselage side",
    )
    ctx.expect_gap(
        airframe,
        right_door,
        axis="y",
        positive_elem="fuselage_shell",
        negative_elem="door_shell",
        max_gap=0.001,
        max_penetration=1e-5,
        name="right door closes flush to fuselage side",
    )
    ctx.expect_gap(
        main_rotor,
        airframe,
        axis="z",
        positive_elem="mast_shaft",
        negative_elem="mast_support",
        max_gap=0.001,
        max_penetration=1e-5,
        name="main rotor mast seats on roof support",
    )
    ctx.expect_gap(
        tail_rotor,
        airframe,
        axis="y",
        positive_elem="shaft",
        negative_elem="tail_rotor_bearing",
        max_gap=0.001,
        max_penetration=1e-5,
        name="tail rotor shaft seats on tail bearing",
    )

    left_closed = ctx.part_element_world_aabb(left_door, elem="door_shell")
    right_closed = ctx.part_element_world_aabb(right_door, elem="door_shell")
    main_blade_rest = ctx.part_element_world_aabb(main_rotor, elem="blade_0")
    tail_blade_rest = ctx.part_element_world_aabb(tail_rotor, elem="blade_span")

    with ctx.pose({left_hinge: 1.0, right_hinge: 1.0}):
        left_open = ctx.part_element_world_aabb(left_door, elem="door_shell")
        right_open = ctx.part_element_world_aabb(right_door, elem="door_shell")

    ctx.check(
        "left door opens outward",
        left_closed is not None
        and left_open is not None
        and left_open[1][1] > left_closed[1][1] + 0.45,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right door opens outward",
        right_closed is not None
        and right_open is not None
        and right_open[0][1] < right_closed[0][1] - 0.45,
        details=f"closed={right_closed}, open={right_open}",
    )

    with ctx.pose(main_rotor_spin=math.pi / 2.0, tail_rotor_spin=math.pi / 2.0):
        main_blade_spun = ctx.part_element_world_aabb(main_rotor, elem="blade_0")
        tail_blade_spun = ctx.part_element_world_aabb(tail_rotor, elem="blade_span")

    ctx.check(
        "main rotor spins about vertical mast axis",
        main_blade_rest is not None
        and main_blade_spun is not None
        and (main_blade_spun[1][1] - main_blade_spun[0][1]) > 2.0
        and (main_blade_rest[1][0] - main_blade_rest[0][0]) > 2.0,
        details=f"rest={main_blade_rest}, spun={main_blade_spun}",
    )
    ctx.check(
        "tail rotor spins about transverse axis",
        tail_blade_rest is not None
        and tail_blade_spun is not None
        and (tail_blade_rest[1][0] - tail_blade_rest[0][0]) > 0.35
        and (tail_blade_spun[1][2] - tail_blade_spun[0][2]) > 0.35,
        details=f"rest={tail_blade_rest}, spun={tail_blade_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

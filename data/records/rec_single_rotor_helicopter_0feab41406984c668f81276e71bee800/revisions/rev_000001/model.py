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
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, name)


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x_pos: float,
    *,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_pos + z_center)
        for z_pos, y_pos in rounded_rect_profile(height, width, radius, corner_segments=7)
    ]


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x_pos, -y_pos, z_pos) for x_pos, y_pos, z_pos in points]


def _rotor_blade_section(
    x_pos: float,
    chord: float,
    thickness: float,
    *,
    z_offset: float = 0.0,
    twist: float = 0.0,
) -> list[tuple[float, float, float]]:
    c = math.cos(twist)
    s = math.sin(twist)
    base = [
        (-0.50 * chord, 0.0),
        (-0.18 * chord, 0.50 * thickness),
        (0.30 * chord, 0.38 * thickness),
        (0.50 * chord, 0.0),
        (0.18 * chord, -0.42 * thickness),
        (-0.20 * chord, -0.28 * thickness),
    ]
    return [
        (x_pos, (c * y_pos) - (s * z_pos), (s * y_pos) + (c * z_pos) + z_offset)
        for y_pos, z_pos in base
    ]


def _build_airframe_shell() -> MeshGeometry:
    sections = [
        _yz_section(0.12, 0.18, 0.04, 1.45, z_center=0.92),
        _yz_section(0.56, 0.82, 0.16, 0.95, z_center=1.15),
        _yz_section(1.08, 1.24, 0.22, 0.25, z_center=1.24),
        _yz_section(1.18, 1.30, 0.22, -0.45, z_center=1.26),
        _yz_section(0.94, 1.04, 0.18, -1.05, z_center=1.18),
        _yz_section(0.42, 0.42, 0.10, -1.42, z_center=1.13),
    ]
    return section_loft(sections)


def _build_canopy_glass() -> MeshGeometry:
    sections = [
        _yz_section(0.16, 0.10, 0.03, 1.12, z_center=1.23),
        _yz_section(0.82, 0.72, 0.10, 0.58, z_center=1.38),
        _yz_section(0.92, 0.86, 0.12, 0.05, z_center=1.42),
        _yz_section(0.70, 0.62, 0.10, -0.28, z_center=1.36),
    ]
    return section_loft(sections)


def _build_main_rotor_mesh() -> MeshGeometry:
    rotor = MeshGeometry()
    rotor.merge(CylinderGeometry(radius=0.06, height=0.26, radial_segments=28).translate(0.0, 0.0, 0.13))
    rotor.merge(CylinderGeometry(radius=0.16, height=0.05, radial_segments=36).translate(0.0, 0.0, 0.29))
    rotor.merge(CylinderGeometry(radius=0.11, height=0.08, radial_segments=28).translate(0.0, 0.0, 0.23))

    blade = section_loft(
        [
            _rotor_blade_section(0.16, 0.18, 0.034, z_offset=0.01, twist=0.18),
            _rotor_blade_section(1.70, 0.14, 0.022, z_offset=0.03, twist=0.10),
            _rotor_blade_section(3.25, 0.10, 0.013, z_offset=0.04, twist=0.02),
            _rotor_blade_section(3.75, 0.06, 0.008, z_offset=0.04, twist=0.00),
        ]
    )
    for blade_index in range(3):
        rotor.merge(blade.copy().rotate_z(blade_index * (2.0 * math.pi / 3.0)).translate(0.0, 0.0, 0.29))

    return rotor


def _build_tail_rotor_mesh() -> MeshGeometry:
    rotor = MeshGeometry()
    rotor.merge(
        CylinderGeometry(radius=0.026, height=0.07, radial_segments=22)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.035, 0.0)
    )
    rotor.merge(
        CylinderGeometry(radius=0.046, height=0.10, radial_segments=24)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.10, 0.0)
    )
    rotor.merge(
        CylinderGeometry(radius=0.020, height=0.12, radial_segments=20)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.16, 0.0)
    )
    rotor.merge(CylinderGeometry(radius=0.022, height=0.56, radial_segments=18).translate(0.0, 0.16, 0.0))
    rotor.merge(
        CylinderGeometry(radius=0.016, height=0.40, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.16, 0.0)
    )
    return rotor


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trainer_helicopter")

    fuselage_paint = model.material("fuselage_paint", rgba=(0.92, 0.18, 0.14, 1.0))
    white_paint = model.material("white_paint", rgba=(0.95, 0.95, 0.94, 1.0))
    charcoal = model.material("charcoal", rgba=(0.13, 0.14, 0.16, 1.0))
    glass = model.material("glass", rgba=(0.16, 0.22, 0.26, 0.65))
    aluminum = model.material("aluminum", rgba=(0.71, 0.74, 0.77, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.27, 1.0))

    airframe = model.part("airframe")
    airframe.visual(_mesh("airframe_shell", _build_airframe_shell()), material=fuselage_paint, name="shell")
    airframe.visual(_mesh("canopy_glass", _build_canopy_glass()), material=glass, name="canopy")

    airframe.visual(
        Box((1.30, 0.82, 0.24)),
        origin=Origin(xyz=(-0.18, 0.0, 0.62)),
        material=white_paint,
        name="belly",
    )
    airframe.visual(
        Box((0.92, 0.52, 0.26)),
        origin=Origin(xyz=(-0.46, 0.0, 1.70)),
        material=fuselage_paint,
        name="engine_house",
    )
    airframe.visual(
        Box((0.76, 0.06, 0.30)),
        origin=Origin(xyz=(-0.50, 0.575, 1.58)),
        material=white_paint,
        name="left_cowling_sill",
    )
    airframe.visual(
        Box((0.76, 0.06, 0.30)),
        origin=Origin(xyz=(-0.50, -0.575, 1.58)),
        material=white_paint,
        name="right_cowling_sill",
    )
    airframe.visual(
        Cylinder(radius=0.018, length=0.74),
        origin=Origin(xyz=(-0.50, 0.575, 1.72), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="left_hinge_rail",
    )
    airframe.visual(
        Cylinder(radius=0.018, length=0.74),
        origin=Origin(xyz=(-0.50, -0.575, 1.72), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="right_hinge_rail",
    )
    airframe.visual(
        Cylinder(radius=0.11, length=0.44),
        origin=Origin(xyz=(-0.02, 0.0, 1.95)),
        material=charcoal,
        name="mast_fairing",
    )
    airframe.visual(
        Cylinder(radius=0.06, length=0.38),
        origin=Origin(xyz=(-0.02, 0.0, 2.16)),
        material=dark_metal,
        name="mast_post",
    )

    tail_boom = tube_from_spline_points(
        [
            (-1.30, 0.0, 1.15),
            (-2.10, 0.0, 1.20),
            (-3.00, 0.0, 1.25),
            (-3.88, 0.0, 1.32),
        ],
        radius=0.11,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )
    airframe.visual(_mesh("tail_boom", tail_boom), material=white_paint, name="tail_boom")
    airframe.visual(
        Box((0.12, 0.72, 0.82)),
        origin=Origin(xyz=(-3.72, 0.0, 1.55)),
        material=white_paint,
        name="tail_fin",
    )
    airframe.visual(
        Box((0.54, 0.10, 0.10)),
        origin=Origin(xyz=(-3.42, 0.0, 1.12)),
        material=white_paint,
        name="tailplane",
    )
    airframe.visual(
        Box((0.12, 0.24, 0.28)),
        origin=Origin(xyz=(-3.80, 0.325, 1.44)),
        material=dark_metal,
        name="tail_rotor_pylon",
    )

    left_skid = tube_from_spline_points(
        [
            (1.10, 0.82, 0.22),
            (0.55, 0.80, 0.12),
            (-0.55, 0.79, 0.10),
            (-1.55, 0.79, 0.13),
            (-2.20, 0.82, 0.23),
        ],
        radius=0.05,
        samples_per_segment=12,
        radial_segments=18,
    )
    right_skid = tube_from_spline_points(
        _mirror_y(
            [
                (1.10, 0.82, 0.22),
                (0.55, 0.80, 0.12),
                (-0.55, 0.79, 0.10),
                (-1.55, 0.79, 0.13),
                (-2.20, 0.82, 0.23),
            ]
        ),
        radius=0.05,
        samples_per_segment=12,
        radial_segments=18,
    )
    airframe.visual(_mesh("left_skid", left_skid), material=aluminum, name="left_skid")
    airframe.visual(_mesh("right_skid", right_skid), material=aluminum, name="right_skid")

    for name, points in [
        (
            "left_front_gear",
            [(0.56, 0.30, 0.74), (0.76, 0.54, 0.42), (0.79, 0.80, 0.14)],
        ),
        (
            "left_rear_gear",
            [(-0.68, 0.28, 0.74), (-0.82, 0.55, 0.40), (-0.82, 0.79, 0.13)],
        ),
        (
            "right_front_gear",
            _mirror_y([(0.56, 0.30, 0.74), (0.76, 0.54, 0.42), (0.79, 0.80, 0.14)]),
        ),
        (
            "right_rear_gear",
            _mirror_y([(-0.68, 0.28, 0.74), (-0.82, 0.55, 0.40), (-0.82, 0.79, 0.13)]),
        ),
        (
            "front_cross_tube",
            [(0.78, 0.80, 0.14), (0.66, 0.38, 0.44), (0.0, 0.0, 0.54), (-0.66, -0.38, 0.44), (-0.78, -0.80, 0.14)],
        ),
        (
            "rear_cross_tube",
            [(-0.82, 0.79, 0.13), (-0.70, 0.36, 0.42), (0.0, 0.0, 0.50), (0.70, -0.36, 0.42), (0.82, -0.79, 0.13)],
        ),
    ]:
        airframe.visual(
            _mesh(
                name,
                tube_from_spline_points(
                    points,
                    radius=0.03 if "cross" not in name else 0.026,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=aluminum,
            name=name,
        )

    airframe.inertial = Inertial.from_geometry(
        Box((6.0, 1.9, 2.4)),
        mass=720.0,
        origin=Origin(xyz=(-1.1, 0.0, 1.22)),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(_mesh("main_rotor", _build_main_rotor_mesh()), material=charcoal, name="rotor")
    main_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=3.8, length=0.34),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(_mesh("tail_rotor", _build_tail_rotor_mesh()), material=charcoal, name="rotor")
    tail_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=0.18),
        mass=6.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    left_cowling_door = model.part("left_cowling_door")
    left_cowling_door.visual(
        Box((0.74, 0.03, 0.28)),
        origin=Origin(xyz=(0.0, 0.08, -0.14)),
        material=fuselage_paint,
        name="panel",
    )
    left_cowling_door.inertial = Inertial.from_geometry(
        Box((0.74, 0.08, 0.30)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.07, -0.14)),
    )

    right_cowling_door = model.part("right_cowling_door")
    right_cowling_door.visual(
        Box((0.74, 0.03, 0.28)),
        origin=Origin(xyz=(0.0, -0.08, -0.14)),
        material=fuselage_paint,
        name="panel",
    )
    right_cowling_door.inertial = Inertial.from_geometry(
        Box((0.74, 0.08, 0.30)),
        mass=7.0,
        origin=Origin(xyz=(0.0, -0.07, -0.14)),
    )

    model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=main_rotor,
        origin=Origin(xyz=(-0.02, 0.0, 2.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=40.0),
    )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=tail_rotor,
        origin=Origin(xyz=(-3.86, 0.445, 1.44)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=70.0),
    )
    model.articulation(
        "left_cowling_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=left_cowling_door,
        origin=Origin(xyz=(-0.50, 0.54, 1.72)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "right_cowling_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=right_cowling_door,
        origin=Origin(xyz=(-0.50, -0.54, 1.72)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_corner, max_corner) = aabb
    return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    airframe = object_model.get_part("airframe")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    left_door = object_model.get_part("left_cowling_door")
    right_door = object_model.get_part("right_cowling_door")

    main_spin = object_model.get_articulation("main_rotor_spin")
    tail_spin = object_model.get_articulation("tail_rotor_spin")
    left_hinge = object_model.get_articulation("left_cowling_hinge")
    right_hinge = object_model.get_articulation("right_cowling_hinge")

    ctx.check(
        "main rotor uses continuous vertical mast spin",
        main_spin.joint_type == ArticulationType.CONTINUOUS
        and main_spin.axis == (0.0, 0.0, 1.0)
        and main_spin.motion_limits is not None
        and main_spin.motion_limits.lower is None
        and main_spin.motion_limits.upper is None,
        details=f"type={main_spin.joint_type}, axis={main_spin.axis}, limits={main_spin.motion_limits}",
    )
    ctx.check(
        "tail rotor uses continuous transverse spin",
        tail_spin.joint_type == ArticulationType.CONTINUOUS
        and tail_spin.axis == (0.0, 1.0, 0.0)
        and tail_spin.motion_limits is not None
        and tail_spin.motion_limits.lower is None
        and tail_spin.motion_limits.upper is None,
        details=f"type={tail_spin.joint_type}, axis={tail_spin.axis}, limits={tail_spin.motion_limits}",
    )
    ctx.check(
        "cowling hinges are matched longitudinal joints",
        left_hinge.axis == (1.0, 0.0, 0.0)
        and right_hinge.axis == (-1.0, 0.0, 0.0)
        and left_hinge.motion_limits is not None
        and right_hinge.motion_limits is not None
        and left_hinge.motion_limits.lower == 0.0
        and right_hinge.motion_limits.lower == 0.0
        and abs(left_hinge.motion_limits.upper - right_hinge.motion_limits.upper) < 1e-6,
        details=f"left={left_hinge.axis}/{left_hinge.motion_limits}, right={right_hinge.axis}/{right_hinge.motion_limits}",
    )

    ctx.expect_gap(
        left_door,
        airframe,
        axis="y",
        positive_elem="panel",
        negative_elem="left_cowling_sill",
        min_gap=0.0,
        max_gap=0.05,
        name="left cowling door closes against left sill",
    )
    ctx.expect_overlap(
        left_door,
        airframe,
        axes="xz",
        elem_a="panel",
        elem_b="left_cowling_sill",
        min_overlap=0.20,
        name="left cowling door covers left sill opening",
    )
    ctx.expect_gap(
        airframe,
        right_door,
        axis="y",
        positive_elem="right_cowling_sill",
        negative_elem="panel",
        min_gap=0.0,
        max_gap=0.05,
        name="right cowling door closes against right sill",
    )
    ctx.expect_overlap(
        right_door,
        airframe,
        axes="xz",
        elem_a="panel",
        elem_b="right_cowling_sill",
        min_overlap=0.20,
        name="right cowling door covers right sill opening",
    )

    main_pos = ctx.part_world_position(main_rotor)
    tail_pos = ctx.part_world_position(tail_rotor)
    ctx.check(
        "tail rotor sits rearward of the mast",
        main_pos is not None and tail_pos is not None and tail_pos[0] < main_pos[0] - 3.0,
        details=f"main={main_pos}, tail={tail_pos}",
    )

    left_closed = _aabb_center(ctx.part_element_world_aabb(left_door, elem="panel"))
    right_closed = _aabb_center(ctx.part_element_world_aabb(right_door, elem="panel"))
    with ctx.pose(
        {
            left_hinge: left_hinge.motion_limits.upper,
            right_hinge: right_hinge.motion_limits.upper,
        }
    ):
        left_open = _aabb_center(ctx.part_element_world_aabb(left_door, elem="panel"))
        right_open = _aabb_center(ctx.part_element_world_aabb(right_door, elem="panel"))

    ctx.check(
        "left cowling door opens outward",
        left_closed is not None and left_open is not None and left_open[1] > left_closed[1] + 0.03,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right cowling door opens outward",
        right_closed is not None and right_open is not None and right_open[1] < right_closed[1] - 0.03,
        details=f"closed={right_closed}, open={right_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

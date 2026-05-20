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
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _loop_x(
    x_pos: float,
    width_y: float,
    height_z: float,
    *,
    y_center: float = 0.0,
    z_center: float = 0.0,
    exponent: float = 2.1,
    segments: int = 28,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_center + y_val, z_center + z_val)
        for y_val, z_val in superellipse_profile(
            width_y,
            height_z,
            exponent=exponent,
            segments=segments,
        )
    ]


def _build_main_blade(span: float, root_chord: float, tip_chord: float) -> MeshGeometry:
    return section_loft(
        [
            _loop_x(0.00, root_chord, 0.050, exponent=2.8, segments=20),
            _loop_x(span * 0.18, root_chord * 0.96, 0.046, exponent=2.8, segments=20),
            _loop_x(span * 0.52, root_chord * 0.72, 0.038, exponent=2.8, segments=20),
            _loop_x(span * 0.88, tip_chord * 1.15, 0.030, exponent=2.8, segments=20),
            _loop_x(span, tip_chord, 0.024, exponent=2.8, segments=20),
        ]
    )


def _build_main_rotor_blades() -> MeshGeometry:
    span = 4.15
    blade = _build_main_blade(span, 0.26, 0.12)
    blades = MeshGeometry()
    blades.merge(blade.copy())
    blades.merge(blade.copy().rotate_z(math.pi))
    return blades


def _build_door_shell(side_sign: float) -> MeshGeometry:
    return section_loft(
        [
            _loop_x(0.00, 0.028, 1.12, y_center=0.014 * side_sign, exponent=2.8, segments=24),
            _loop_x(-0.18, 0.090, 1.24, y_center=0.050 * side_sign, exponent=2.2, segments=24),
            _loop_x(-0.60, 0.200, 1.34, y_center=0.106 * side_sign, exponent=2.0, segments=24),
            _loop_x(-1.02, 0.154, 1.26, y_center=0.082 * side_sign, exponent=2.1, segments=24),
            _loop_x(-1.22, 0.086, 1.10, y_center=0.046 * side_sign, exponent=2.4, segments=24),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observation_helicopter")

    body_white = model.material("body_white", rgba=(0.94, 0.95, 0.97, 1.0))
    body_blue = model.material("body_blue", rgba=(0.16, 0.28, 0.55, 1.0))
    glass = model.material("glass", rgba=(0.42, 0.64, 0.76, 0.34))
    skid_gray = model.material("skid_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.19, 1.0))

    airframe = model.part("airframe")
    airframe.inertial = Inertial.from_geometry(
        Box((8.8, 2.0, 3.2)),
        mass=780.0,
        origin=Origin(xyz=(-2.2, 0.0, 0.7)),
    )

    canopy_geom = section_loft(
        [
            _loop_x(1.42, 0.54, 0.72, z_center=0.54, exponent=1.8, segments=28),
            _loop_x(0.92, 1.12, 1.34, z_center=0.58, exponent=1.9, segments=28),
            _loop_x(0.20, 1.74, 1.64, z_center=0.64, exponent=2.0, segments=30),
            _loop_x(-0.52, 1.64, 1.56, z_center=0.72, exponent=2.1, segments=30),
            _loop_x(-1.02, 1.02, 1.18, z_center=0.88, exponent=2.3, segments=28),
        ]
    )
    airframe.visual(_mesh("canopy_shell", canopy_geom), material=glass, name="canopy_shell")

    lower_body_geom = section_loft(
        [
            _loop_x(1.02, 0.92, 0.58, z_center=-0.06, exponent=2.3, segments=24),
            _loop_x(0.22, 1.46, 0.72, z_center=-0.12, exponent=2.4, segments=28),
            _loop_x(-0.62, 1.30, 0.72, z_center=-0.08, exponent=2.4, segments=28),
            _loop_x(-1.20, 0.84, 0.54, z_center=0.06, exponent=2.6, segments=24),
        ]
    )
    airframe.visual(_mesh("lower_body", lower_body_geom), material=body_blue, name="lower_body")

    engine_cowling_geom = section_loft(
        [
            _loop_x(-0.12, 0.64, 0.42, z_center=1.18, exponent=2.4, segments=22),
            _loop_x(-0.70, 0.94, 0.62, z_center=1.24, exponent=2.2, segments=24),
            _loop_x(-1.28, 0.60, 0.40, z_center=1.04, exponent=2.5, segments=22),
        ]
    )
    airframe.visual(
        _mesh("engine_cowling", engine_cowling_geom),
        material=body_white,
        name="engine_cowling",
    )

    boom_geom = tube_from_spline_points(
        [
            (-1.10, 0.0, 0.88),
            (-2.60, 0.0, 0.86),
            (-4.50, 0.0, 0.94),
            (-6.08, 0.0, 1.04),
        ],
        radius=0.090,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    airframe.visual(_mesh("tail_boom", boom_geom), material=body_white, name="tail_boom")

    tail_cone_geom = tube_from_spline_points(
        [
            (-5.90, 0.0, 1.02),
            (-6.38, 0.0, 0.98),
        ],
        radius=0.068,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    airframe.visual(_mesh("tail_cone", tail_cone_geom), material=body_blue, name="tail_cone")

    airframe.visual(
        Box((0.14, 0.30, 1.48)),
        origin=Origin(xyz=(-5.86, 0.0, 1.48)),
        material=body_blue,
        name="vertical_fin",
    )
    airframe.visual(
        Box((0.72, 1.26, 0.06)),
        origin=Origin(xyz=(-5.22, 0.0, 0.92)),
        material=body_white,
        name="stabilizer",
    )
    airframe.visual(
        Box((0.22, 0.28, 0.22)),
        origin=Origin(xyz=(-6.12, 0.14, 1.02)),
        material=body_white,
        name="tail_gearbox",
    )

    airframe.visual(
        Cylinder(radius=0.12, length=0.34),
        origin=Origin(xyz=(-0.16, 0.0, 1.76)),
        material=body_white,
        name="mast_fairing",
    )
    airframe.visual(
        Cylinder(radius=0.072, length=0.16),
        origin=Origin(xyz=(-0.16, 0.0, 1.98)),
        material=dark_metal,
        name="mast_cap",
    )
    airframe.visual(
        Box((0.34, 0.34, 0.46)),
        origin=Origin(xyz=(-0.16, 0.0, 1.58)),
        material=body_white,
        name="mast_pylon",
    )
    airframe.visual(
        Cylinder(radius=0.040, length=1.24),
        origin=Origin(xyz=(0.96, 0.861, 0.46)),
        material=dark_metal,
        name="left_hinge_post",
    )
    airframe.visual(
        Cylinder(radius=0.040, length=1.24),
        origin=Origin(xyz=(0.96, -0.861, 0.46)),
        material=dark_metal,
        name="right_hinge_post",
    )
    airframe.visual(
        Box((1.02, 0.06, 0.07)),
        origin=Origin(xyz=(0.42, 0.82, 1.02)),
        material=body_white,
        name="left_upper_frame",
    )
    airframe.visual(
        Box((1.12, 0.06, 0.07)),
        origin=Origin(xyz=(0.44, 0.80, -0.08)),
        material=body_white,
        name="left_lower_frame",
    )
    airframe.visual(
        Box((1.02, 0.06, 0.07)),
        origin=Origin(xyz=(0.42, -0.82, 1.02)),
        material=body_white,
        name="right_upper_frame",
    )
    airframe.visual(
        Box((1.12, 0.06, 0.07)),
        origin=Origin(xyz=(0.44, -0.80, -0.08)),
        material=body_white,
        name="right_lower_frame",
    )

    left_skid_geom = tube_from_spline_points(
        [
            (-1.24, 0.94, -0.98),
            (-0.10, 0.94, -1.00),
            (1.04, 0.94, -0.96),
            (1.58, 0.94, -0.88),
        ],
        radius=0.050,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    right_skid_geom = tube_from_spline_points(
        [
            (-1.24, -0.94, -0.98),
            (-0.10, -0.94, -1.00),
            (1.04, -0.94, -0.96),
            (1.58, -0.94, -0.88),
        ],
        radius=0.050,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    airframe.visual(_mesh("left_skid", left_skid_geom), material=skid_gray, name="left_skid")
    airframe.visual(_mesh("right_skid", right_skid_geom), material=skid_gray, name="right_skid")

    for name, points in (
        (
            "left_front_strut",
            [(0.56, 0.34, -0.32), (0.60, 0.60, -0.62), (0.54, 0.94, -0.95)],
        ),
        (
            "left_rear_strut",
            [(-0.56, 0.30, -0.30), (-0.62, 0.58, -0.60), (-0.58, 0.94, -0.95)],
        ),
        (
            "right_front_strut",
            [(0.56, -0.34, -0.32), (0.60, -0.60, -0.62), (0.54, -0.94, -0.95)],
        ),
        (
            "right_rear_strut",
            [(-0.56, -0.30, -0.30), (-0.62, -0.58, -0.60), (-0.58, -0.94, -0.95)],
        ),
    ):
        strut_geom = tube_from_spline_points(
            points,
            radius=0.044,
            samples_per_segment=12,
            radial_segments=16,
            cap_ends=True,
        )
        airframe.visual(_mesh(name, strut_geom), material=skid_gray, name=name)

    left_door = model.part("left_door")
    left_door.visual(
        _mesh("left_door_shell", _build_door_shell(1.0)),
        material=glass,
        name="door_shell",
    )
    left_door.visual(
        Cylinder(radius=0.024, length=1.22),
        origin=Origin(xyz=(-0.02, 0.02, 0.0)),
        material=dark_metal,
        name="hinge_bar",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((1.26, 0.24, 1.36)),
        mass=22.0,
        origin=Origin(xyz=(-0.62, 0.11, 0.0)),
    )

    right_door = model.part("right_door")
    right_door.visual(
        _mesh("right_door_shell", _build_door_shell(-1.0)),
        material=glass,
        name="door_shell",
    )
    right_door.visual(
        Cylinder(radius=0.024, length=1.22),
        origin=Origin(xyz=(-0.02, -0.02, 0.0)),
        material=dark_metal,
        name="hinge_bar",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((1.26, 0.24, 1.36)),
        mass=22.0,
        origin=Origin(xyz=(-0.62, -0.11, 0.0)),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        _mesh("main_rotor_blades", _build_main_rotor_blades()),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=dark_metal,
        name="rotor_blades",
    )
    main_rotor.visual(
        Cylinder(radius=0.09, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=dark_metal,
        name="hub",
    )
    main_rotor.visual(
        Cylinder(radius=0.045, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=dark_metal,
        name="shaft",
    )
    main_rotor.inertial = Inertial.from_geometry(
        Box((8.4, 0.40, 0.34)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Box((0.07, 0.04, 0.92)),
        origin=Origin(xyz=(0.0, 0.07, 0.0)),
        material=dark_metal,
        name="rotor_blades",
    )
    tail_rotor.visual(
        Cylinder(radius=0.08, length=0.14),
        origin=Origin(xyz=(0.0, 0.07, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hub",
    )
    tail_rotor.inertial = Inertial.from_geometry(
        Box((0.16, 0.18, 0.96)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.07, 0.0)),
    )

    model.articulation(
        "airframe_to_left_door",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=left_door,
        origin=Origin(xyz=(0.98, 0.905, 0.46)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "airframe_to_right_door",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=right_door,
        origin=Origin(xyz=(0.98, -0.905, 0.46)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "airframe_to_main_rotor",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=main_rotor,
        origin=Origin(xyz=(-0.16, 0.0, 2.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=35.0),
    )
    model.articulation(
        "airframe_to_tail_rotor",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=tail_rotor,
        origin=Origin(xyz=(-6.12, 0.28, 1.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=42.0, velocity=60.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    airframe = object_model.get_part("airframe")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")

    left_hinge = object_model.get_articulation("airframe_to_left_door")
    right_hinge = object_model.get_articulation("airframe_to_right_door")
    main_spin = object_model.get_articulation("airframe_to_main_rotor")
    tail_spin = object_model.get_articulation("airframe_to_tail_rotor")

    ctx.check(
        "main rotor articulation is continuous about the mast",
        main_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(main_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={main_spin.articulation_type}, axis={main_spin.axis}",
    )
    ctx.check(
        "tail rotor articulation is continuous about the transverse tail axis",
        tail_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(tail_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={tail_spin.articulation_type}, axis={tail_spin.axis}",
    )

    ctx.expect_gap(
        main_rotor,
        airframe,
        axis="z",
        min_gap=0.0,
        max_gap=0.05,
        positive_elem="shaft",
        negative_elem="mast_cap",
        name="main rotor clears the mast cap",
    )
    ctx.expect_gap(
        tail_rotor,
        airframe,
        axis="y",
        min_gap=0.0,
        max_gap=0.12,
        positive_elem="hub",
        negative_elem="tail_gearbox",
        name="tail rotor sits outboard of the tail gearbox",
    )

    def center_y(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    closed_left_aabb = ctx.part_world_aabb(left_door)
    closed_right_aabb = ctx.part_world_aabb(right_door)

    with ctx.pose({left_hinge: 1.15, right_hinge: 1.15}):
        open_left_aabb = ctx.part_world_aabb(left_door)
        open_right_aabb = ctx.part_world_aabb(right_door)

    closed_left_y = center_y(closed_left_aabb)
    closed_right_y = center_y(closed_right_aabb)
    open_left_y = center_y(open_left_aabb)
    open_right_y = center_y(open_right_aabb)

    ctx.check(
        "left door swings outward",
        closed_left_y is not None and open_left_y is not None and open_left_y > closed_left_y + 0.18,
        details=f"closed_y={closed_left_y}, open_y={open_left_y}",
    )
    ctx.check(
        "right door swings outward",
        closed_right_y is not None and open_right_y is not None and open_right_y < closed_right_y - 0.18,
        details=f"closed_y={closed_right_y}, open_y={open_right_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

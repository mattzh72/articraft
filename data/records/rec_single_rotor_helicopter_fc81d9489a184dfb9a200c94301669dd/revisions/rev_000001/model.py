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
    tube_from_spline_points,
)


def _x_superellipse_section(
    x_pos: float,
    width: float,
    height: float,
    z_center: float,
    *,
    exponent: float = 2.4,
    segments: int = 44,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y, z_center + z)
        for y, z in superellipse_profile(width, height, exponent=exponent, segments=segments)
    ]


def _build_fuselage_lower_mesh() -> MeshGeometry:
    specs = [
        (1.28, 0.18, 0.18, 0.84),
        (0.92, 0.68, 0.42, 0.82),
        (0.35, 1.04, 0.58, 0.80),
        (-0.35, 1.16, 0.62, 0.82),
        (-0.95, 1.04, 0.66, 0.86),
        (-1.55, 0.68, 0.48, 0.92),
        (-2.05, 0.34, 0.28, 0.98),
    ]
    return section_loft([_x_superellipse_section(*spec, exponent=2.25) for spec in specs])


def _build_canopy_mesh() -> MeshGeometry:
    specs = [
        (1.22, 0.16, 0.22, 1.20),
        (0.92, 0.92, 0.72, 1.28),
        (0.25, 1.26, 0.92, 1.38),
        (-0.45, 1.18, 0.86, 1.40),
        (-0.95, 0.78, 0.56, 1.28),
    ]
    return section_loft([_x_superellipse_section(*spec, exponent=2.0) for spec in specs])


def _build_vertical_fin_mesh() -> MeshGeometry:
    sections = [
        [
            (-4.35, -0.03, 1.08),
            (-4.35, 0.03, 1.08),
            (-4.35, 0.03, 1.42),
            (-4.35, -0.03, 1.42),
        ],
        [
            (-4.72, -0.028, 1.10),
            (-4.72, 0.028, 1.10),
            (-4.72, 0.028, 1.92),
            (-4.72, -0.028, 1.92),
        ],
        [
            (-5.06, -0.022, 1.18),
            (-5.06, 0.022, 1.18),
            (-5.06, 0.022, 1.70),
            (-5.06, -0.022, 1.70),
        ],
    ]
    return section_loft(sections)


def _build_landing_gear_mesh() -> MeshGeometry:
    gear = MeshGeometry()

    skid_radius = 0.038
    strut_radius = 0.024

    left_skid = tube_from_spline_points(
        [
            (0.96, 0.74, 0.24),
            (0.72, 0.76, 0.18),
            (-0.18, 0.76, 0.14),
            (-1.34, 0.76, 0.14),
            (-1.76, 0.74, 0.20),
        ],
        radius=skid_radius,
        samples_per_segment=16,
        radial_segments=18,
    )
    right_skid = left_skid.copy().scale(1.0, -1.0, 1.0)
    gear.merge(left_skid)
    gear.merge(right_skid)

    upper_points = [
        (0.46, 0.34, 0.76),
        (-0.86, 0.35, 0.78),
    ]
    lower_points = [
        (0.46, 0.74, 0.23),
        (-0.86, 0.74, 0.18),
    ]
    for upper, lower in zip(upper_points, lower_points):
        strut = tube_from_spline_points(
            [
                upper,
                (
                    0.5 * (upper[0] + lower[0]),
                    0.5 * (upper[1] + lower[1]),
                    lower[2] + 0.18,
                ),
                lower,
            ],
            radius=strut_radius,
            samples_per_segment=14,
            radial_segments=14,
        )
        gear.merge(strut)
        gear.merge(strut.copy().scale(1.0, -1.0, 1.0))

    cross_tube = CylinderGeometry(radius=0.022, height=1.20, radial_segments=16)
    cross_tube.rotate_x(math.pi / 2.0).translate(-0.22, 0.0, 0.50)
    gear.merge(cross_tube)

    return gear


def _blade_section(
    span_pos: float,
    chord: float,
    thickness: float,
    y_shift: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        chord,
        thickness,
        radius=min(chord * 0.08, thickness * 0.45),
        corner_segments=4,
    )
    return [(span_pos, y + y_shift, z) for y, z in profile]


def _build_two_blade_rotor_mesh(
    *,
    radius: float,
    hub_radius: float,
    hub_height: float,
    mast_radius: float,
    mast_length: float,
    root_cutout: float,
    chord_root: float,
    chord_mid: float,
    chord_tip: float,
    thickness_root: float,
    thickness_tip: float,
    sweep: float,
) -> MeshGeometry:
    rotor = MeshGeometry()
    mast = CylinderGeometry(radius=mast_radius, height=mast_length, radial_segments=24)
    mast.translate(0.0, 0.0, -0.5 * mast_length + 0.04)
    rotor.merge(mast)

    hub = CylinderGeometry(radius=hub_radius, height=hub_height, radial_segments=28)
    rotor.merge(hub)

    pitch_link = CylinderGeometry(radius=0.020, height=0.28, radial_segments=16)
    pitch_link.rotate_y(math.pi / 2.0)
    rotor.merge(pitch_link.copy().translate(0.02, 0.0, 0.0))
    rotor.merge(pitch_link.copy().rotate_z(math.pi).translate(-0.02, 0.0, 0.0))

    blade = section_loft(
        [
            _blade_section(root_cutout, chord_root, thickness_root, 0.00),
            _blade_section(radius * 0.45, chord_mid, 0.5 * (thickness_root + thickness_tip), -0.45 * sweep),
            _blade_section(radius, chord_tip, thickness_tip, -sweep),
        ]
    )
    rotor.merge(blade)
    rotor.merge(blade.copy().rotate_z(math.pi))
    return rotor


def _build_main_rotor_mesh() -> MeshGeometry:
    rotor = MeshGeometry()

    mast = CylinderGeometry(radius=0.050, height=0.24, radial_segments=24)
    mast.translate(0.0, 0.0, 0.12)
    rotor.merge(mast)

    hub = CylinderGeometry(radius=0.14, height=0.08, radial_segments=28)
    hub.translate(0.0, 0.0, 0.24)
    rotor.merge(hub)

    pitch_link = CylinderGeometry(radius=0.022, height=0.34, radial_segments=16)
    pitch_link.rotate_y(math.pi / 2.0).translate(0.0, 0.0, 0.24)
    rotor.merge(pitch_link)

    blade = section_loft(
        [
            _blade_section(0.14, 0.26, 0.032, 0.00),
            _blade_section(1.46, 0.19, 0.024, -0.06),
            _blade_section(3.25, 0.11, 0.016, -0.14),
        ]
    )
    blade.translate(0.0, 0.0, 0.24)
    rotor.merge(blade)
    rotor.merge(blade.copy().rotate_z(math.pi))
    return rotor


def _build_tail_rotor_mesh() -> MeshGeometry:
    tail_rotor = MeshGeometry()

    shaft = CylinderGeometry(radius=0.032, height=0.22, radial_segments=20)
    shaft.rotate_x(math.pi / 2.0)
    tail_rotor.merge(shaft)

    hub = CylinderGeometry(radius=0.075, height=0.065, radial_segments=24)
    hub.rotate_x(math.pi / 2.0)
    tail_rotor.merge(hub)

    blade = section_loft(
        [
            _blade_section(0.05, 0.12, 0.020, 0.00),
            _blade_section(0.30, 0.08, 0.014, -0.03),
            _blade_section(0.55, 0.045, 0.010, -0.05),
        ]
    )
    blade.rotate_x(math.pi / 2.0)
    tail_rotor.merge(blade)
    tail_rotor.merge(blade.copy().rotate_y(math.pi))

    pitch_bar = CylinderGeometry(radius=0.014, height=0.18, radial_segments=14)
    pitch_bar.rotate_z(math.pi / 2.0)
    tail_rotor.merge(pitch_bar)

    blade_grip = CylinderGeometry(radius=0.026, height=0.14, radial_segments=16)
    blade_grip.rotate_y(math.pi / 2.0)
    tail_rotor.merge(blade_grip.copy().translate(0.07, 0.0, 0.0))
    tail_rotor.merge(blade_grip.copy().translate(-0.07, 0.0, 0.0))
    return tail_rotor


def _build_baggage_door_mesh(width: float, height: float, thickness: float) -> MeshGeometry:
    door = ExtrudeGeometry(
        rounded_rect_profile(width, height, 0.07, corner_segments=6),
        thickness,
        center=True,
    )
    door.rotate_x(math.pi / 2.0)
    door.translate(-0.5 * width, -0.5 * thickness, 0.0)
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="civilian_helicopter")

    body_paint = model.material("body_paint", rgba=(0.93, 0.94, 0.95, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.12, 0.34, 0.58, 1.0))
    canopy_glass = model.material("canopy_glass", rgba=(0.35, 0.50, 0.58, 0.35))
    metal_dark = model.material("metal_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    metal_mid = model.material("metal_mid", rgba=(0.45, 0.47, 0.50, 1.0))

    airframe = model.part("airframe")
    airframe.visual(
        mesh_from_geometry(_build_fuselage_lower_mesh(), "fuselage_lower"),
        material=body_paint,
        name="fuselage_lower",
    )
    airframe.visual(
        mesh_from_geometry(_build_canopy_mesh(), "canopy_shell"),
        material=canopy_glass,
        name="canopy_shell",
    )
    airframe.visual(
        Box((1.20, 0.12, 0.06)),
        origin=Origin(xyz=(-0.05, 0.0, 0.96)),
        material=accent_blue,
        name="accent_stripe",
    )
    airframe.visual(
        Cylinder(radius=0.12, length=3.00),
        origin=Origin(xyz=(-3.55, 0.0, 1.20), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_paint,
        name="tail_boom",
    )
    airframe.visual(
        mesh_from_geometry(_build_vertical_fin_mesh(), "vertical_fin"),
        material=body_paint,
        name="vertical_fin",
    )
    airframe.visual(
        Box((0.78, 0.90, 0.05)),
        origin=Origin(xyz=(-4.42, 0.0, 1.08)),
        material=body_paint,
        name="horizontal_stabilizer",
    )
    airframe.visual(
        Box((0.24, 0.315, 0.22)),
        origin=Origin(xyz=(-5.00, 0.19, 1.33)),
        material=body_paint,
        name="tail_gearbox",
    )
    airframe.visual(
        Cylinder(radius=0.17, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 1.87)),
        material=body_paint,
        name="mast_pylon",
    )
    airframe.visual(
        mesh_from_geometry(_build_landing_gear_mesh(), "landing_gear"),
        material=metal_dark,
        name="landing_gear",
    )
    airframe.visual(
        Cylinder(radius=0.026, length=0.14),
        origin=Origin(xyz=(-0.55, -0.590, 1.38)),
        material=metal_mid,
        name="door_hinge_upper",
    )
    airframe.visual(
        Cylinder(radius=0.026, length=0.14),
        origin=Origin(xyz=(-0.55, -0.596, 0.92)),
        material=metal_mid,
        name="door_hinge_lower",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((6.60, 1.80, 2.20)),
        mass=750.0,
        origin=Origin(xyz=(-1.95, 0.0, 1.10)),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        mesh_from_geometry(_build_main_rotor_mesh(), "main_rotor_assembly"),
        material=metal_dark,
        name="main_rotor_assembly",
    )
    main_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=3.25, length=0.46),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        mesh_from_geometry(_build_tail_rotor_mesh(), "tail_rotor_assembly"),
        material=metal_dark,
        name="tail_rotor_assembly",
    )
    tail_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.55, length=0.22),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    baggage_door = model.part("baggage_door")
    baggage_door.visual(
        mesh_from_geometry(_build_baggage_door_mesh(0.62, 0.68, 0.03), "baggage_door"),
        material=body_paint,
        name="door_panel",
    )
    baggage_door.visual(
        Cylinder(radius=0.015, length=0.66),
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        material=metal_mid,
        name="hinge_barrel",
    )
    baggage_door.visual(
        Box((0.05, 0.015, 0.06)),
        origin=Origin(xyz=(-0.46, -0.030, 0.02)),
        material=metal_mid,
        name="door_handle",
    )
    baggage_door.inertial = Inertial.from_geometry(
        Box((0.64, 0.04, 0.70)),
        mass=8.0,
        origin=Origin(xyz=(-0.31, -0.02, 0.0)),
    )

    model.articulation(
        "airframe_to_main_rotor",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=main_rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.96)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=60.0),
    )
    model.articulation(
        "airframe_to_tail_rotor",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=tail_rotor,
        origin=Origin(xyz=(-5.04, 0.4575, 1.46)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=90.0),
    )
    model.articulation(
        "airframe_to_baggage_door",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=baggage_door,
        origin=Origin(xyz=(-0.55, -0.625, 1.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    airframe = object_model.get_part("airframe")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    baggage_door = object_model.get_part("baggage_door")

    main_joint = object_model.get_articulation("airframe_to_main_rotor")
    tail_joint = object_model.get_articulation("airframe_to_tail_rotor")
    door_joint = object_model.get_articulation("airframe_to_baggage_door")

    ctx.check(
        "main rotor uses vertical continuous spin",
        main_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(main_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={main_joint.articulation_type}, axis={main_joint.axis}",
    )
    ctx.check(
        "tail rotor uses transverse continuous spin",
        tail_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(tail_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={tail_joint.articulation_type}, axis={tail_joint.axis}",
    )
    ctx.check(
        "baggage door uses vertical hinge",
        door_joint.articulation_type == ArticulationType.REVOLUTE and tuple(door_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={door_joint.articulation_type}, axis={door_joint.axis}",
    )

    ctx.expect_overlap(
        baggage_door,
        airframe,
        axes="xz",
        elem_a="door_panel",
        elem_b="fuselage_lower",
        min_overlap=0.35,
        name="baggage door sits within the aft side bay area",
    )
    ctx.expect_gap(
        airframe,
        baggage_door,
        axis="y",
        positive_elem="fuselage_lower",
        negative_elem="door_panel",
        min_gap=0.0,
        max_gap=0.05,
        name="closed baggage door stays close to the fuselage side",
    )
    ctx.expect_gap(
        main_rotor,
        airframe,
        axis="z",
        positive_elem="main_rotor_assembly",
        negative_elem="mast_pylon",
        max_gap=0.05,
        max_penetration=1e-6,
        name="main rotor mast seats on the pylon without penetrating it",
    )

    closed_door_aabb = ctx.part_world_aabb(baggage_door)
    with ctx.pose({door_joint: math.radians(70.0)}):
        open_door_aabb = ctx.part_world_aabb(baggage_door)
        ctx.expect_origin_gap(
            tail_rotor,
            airframe,
            axis="x",
            min_gap=-6.0,
            max_gap=-4.0,
            name="tail rotor remains aft of the cabin",
        )

    opened_outward = (
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.18
    )
    ctx.check(
        "baggage door swings outward from the starboard side",
        opened_outward,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

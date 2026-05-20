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
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xz_section(
    y: float,
    width: float,
    height: float,
    radius: float,
    *,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_center) for x, z in rounded_rect_profile(width, height, radius, corner_segments=8)]


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


def _add_member(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_helicopter")

    body_paint = model.material("body_paint", rgba=(0.86, 0.15, 0.11, 1.0))
    body_trim = model.material("body_trim", rgba=(0.92, 0.93, 0.94, 1.0))
    window_tint = model.material("window_tint", rgba=(0.36, 0.50, 0.62, 0.38))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    rotor_gray = model.material("rotor_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    skid_black = model.material("skid_black", rgba=(0.09, 0.09, 0.10, 1.0))

    fuselage = model.part("fuselage")
    fuselage.inertial = Inertial.from_geometry(
        Box((1.7, 6.8, 2.2)),
        mass=980.0,
        origin=Origin(xyz=(0.0, -1.75, 1.10)),
    )

    fuselage_shell = section_loft(
        [
            _xz_section(1.45, 0.10, 0.12, 0.03, z_center=1.02),
            _xz_section(1.00, 0.62, 0.48, 0.10, z_center=1.05),
            _xz_section(0.55, 1.20, 0.88, 0.20, z_center=1.18),
            _xz_section(0.05, 1.36, 1.16, 0.22, z_center=1.28),
            _xz_section(-0.55, 1.22, 1.02, 0.18, z_center=1.20),
            _xz_section(-1.10, 0.76, 0.70, 0.14, z_center=1.08),
            _xz_section(-1.55, 0.40, 0.40, 0.09, z_center=1.08),
            _xz_section(-3.20, 0.22, 0.22, 0.06, z_center=1.16),
            _xz_section(-4.88, 0.13, 0.13, 0.04, z_center=1.26),
        ]
    )
    fuselage.visual(_mesh("fuselage_shell", fuselage_shell), material=body_paint, name="shell")

    canopy = section_loft(
        [
            _xz_section(0.92, 0.74, 0.34, 0.08, z_center=1.24),
            _xz_section(0.56, 0.98, 0.56, 0.12, z_center=1.31),
            _xz_section(0.12, 1.00, 0.62, 0.12, z_center=1.34),
        ]
    )
    fuselage.visual(_mesh("canopy_glazing", canopy), material=window_tint, name="canopy")

    fuselage.visual(
        Box((0.56, 0.52, 0.16)),
        origin=Origin(xyz=(0.0, 0.74, 0.78)),
        material=dark_metal,
        name="chin_fairing",
    )
    fuselage.visual(
        Box((0.62, 0.78, 0.24)),
        origin=Origin(xyz=(0.0, -0.52, 1.50)),
        material=body_trim,
        name="engine_cowl",
    )
    fuselage.visual(
        Box((0.46, 0.32, 0.12)),
        origin=Origin(xyz=(0.0, -0.03, 1.64)),
        material=body_trim,
        name="roof_housing",
    )
    fuselage.visual(
        Cylinder(radius=0.095, length=0.24),
        origin=Origin(xyz=(0.0, -0.03, 1.80)),
        material=dark_metal,
        name="mast_cap",
    )
    fuselage.visual(
        Box((0.050, 0.74, 0.64)),
        origin=Origin(xyz=(0.0, -4.42, 1.60)),
        material=body_trim,
        name="tail_fin",
    )
    fuselage.visual(
        Box((0.74, 0.24, 0.050)),
        origin=Origin(xyz=(0.0, -4.03, 1.14)),
        material=body_trim,
        name="stabilizer",
    )
    fuselage.visual(
        Box((0.20, 0.10, 0.10)),
        origin=Origin(xyz=(0.10, -4.92, 1.28)),
        material=dark_metal,
        name="tail_gearbox",
    )
    fuselage.visual(
        Box((0.040, 0.40, 0.40)),
        origin=Origin(xyz=(0.194, -1.89, 1.05)),
        material=body_trim,
        name="hatch_seat",
    )
    fuselage.visual(
        Box((0.028, 0.04, 0.34)),
        origin=Origin(xyz=(0.200, -1.72, 1.05)),
        material=dark_metal,
        name="hatch_hinge_jamb",
    )

    left_skid = tube_from_spline_points(
        [
            (-0.72, 0.88, 0.16),
            (-0.75, 0.34, 0.12),
            (-0.755, 0.22, 0.115),
            (-0.77, -0.30, 0.11),
            (-0.765, -0.72, 0.115),
            (-0.75, -1.04, 0.12),
            (-0.70, -1.72, 0.18),
        ],
        radius=0.038,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    right_skid = tube_from_spline_points(
        [
            (0.72, 0.88, 0.16),
            (0.75, 0.34, 0.12),
            (0.755, 0.22, 0.115),
            (0.77, -0.30, 0.11),
            (0.765, -0.72, 0.115),
            (0.75, -1.04, 0.12),
            (0.70, -1.72, 0.18),
        ],
        radius=0.038,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    fuselage.visual(_mesh("left_skid", left_skid), material=skid_black, name="left_skid")
    fuselage.visual(_mesh("right_skid", right_skid), material=skid_black, name="right_skid")
    fuselage.visual(
        Cylinder(radius=0.030, length=1.44),
        origin=Origin(xyz=(0.0, 0.22, 0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="front_cross_tube",
    )
    fuselage.visual(
        Cylinder(radius=0.030, length=1.44),
        origin=Origin(xyz=(0.0, -0.72, 0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_cross_tube",
    )
    _add_member(
        fuselage,
        (-0.23, 0.22, 0.72),
        (-0.42, 0.22, 0.18),
        radius=0.024,
        material=steel,
        name="front_port_strut",
    )
    _add_member(
        fuselage,
        (0.23, 0.22, 0.72),
        (0.42, 0.22, 0.18),
        radius=0.024,
        material=steel,
        name="front_starboard_strut",
    )
    _add_member(
        fuselage,
        (-0.14, -0.58, 0.78),
        (-0.40, -0.72, 0.18),
        radius=0.024,
        material=steel,
        name="rear_port_strut",
    )
    _add_member(
        fuselage,
        (0.14, -0.58, 0.78),
        (0.40, -0.72, 0.18),
        radius=0.024,
        material=steel,
        name="rear_starboard_strut",
    )
    _add_member(
        fuselage,
        (-0.72, 0.22, 0.18),
        (-0.755, 0.22, 0.115),
        radius=0.024,
        material=steel,
        name="front_port_drop",
    )
    _add_member(
        fuselage,
        (0.72, 0.22, 0.18),
        (0.755, 0.22, 0.115),
        radius=0.024,
        material=steel,
        name="front_starboard_drop",
    )
    _add_member(
        fuselage,
        (-0.72, -0.72, 0.18),
        (-0.765, -0.72, 0.115),
        radius=0.024,
        material=steel,
        name="rear_port_drop",
    )
    _add_member(
        fuselage,
        (0.72, -0.72, 0.18),
        (0.765, -0.72, 0.115),
        radius=0.024,
        material=steel,
        name="rear_starboard_drop",
    )

    main_rotor = model.part("main_rotor")
    main_rotor.inertial = Inertial.from_geometry(
        Box((8.5, 8.5, 0.20)),
        mass=65.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )
    main_rotor.visual(
        Cylinder(radius=0.12, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_metal,
        name="hub",
    )
    main_rotor.visual(
        Cylinder(radius=0.035, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=dark_metal,
        name="shaft",
    )
    main_rotor.visual(
        Box((7.20, 0.18, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=rotor_gray,
        name="blade_bar_x",
    )
    main_rotor.visual(
        Box((0.18, 7.20, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=rotor_gray,
        name="blade_bar_y",
    )
    main_rotor.visual(
        Box((0.78, 0.22, 0.05)),
        origin=Origin(xyz=(0.58, 0.0, 0.055)),
        material=dark_metal,
        name="root_x_pos",
    )
    main_rotor.visual(
        Box((0.78, 0.22, 0.05)),
        origin=Origin(xyz=(-0.58, 0.0, 0.055)),
        material=dark_metal,
        name="root_x_neg",
    )
    main_rotor.visual(
        Box((0.22, 0.78, 0.05)),
        origin=Origin(xyz=(0.0, 0.58, 0.055)),
        material=dark_metal,
        name="root_y_pos",
    )
    main_rotor.visual(
        Box((0.22, 0.78, 0.05)),
        origin=Origin(xyz=(0.0, -0.58, 0.055)),
        material=dark_metal,
        name="root_y_neg",
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.inertial = Inertial.from_geometry(
        Box((0.18, 0.24, 1.40)),
        mass=6.5,
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
    )
    tail_rotor.visual(
        Cylinder(radius=0.07, length=0.08),
        origin=Origin(xyz=(0.04, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub",
    )
    tail_rotor.visual(
        Box((0.06, 0.18, 0.10)),
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
        material=dark_metal,
        name="root",
    )
    tail_rotor.visual(
        Box((0.025, 0.12, 1.34)),
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
        material=rotor_gray,
        name="blade",
    )

    baggage_hatch = model.part("baggage_hatch")
    baggage_hatch.inertial = Inertial.from_geometry(
        Box((0.05, 0.40, 0.38)),
        mass=8.0,
        origin=Origin(xyz=(0.018, -0.17, 0.0)),
    )
    baggage_hatch.visual(
        Box((0.022, 0.34, 0.36)),
        origin=Origin(xyz=(0.0113, -0.17, 0.0)),
        material=body_trim,
        name="panel",
    )
    baggage_hatch.visual(
        Cylinder(radius=0.014, length=0.10),
        origin=Origin(xyz=(0.014, 0.0, 0.12)),
        material=dark_metal,
        name="hinge_upper",
    )
    baggage_hatch.visual(
        Cylinder(radius=0.014, length=0.10),
        origin=Origin(xyz=(0.014, 0.0, -0.12)),
        material=dark_metal,
        name="hinge_lower",
    )
    baggage_hatch.visual(
        Cylinder(radius=0.010, length=0.05),
        origin=Origin(xyz=(0.042, -0.25, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="handle",
    )

    model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=main_rotor,
        origin=Origin(xyz=(0.0, -0.03, 1.92)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=35.0),
    )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=tail_rotor,
        origin=Origin(xyz=(0.20, -4.92, 1.28)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=55.0),
    )
    model.articulation(
        "baggage_hatch_swing",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=baggage_hatch,
        origin=Origin(xyz=(0.214, -1.72, 1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fuselage = object_model.get_part("fuselage")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    baggage_hatch = object_model.get_part("baggage_hatch")

    main_joint = object_model.get_articulation("main_rotor_spin")
    tail_joint = object_model.get_articulation("tail_rotor_spin")
    hatch_joint = object_model.get_articulation("baggage_hatch_swing")

    ctx.check(
        "main rotor uses vertical mast axis",
        tuple(main_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={main_joint.axis}",
    )
    ctx.check(
        "tail rotor uses transverse axis",
        tuple(tail_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={tail_joint.axis}",
    )
    ctx.check(
        "baggage hatch uses vertical side hinge",
        tuple(hatch_joint.axis) == (0.0, 0.0, 1.0)
        and hatch_joint.motion_limits is not None
        and hatch_joint.motion_limits.lower == 0.0
        and hatch_joint.motion_limits.upper is not None
        and hatch_joint.motion_limits.upper >= 1.25,
        details=f"axis={hatch_joint.axis}, limits={hatch_joint.motion_limits}",
    )

    ctx.expect_gap(
        main_rotor,
        fuselage,
        axis="z",
        positive_elem="hub",
        negative_elem="mast_cap",
        min_gap=0.0,
        max_gap=0.10,
        name="main rotor hub sits above mast cap",
    )
    ctx.expect_gap(
        tail_rotor,
        fuselage,
        axis="x",
        positive_elem="hub",
        negative_elem="tail_gearbox",
        min_gap=0.0,
        max_gap=0.02,
        name="tail rotor hub mounts outboard of tail gearbox",
    )
    ctx.expect_gap(
        baggage_hatch,
        fuselage,
        axis="x",
        positive_elem="panel",
        negative_elem="hatch_seat",
        min_gap=0.0,
        max_gap=0.01,
        name="closed baggage hatch sits flush on hatch seat",
    )

    rest_aabb = ctx.part_world_aabb(baggage_hatch)
    with ctx.pose({hatch_joint: 1.10}):
        open_aabb = ctx.part_world_aabb(baggage_hatch)
    ctx.check(
        "baggage hatch swings outward",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > rest_aabb[1][0] + 0.18,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
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


def _yz_section(
    x_pos: float,
    width: float,
    height: float,
    *,
    z_offset: float,
    exponent: float = 2.4,
    samples: int = 32,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y, z + z_offset)
        for y, z in superellipse_profile(width, height, exponent=exponent, segments=samples)
    ]


def _blade_loop(
    span_x: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    return [
        (span_x, -0.50 * chord, 0.00 * thickness),
        (span_x, -0.22 * chord, -0.52 * thickness),
        (span_x, 0.18 * chord, -0.38 * thickness),
        (span_x, 0.50 * chord, 0.00 * thickness),
        (span_x, 0.12 * chord, 0.52 * thickness),
        (span_x, -0.34 * chord, 0.30 * thickness),
    ]


def _tail_fin_section(
    y_pos: float,
    *,
    root_x: float,
    tip_x: float,
    z_base: float,
    z_top: float,
) -> list[tuple[float, float, float]]:
    return [
        (root_x, y_pos, z_base),
        (root_x + 0.040, y_pos, z_base + 0.010),
        (tip_x - 0.020, y_pos, z_top - 0.040),
        (tip_x, y_pos, z_top - 0.014),
        (tip_x - 0.012, y_pos, z_top + 0.006),
        (root_x + 0.010, y_pos, z_base + 0.070),
    ]


def _tail_plane_section(
    z_pos: float,
    *,
    root_x: float,
    tip_x: float,
    half_span: float,
) -> list[tuple[float, float, float]]:
    return [
        (root_x - 0.010, -0.018, z_pos),
        (root_x, -half_span, z_pos),
        (tip_x, -half_span * 0.72, z_pos),
        (tip_x + 0.014, -half_span * 0.20, z_pos),
        (tip_x + 0.014, half_span * 0.20, z_pos),
        (tip_x, half_span * 0.72, z_pos),
        (root_x, half_span, z_pos),
        (root_x - 0.010, 0.018, z_pos),
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_rotor_helicopter_drone")

    body_paint = model.material("body_paint", rgba=(0.84, 0.86, 0.90, 1.0))
    canopy_glass = model.material("canopy_glass", rgba=(0.12, 0.16, 0.20, 0.55))
    carbon_black = model.material("carbon_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.24, 0.27, 1.0))
    light_metal = model.material("light_metal", rgba=(0.64, 0.66, 0.70, 1.0))
    warning_orange = model.material("warning_orange", rgba=(0.90, 0.42, 0.08, 1.0))

    fuselage = model.part("fuselage")

    fuselage_sections = [
        _yz_section(-0.24, 0.022, 0.042, z_offset=0.098, exponent=2.0),
        _yz_section(-0.16, 0.072, 0.110, z_offset=0.113, exponent=2.1),
        _yz_section(-0.03, 0.116, 0.150, z_offset=0.120, exponent=2.4),
        _yz_section(0.10, 0.098, 0.124, z_offset=0.114, exponent=2.5),
        _yz_section(0.20, 0.052, 0.070, z_offset=0.108, exponent=2.2),
        _yz_section(0.25, 0.028, 0.042, z_offset=0.106, exponent=2.0),
    ]
    fuselage.visual(
        _save_mesh(section_loft(fuselage_sections), "fuselage_shell"),
        material=body_paint,
        name="body_shell",
    )

    canopy_sections = [
        _yz_section(-0.17, 0.055, 0.072, z_offset=0.132, exponent=2.2, samples=24),
        _yz_section(-0.08, 0.080, 0.100, z_offset=0.139, exponent=2.4, samples=24),
        _yz_section(0.01, 0.050, 0.060, z_offset=0.131, exponent=2.2, samples=24),
    ]
    fuselage.visual(
        _save_mesh(section_loft(canopy_sections), "canopy_shell"),
        material=canopy_glass,
        name="canopy",
    )

    fuselage.visual(
        Cylinder(radius=0.016, length=0.48),
        origin=Origin(xyz=(0.49, 0.0, 0.106), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=carbon_black,
        name="tail_boom",
    )
    tail_fin_mesh = _save_mesh(
        section_loft(
            [
                _tail_fin_section(-0.0035, root_x=0.612, tip_x=0.722, z_base=0.100, z_top=0.248),
                _tail_fin_section(0.0, root_x=0.612, tip_x=0.722, z_base=0.100, z_top=0.248),
                _tail_fin_section(0.0035, root_x=0.612, tip_x=0.722, z_base=0.100, z_top=0.248),
            ]
        ),
        "tail_fin",
    )
    fuselage.visual(
        tail_fin_mesh,
        material=warning_orange,
        name="tail_fin",
    )
    tail_plane_mesh = _save_mesh(
        section_loft(
            [
                _tail_plane_section(0.1045, root_x=0.586, tip_x=0.664, half_span=0.060),
                _tail_plane_section(0.1080, root_x=0.586, tip_x=0.664, half_span=0.060),
                _tail_plane_section(0.1115, root_x=0.586, tip_x=0.664, half_span=0.060),
            ]
        ),
        "tail_stabilizer",
    )
    fuselage.visual(
        tail_plane_mesh,
        material=warning_orange,
        name="tail_stabilizer",
    )
    fuselage.visual(
        Box((0.050, 0.074, 0.030)),
        origin=Origin(xyz=(0.72, 0.037, 0.160)),
        material=dark_metal,
        name="tail_mount",
    )
    fuselage.visual(
        Box((0.022, 0.030, 0.050)),
        origin=Origin(xyz=(0.702, 0.018, 0.162)),
        material=dark_metal,
        name="tail_mount_bracket",
    )
    fuselage.visual(
        Box((0.20, 0.10, 0.018)),
        origin=Origin(xyz=(-0.03, 0.0, 0.058)),
        material=dark_metal,
        name="belly_frame",
    )

    fuselage.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        material=dark_metal,
        name="mast_fairing",
    )
    fuselage.visual(
        Cylinder(radius=0.013, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.193)),
        material=light_metal,
        name="mast_shaft",
    )
    fuselage.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=dark_metal,
        name="mast_cap",
    )

    _add_member(
        fuselage,
        (-0.19, 0.115, 0.016),
        (0.12, 0.115, 0.016),
        radius=0.008,
        material=light_metal,
        name="left_skid",
    )
    _add_member(
        fuselage,
        (-0.19, -0.115, 0.016),
        (0.12, -0.115, 0.016),
        radius=0.008,
        material=light_metal,
        name="right_skid",
    )
    for side in (-1.0, 1.0):
        _add_member(
            fuselage,
            (-0.19, 0.115 * side, 0.016),
            (-0.23, 0.115 * side, 0.028),
            radius=0.007,
            material=light_metal,
        )
        _add_member(
            fuselage,
            (0.12, 0.115 * side, 0.016),
            (0.16, 0.115 * side, 0.024),
            radius=0.007,
            material=light_metal,
        )

    for side in (-1.0, 1.0):
        _add_member(
            fuselage,
            (-0.10, 0.040 * side, 0.058),
            (-0.12, 0.115 * side, 0.018),
            radius=0.006,
            material=light_metal,
        )
        _add_member(
            fuselage,
            (0.03, 0.036 * side, 0.058),
            (0.05, 0.115 * side, 0.018),
            radius=0.006,
            material=light_metal,
        )

    fuselage.inertial = Inertial.from_geometry(
        Box((0.98, 0.26, 0.30)),
        mass=3.4,
        origin=Origin(xyz=(0.24, 0.0, 0.120)),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.033, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_metal,
        name="main_hub",
    )
    main_rotor.visual(
        Cylinder(radius=0.012, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=light_metal,
        name="main_spindle",
    )
    main_rotor.visual(
        Box((0.060, 0.016, 0.010)),
        origin=Origin(xyz=(0.030, 0.0, 0.010)),
        material=dark_metal,
        name="main_grip_right",
    )
    main_rotor.visual(
        Box((0.060, 0.016, 0.010)),
        origin=Origin(xyz=(-0.030, 0.0, 0.010)),
        material=dark_metal,
        name="main_grip_left",
    )

    main_blade_mesh = _save_mesh(
        section_loft(
            [
                _blade_loop(0.0, 0.040, 0.0050),
                _blade_loop(0.18, 0.032, 0.0042),
                _blade_loop(0.36, 0.023, 0.0032),
            ]
        ),
        "main_rotor_blade",
    )
    main_rotor.visual(
        main_blade_mesh,
        origin=Origin(xyz=(0.030, 0.0, 0.010), rpy=(math.radians(4.0), 0.0, 0.0)),
        material=carbon_black,
        name="main_blade_right",
    )
    main_rotor.visual(
        main_blade_mesh,
        origin=Origin(
            xyz=(-0.030, 0.0, 0.010),
            rpy=(math.radians(4.0), 0.0, math.pi),
        ),
        material=carbon_black,
        name="main_blade_left",
    )
    main_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.41, length=0.060),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.018, length=0.045),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="tail_hub",
    )
    tail_rotor.visual(
        Box((0.010, 0.060, 0.004)),
        origin=Origin(xyz=(0.0, 0.048, 0.0)),
        material=carbon_black,
        name="tail_blade_starboard",
    )
    tail_rotor.visual(
        Box((0.010, 0.060, 0.004)),
        origin=Origin(xyz=(0.0, -0.048, 0.0)),
        material=carbon_black,
        name="tail_blade_port",
    )
    tail_rotor.visual(
        Box((0.010, 0.004, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=carbon_black,
        name="tail_blade_top",
    )
    tail_rotor.visual(
        Box((0.010, 0.004, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=carbon_black,
        name="tail_blade_bottom",
    )
    tail_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.08, length=0.045),
        mass=0.08,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "fuselage_to_main_rotor",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=main_rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=40.0),
    )
    model.articulation(
        "fuselage_to_tail_rotor",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=tail_rotor,
        origin=Origin(xyz=(0.72, 0.092, 0.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fuselage = object_model.get_part("fuselage")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    main_joint = object_model.get_articulation("fuselage_to_main_rotor")
    tail_joint = object_model.get_articulation("fuselage_to_tail_rotor")

    mast_cap = fuselage.get_visual("mast_cap")
    tail_mount = fuselage.get_visual("tail_mount")
    main_hub = main_rotor.get_visual("main_hub")
    tail_hub = tail_rotor.get_visual("tail_hub")
    main_blade = main_rotor.get_visual("main_blade_right")
    tail_blade = tail_rotor.get_visual("tail_blade_starboard")

    ctx.check(
        "main rotor uses a continuous vertical joint",
        main_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(main_joint.axis) == (0.0, 0.0, 1.0)
        and main_joint.motion_limits is not None
        and main_joint.motion_limits.lower is None
        and main_joint.motion_limits.upper is None,
        details=f"type={main_joint.articulation_type}, axis={main_joint.axis}, limits={main_joint.motion_limits}",
    )
    ctx.check(
        "tail rotor uses a continuous horizontal joint",
        tail_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(tail_joint.axis) == (1.0, 0.0, 0.0)
        and tail_joint.motion_limits is not None
        and tail_joint.motion_limits.lower is None
        and tail_joint.motion_limits.upper is None,
        details=f"type={tail_joint.articulation_type}, axis={tail_joint.axis}, limits={tail_joint.motion_limits}",
    )

    with ctx.pose({main_joint: 0.0, tail_joint: 0.0}):
        ctx.expect_gap(
            main_rotor,
            fuselage,
            axis="z",
            positive_elem=main_hub,
            negative_elem=mast_cap,
            max_gap=0.0015,
            max_penetration=0.0,
            name="main rotor hub seats on the mast cap",
        )
        ctx.expect_overlap(
            main_rotor,
            fuselage,
            axes="xy",
            elem_a=main_hub,
            elem_b=mast_cap,
            min_overlap=0.020,
            name="main rotor axis stays centered over the mast",
        )
        ctx.expect_gap(
            tail_rotor,
            fuselage,
            axis="y",
            positive_elem=tail_hub,
            negative_elem=tail_mount,
            max_gap=0.0015,
            max_penetration=0.0,
            name="tail rotor hub seats against the tail mount",
        )
        ctx.expect_overlap(
            tail_rotor,
            fuselage,
            axes="xz",
            elem_a=tail_hub,
            elem_b=tail_mount,
            min_overlap=0.020,
            name="tail rotor axis stays aligned with the tail mount",
        )

        fuselage_pos = ctx.part_world_position(fuselage)
        main_pos = ctx.part_world_position(main_rotor)
        tail_pos = ctx.part_world_position(tail_rotor)
        ctx.check(
            "rotors are mounted above and behind the slim fuselage",
            fuselage_pos is not None
            and main_pos is not None
            and tail_pos is not None
            and main_pos[2] > fuselage_pos[2] + 0.22
            and tail_pos[0] > fuselage_pos[0] + 0.65
            and tail_pos[1] > fuselage_pos[1] + 0.07,
            details=f"fuselage={fuselage_pos}, main={main_pos}, tail={tail_pos}",
        )

    with ctx.pose({main_joint: 0.0}):
        main_blade_rest = _aabb_center(ctx.part_element_world_aabb(main_rotor, elem=main_blade))
    with ctx.pose({main_joint: math.pi / 2.0}):
        main_blade_quarter = _aabb_center(ctx.part_element_world_aabb(main_rotor, elem=main_blade))
    ctx.check(
        "main rotor blade sweeps in the horizontal plane about the mast",
        main_blade_rest is not None
        and main_blade_quarter is not None
        and main_blade_rest[0] > 0.15
        and abs(main_blade_rest[1]) < 0.03
        and abs(main_blade_quarter[0]) < 0.03
        and main_blade_quarter[1] > 0.15
        and abs(main_blade_rest[2] - main_blade_quarter[2]) < 0.03,
        details=f"rest={main_blade_rest}, quarter_turn={main_blade_quarter}",
    )

    with ctx.pose({tail_joint: 0.0}):
        tail_blade_rest = _aabb_center(ctx.part_element_world_aabb(tail_rotor, elem=tail_blade))
    with ctx.pose({tail_joint: math.pi / 2.0}):
        tail_blade_quarter = _aabb_center(ctx.part_element_world_aabb(tail_rotor, elem=tail_blade))
    ctx.check(
        "tail rotor blade sweeps in the vertical plane about the horizontal tail axis",
        tail_blade_rest is not None
        and tail_blade_quarter is not None
        and tail_blade_rest[1] > 0.11
        and abs(tail_blade_rest[2] - 0.160) < 0.03
        and abs(tail_blade_quarter[1] - 0.090) < 0.03
        and tail_blade_quarter[2] > 0.20
        and abs(tail_blade_rest[0] - tail_blade_quarter[0]) < 0.02,
        details=f"rest={tail_blade_rest}, quarter_turn={tail_blade_quarter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

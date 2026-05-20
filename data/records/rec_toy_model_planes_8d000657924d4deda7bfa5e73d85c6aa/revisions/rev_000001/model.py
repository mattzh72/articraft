from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    Inertial,
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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_superellipse_section(
    x: float,
    width: float,
    height: float,
    z_center: float,
    *,
    exponent: float = 2.45,
    segments: int = 32,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_center + z)
        for y, z in superellipse_profile(width, height, exponent=exponent, segments=segments)
    ]


def _airfoil_section_at_y(
    y: float,
    chord: float,
    thickness: float,
    x_le: float,
    z0: float,
) -> list[tuple[float, float, float]]:
    profile = (
        (0.00, 0.00),
        (0.05, 0.55),
        (0.18, 0.92),
        (0.42, 1.00),
        (0.70, 0.62),
        (0.92, 0.16),
        (1.00, 0.03),
        (0.94, -0.02),
        (0.72, -0.10),
        (0.42, -0.15),
        (0.18, -0.13),
        (0.05, -0.06),
    )
    return [(x_le + chord * u, y, z0 + thickness * v) for u, v in profile]


def _fin_section_at_z(
    z: float,
    chord: float,
    thickness: float,
    x_le: float,
    y0: float,
) -> list[tuple[float, float, float]]:
    profile = (
        (0.00, 0.00),
        (0.10, 0.55),
        (0.34, 0.92),
        (0.68, 1.00),
        (0.92, 0.35),
        (1.00, 0.06),
        (0.94, -0.02),
        (0.68, -0.08),
        (0.28, -0.10),
        (0.06, -0.04),
    )
    return [(x_le + chord * u, y0 + thickness * v, z) for u, v in profile]


def _add_side_hatch_bolts(
    part,
    *,
    side_y: float,
    z_center: float,
    x_positions: tuple[float, ...],
    radius: float,
    material,
    prefix: str,
) -> None:
    for index, x in enumerate(x_positions):
        part.visual(
            Cylinder(radius=radius, length=0.003),
            origin=Origin(
                xyz=(x, side_y, z_center),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=material,
            name=f"{prefix}_{index}",
        )


def _add_top_hatch_bolts(
    part,
    *,
    z: float,
    x_positions: tuple[float, ...],
    y_positions: tuple[float, ...],
    radius: float,
    material,
    prefix: str,
) -> None:
    count = 0
    for x in x_positions:
        for y in y_positions:
            part.visual(
                Cylinder(radius=radius, length=0.003),
                origin=Origin(xyz=(x, y, z)),
                material=material,
                name=f"{prefix}_{count}",
            )
            count += 1


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_model_plane")

    cream = model.material("cream_dope", rgba=(0.90, 0.87, 0.76, 1.0))
    crimson = model.material("crimson_trim", rgba=(0.67, 0.12, 0.12, 1.0))
    wood = model.material("varnished_wood", rgba=(0.58, 0.38, 0.20, 1.0))
    aluminum = model.material("machined_aluminum", rgba=(0.71, 0.74, 0.78, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.24, 0.26, 0.29, 1.0))
    stand_black = model.material("stand_black", rgba=(0.12, 0.12, 0.13, 1.0))
    canopy_smoke = model.material("canopy_smoke", rgba=(0.50, 0.62, 0.68, 0.45))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    stand = model.part("display_stand")
    stand.visual(
        Box((0.180, 0.120, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=stand_black,
        name="base_plate",
    )
    stand.visual(
        Box((0.092, 0.060, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=gunmetal,
        name="base_plinth",
    )
    stand.visual(
        Cylinder(radius=0.013, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=gunmetal,
        name="center_column",
    )
    stand.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=stand_black,
        name="yoke_collar",
    )
    stand.visual(
        Box((0.012, 0.022, 0.054)),
        origin=Origin(xyz=(-0.024, 0.041, 0.171)),
        material=gunmetal,
        name="left_yoke_arm",
    )
    stand.visual(
        Box((0.012, 0.022, 0.054)),
        origin=Origin(xyz=(-0.024, -0.041, 0.171)),
        material=gunmetal,
        name="right_yoke_arm",
    )
    stand.visual(
        Box((0.026, 0.010, 0.018)),
        origin=Origin(xyz=(-0.016, 0.018, 0.161)),
        material=gunmetal,
        name="left_lower_bracket",
    )
    stand.visual(
        Box((0.026, 0.010, 0.018)),
        origin=Origin(xyz=(-0.016, -0.018, 0.161)),
        material=gunmetal,
        name="right_lower_bracket",
    )
    stand.visual(
        Box((0.012, 0.016, 0.024)),
        origin=Origin(xyz=(-0.024, 0.029, 0.169)),
        material=gunmetal,
        name="left_upright_brace",
    )
    stand.visual(
        Box((0.012, 0.016, 0.024)),
        origin=Origin(xyz=(-0.024, -0.029, 0.169)),
        material=gunmetal,
        name="right_upright_brace",
    )
    stand.visual(
        Box((0.036, 0.008, 0.016)),
        origin=Origin(xyz=(-0.022, 0.0, 0.160)),
        material=gunmetal,
        name="yoke_crossbrace",
    )
    stand.visual(
        Box((0.026, 0.016, 0.026)),
        origin=Origin(xyz=(-0.018, 0.043, 0.190)),
        material=aluminum,
        name="left_pivot_pad",
    )
    stand.visual(
        Box((0.026, 0.016, 0.026)),
        origin=Origin(xyz=(-0.018, -0.043, 0.190)),
        material=aluminum,
        name="right_pivot_pad",
    )
    for bolt_index, (bx, by) in enumerate(
        ((0.058, 0.038), (0.058, -0.038), (-0.058, 0.038), (-0.058, -0.038))
    ):
        stand.visual(
            Cylinder(radius=0.005, length=0.005),
            origin=Origin(xyz=(bx, by, 0.0185)),
            material=aluminum,
            name=f"base_bolt_{bolt_index}",
        )
    stand.visual(
        Box((0.052, 0.010, 0.030)),
        origin=Origin(xyz=(0.018, 0.0, 0.026)),
        material=gunmetal,
        name="front_reinforcement_bar",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.180, 0.120, 0.200)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
    )

    fuselage = model.part("fuselage")
    fuselage_shell = section_loft(
        [
            _yz_superellipse_section(0.208, 0.016, 0.020, 0.020, exponent=2.1),
            _yz_superellipse_section(0.172, 0.048, 0.060, 0.035, exponent=2.2),
            _yz_superellipse_section(0.092, 0.074, 0.086, 0.050, exponent=2.45),
            _yz_superellipse_section(0.010, 0.078, 0.090, 0.052, exponent=2.55),
            _yz_superellipse_section(-0.080, 0.058, 0.068, 0.044, exponent=2.45),
            _yz_superellipse_section(-0.170, 0.026, 0.038, 0.028, exponent=2.2),
            _yz_superellipse_section(-0.220, 0.010, 0.016, 0.020, exponent=2.0),
        ]
    )
    fuselage.visual(
        _save_mesh("plane_fuselage_shell", fuselage_shell),
        material=cream,
        name="fuselage_shell",
    )
    fuselage.visual(
        Box((0.060, 0.030, 0.014)),
        origin=Origin(xyz=(0.000, 0.0, 0.014)),
        material=aluminum,
        name="belly_adapter",
    )
    fuselage.visual(
        Cylinder(radius=0.013, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="pivot_barrel",
    )
    fuselage.visual(
        Box((0.148, 0.088, 0.012)),
        origin=Origin(xyz=(0.006, 0.0, 0.096)),
        material=aluminum,
        name="wing_saddle",
    )
    fuselage.visual(
        Box((0.124, 0.010, 0.022)),
        origin=Origin(xyz=(0.000, 0.036, 0.056)),
        material=aluminum,
        name="left_wing_doubler",
    )
    fuselage.visual(
        Box((0.124, 0.010, 0.022)),
        origin=Origin(xyz=(0.000, -0.036, 0.056)),
        material=aluminum,
        name="right_wing_doubler",
    )
    fuselage.visual(
        Box((0.022, 0.028, 0.012)),
        origin=Origin(xyz=(-0.194, 0.0, 0.046)),
        material=aluminum,
        name="tail_mount_pad",
    )
    fuselage.visual(
        Box((0.050, 0.024, 0.020)),
        origin=Origin(xyz=(-0.179, 0.0, 0.040)),
        material=aluminum,
        name="tail_reinforcement_block",
    )
    fuselage.visual(
        Box((0.095, 0.004, 0.032)),
        origin=Origin(xyz=(0.025, 0.040, 0.045)),
        material=aluminum,
        name="left_service_hatch",
    )
    fuselage.visual(
        Box((0.095, 0.004, 0.032)),
        origin=Origin(xyz=(0.025, -0.040, 0.045)),
        material=aluminum,
        name="right_service_hatch",
    )
    fuselage.visual(
        Box((0.092, 0.050, 0.004)),
        origin=Origin(xyz=(0.005, 0.0, 0.097)),
        material=aluminum,
        name="top_service_hatch",
    )
    _add_side_hatch_bolts(
        fuselage,
        side_y=0.0435,
        z_center=0.045,
        x_positions=(-0.012, 0.020, 0.052, 0.068),
        radius=0.0028,
        material=gunmetal,
        prefix="left_hatch_bolt",
    )
    _add_side_hatch_bolts(
        fuselage,
        side_y=-0.0435,
        z_center=0.045,
        x_positions=(-0.012, 0.020, 0.052, 0.068),
        radius=0.0028,
        material=gunmetal,
        prefix="right_hatch_bolt",
    )
    _add_top_hatch_bolts(
        fuselage,
        z=0.0995,
        x_positions=(-0.028, 0.040),
        y_positions=(-0.018, 0.018),
        radius=0.0028,
        material=gunmetal,
        prefix="top_hatch_bolt",
    )
    fuselage.visual(
        Box((0.078, 0.040, 0.026)),
        origin=Origin(xyz=(0.055, 0.0, 0.084)),
        material=canopy_smoke,
        name="cockpit_canopy",
    )
    fuselage.visual(
        Box((0.034, 0.040, 0.018)),
        origin=Origin(xyz=(0.095, 0.0, 0.078), rpy=(0.0, -0.28, 0.0)),
        material=canopy_smoke,
        name="windscreen",
    )
    fuselage.visual(
        Cylinder(radius=0.026, length=0.022),
        origin=Origin(xyz=(0.184, 0.0, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=crimson,
        name="nose_cowl_band",
    )
    fuselage.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.211, 0.0, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="nose_bushing",
    )
    fuselage.visual(
        Box((0.090, 0.010, 0.018)),
        origin=Origin(xyz=(-0.030, 0.0, 0.078)),
        material=crimson,
        name="dorsal_trim_band",
    )
    fuselage.inertial = Inertial.from_geometry(
        Box((0.460, 0.090, 0.120)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    wing_set = model.part("wing_set")
    left_wing = section_loft(
        [
            _airfoil_section_at_y(0.060, 0.160, 0.012, -0.055, 0.010),
            _airfoil_section_at_y(0.190, 0.112, 0.010, -0.030, 0.023),
            _airfoil_section_at_y(0.310, 0.070, 0.007, 0.004, 0.036),
        ]
    )
    right_wing = section_loft(
        [
            _airfoil_section_at_y(-0.060, 0.160, 0.012, -0.055, 0.010),
            _airfoil_section_at_y(-0.190, 0.112, 0.010, -0.030, 0.023),
            _airfoil_section_at_y(-0.310, 0.070, 0.007, 0.004, 0.036),
        ]
    )
    wing_set.visual(
        _save_mesh("plane_left_wing", left_wing),
        material=cream,
        name="left_wing_panel",
    )
    wing_set.visual(
        _save_mesh("plane_right_wing", right_wing),
        material=cream,
        name="right_wing_panel",
    )
    wing_set.visual(
        Box((0.150, 0.160, 0.018)),
        origin=Origin(xyz=(0.016, 0.0, 0.009)),
        material=aluminum,
        name="center_spar",
    )
    wing_set.visual(
        Box((0.110, 0.050, 0.004)),
        origin=Origin(xyz=(0.010, 0.052, 0.002)),
        material=aluminum,
        name="left_adapter_plate",
    )
    wing_set.visual(
        Box((0.110, 0.050, 0.004)),
        origin=Origin(xyz=(0.010, -0.052, 0.002)),
        material=aluminum,
        name="right_adapter_plate",
    )
    for index, y in enumerate((0.036, 0.068, -0.036, -0.068)):
        wing_set.visual(
            Cylinder(radius=0.003, length=0.004),
            origin=Origin(xyz=(0.008, y, 0.004)),
            material=gunmetal,
            name=f"wing_adapter_bolt_{index}",
        )
    wing_set.visual(
        Box((0.026, 0.030, 0.008)),
        origin=Origin(xyz=(0.050, 0.296, 0.036)),
        material=crimson,
        name="left_tip_cuff",
    )
    wing_set.visual(
        Box((0.026, 0.030, 0.008)),
        origin=Origin(xyz=(0.050, -0.296, 0.036)),
        material=crimson,
        name="right_tip_cuff",
    )
    wing_set.inertial = Inertial.from_geometry(
        Box((0.170, 0.640, 0.060)),
        mass=0.24,
        origin=Origin(xyz=(0.020, 0.0, 0.026)),
    )

    tail_set = model.part("tail_set")
    left_tail = section_loft(
        [
            _airfoil_section_at_y(0.040, 0.065, 0.006, -0.075, 0.012),
            _airfoil_section_at_y(0.086, 0.046, 0.0045, -0.060, 0.014),
            _airfoil_section_at_y(0.122, 0.030, 0.003, -0.048, 0.016),
        ]
    )
    right_tail = section_loft(
        [
            _airfoil_section_at_y(-0.040, 0.065, 0.006, -0.075, 0.012),
            _airfoil_section_at_y(-0.086, 0.046, 0.0045, -0.060, 0.014),
            _airfoil_section_at_y(-0.122, 0.030, 0.003, -0.048, 0.016),
        ]
    )
    fin_taper = section_loft(
        [
            _fin_section_at_z(0.014, 0.050, 0.012, -0.058, 0.0),
            _fin_section_at_z(0.070, 0.034, 0.009, -0.046, 0.0),
            _fin_section_at_z(0.120, 0.014, 0.004, -0.020, 0.0),
        ]
    )
    tail_set.visual(
        Box((0.028, 0.024, 0.012)),
        origin=Origin(xyz=(-0.014, 0.0, 0.006)),
        material=aluminum,
        name="tail_root_plate",
    )
    tail_set.visual(
        Box((0.082, 0.082, 0.014)),
        origin=Origin(xyz=(-0.052, 0.0, 0.013)),
        material=aluminum,
        name="tail_center_hub",
    )
    tail_set.visual(_save_mesh("plane_left_tail", left_tail), material=cream, name="left_stabilizer")
    tail_set.visual(_save_mesh("plane_right_tail", right_tail), material=cream, name="right_stabilizer")
    tail_set.visual(
        Box((0.022, 0.012, 0.060)),
        origin=Origin(xyz=(-0.052, 0.0, 0.042)),
        material=aluminum,
        name="fin_root_brace",
    )
    tail_set.visual(
        _save_mesh("plane_tail_fin_taper", fin_taper),
        material=cream,
        name="vertical_fin",
    )
    tail_set.visual(
        Box((0.024, 0.010, 0.046)),
        origin=Origin(xyz=(-0.022, 0.0, 0.095)),
        material=aluminum,
        name="fin_upper_fairing",
    )
    tail_set.visual(
        Box((0.014, 0.010, 0.010)),
        origin=Origin(xyz=(-0.010, 0.0, 0.123)),
        material=crimson,
        name="tail_tip_accent",
    )
    for index, y in enumerate((0.024, -0.024)):
        tail_set.visual(
            Cylinder(radius=0.0028, length=0.004),
            origin=Origin(xyz=(-0.014, y, 0.006)),
            material=gunmetal,
            name=f"tail_bolt_{index}",
        )
    tail_set.inertial = Inertial.from_geometry(
        Box((0.100, 0.270, 0.105)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    propeller = model.part("propeller")
    spinner = ConeGeometry(radius=0.016, height=0.040, radial_segments=24, closed=True)
    spinner.rotate_y(math.pi / 2.0).translate(0.028, 0.0, 0.0)
    propeller.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="hub_backplate",
    )
    propeller.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="hub_core",
    )
    propeller.visual(
        _save_mesh("plane_spinner", spinner),
        material=aluminum,
        name="spinner",
    )
    propeller.visual(
        Box((0.014, 0.078, 0.005)),
        origin=Origin(xyz=(0.018, 0.040, 0.0), rpy=(0.22, 0.10, 0.0)),
        material=wood,
        name="blade_upper",
    )
    propeller.visual(
        Box((0.014, 0.078, 0.005)),
        origin=Origin(xyz=(0.018, -0.040, 0.0), rpy=(-0.22, 0.10, 0.0)),
        material=wood,
        name="blade_lower",
    )
    propeller.visual(
        Box((0.010, 0.034, 0.004)),
        origin=Origin(xyz=(0.020, 0.060, 0.0), rpy=(0.22, 0.10, 0.0)),
        material=wood,
        name="blade_upper_tip",
    )
    propeller.visual(
        Box((0.010, 0.034, 0.004)),
        origin=Origin(xyz=(0.020, -0.060, 0.0), rpy=(-0.22, 0.10, 0.0)),
        material=wood,
        name="blade_lower_tip",
    )
    propeller.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="hub_bushing",
    )
    propeller.inertial = Inertial.from_geometry(
        Box((0.070, 0.160, 0.020)),
        mass=0.05,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    model.articulation(
        "stand_pitch",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=fuselage,
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.0,
            lower=math.radians(-10.0),
            upper=math.radians(25.0),
        ),
    )
    model.articulation(
        "fuselage_to_wing",
        ArticulationType.FIXED,
        parent=fuselage,
        child=wing_set,
        origin=Origin(xyz=(0.000, 0.0, 0.102)),
    )
    model.articulation(
        "fuselage_to_tail",
        ArticulationType.FIXED,
        parent=fuselage,
        child=tail_set,
        origin=Origin(xyz=(-0.205, 0.0, 0.040)),
    )
    model.articulation(
        "propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=propeller,
        origin=Origin(xyz=(0.226, 0.0, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("display_stand")
    fuselage = object_model.get_part("fuselage")
    wing_set = object_model.get_part("wing_set")
    tail_set = object_model.get_part("tail_set")
    propeller = object_model.get_part("propeller")
    stand_pitch = object_model.get_articulation("stand_pitch")
    propeller_spin = object_model.get_articulation("propeller_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        fuselage,
        stand,
        elem_a="pivot_barrel",
        elem_b="left_pivot_pad",
        name="left trunnion pad supports pivot barrel",
    )
    ctx.expect_contact(
        fuselage,
        stand,
        elem_a="pivot_barrel",
        elem_b="right_pivot_pad",
        name="right trunnion pad supports pivot barrel",
    )
    ctx.expect_contact(
        wing_set,
        fuselage,
        elem_a="center_spar",
        elem_b="wing_saddle",
        name="wing center spar seats on fuselage saddle",
    )
    ctx.expect_contact(
        tail_set,
        fuselage,
        elem_a="tail_root_plate",
        elem_b="tail_mount_pad",
        name="tail root plate seats on tail mount pad",
    )
    ctx.expect_contact(
        propeller,
        fuselage,
        elem_a="hub_backplate",
        elem_b="nose_bushing",
        name="propeller hub seats on nose bushing",
    )

    ctx.check(
        "stand pitch articulation is nose-up",
        tuple(round(value, 4) for value in stand_pitch.axis) == (0.0, -1.0, 0.0),
        f"expected axis (0, -1, 0), got {stand_pitch.axis}",
    )
    ctx.check(
        "propeller spins around longitudinal axis",
        tuple(round(value, 4) for value in propeller_spin.axis) == (1.0, 0.0, 0.0),
        f"expected axis (1, 0, 0), got {propeller_spin.axis}",
    )

    with ctx.pose({stand_pitch: 0.0}):
        closed_prop_pos = ctx.part_world_position(propeller)
    with ctx.pose({stand_pitch: math.radians(20.0)}):
        raised_prop_pos = ctx.part_world_position(propeller)
    ctx.check(
        "stand pitch raises propeller arc",
        closed_prop_pos is not None
        and raised_prop_pos is not None
        and raised_prop_pos[2] > closed_prop_pos[2] + 0.045,
        (
            f"closed={closed_prop_pos}, raised={raised_prop_pos}; "
            "positive stand pitch should lift the nose noticeably"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

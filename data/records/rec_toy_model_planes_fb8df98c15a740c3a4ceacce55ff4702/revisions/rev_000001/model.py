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
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _section_at_x(
    x_pos: float,
    width: float,
    height: float,
    corner_radius: float,
) -> tuple[tuple[float, float, float], ...]:
    profile = rounded_rect_profile(
        width,
        height,
        corner_radius,
        corner_segments=8,
    )
    return tuple((x_pos, y_pos, z_pos) for y_pos, z_pos in profile)


def _wing_profile(span: float, *, left: bool) -> list[tuple[float, float]]:
    if left:
        return [
            (0.080, 0.0),
            (-0.074, 0.0),
            (-0.024, -span),
            (0.036, -span),
        ]
    return [
        (0.080, 0.0),
        (0.036, span),
        (-0.024, span),
        (-0.074, 0.0),
    ]


def _tail_profile(span: float, *, left: bool) -> list[tuple[float, float]]:
    if left:
        return [
            (0.046, 0.0),
            (-0.050, 0.0),
            (-0.022, -span),
            (0.024, -span),
        ]
    return [
        (0.046, 0.0),
        (0.024, span),
        (-0.022, span),
        (-0.050, 0.0),
    ]


def _plate_mesh(profile: list[tuple[float, float]], thickness: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry.centered(profile, thickness, cap=True, closed=True),
        name,
    )


def _vertical_fin_mesh(name: str):
    fin_geom = ExtrudeGeometry.centered(
        [
            (0.030, 0.000),
            (0.046, 0.092),
            (-0.006, 0.112),
            (-0.052, 0.000),
        ],
        0.014,
        cap=True,
        closed=True,
    )
    fin_geom.rotate_x(pi / 2.0)
    return mesh_from_geometry(fin_geom, name)


def _prop_blade_mesh(name: str):
    blade_geom = ExtrudeGeometry.centered(
        [
            (0.014, 0.000),
            (0.018, 0.022),
            (0.016, 0.060),
            (0.010, 0.102),
            (-0.004, 0.112),
            (-0.010, 0.072),
            (-0.008, 0.020),
        ],
        0.010,
        cap=True,
        closed=True,
    )
    blade_geom.rotate_y(pi / 2.0)
    return mesh_from_geometry(blade_geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_model_plane")

    airframe = model.material("airframe", rgba=(0.42, 0.47, 0.50, 1.0))
    service_yellow = model.material("service_yellow", rgba=(0.77, 0.58, 0.18, 1.0))
    hardware = model.material("hardware", rgba=(0.63, 0.65, 0.68, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    canopy = model.material("canopy", rgba=(0.27, 0.34, 0.39, 1.0))
    prop_black = model.material("prop_black", rgba=(0.10, 0.10, 0.11, 1.0))

    fuselage = model.part("fuselage")
    fuselage.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _section_at_x(0.288, 0.036, 0.036, 0.012),
                    _section_at_x(0.208, 0.096, 0.092, 0.024),
                    _section_at_x(0.056, 0.130, 0.122, 0.030),
                    _section_at_x(-0.112, 0.100, 0.096, 0.024),
                    _section_at_x(-0.246, 0.058, 0.062, 0.014),
                    _section_at_x(-0.316, 0.030, 0.040, 0.010),
                ]
            ),
            "fuselage_shell",
        ),
        material=airframe,
        name="shell",
    )
    fuselage.visual(
        Box((0.138, 0.082, 0.036)),
        origin=Origin(xyz=(0.072, 0.0, 0.068)),
        material=canopy,
        name="canopy_block",
    )
    fuselage.visual(
        Box((0.182, 0.040, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, -0.054)),
        material=panel_dark,
        name="belly_skid",
    )
    fuselage.visual(
        Box((0.068, 0.018, 0.030)),
        origin=Origin(xyz=(0.030, 0.066, 0.004)),
        material=hardware,
        name="right_wing_saddle",
    )
    fuselage.visual(
        Box((0.068, 0.018, 0.030)),
        origin=Origin(xyz=(0.030, -0.066, 0.004)),
        material=hardware,
        name="left_wing_saddle",
    )
    fuselage.visual(
        Box((0.042, 0.018, 0.018)),
        origin=Origin(xyz=(-0.248, 0.023, 0.018)),
        material=hardware,
        name="right_tail_saddle",
    )
    fuselage.visual(
        Box((0.042, 0.018, 0.018)),
        origin=Origin(xyz=(-0.248, -0.023, 0.018)),
        material=hardware,
        name="left_tail_saddle",
    )
    fuselage.visual(
        Box((0.050, 0.024, 0.016)),
        origin=Origin(xyz=(-0.234, 0.0, 0.038)),
        material=hardware,
        name="fin_mount",
    )
    fuselage.visual(
        Box((0.060, 0.150, 0.012)),
        origin=Origin(xyz=(-0.294, 0.0, 0.018)),
        material=hardware,
        name="tail_carry_beam",
    )
    fuselage.visual(
        Box((0.118, 0.006, 0.014)),
        origin=Origin(xyz=(0.158, -0.062, 0.034)),
        material=hardware,
        name="panel_rail_upper",
    )
    fuselage.visual(
        Box((0.118, 0.006, 0.014)),
        origin=Origin(xyz=(0.158, -0.062, -0.020)),
        material=hardware,
        name="panel_rail_lower",
    )
    fuselage.visual(
        Box((0.008, 0.006, 0.066)),
        origin=Origin(xyz=(0.100, -0.062, 0.007)),
        material=hardware,
        name="panel_rail_front",
    )
    fuselage.visual(
        Box((0.008, 0.006, 0.066)),
        origin=Origin(xyz=(0.216, -0.062, 0.007)),
        material=hardware,
        name="panel_rail_rear",
    )
    fuselage.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.290, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware,
        name="nose_bearing",
    )
    fuselage.visual(
        Box((0.032, 0.064, 0.010)),
        origin=Origin(xyz=(0.078, 0.0, -0.053)),
        material=hardware,
        name="stand_crossmember",
    )
    fuselage.visual(
        Box((0.018, 0.014, 0.036)),
        origin=Origin(xyz=(0.078, 0.031, -0.052)),
        material=hardware,
        name="stand_bracket_right",
    )
    fuselage.visual(
        Box((0.018, 0.014, 0.036)),
        origin=Origin(xyz=(0.078, -0.031, -0.052)),
        material=hardware,
        name="stand_bracket_left",
    )
    fuselage.visual(
        Cylinder(radius=0.004, length=0.068),
        origin=Origin(xyz=(0.078, 0.0, -0.070), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="stand_pivot_pin",
    )
    for bolt_x in (0.010, 0.050):
        fuselage.visual(
            Cylinder(radius=0.0055, length=0.006),
            origin=Origin(xyz=(bolt_x, 0.072, 0.014), rpy=(pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"right_wing_bolt_{int((bolt_x + 0.01) * 1000)}",
        )
        fuselage.visual(
            Cylinder(radius=0.0055, length=0.006),
            origin=Origin(xyz=(bolt_x, -0.072, 0.014), rpy=(pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"left_wing_bolt_{int((bolt_x + 0.01) * 1000)}",
        )

    left_wing = model.part("left_wing")
    left_wing.visual(
        _plate_mesh(_wing_profile(0.312, left=True), 0.018, "left_wing_panel"),
        material=airframe,
        name="panel",
    )
    left_wing.visual(
        Box((0.094, 0.034, 0.024)),
        origin=Origin(xyz=(0.004, -0.017, 0.0)),
        material=hardware,
        name="root_block",
    )

    right_wing = model.part("right_wing")
    right_wing.visual(
        _plate_mesh(_wing_profile(0.312, left=False), 0.018, "right_wing_panel"),
        material=airframe,
        name="panel",
    )
    right_wing.visual(
        Box((0.094, 0.034, 0.024)),
        origin=Origin(xyz=(0.004, 0.017, 0.0)),
        material=hardware,
        name="root_block",
    )

    left_tail = model.part("left_tail")
    left_tail.visual(
        _plate_mesh(_tail_profile(0.122, left=True), 0.012, "left_tail_plane"),
        material=service_yellow,
        name="plane",
    )
    left_tail.visual(
        Box((0.054, 0.026, 0.016)),
        origin=Origin(xyz=(-0.002, -0.013, 0.0)),
        material=hardware,
        name="root_block",
    )

    right_tail = model.part("right_tail")
    right_tail.visual(
        _plate_mesh(_tail_profile(0.122, left=False), 0.012, "right_tail_plane"),
        material=service_yellow,
        name="plane",
    )
    right_tail.visual(
        Box((0.054, 0.026, 0.016)),
        origin=Origin(xyz=(-0.002, 0.013, 0.0)),
        material=hardware,
        name="root_block",
    )

    vertical_tail = model.part("vertical_tail")
    vertical_tail.visual(
        _vertical_fin_mesh("vertical_tail_fin"),
        material=service_yellow,
        name="fin",
    )
    vertical_tail.visual(
        Box((0.060, 0.020, 0.024)),
        origin=Origin(xyz=(-0.004, 0.0, 0.012)),
        material=hardware,
        name="root_block",
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((0.122, 0.008, 0.070)),
        material=service_yellow,
        name="door",
    )
    service_panel.visual(
        Box((0.020, 0.010, 0.020)),
        origin=Origin(xyz=(-0.036, -0.009, 0.020)),
        material=panel_dark,
        name="front_latch",
    )
    service_panel.visual(
        Box((0.020, 0.010, 0.020)),
        origin=Origin(xyz=(0.036, -0.009, -0.016)),
        material=panel_dark,
        name="rear_latch",
    )
    service_panel.visual(
        Box((0.050, 0.010, 0.012)),
        origin=Origin(xyz=(0.000, -0.009, 0.000)),
        material=hardware,
        name="pull_handle",
    )

    propeller = model.part("propeller")
    prop_blade = _prop_blade_mesh("prop_blade")
    propeller.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware,
        name="hub",
    )
    propeller.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware,
        name="retainer",
    )
    propeller.visual(
        prop_blade,
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(0.0, 0.20, 0.0)),
        material=prop_black,
        name="blade_0",
    )
    propeller.visual(
        prop_blade,
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(2.0 * pi / 3.0, 0.20, 0.0)),
        material=prop_black,
        name="blade_1",
    )
    propeller.visual(
        prop_blade,
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(4.0 * pi / 3.0, 0.20, 0.0)),
        material=prop_black,
        name="blade_2",
    )

    service_stand = model.part("service_stand")
    service_stand.visual(
        Cylinder(radius=0.0065, length=0.046),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="hinge_barrel",
    )
    service_stand.visual(
        Box((0.026, 0.046, 0.010)),
        origin=Origin(xyz=(-0.010, 0.0, -0.008)),
        material=hardware,
        name="hinge_saddle",
    )
    service_stand.visual(
        Box((0.018, 0.012, 0.020)),
        origin=Origin(xyz=(-0.010, 0.024, -0.015)),
        material=hardware,
        name="right_hinge_cheek",
    )
    service_stand.visual(
        Box((0.018, 0.012, 0.020)),
        origin=Origin(xyz=(-0.010, -0.024, -0.015)),
        material=hardware,
        name="left_hinge_cheek",
    )
    service_stand.visual(
        Box((0.090, 0.012, 0.018)),
        origin=Origin(xyz=(-0.053, 0.024, -0.024)),
        material=panel_dark,
        name="right_leg",
    )
    service_stand.visual(
        Box((0.090, 0.012, 0.018)),
        origin=Origin(xyz=(-0.053, -0.024, -0.024)),
        material=panel_dark,
        name="left_leg",
    )
    service_stand.visual(
        Box((0.014, 0.058, 0.014)),
        origin=Origin(xyz=(-0.094, 0.0, -0.024)),
        material=hardware,
        name="tie_bar",
    )
    service_stand.visual(
        Cylinder(radius=0.010, length=0.074),
        origin=Origin(xyz=(-0.105, 0.0, -0.034), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="foot_pad",
    )

    model.articulation(
        "fuselage_to_left_wing",
        ArticulationType.FIXED,
        parent=fuselage,
        child=left_wing,
        origin=Origin(xyz=(0.030, -0.075, 0.004)),
    )
    model.articulation(
        "fuselage_to_right_wing",
        ArticulationType.FIXED,
        parent=fuselage,
        child=right_wing,
        origin=Origin(xyz=(0.030, 0.075, 0.004)),
    )
    model.articulation(
        "fuselage_to_left_tail",
        ArticulationType.FIXED,
        parent=fuselage,
        child=left_tail,
        origin=Origin(xyz=(-0.289, -0.070, 0.018)),
    )
    model.articulation(
        "fuselage_to_right_tail",
        ArticulationType.FIXED,
        parent=fuselage,
        child=right_tail,
        origin=Origin(xyz=(-0.289, 0.070, 0.018)),
    )
    model.articulation(
        "fuselage_to_vertical_tail",
        ArticulationType.FIXED,
        parent=fuselage,
        child=vertical_tail,
        origin=Origin(xyz=(-0.234, 0.0, 0.046)),
    )
    model.articulation(
        "fuselage_to_service_panel",
        ArticulationType.FIXED,
        parent=fuselage,
        child=service_panel,
        origin=Origin(xyz=(0.158, -0.069, 0.007)),
    )
    model.articulation(
        "fuselage_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=propeller,
        origin=Origin(xyz=(0.292, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "fuselage_to_service_stand",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=service_stand,
        origin=Origin(xyz=(0.078, 0.0, -0.070)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fuselage = object_model.get_part("fuselage")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    left_tail = object_model.get_part("left_tail")
    right_tail = object_model.get_part("right_tail")
    vertical_tail = object_model.get_part("vertical_tail")
    service_panel = object_model.get_part("service_panel")
    propeller = object_model.get_part("propeller")
    service_stand = object_model.get_part("service_stand")
    prop_joint = object_model.get_articulation("fuselage_to_propeller")
    stand_joint = object_model.get_articulation("fuselage_to_service_stand")

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
    ctx.allow_overlap(
        fuselage,
        service_stand,
        elem_a="stand_pivot_pin",
        elem_b="hinge_barrel",
        reason="captured maintenance stand rotates on a real steel pivot pin through the hinge barrel",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_wing, fuselage, contact_tol=0.001, name="left_wing_is_bolted_on")
    ctx.expect_contact(right_wing, fuselage, contact_tol=0.001, name="right_wing_is_bolted_on")
    ctx.expect_contact(left_tail, fuselage, contact_tol=0.001, name="left_tail_is_supported")
    ctx.expect_contact(right_tail, fuselage, contact_tol=0.001, name="right_tail_is_supported")
    ctx.expect_contact(vertical_tail, fuselage, contact_tol=0.001, name="vertical_tail_is_supported")
    ctx.expect_contact(service_panel, fuselage, contact_tol=0.001, name="service_panel_is_rail_mounted")
    ctx.expect_overlap(
        service_panel,
        fuselage,
        axes="xz",
        min_overlap=0.060,
        name="service_panel_covers_access_bay",
    )
    ctx.expect_gap(
        propeller,
        fuselage,
        axis="x",
        positive_elem="hub",
        negative_elem="nose_bearing",
        min_gap=0.0,
        max_gap=0.001,
        name="propeller_hub_seats_on_nose_bearing",
    )
    ctx.expect_overlap(
        propeller,
        fuselage,
        axes="yz",
        elem_a="hub",
        elem_b="nose_bearing",
        min_overlap=0.028,
        name="propeller_stays_centered_on_nose",
    )
    with ctx.pose({prop_joint: 1.0}):
        ctx.expect_gap(
            propeller,
            fuselage,
            axis="x",
            positive_elem="hub",
            negative_elem="nose_bearing",
            min_gap=0.0,
            max_gap=0.001,
            name="propeller_spin_keeps_axial_mount",
        )
    with ctx.pose({stand_joint: 0.95}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_with_stand_deployed")
        ctx.expect_overlap(
            service_stand,
            fuselage,
            axes="yz",
            elem_a="hinge_barrel",
            elem_b="stand_pivot_pin",
            min_overlap=0.008,
            name="stand_hinge_remains_mounted_when_deployed",
        )
        ctx.expect_gap(
            fuselage,
            service_stand,
            axis="z",
            negative_elem="foot_pad",
            min_gap=0.070,
            max_gap=0.130,
            name="stand_swings_below_fuselage",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

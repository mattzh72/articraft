from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
)


def _yz_superellipse_section(
    x: float,
    width: float,
    height: float,
    *,
    center_z: float,
    exponent: float = 2.5,
    segments: int = 36,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + center_z)
        for y, z in superellipse_profile(width, height, exponent=exponent, segments=segments)
    ]


def _symmetric_planform(
    *,
    span: float,
    root_chord: float,
    tip_chord: float,
    sweep: float,
) -> list[tuple[float, float]]:
    half_span = span * 0.5
    leading_root = root_chord * 0.45
    trailing_root = leading_root - root_chord
    leading_tip = leading_root - sweep
    trailing_tip = leading_tip - tip_chord
    return [
        (leading_root, 0.0),
        (leading_tip, half_span),
        (trailing_tip, half_span),
        (trailing_root, 0.0),
        (trailing_tip, -half_span),
        (leading_tip, -half_span),
    ]


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_toy_model_plane")

    body_red = model.material("body_red", rgba=(0.74, 0.17, 0.15, 1.0))
    trim_cream = model.material("trim_cream", rgba=(0.93, 0.92, 0.86, 1.0))
    stand_paint = model.material("stand_paint", rgba=(0.20, 0.27, 0.31, 1.0))
    canopy_smoke = model.material("canopy_smoke", rgba=(0.52, 0.63, 0.71, 0.88))
    hardware = model.material("hardware", rgba=(0.69, 0.71, 0.74, 1.0))
    prop_black = model.material("prop_black", rgba=(0.12, 0.12, 0.13, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.26, 0.19, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=stand_paint,
        name="base_plate",
    )
    stand.visual(
        Box((0.22, 0.15, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=stand_paint,
        name="base_cap",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=hardware,
        name="mast_collar",
    )
    stand.visual(
        Cylinder(radius=0.020, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, 0.182)),
        material=stand_paint,
        name="mast",
    )
    stand.visual(
        Box((0.084, 0.060, 0.060)),
        origin=Origin(xyz=(0.006, 0.0, 0.322)),
        material=stand_paint,
        name="joint_housing",
    )
    stand.visual(
        Box((0.058, 0.044, 0.010)),
        origin=Origin(xyz=(0.006, 0.0, 0.351)),
        material=trim_cream,
        name="saddle_pad",
    )
    stand.visual(
        Box((0.030, 0.024, 0.060)),
        origin=Origin(xyz=(0.006, 0.018, 0.326)),
        material=stand_paint,
        name="left_cheek",
    )
    stand.visual(
        Box((0.030, 0.024, 0.060)),
        origin=Origin(xyz=(0.006, -0.018, 0.326)),
        material=stand_paint,
        name="right_cheek",
    )
    stand.visual(
        Box((0.042, 0.050, 0.018)),
        origin=Origin(xyz=(0.034, 0.0, 0.340)),
        material=stand_paint,
        name="joint_hood",
    )
    stand.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.006, 0.020, 0.332), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_pivot_cap",
    )
    stand.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.006, -0.020, 0.332), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_pivot_cap",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.26, 0.19, 0.40)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    fuselage_geom = section_loft(
        [
            _yz_superellipse_section(0.280, 0.016, 0.022, center_z=0.048, exponent=2.0),
            _yz_superellipse_section(0.215, 0.082, 0.090, center_z=0.056, exponent=2.2),
            _yz_superellipse_section(0.090, 0.112, 0.108, center_z=0.060, exponent=2.5),
            _yz_superellipse_section(-0.020, 0.102, 0.098, center_z=0.056, exponent=2.5),
            _yz_superellipse_section(-0.160, 0.058, 0.066, center_z=0.062, exponent=2.2),
            _yz_superellipse_section(-0.268, 0.016, 0.026, center_z=0.074, exponent=2.0),
        ]
    )
    main_wing_geom = ExtrudeGeometry.centered(
        _symmetric_planform(span=0.780, root_chord=0.205, tip_chord=0.092, sweep=0.052),
        0.018,
    )
    main_wing_geom.translate(0.010, 0.0, 0.074)

    tailplane_geom = ExtrudeGeometry.centered(
        _symmetric_planform(span=0.290, root_chord=0.112, tip_chord=0.058, sweep=0.026),
        0.012,
    )
    tailplane_geom.translate(-0.215, 0.0, 0.088)

    fin_profile = [
        (-0.080, 0.0),
        (0.000, 0.0),
        (0.022, 0.074),
        (-0.008, 0.136),
        (-0.070, 0.084),
    ]
    fin_geom = ExtrudeGeometry.centered(fin_profile, 0.014)
    fin_geom.rotate_x(pi / 2.0)
    fin_geom.translate(-0.230, 0.0, 0.074)

    plane = model.part("plane")
    plane.visual(
        mesh_from_geometry(fuselage_geom, "plane_fuselage"),
        material=body_red,
        name="fuselage",
    )
    plane.visual(
        mesh_from_geometry(main_wing_geom, "plane_main_wing"),
        material=trim_cream,
        name="main_wing",
    )
    plane.visual(
        mesh_from_geometry(tailplane_geom, "plane_tailplane"),
        material=trim_cream,
        name="tailplane",
    )
    plane.visual(
        mesh_from_geometry(fin_geom, "plane_vertical_fin"),
        material=body_red,
        name="vertical_fin",
    )
    plane.visual(
        Cylinder(radius=0.030, length=0.100),
        origin=Origin(xyz=(0.055, 0.0, 0.102), rpy=(0.0, pi / 2.0, 0.0)),
        material=canopy_smoke,
        name="canopy_shell",
    )
    plane.visual(
        Box((0.112, 0.064, 0.008)),
        origin=Origin(xyz=(0.055, 0.0, 0.081)),
        material=trim_cream,
        name="canopy_lip",
    )
    plane.visual(
        Box((0.084, 0.060, 0.022)),
        origin=Origin(xyz=(0.000, 0.0, 0.015)),
        material=body_red,
        name="pivot_shroud",
    )
    plane.visual(
        Box((0.052, 0.040, 0.008)),
        origin=Origin(xyz=(0.000, 0.0, 0.004)),
        material=trim_cream,
        name="belly_pad",
    )
    plane.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.275, 0.0, 0.048), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware,
        name="nose_plate",
    )
    plane.inertial = Inertial.from_geometry(
        Box((0.58, 0.78, 0.20)),
        mass=0.85,
        origin=Origin(xyz=(0.005, 0.0, 0.090)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware,
        name="hub",
    )
    propeller.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.019, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware,
        name="shaft_cap",
    )
    propeller.visual(
        Box((0.006, 0.024, 0.112)),
        origin=Origin(xyz=(0.016, 0.0, 0.056), rpy=(0.18, 0.0, 0.0)),
        material=prop_black,
        name="upper_blade",
    )
    propeller.visual(
        Box((0.006, 0.024, 0.112)),
        origin=Origin(xyz=(0.016, 0.0, -0.056), rpy=(-0.18, 0.0, 0.0)),
        material=prop_black,
        name="lower_blade",
    )
    propeller.inertial = Inertial.from_geometry(
        Box((0.024, 0.032, 0.224)),
        mass=0.05,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_plane",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=plane,
        origin=Origin(xyz=(0.006, 0.0, 0.356)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.2,
            lower=-radians(18.0),
            upper=radians(32.0),
        ),
    )
    model.articulation(
        "plane_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=plane,
        child=propeller,
        origin=Origin(xyz=(0.278, 0.0, 0.048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=15.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    plane = object_model.get_part("plane")
    propeller = object_model.get_part("propeller")
    stand_pitch = object_model.get_articulation("stand_to_plane")
    prop_spin = object_model.get_articulation("plane_to_propeller")

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
        plane,
        stand,
        elem_a="belly_pad",
        elem_b="saddle_pad",
        contact_tol=1e-4,
        name="plane_belly_pad_seats_on_saddle",
    )
    ctx.expect_overlap(
        plane,
        stand,
        axes="xy",
        elem_a="belly_pad",
        elem_b="saddle_pad",
        min_overlap=0.038,
        name="plane_pad_has_broad_saddle_support",
    )
    ctx.expect_overlap(
        plane,
        stand,
        axes="xy",
        elem_a="pivot_shroud",
        elem_b="joint_housing",
        min_overlap=0.055,
        name="joint_housing_sits_under_pivot_shroud",
    )
    ctx.expect_gap(
        plane,
        stand,
        axis="z",
        positive_elem="pivot_shroud",
        negative_elem="joint_housing",
        min_gap=0.006,
        max_gap=0.012,
        name="pivot_shroud_covers_joint_housing_from_above",
    )
    ctx.expect_gap(
        plane,
        stand,
        axis="z",
        positive_elem="belly_pad",
        negative_elem="joint_hood",
        min_gap=0.006,
        max_gap=0.018,
        name="front_drip_lip_sits_below_belly_pad",
    )
    ctx.expect_contact(
        propeller,
        plane,
        elem_a="hub",
        elem_b="nose_plate",
        contact_tol=1e-5,
        name="propeller_hub_bears_on_nose_plate",
    )
    ctx.expect_gap(
        propeller,
        plane,
        axis="x",
        positive_elem="hub",
        negative_elem="nose_plate",
        min_gap=0.0,
        max_gap=5e-4,
        name="propeller_hub_starts_at_sealed_nose_face",
    )

    ctx.check(
        "propeller_joint_is_axial_and_continuous",
        prop_spin.articulation_type == ArticulationType.CONTINUOUS and prop_spin.axis == (1.0, 0.0, 0.0),
        f"expected continuous axial spin, got type={prop_spin.articulation_type!r} axis={prop_spin.axis!r}",
    )

    rest_hub = ctx.part_element_world_aabb(propeller, elem="hub")
    with ctx.pose({stand_pitch: radians(28.0)}):
        raised_hub = ctx.part_element_world_aabb(propeller, elem="hub")
        ctx.expect_contact(
            propeller,
            plane,
            elem_a="hub",
            elem_b="nose_plate",
            contact_tol=1e-5,
            name="propeller_mount_stays_seated_when_plane_tilts",
        )

    rest_hub_z = _aabb_center_z(rest_hub)
    raised_hub_z = _aabb_center_z(raised_hub)
    ctx.check(
        "stand_pitch_raises_the_nose",
        rest_hub_z is not None and raised_hub_z is not None and raised_hub_z > rest_hub_z + 0.09,
        f"rest_hub_z={rest_hub_z!r}, raised_hub_z={raised_hub_z!r}",
    )

    with ctx.pose({prop_spin: pi / 2.0}):
        ctx.expect_contact(
            propeller,
            plane,
            elem_a="hub",
            elem_b="nose_plate",
            contact_tol=1e-5,
            name="propeller_spin_preserves_bearing_contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

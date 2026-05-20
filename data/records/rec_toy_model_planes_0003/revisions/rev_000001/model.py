from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _write_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _superellipse_section(
    x: float,
    width: float,
    height: float,
    z_center: float,
    *,
    exponent: float = 2.45,
    segments: int = 36,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for y, z in superellipse_profile(
            width,
            height,
            exponent=exponent,
            segments=segments,
        )
    ]


def _airfoil_loop(
    *,
    leading_x: float,
    chord: float,
    thickness: float,
    y: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    t = thickness * 0.5
    points = (
        (leading_x, 0.00),
        (leading_x - 0.08 * chord, 0.70 * t),
        (leading_x - 0.28 * chord, 1.00 * t),
        (leading_x - 0.58 * chord, 0.72 * t),
        (leading_x - 0.88 * chord, 0.22 * t),
        (leading_x - chord, 0.03 * t),
        (leading_x - 0.96 * chord, -0.03 * t),
        (leading_x - 0.68 * chord, -0.16 * t),
        (leading_x - 0.30 * chord, -0.20 * t),
        (leading_x - 0.10 * chord, -0.08 * t),
    )
    return [(x, y, z_center + z) for x, z in points]


def _wing_mesh(
    name: str,
    *,
    side: float,
    span: float,
    root_leading_x: float,
    mid_leading_x: float,
    tip_leading_x: float,
    root_chord: float,
    mid_chord: float,
    tip_chord: float,
    root_thickness: float,
    mid_thickness: float,
    tip_thickness: float,
    root_z: float,
    mid_z: float,
    tip_z: float,
):
    sections = [
        _airfoil_loop(
            leading_x=root_leading_x,
            chord=root_chord,
            thickness=root_thickness,
            y=0.0,
            z_center=root_z,
        ),
        _airfoil_loop(
            leading_x=mid_leading_x,
            chord=mid_chord,
            thickness=mid_thickness,
            y=side * span * 0.58,
            z_center=mid_z,
        ),
        _airfoil_loop(
            leading_x=tip_leading_x,
            chord=tip_chord,
            thickness=tip_thickness,
            y=side * span,
            z_center=tip_z,
        ),
    ]
    return _write_mesh(name, section_loft(sections))


def _control_surface_mesh(
    name: str,
    *,
    half_span: float,
    root_chord: float,
    tip_chord: float,
    root_thickness: float,
    tip_thickness: float,
):
    sections = [
        _airfoil_loop(
            leading_x=-0.001,
            chord=tip_chord,
            thickness=tip_thickness,
            y=-half_span,
            z_center=0.0,
        ),
        _airfoil_loop(
            leading_x=-0.001,
            chord=root_chord,
            thickness=root_thickness,
            y=0.0,
            z_center=0.0,
        ),
        _airfoil_loop(
            leading_x=-0.001,
            chord=tip_chord,
            thickness=tip_thickness,
            y=half_span,
            z_center=0.0,
        ),
    ]
    return _write_mesh(name, section_loft(sections))


def _thin_profile_mesh(name: str, profile_xz: list[tuple[float, float]], thickness: float):
    geom = ExtrudeGeometry.centered(profile_xz, thickness, cap=True, closed=True)
    geom.rotate_x(math.pi / 2.0)
    return _write_mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_toy_model_plane", assets=ASSETS)

    body_paint = model.material("body_paint", rgba=(0.93, 0.92, 0.88, 1.0))
    satin_trim = model.material("satin_trim", rgba=(0.23, 0.29, 0.38, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.26, 0.33, 0.40, 0.55))
    prop_dark = model.material("prop_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    spinner_metal = model.material("spinner_metal", rgba=(0.79, 0.80, 0.81, 1.0))
    stand_metal = model.material("stand_metal", rgba=(0.69, 0.70, 0.73, 1.0))
    stand_pad = model.material("stand_pad", rgba=(0.16, 0.17, 0.18, 1.0))

    stand_base = model.part("stand_base")
    stand_base.visual(
        _write_mesh(
            "stand_base_plinth.obj",
            LatheGeometry(
                [
                    (0.056, 0.000),
                    (0.060, 0.0015),
                    (0.060, 0.0045),
                    (0.052, 0.0070),
                    (0.040, 0.0090),
                    (0.032, 0.0105),
                ],
                segments=48,
            ),
        ),
        material=stand_metal,
        name="base_plinth",
    )
    stand_base.visual(
        Cylinder(radius=0.040, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=stand_pad,
        name="base_cap",
    )
    stand_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.012),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    stand_stem = model.part("stand_stem")
    stand_stem.visual(
        Cylinder(radius=0.006, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=stand_metal,
        name="stem_shaft",
    )
    stand_stem.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=stand_metal,
        name="stem_collar",
    )
    stand_stem.visual(
        Box((0.010, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.010, 0.122)),
        material=stand_metal,
        name="yoke_left_lug",
    )
    stand_stem.visual(
        Box((0.010, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, -0.010, 0.122)),
        material=stand_metal,
        name="yoke_right_lug",
    )
    stand_stem.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.112),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    model.articulation(
        "base_to_stem",
        ArticulationType.FIXED,
        parent=stand_base,
        child=stand_stem,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    stand_cradle = model.part("stand_cradle")
    stand_cradle.visual(
        Box((0.016, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=stand_metal,
        name="cradle_post",
    )
    stand_cradle.visual(
        Box((0.034, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=stand_pad,
        name="cradle_pad",
    )
    stand_cradle.visual(
        Box((0.010, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.012, 0.018)),
        material=stand_metal,
        name="cradle_left_cheek",
    )
    stand_cradle.visual(
        Box((0.010, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, -0.012, 0.018)),
        material=stand_metal,
        name="cradle_right_cheek",
    )
    stand_cradle.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spinner_metal,
        name="cradle_pivot_pin",
    )
    stand_cradle.inertial = Inertial.from_geometry(
        Box((0.034, 0.028, 0.026)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    model.articulation(
        "stem_to_cradle",
        ArticulationType.REVOLUTE,
        parent=stand_stem,
        child=stand_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.12,
            velocity=1.2,
            lower=-math.radians(22.0),
            upper=math.radians(22.0),
        ),
    )

    airframe = model.part("airframe")

    fuselage_mesh = _write_mesh(
        "airframe_fuselage.obj",
        section_loft(
            [
                _superellipse_section(-0.130, 0.004, 0.006, 0.015),
                _superellipse_section(-0.102, 0.018, 0.024, 0.025),
                _superellipse_section(-0.058, 0.032, 0.040, 0.032),
                _superellipse_section(0.000, 0.050, 0.060, 0.040),
                _superellipse_section(0.045, 0.054, 0.064, 0.044),
                _superellipse_section(0.088, 0.048, 0.058, 0.040),
                _superellipse_section(0.118, 0.034, 0.046, 0.036),
            ]
        ),
    )
    airframe.visual(fuselage_mesh, material=body_paint, name="fuselage_shell")

    canopy_mesh = _write_mesh(
        "airframe_canopy.obj",
        section_loft(
            [
                _superellipse_section(0.015, 0.022, 0.018, 0.0528, exponent=2.15),
                _superellipse_section(0.040, 0.033, 0.030, 0.0574, exponent=2.10),
                _superellipse_section(0.065, 0.024, 0.020, 0.0548, exponent=2.10),
            ]
        ),
    )
    canopy = model.part("canopy")
    canopy.visual(canopy_mesh, material=canopy_tint, name="canopy_shell")
    canopy.inertial = Inertial.from_geometry(
        Box((0.055, 0.035, 0.030)),
        mass=0.02,
        origin=Origin(xyz=(0.040, 0.0, 0.058)),
    )

    left_wing_mesh = _wing_mesh(
        "left_wing.obj",
        side=1.0,
        span=0.125,
        root_leading_x=0.044,
        mid_leading_x=0.048,
        tip_leading_x=0.052,
        root_chord=0.061,
        mid_chord=0.050,
        tip_chord=0.039,
        root_thickness=0.012,
        mid_thickness=0.009,
        tip_thickness=0.0055,
        root_z=0.037,
        mid_z=0.044,
        tip_z=0.052,
    )
    right_wing_mesh = _wing_mesh(
        "right_wing.obj",
        side=-1.0,
        span=0.125,
        root_leading_x=0.044,
        mid_leading_x=0.048,
        tip_leading_x=0.052,
        root_chord=0.061,
        mid_chord=0.050,
        tip_chord=0.039,
        root_thickness=0.012,
        mid_thickness=0.009,
        tip_thickness=0.0055,
        root_z=0.037,
        mid_z=0.044,
        tip_z=0.052,
    )
    airframe.visual(left_wing_mesh, material=body_paint, name="left_wing")
    airframe.visual(right_wing_mesh, material=body_paint, name="right_wing")

    left_tail_mesh = _wing_mesh(
        "left_tailplane.obj",
        side=1.0,
        span=0.056,
        root_leading_x=-0.061,
        mid_leading_x=-0.060,
        tip_leading_x=-0.059,
        root_chord=0.027,
        mid_chord=0.023,
        tip_chord=0.019,
        root_thickness=0.0050,
        mid_thickness=0.0042,
        tip_thickness=0.0032,
        root_z=0.048,
        mid_z=0.050,
        tip_z=0.053,
    )
    right_tail_mesh = _wing_mesh(
        "right_tailplane.obj",
        side=-1.0,
        span=0.056,
        root_leading_x=-0.061,
        mid_leading_x=-0.060,
        tip_leading_x=-0.059,
        root_chord=0.027,
        mid_chord=0.023,
        tip_chord=0.019,
        root_thickness=0.0050,
        mid_thickness=0.0042,
        tip_thickness=0.0032,
        root_z=0.048,
        mid_z=0.050,
        tip_z=0.053,
    )
    airframe.visual(left_tail_mesh, material=body_paint, name="left_tailplane")
    airframe.visual(right_tail_mesh, material=body_paint, name="right_tailplane")

    fin_profile = [
        (-0.100, 0.028),
        (-0.091, 0.057),
        (-0.098, 0.080),
        (-0.118, 0.074),
        (-0.120, 0.042),
        (-0.118, 0.026),
    ]
    airframe.visual(
        _thin_profile_mesh("vertical_fin.obj", fin_profile, 0.006),
        material=body_paint,
        name="vertical_fin",
    )

    airframe.visual(
        Cylinder(radius=0.027, length=0.010),
        origin=Origin(xyz=(0.103, 0.0, 0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_trim,
        name="cowl_band",
    )
    airframe.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.122, 0.0, 0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spinner_metal,
        name="nose_bearing",
    )
    airframe.visual(
        Box((0.028, 0.022, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=satin_trim,
        name="mount_puck",
    )
    airframe.visual(
        Box((0.003, 0.003, 0.036)),
        origin=Origin(xyz=(-0.108, 0.0, 0.046)),
        material=satin_trim,
        name="rudder_break",
    )

    airframe.inertial = Inertial.from_geometry(
        Box((0.260, 0.260, 0.080)),
        mass=0.55,
        origin=Origin(xyz=(-0.005, 0.0, 0.042)),
    )

    model.articulation(
        "cradle_to_airframe",
        ArticulationType.FIXED,
        parent=stand_cradle,
        child=airframe,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    model.articulation(
        "airframe_to_canopy",
        ArticulationType.FIXED,
        parent=airframe,
        child=canopy,
        origin=Origin(),
    )

    propeller = model.part("propeller")
    spinner_geom = LatheGeometry(
        [
            (0.012, 0.0),
            (0.0115, 0.004),
            (0.0075, 0.016),
            (0.0, 0.024),
        ],
        segments=36,
    )
    spinner_geom.rotate_y(math.pi / 2.0)
    propeller.visual(
        _write_mesh("propeller_spinner.obj", spinner_geom),
        material=spinner_metal,
        name="spinner",
    )
    propeller.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spinner_metal,
        name="prop_hub",
    )
    upper_blade_mesh = _thin_profile_mesh(
        "prop_blade_upper.obj",
        [
            (0.0055, -0.0025),
            (0.0100, 0.0060),
            (0.0150, 0.0320),
            (0.0215, 0.0580),
            (0.0270, 0.0730),
            (0.0315, 0.0780),
            (0.0290, 0.0680),
            (0.0230, 0.0510),
            (0.0170, 0.0250),
            (0.0110, 0.0035),
        ],
        0.0035,
    )
    propeller.visual(
        upper_blade_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.24, 0.0)),
        material=prop_dark,
        name="prop_blade_upper",
    )
    propeller.visual(
        upper_blade_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi, -0.24, 0.0)),
        material=prop_dark,
        name="prop_blade_lower",
    )
    propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.028),
        mass=0.04,
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "airframe_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        origin=Origin(xyz=(0.126, 0.0, 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    stand_base = object_model.get_part("stand_base")
    stand_stem = object_model.get_part("stand_stem")
    stand_cradle = object_model.get_part("stand_cradle")
    airframe = object_model.get_part("airframe")
    canopy = object_model.get_part("canopy")
    propeller = object_model.get_part("propeller")

    cradle_joint = object_model.get_articulation("stem_to_cradle")
    propeller_joint = object_model.get_articulation("airframe_to_propeller")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        stand_stem,
        stand_cradle,
        reason="The display cradle uses a captured pivot pin through unmodeled lug bores.",
    )
    ctx.allow_overlap(
        airframe,
        canopy,
        reason="The canopy is seated with a shallow inset lip for a tight premium seam.",
    )

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts(max_pose_samples=12)
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

    ctx.check(
        "all_key_parts_present",
        all(
            part is not None
            for part in (
                stand_base,
                stand_stem,
                stand_cradle,
                airframe,
                canopy,
                propeller,
            )
        ),
        "Expected stand, airframe, canopy, and propeller parts.",
    )

    ctx.check(
        "articulation_axes_plausible",
        cradle_joint.axis == (0.0, 1.0, 0.0) and propeller_joint.axis == (1.0, 0.0, 0.0),
        "The stand should tilt about a lateral pivot and the propeller should spin about the fuselage axis.",
    )

    ctx.expect_gap(
        stand_stem,
        stand_base,
        axis="z",
        positive_elem="stem_shaft",
        negative_elem="base_cap",
        max_gap=0.00001,
        max_penetration=0.0,
    )
    ctx.expect_overlap(stand_stem, stand_base, axes="xy", min_overlap=0.012)
    ctx.expect_overlap(stand_cradle, stand_stem, axes="xy", min_overlap=0.018)
    ctx.expect_gap(
        airframe,
        stand_cradle,
        axis="z",
        positive_elem="mount_puck",
        negative_elem="cradle_pad",
        max_gap=0.0005,
        max_penetration=0.001,
    )
    ctx.expect_contact(propeller, airframe, elem_a="prop_hub", elem_b="nose_bearing")

    ctx.expect_overlap(airframe, stand_cradle, axes="xy", min_overlap=0.018)
    ctx.expect_overlap(propeller, airframe, axes="yz", min_overlap=0.012)
    ctx.expect_gap(propeller, airframe, axis="x", max_gap=0.0005, max_penetration=0.0)
    ctx.expect_within(canopy, airframe, axes="xy", margin=0.002)
    ctx.expect_overlap(canopy, airframe, axes="xy", min_overlap=0.025)
    ctx.expect_origin_gap(airframe, stand_base, axis="z", min_gap=0.15, max_gap=0.19)

    airframe_aabb = ctx.part_world_aabb(airframe)
    if airframe_aabb is not None:
        airframe_dx = airframe_aabb[1][0] - airframe_aabb[0][0]
        airframe_dy = airframe_aabb[1][1] - airframe_aabb[0][1]
        airframe_dz = airframe_aabb[1][2] - airframe_aabb[0][2]
        ctx.check(
            "airframe_compact_display_scale",
            0.23 <= airframe_dx <= 0.30
            and 0.22 <= airframe_dy <= 0.30
            and 0.05 <= airframe_dz <= 0.10,
            f"Airframe dims were {(airframe_dx, airframe_dy, airframe_dz)}.",
        )
    else:
        ctx.fail("airframe_compact_display_scale", "Could not resolve airframe AABB.")

    cradle_limits = cradle_joint.motion_limits
    if cradle_limits is None or cradle_limits.lower is None or cradle_limits.upper is None:
        ctx.fail("stem_to_cradle_limits_present", "Expected bounded cradle tilt limits.")
    else:
        rest_pos = ctx.part_world_position(airframe)
        assert rest_pos is not None
        with ctx.pose({cradle_joint: cradle_limits.lower}):
            low_pos = ctx.part_world_position(airframe)
            ctx.fail_if_parts_overlap_in_current_pose(name="stem_to_cradle_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="stem_to_cradle_lower_no_floating")
        with ctx.pose({cradle_joint: cradle_limits.upper}):
            high_pos = ctx.part_world_position(airframe)
            ctx.fail_if_parts_overlap_in_current_pose(name="stem_to_cradle_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="stem_to_cradle_upper_no_floating")
        ctx.check(
            "stand_tilt_moves_airframe_side_to_side",
            low_pos is not None
            and high_pos is not None
            and low_pos[0] < rest_pos[0] - 0.008
            and high_pos[0] > rest_pos[0] + 0.008,
            f"Airframe positions were rest={rest_pos}, low={low_pos}, high={high_pos}.",
        )

    for angle_name, angle_value in (("rest", 0.0), ("quarter_turn", math.pi / 2.0)):
        with ctx.pose({propeller_joint: angle_value}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"propeller_{angle_name}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"propeller_{angle_name}_no_floating")
            ctx.expect_contact(
                propeller,
                airframe,
                elem_a="prop_hub",
                elem_b="nose_bearing",
                name=f"propeller_{angle_name}_hub_contact",
            )

    with ctx.pose({propeller_joint: 0.0}):
        blade_rest = ctx.part_element_world_aabb(propeller, elem="prop_blade_upper")
    with ctx.pose({propeller_joint: math.pi / 2.0}):
        blade_quarter = ctx.part_element_world_aabb(propeller, elem="prop_blade_upper")
    if blade_rest is not None and blade_quarter is not None:
        rest_y = blade_rest[1][1] - blade_rest[0][1]
        rest_z = blade_rest[1][2] - blade_rest[0][2]
        quarter_y = blade_quarter[1][1] - blade_quarter[0][1]
        quarter_z = blade_quarter[1][2] - blade_quarter[0][2]
        ctx.check(
            "propeller_rotates_about_nose_axis",
            rest_z > rest_y * 3.0 and quarter_y > quarter_z * 3.0,
            f"Blade extents were rest {(rest_y, rest_z)} and quarter-turn {(quarter_y, quarter_z)}.",
        )
    else:
        ctx.fail("propeller_rotates_about_nose_axis", "Could not resolve propeller blade AABBs.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

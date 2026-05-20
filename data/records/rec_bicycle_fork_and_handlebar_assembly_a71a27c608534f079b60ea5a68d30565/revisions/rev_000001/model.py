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
    superellipse_profile,
    tube_from_spline_points,
)


def _bezier_point(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    u = 1.0 - t
    x = (
        (u**3) * p0[0]
        + 3.0 * (u**2) * t * p1[0]
        + 3.0 * u * (t**2) * p2[0]
        + (t**3) * p3[0]
    )
    y = (
        (u**3) * p0[1]
        + 3.0 * (u**2) * t * p1[1]
        + 3.0 * u * (t**2) * p2[1]
        + (t**3) * p3[1]
    )
    return (x, y)


def _aero_profile(
    chord: float,
    thickness: float,
    *,
    samples_per_side: int = 16,
) -> list[tuple[float, float]]:
    trailing = (-0.50 * chord, 0.0)
    leading = (0.50 * chord, 0.0)
    top = [
        _bezier_point(
            trailing,
            (-0.24 * chord, 0.55 * thickness),
            (0.14 * chord, 0.53 * thickness),
            leading,
            i / samples_per_side,
        )
        for i in range(samples_per_side)
    ]
    bottom = [
        _bezier_point(
            leading,
            (0.12 * chord, -0.50 * thickness),
            (-0.30 * chord, -0.42 * thickness),
            trailing,
            i / samples_per_side,
        )
        for i in range(samples_per_side)
    ]
    return top + bottom


def _aero_section_xy(
    *,
    center_x: float,
    center_y: float,
    z: float,
    chord: float,
    thickness: float,
    yaw: float = 0.0,
) -> list[tuple[float, float, float]]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [
        (
            center_x + c * px - s * py,
            center_y + s * px + c * py,
            z,
        )
        for px, py in _aero_profile(chord, thickness)
    ]


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for z, y in rounded_rect_profile(height, width, radius)]


def _xy_superellipse_section(
    z: float,
    *,
    width: float,
    depth: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
    exponent: float = 2.35,
    segments: int = 40,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + x, center_y + y, z)
        for x, y in superellipse_profile(width, depth, exponent=exponent, segments=segments)
    ]


def _circle_section(
    z: float,
    radius: float,
    *,
    samples: int = 24,
) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / samples),
            radius * math.sin(2.0 * math.pi * i / samples),
            z,
        )
        for i in range(samples)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triathlon_tt_fork_front_end")

    carbon = model.material("carbon", rgba=(0.11, 0.12, 0.13, 1.0))
    satin_carbon = model.material("satin_carbon", rgba=(0.18, 0.19, 0.20, 1.0))
    alloy = model.material("alloy", rgba=(0.63, 0.65, 0.69, 1.0))
    black_hardware = model.material("black_hardware", rgba=(0.08, 0.08, 0.09, 1.0))
    pad_foam = model.material("pad_foam", rgba=(0.05, 0.05, 0.05, 1.0))

    stem = model.part("stem")
    stem.inertial = Inertial.from_geometry(
        Box((0.24, 0.12, 0.09)),
        mass=1.4,
        origin=Origin(xyz=(0.08, 0.0, -0.01)),
    )

    stem_shell_geom = section_loft(
        [
            _yz_section(0.018, width=0.060, height=0.052, radius=0.018),
            _yz_section(0.105, width=0.068, height=0.041, radius=0.014),
            _yz_section(0.195, width=0.050, height=0.030, radius=0.010),
        ]
    )
    stem.visual(
        mesh_from_geometry(stem_shell_geom, "tt_stem_shell"),
        material=carbon,
        name="stem_shell",
    )
    stem.visual(
        Box((0.024, 0.018, 0.012)),
        origin=Origin(xyz=(0.030, 0.046, -0.031)),
        material=satin_carbon,
        name="left_rail_root",
    )
    stem.visual(
        Box((0.024, 0.018, 0.012)),
        origin=Origin(xyz=(0.030, -0.046, -0.031)),
        material=satin_carbon,
        name="right_rail_root",
    )
    stem.visual(
        Box((0.098, 0.044, 0.012)),
        origin=Origin(xyz=(0.069, 0.0, -0.031)),
        material=satin_carbon,
        name="center_keel",
    )
    stem.visual(
        Box((0.136, 0.014, 0.010)),
        origin=Origin(xyz=(0.086, 0.046, -0.042)),
        material=alloy,
        name="left_rail",
    )
    stem.visual(
        Box((0.136, 0.014, 0.010)),
        origin=Origin(xyz=(0.086, -0.046, -0.042)),
        material=alloy,
        name="right_rail",
    )
    stem.visual(
        Box((0.044, 0.012, 0.054)),
        origin=Origin(xyz=(-0.004, 0.0241, 0.0)),
        material=satin_carbon,
        name="left_clamp_cheek",
    )
    stem.visual(
        Box((0.044, 0.012, 0.054)),
        origin=Origin(xyz=(-0.004, -0.0241, 0.0)),
        material=satin_carbon,
        name="right_clamp_cheek",
    )
    stem.visual(
        Box((0.030, 0.004, 0.012)),
        origin=Origin(xyz=(-0.002, 0.0161, -0.014)),
        material=satin_carbon,
        name="left_clamp_pad",
    )
    stem.visual(
        Box((0.030, 0.004, 0.012)),
        origin=Origin(xyz=(-0.002, -0.0161, -0.014)),
        material=satin_carbon,
        name="right_clamp_pad",
    )
    stem.visual(
        Box((0.020, 0.030, 0.012)),
        origin=Origin(xyz=(0.030, 0.030, -0.031)),
        material=satin_carbon,
        name="left_rail_support",
    )
    stem.visual(
        Box((0.020, 0.030, 0.012)),
        origin=Origin(xyz=(0.030, -0.030, -0.031)),
        material=satin_carbon,
        name="right_rail_support",
    )
    stem.visual(
        Box((0.030, 0.063, 0.012)),
        origin=Origin(xyz=(0.003, 0.0, 0.033)),
        material=satin_carbon,
        name="clamp_cap",
    )
    stem.visual(
        Box((0.032, 0.0362, 0.010)),
        origin=Origin(xyz=(0.000, 0.0, 0.027)),
        material=satin_carbon,
        name="steerer_bridge",
    )
    stem.visual(
        Box((0.034, 0.068, 0.008)),
        origin=Origin(xyz=(0.055, 0.0, -0.021)),
        material=black_hardware,
        name="underside_fairing",
    )

    fork = model.part("fork")
    fork.inertial = Inertial.from_geometry(
        Box((0.16, 0.16, 0.80)),
        mass=1.1,
        origin=Origin(xyz=(0.015, 0.0, -0.36)),
    )

    steerer_geom = section_loft(
        [
            _circle_section(0.018, 0.0135),
            _circle_section(-0.110, 0.0155),
            _circle_section(-0.245, 0.0190),
        ]
    )
    crown_geom = section_loft(
        [
            _xy_superellipse_section(-0.245, width=0.050, depth=0.058, center_x=0.0),
            _xy_superellipse_section(-0.290, width=0.088, depth=0.142, center_x=0.018),
            _xy_superellipse_section(-0.325, width=0.070, depth=0.128, center_x=0.015),
        ]
    )
    left_blade_geom = section_loft(
        [
            _aero_section_xy(center_x=0.010, center_y=0.053, z=-0.325, chord=0.052, thickness=0.018, yaw=0.02),
            _aero_section_xy(center_x=0.020, center_y=0.056, z=-0.470, chord=0.047, thickness=0.015, yaw=0.03),
            _aero_section_xy(center_x=0.026, center_y=0.060, z=-0.610, chord=0.041, thickness=0.0125, yaw=0.03),
            _aero_section_xy(center_x=0.022, center_y=0.064, z=-0.720, chord=0.034, thickness=0.0105, yaw=0.01),
        ]
    )
    right_blade_geom = section_loft(
        [
            _aero_section_xy(center_x=0.010, center_y=-0.053, z=-0.325, chord=0.052, thickness=0.018, yaw=-0.02),
            _aero_section_xy(center_x=0.020, center_y=-0.056, z=-0.470, chord=0.047, thickness=0.015, yaw=-0.03),
            _aero_section_xy(center_x=0.026, center_y=-0.060, z=-0.610, chord=0.041, thickness=0.0125, yaw=-0.03),
            _aero_section_xy(center_x=0.022, center_y=-0.064, z=-0.720, chord=0.034, thickness=0.0105, yaw=-0.01),
        ]
    )
    fork.visual(
        mesh_from_geometry(steerer_geom, "tt_tapered_steerer"),
        material=carbon,
        name="steerer",
    )
    fork.visual(
        Cylinder(radius=0.0165, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=alloy,
        name="compression_spacer",
    )
    fork.visual(
        mesh_from_geometry(crown_geom, "tt_fork_crown"),
        material=carbon,
        name="crown",
    )
    fork.visual(
        mesh_from_geometry(left_blade_geom, "tt_left_blade"),
        material=carbon,
        name="left_blade",
    )
    fork.visual(
        mesh_from_geometry(right_blade_geom, "tt_right_blade"),
        material=carbon,
        name="right_blade",
    )
    fork.visual(
        Box((0.018, 0.022, 0.016)),
        origin=Origin(xyz=(0.022, 0.064, -0.728)),
        material=alloy,
        name="left_dropout",
    )
    fork.visual(
        Box((0.018, 0.022, 0.016)),
        origin=Origin(xyz=(0.022, -0.064, -0.728)),
        material=alloy,
        name="right_dropout",
    )
    fork.visual(
        Cylinder(radius=0.0065, length=0.150),
        origin=Origin(xyz=(0.022, 0.0, -0.728), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="through_axle",
    )

    extension_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.035, 0.0, 0.0),
                (0.045, 0.0, 0.0),
                (0.200, 0.0, 0.015),
                (0.305, 0.0, 0.060),
                (0.355, 0.0, 0.108),
            ],
            radius=0.0111,
            samples_per_segment=18,
            radial_segments=18,
        ),
        "tt_extension_tube",
    )

    for side, y_sign in (("left", 1.0), ("right", -1.0)):
        bracket = model.part(f"{side}_extension_bracket")
        bracket.visual(
            Box((0.042, 0.022, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, 0.009)),
            material=alloy,
            name="carriage_body",
        )
        bracket.visual(
            Box((0.046, 0.020, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.023)),
            material=black_hardware,
            name="extension_clamp",
        )
        bracket.inertial = Inertial.from_geometry(
            Box((0.046, 0.024, 0.030)),
            mass=0.18,
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
        )

        extension = model.part(f"{side}_extension")
        extension.visual(extension_mesh, material=carbon, name="extension_tube")
        extension.visual(
            Box((0.170, 0.020, 0.010)),
            origin=Origin(xyz=(0.155, 0.0, 0.028)),
            material=black_hardware,
            name="pad_track",
        )
        extension.inertial = Inertial.from_geometry(
            Box((0.400, 0.030, 0.130)),
            mass=0.22,
            origin=Origin(xyz=(0.160, 0.0, 0.050)),
        )

        pad = model.part(f"{side}_pad_block")
        pad.visual(
            Box((0.040, 0.028, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=black_hardware,
            name="pad_base",
        )
        pad.visual(
            Box((0.026, 0.022, 0.024)),
            origin=Origin(xyz=(0.0, 0.0, 0.022)),
            material=alloy,
            name="pad_riser",
        )
        pad.visual(
            Box((0.105, 0.045, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.040)),
            material=black_hardware,
            name="pad_tray",
        )
        pad.visual(
            Box((0.110, 0.055, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, 0.055)),
            material=pad_foam,
            name="pad_surface",
        )
        pad.inertial = Inertial.from_geometry(
            Box((0.115, 0.060, 0.075)),
            mass=0.16,
            origin=Origin(xyz=(0.0, 0.0, 0.038)),
        )

        model.articulation(
            f"stem_to_{side}_bracket",
            ArticulationType.PRISMATIC,
            parent=stem,
            child=bracket,
            origin=Origin(xyz=(0.065, 0.046 * y_sign, -0.037)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=40.0,
                velocity=0.08,
                lower=0.0,
                upper=0.050,
            ),
        )
        model.articulation(
            f"{side}_bracket_to_extension",
            ArticulationType.FIXED,
            parent=bracket,
            child=extension,
            origin=Origin(xyz=(0.0, 0.0, 0.0391)),
        )
        model.articulation(
            f"{side}_extension_to_pad",
            ArticulationType.PRISMATIC,
            parent=extension,
            child=pad,
            origin=Origin(xyz=(0.118, 0.0, 0.033)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=0.12,
                lower=-0.030,
                upper=0.070,
            ),
        )

    model.articulation(
        "stem_to_fork",
        ArticulationType.FIXED,
        parent=stem,
        child=fork,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stem = object_model.get_part("stem")
    fork = object_model.get_part("fork")
    left_bracket = object_model.get_part("left_extension_bracket")
    right_bracket = object_model.get_part("right_extension_bracket")
    left_extension = object_model.get_part("left_extension")
    right_extension = object_model.get_part("right_extension")
    left_pad = object_model.get_part("left_pad_block")
    right_pad = object_model.get_part("right_pad_block")

    stem_to_left_bracket = object_model.get_articulation("stem_to_left_bracket")
    stem_to_right_bracket = object_model.get_articulation("stem_to_right_bracket")
    left_extension_to_pad = object_model.get_articulation("left_extension_to_pad")
    right_extension_to_pad = object_model.get_articulation("right_extension_to_pad")

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

    ctx.expect_contact(fork, stem, name="fork_clamped_into_stem")
    ctx.expect_contact(left_bracket, stem, name="left_bracket_on_rail")
    ctx.expect_contact(right_bracket, stem, name="right_bracket_on_rail")
    ctx.expect_contact(left_extension, left_bracket, name="left_extension_clamped")
    ctx.expect_contact(right_extension, right_bracket, name="right_extension_clamped")
    ctx.expect_contact(left_pad, left_extension, name="left_pad_carriage_contacts_extension")
    ctx.expect_contact(right_pad, right_extension, name="right_pad_carriage_contacts_extension")

    ctx.expect_overlap(left_bracket, stem, axes="x", min_overlap=0.040, name="left_bracket_has_rail_engagement")
    ctx.expect_overlap(right_bracket, stem, axes="x", min_overlap=0.040, name="right_bracket_has_rail_engagement")
    ctx.expect_origin_distance(left_bracket, right_bracket, axes="y", min_dist=0.070, name="brackets_are_separated")

    left_bracket_rest = ctx.part_world_position(left_bracket)
    right_bracket_rest = ctx.part_world_position(right_bracket)
    left_pad_rest = ctx.part_world_position(left_pad)
    right_pad_rest = ctx.part_world_position(right_pad)
    assert left_bracket_rest is not None
    assert right_bracket_rest is not None
    assert left_pad_rest is not None
    assert right_pad_rest is not None

    with ctx.pose({stem_to_left_bracket: 0.050, stem_to_right_bracket: 0.050}):
        left_bracket_fwd = ctx.part_world_position(left_bracket)
        right_bracket_fwd = ctx.part_world_position(right_bracket)
        assert left_bracket_fwd is not None
        assert right_bracket_fwd is not None
        assert left_bracket_fwd[0] > left_bracket_rest[0] + 0.045
        assert right_bracket_fwd[0] > right_bracket_rest[0] + 0.045
        ctx.expect_contact(left_bracket, stem, name="left_bracket_stays_supported_at_max_slide")
        ctx.expect_contact(right_bracket, stem, name="right_bracket_stays_supported_at_max_slide")

    with ctx.pose({left_extension_to_pad: 0.070, right_extension_to_pad: 0.070}):
        left_pad_fwd = ctx.part_world_position(left_pad)
        right_pad_fwd = ctx.part_world_position(right_pad)
        assert left_pad_fwd is not None
        assert right_pad_fwd is not None
        assert left_pad_fwd[0] > left_pad_rest[0] + 0.065
        assert right_pad_fwd[0] > right_pad_rest[0] + 0.065
        ctx.expect_contact(left_pad, left_extension, name="left_pad_keeps_extension_contact")
        ctx.expect_contact(right_pad, right_extension, name="right_pad_keeps_extension_contact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

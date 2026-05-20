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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


HEAD_TILT = 0.0
CROWN_CENTER = (0.0, 0.0, 0.56)
STANCHION_OFFSET_X = 0.055
STANCHION_RADIUS = 0.0175
STANCHION_LENGTH = 0.32
TRAVEL = 0.12


def _rotate_x(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def _add(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _tilted_origin(local_xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=_add(CROWN_CENTER, _rotate_x(local_xyz, HEAD_TILT)), rpy=(HEAD_TILT, 0.0, 0.0))


def _shell_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float, segments: int = 56):
    half = length * 0.5
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
    )
    return mesh_from_geometry(geom, name)


def _aabb_center_x(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> float:
    return (aabb[0][0] + aabb[1][0]) * 0.5


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="full_suspension_mtb_fork")

    crown_black = model.material("crown_black", rgba=(0.12, 0.12, 0.13, 1.0))
    lower_black = model.material("lower_black", rgba=(0.10, 0.10, 0.11, 1.0))
    anodized_silver = model.material("anodized_silver", rgba=(0.78, 0.79, 0.81, 1.0))
    matte_alloy = model.material("matte_alloy", rgba=(0.62, 0.64, 0.67, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))

    crown_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (-0.082, 0.0, -0.002),
                (-0.036, 0.0, 0.008),
                (0.0, 0.0, 0.012),
                (0.036, 0.0, 0.008),
                (0.082, 0.0, -0.002),
            ],
            profile=rounded_rect_profile(0.044, 0.032, radius=0.010, corner_segments=8),
            samples_per_segment=12,
            cap_profile=True,
        ),
        "fork_crown_body_v2",
    )
    lower_bridge_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (-STANCHION_OFFSET_X + 0.012, 0.028, -0.436),
                (-0.040, 0.060, -0.462),
                (0.0, 0.078, -0.474),
                (0.040, 0.060, -0.462),
                (STANCHION_OFFSET_X - 0.012, 0.028, -0.436),
            ],
            profile=rounded_rect_profile(0.034, 0.022, radius=0.006, corner_segments=6),
            samples_per_segment=14,
            cap_profile=True,
        ),
        "fork_lower_bridge_v2",
    )
    stem_body_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (0.0, 0.000, -0.010),
                (0.0, 0.032, -0.008),
                (0.0, 0.062, -0.002),
            ],
            profile=rounded_rect_profile(0.036, 0.020, radius=0.006, corner_segments=6),
            samples_per_segment=12,
            cap_profile=True,
        ),
        "stem_body_v2",
    )
    left_outer_sleeve_mesh = _shell_mesh(
        "left_outer_sleeve_v2",
        outer_radius=0.027,
        inner_radius=0.0195,
        length=0.24,
    )
    right_outer_sleeve_mesh = _shell_mesh(
        "right_outer_sleeve_v2",
        outer_radius=0.027,
        inner_radius=0.0195,
        length=0.24,
    )
    left_wiper_mesh = _shell_mesh(
        "left_wiper_seal_v3",
        outer_radius=0.0245,
        inner_radius=0.0185,
        length=0.008,
    )
    right_wiper_mesh = _shell_mesh(
        "right_wiper_seal_v3",
        outer_radius=0.0245,
        inner_radius=0.0185,
        length=0.008,
    )
    steerer_clamp_mesh = _shell_mesh(
        "steerer_clamp_v2",
        outer_radius=0.024,
        inner_radius=0.0143,
        length=0.040,
    )
    handlebar_clamp_mesh = _shell_mesh(
        "handlebar_clamp_v2",
        outer_radius=0.024,
        inner_radius=0.0163,
        length=0.052,
    )
    handlebar_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.39, -0.032, -0.016),
                (-0.30, -0.046, -0.008),
                (-0.18, -0.022, 0.002),
                (-0.08, -0.008, 0.002),
                (0.0, 0.000, 0.000),
                (0.08, -0.008, 0.002),
                (0.18, -0.022, 0.002),
                (0.30, -0.046, -0.008),
                (0.39, -0.032, -0.016),
            ],
            radius=0.0111,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "wide_flat_handlebar_v2",
    )

    upper_assembly = model.part("upper_assembly")
    upper_assembly.visual(crown_mesh, origin=Origin(xyz=CROWN_CENTER, rpy=(HEAD_TILT, 0.0, 0.0)), material=crown_black, name="crown_body")
    upper_assembly.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=_tilted_origin((-STANCHION_OFFSET_X, 0.0, 0.002)),
        material=crown_black,
        name="left_crown_shoulder",
    )
    upper_assembly.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=_tilted_origin((STANCHION_OFFSET_X, 0.0, 0.002)),
        material=crown_black,
        name="right_crown_shoulder",
    )
    upper_assembly.visual(
        Cylinder(radius=0.025, length=0.036),
        origin=_tilted_origin((0.0, 0.0, 0.018)),
        material=crown_black,
        name="steerer_base_boss",
    )
    upper_assembly.visual(
        Cylinder(radius=0.019, length=0.090),
        origin=_tilted_origin((0.0, 0.0, 0.064)),
        material=matte_alloy,
        name="steerer_lower_taper",
    )
    upper_assembly.visual(
        Cylinder(radius=0.016, length=0.036),
        origin=_tilted_origin((0.0, 0.0, 0.127)),
        material=matte_alloy,
        name="steerer_transition",
    )
    upper_assembly.visual(
        Cylinder(radius=0.0143, length=0.170),
        origin=_tilted_origin((0.0, 0.0, 0.230)),
        material=matte_alloy,
        name="steerer_upper",
    )
    upper_assembly.visual(
        Cylinder(radius=STANCHION_RADIUS, length=STANCHION_LENGTH),
        origin=_tilted_origin((-STANCHION_OFFSET_X, 0.0, -0.172)),
        material=anodized_silver,
        name="left_stanchion",
    )
    upper_assembly.visual(
        Cylinder(radius=STANCHION_RADIUS, length=STANCHION_LENGTH),
        origin=_tilted_origin((STANCHION_OFFSET_X, 0.0, -0.172)),
        material=anodized_silver,
        name="right_stanchion",
    )
    upper_assembly.inertial = Inertial.from_geometry(
        Box((0.22, 0.42, 0.72)),
        mass=2.4,
        origin=Origin(xyz=(0.0, -0.040, 0.56)),
    )

    lower_assembly = model.part("lower_assembly")
    lower_assembly.visual(
        left_outer_sleeve_mesh,
        origin=Origin(xyz=(-STANCHION_OFFSET_X, 0.0, -0.300)),
        material=lower_black,
        name="left_outer_sleeve",
    )
    lower_assembly.visual(
        right_outer_sleeve_mesh,
        origin=Origin(xyz=(STANCHION_OFFSET_X, 0.0, -0.300)),
        material=lower_black,
        name="right_outer_sleeve",
    )
    lower_assembly.visual(
        left_wiper_mesh,
        origin=Origin(xyz=(-STANCHION_OFFSET_X, 0.0, -0.184)),
        material=rubber_black,
        name="left_wiper_seal",
    )
    lower_assembly.visual(
        right_wiper_mesh,
        origin=Origin(xyz=(STANCHION_OFFSET_X, 0.0, -0.184)),
        material=rubber_black,
        name="right_wiper_seal",
    )
    lower_assembly.visual(
        Box((0.026, 0.008, 0.014)),
        origin=Origin(xyz=(-STANCHION_OFFSET_X, -0.0215, -0.184)),
        material=lower_black,
        name="left_guide_pad",
    )
    lower_assembly.visual(
        Box((0.026, 0.008, 0.014)),
        origin=Origin(xyz=(STANCHION_OFFSET_X, -0.0215, -0.184)),
        material=lower_black,
        name="right_guide_pad",
    )
    lower_assembly.visual(
        Box((0.058, 0.066, 0.190)),
        origin=Origin(xyz=(-STANCHION_OFFSET_X, 0.050, -0.500)),
        material=lower_black,
        name="left_lower_leg",
    )
    lower_assembly.visual(
        Box((0.058, 0.066, 0.190)),
        origin=Origin(xyz=(STANCHION_OFFSET_X, 0.050, -0.500)),
        material=lower_black,
        name="right_lower_leg",
    )
    lower_assembly.visual(
        lower_bridge_mesh,
        material=lower_black,
        name="crown_bridge",
    )
    lower_assembly.visual(
        Box((0.040, 0.040, 0.050)),
        origin=Origin(xyz=(-STANCHION_OFFSET_X, 0.050, -0.612)),
        material=lower_black,
        name="left_dropout",
    )
    lower_assembly.visual(
        Box((0.040, 0.040, 0.050)),
        origin=Origin(xyz=(STANCHION_OFFSET_X, 0.050, -0.612)),
        material=lower_black,
        name="right_dropout",
    )
    lower_assembly.visual(
        Cylinder(radius=0.017, length=0.026),
        origin=Origin(xyz=(-STANCHION_OFFSET_X + 0.008, 0.050, -0.612), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lower_black,
        name="left_axle_boss",
    )
    lower_assembly.visual(
        Cylinder(radius=0.017, length=0.026),
        origin=Origin(xyz=(STANCHION_OFFSET_X - 0.008, 0.050, -0.612), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lower_black,
        name="right_axle_boss",
    )
    lower_assembly.inertial = Inertial.from_geometry(
        Box((0.18, 0.16, 0.58)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.024, -0.300)),
    )

    stem = model.part("stem")
    stem.visual(
        Box((0.040, 0.008, 0.040)),
        origin=Origin(xyz=(0.0, -0.0183, 0.0)),
        material=matte_alloy,
        name="steerer_clamp",
    )
    stem.visual(
        Box((0.032, 0.042, 0.014)),
        origin=Origin(xyz=(0.0, 0.036, -0.024)),
        material=matte_alloy,
        name="stem_body",
    )
    stem.visual(
        Box((0.010, 0.036, 0.040)),
        origin=Origin(xyz=(-0.0193, -0.002, 0.0)),
        material=matte_alloy,
        name="left_stem_cheek",
    )
    stem.visual(
        Box((0.010, 0.036, 0.040)),
        origin=Origin(xyz=(0.0193, -0.002, 0.0)),
        material=matte_alloy,
        name="right_stem_cheek",
    )
    stem.visual(
        Box((0.048, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.066, -0.0196)),
        material=matte_alloy,
        name="handlebar_clamp",
    )
    stem.inertial = Inertial.from_geometry(
        Box((0.06, 0.12, 0.06)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.030, 0.000)),
    )

    handlebar = model.part("handlebar")
    handlebar.visual(handlebar_mesh, material=crown_black, name="bar_tube")
    handlebar.visual(
        Cylinder(radius=0.0156, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=crown_black,
        name="bar_center_bulge",
    )
    handlebar.visual(
        Cylinder(radius=0.016, length=0.130),
        origin=Origin(xyz=(-0.335, -0.038, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="left_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.016, length=0.130),
        origin=Origin(xyz=(0.335, -0.038, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="right_grip",
    )
    handlebar.inertial = Inertial.from_geometry(
        Box((0.82, 0.10, 0.06)),
        mass=0.32,
        origin=Origin(xyz=(0.0, -0.020, 0.014)),
    )

    model.articulation(
        "fork_travel",
        ArticulationType.PRISMATIC,
        parent=upper_assembly,
        child=lower_assembly,
        origin=Origin(xyz=CROWN_CENTER, rpy=(HEAD_TILT, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=1.0,
            lower=0.0,
            upper=TRAVEL,
        ),
    )
    model.articulation(
        "upper_to_stem",
        ArticulationType.FIXED,
        parent=upper_assembly,
        child=stem,
        origin=_tilted_origin((0.0, 0.0, 0.230)),
    )
    model.articulation(
        "stem_to_handlebar",
        ArticulationType.FIXED,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.0, 0.066, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    upper_assembly = object_model.get_part("upper_assembly")
    lower_assembly = object_model.get_part("lower_assembly")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    fork_travel = object_model.get_articulation("fork_travel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(lower_assembly, upper_assembly, elem_a="left_guide_pad", elem_b="left_stanchion")
    ctx.expect_contact(lower_assembly, upper_assembly, elem_a="right_guide_pad", elem_b="right_stanchion")
    ctx.expect_contact(stem, upper_assembly, elem_a="steerer_clamp", elem_b="steerer_upper")
    ctx.expect_contact(handlebar, stem, elem_a="bar_center_bulge", elem_b="handlebar_clamp")

    ctx.expect_overlap(lower_assembly, upper_assembly, axes="xy", elem_a="left_outer_sleeve", elem_b="left_stanchion", min_overlap=0.030)
    ctx.expect_overlap(lower_assembly, upper_assembly, axes="xy", elem_a="right_outer_sleeve", elem_b="right_stanchion", min_overlap=0.030)

    handlebar_aabb = ctx.part_world_aabb(handlebar)
    if handlebar_aabb is not None:
        handlebar_width = handlebar_aabb[1][0] - handlebar_aabb[0][0]
        ctx.check(
            "handlebar_width_realistic",
            0.74 <= handlebar_width <= 0.84,
            details=f"Expected wide flat bar width in [0.74, 0.84] m, got {handlebar_width:.3f} m",
        )

    left_stanchion_aabb = ctx.part_element_world_aabb(upper_assembly, elem="left_stanchion")
    right_stanchion_aabb = ctx.part_element_world_aabb(upper_assembly, elem="right_stanchion")
    if left_stanchion_aabb is not None and right_stanchion_aabb is not None:
        spacing = _aabb_center_x(right_stanchion_aabb) - _aabb_center_x(left_stanchion_aabb)
        ctx.check(
            "stanchion_spacing_realistic",
            0.10 <= spacing <= 0.12,
            details=f"Expected fork stanchion spacing in [0.10, 0.12] m, got {spacing:.3f} m",
        )

    lower_rest = ctx.part_world_position(lower_assembly)
    limits = fork_travel.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({fork_travel: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="fork_travel_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="fork_travel_lower_no_floating")
            ctx.expect_contact(lower_assembly, upper_assembly, elem_a="left_guide_pad", elem_b="left_stanchion", name="fork_travel_lower_left_guided")
            ctx.expect_contact(lower_assembly, upper_assembly, elem_a="right_guide_pad", elem_b="right_stanchion", name="fork_travel_lower_right_guided")

        with ctx.pose({fork_travel: limits.upper}):
            lower_compressed = ctx.part_world_position(lower_assembly)
            if lower_rest is not None and lower_compressed is not None:
                ctx.check(
                    "fork_travel_moves_upward",
                    lower_compressed[2] > lower_rest[2] + 0.08,
                    details=(
                        "Expected the lower fork assembly to move substantially upward under compression; "
                        f"rest z={lower_rest[2]:.3f}, compressed z={lower_compressed[2]:.3f}"
                    ),
                )
            ctx.fail_if_parts_overlap_in_current_pose(name="fork_travel_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="fork_travel_upper_no_floating")
            ctx.expect_contact(lower_assembly, upper_assembly, elem_a="left_guide_pad", elem_b="left_stanchion", name="fork_travel_upper_left_guided")
            ctx.expect_contact(lower_assembly, upper_assembly, elem_a="right_guide_pad", elem_b="right_stanchion", name="fork_travel_upper_right_guided")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

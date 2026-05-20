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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _section_loop(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _section_loop_y(width: float, height: float, radius: float, y: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, z in rounded_rect_profile(width, height, radius, corner_segments=8)]


def _mirror_path_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _aabb_size(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        upper[0] - lower[0],
        upper[1] - lower[1],
        upper[2] - lower[2],
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_searchlight_tower", assets=ASSETS)

    matte_charcoal = model.material("matte_charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.31, 0.33, 0.36, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    coated_steel = model.material("coated_steel", rgba=(0.48, 0.50, 0.53, 1.0))
    deck_gray = model.material("deck_gray", rgba=(0.24, 0.26, 0.28, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.84, 0.90, 0.32))
    anti_glare_black = model.material("anti_glare_black", rgba=(0.07, 0.07, 0.08, 1.0))

    mast_mesh = _save_mesh(
        "searchlight_tower_mast.obj",
        section_loft(
            [
                _section_loop(0.26, 0.30, 0.030, 0.32),
                _section_loop(0.23, 0.26, 0.027, 0.92),
                _section_loop(0.19, 0.22, 0.022, 1.48),
            ]
        ),
    )
    upper_guard_mesh = _save_mesh(
        "searchlight_upper_guard.obj",
        wire_from_points(
            [
                (-0.26, 0.13, 1.565),
                (-0.26, 0.13, 1.80),
                (-0.31, 0.44, 1.80),
                (0.31, 0.44, 1.80),
                (0.26, 0.13, 1.80),
                (0.26, 0.13, 1.565),
            ],
            radius=0.011,
            radial_segments=16,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.035,
            corner_segments=8,
        ),
    )
    mid_guard_mesh = _save_mesh(
        "searchlight_mid_guard.obj",
        wire_from_points(
            [
                (-0.24, 0.15, 1.565),
                (-0.24, 0.15, 1.69),
                (-0.29, 0.42, 1.69),
                (0.29, 0.42, 1.69),
                (0.24, 0.15, 1.69),
                (0.24, 0.15, 1.565),
            ],
            radius=0.009,
            radial_segments=14,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.030,
            corner_segments=8,
        ),
    )
    conduit_mesh = _save_mesh(
        "searchlight_service_conduit.obj",
        tube_from_spline_points(
            [
                (0.090, 0.102, 1.495),
                (0.135, 0.108, 1.528),
                (0.190, 0.116, 1.560),
                (0.248, 0.126, 1.594),
            ],
            radius=0.008,
            samples_per_segment=12,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    yoke_profile = rounded_rect_profile(0.035, 0.075, radius=0.008, corner_segments=6)
    left_arm_path = [
        (-0.095, -0.01, 0.11),
        (-0.115, -0.01, 0.22),
        (-0.140, -0.005, 0.34),
        (-0.182, 0.0, 0.43),
    ]
    right_arm_path = _mirror_path_x(left_arm_path)
    left_yoke_arm = _save_mesh(
        "searchlight_left_yoke_arm.obj",
        sweep_profile_along_spline(
            left_arm_path,
            profile=yoke_profile,
            samples_per_segment=16,
            cap_profile=True,
        ),
    )
    right_yoke_arm = _save_mesh(
        "searchlight_right_yoke_arm.obj",
        sweep_profile_along_spline(
            right_arm_path,
            profile=yoke_profile,
            samples_per_segment=16,
            cap_profile=True,
        ),
    )
    yoke_pedestal_mesh = _save_mesh(
        "searchlight_yoke_pedestal.obj",
        section_loft(
            [
                _section_loop(0.16, 0.11, 0.018, 0.09),
                _section_loop(0.18, 0.10, 0.020, 0.16),
                _section_loop(0.22, 0.08, 0.020, 0.24),
                _section_loop(0.20, 0.075, 0.018, 0.27),
            ]
        ),
    )
    head_shell_mesh = _save_mesh(
        "searchlight_head_shell.obj",
        section_loft(
            [
                _section_loop_y(0.150, 0.165, 0.028, -0.140),
                _section_loop_y(0.196, 0.210, 0.034, -0.040),
                _section_loop_y(0.222, 0.232, 0.040, 0.080),
                _section_loop_y(0.206, 0.220, 0.038, 0.170),
            ]
        ),
    )

    tower = model.part("tower")
    tower.visual(
        Box((0.70, 0.70, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=deck_gray,
        name="foundation_plinth",
    )
    tower.visual(
        Box((0.42, 0.42, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=coated_steel,
        name="foundation_pad",
    )
    tower.visual(
        Box((0.30, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=satin_graphite,
        name="base_collar",
    )
    tower.visual(
        mast_mesh,
        material=matte_charcoal,
        name="mast_shell",
    )
    tower.visual(
        Box((0.104, 0.012, 0.224)),
        origin=Origin(xyz=(0.0, 0.147, 0.52)),
        material=satin_graphite,
        name="service_door",
    )
    tower.visual(
        Box((0.24, 0.26, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.51)),
        material=satin_graphite,
        name="top_cap",
    )
    tower.visual(
        Box((0.12, 0.12, 0.05)),
        origin=Origin(xyz=(-0.13, 0.17, 1.515)),
        material=satin_graphite,
        name="left_support_bracket",
    )
    tower.visual(
        Box((0.12, 0.12, 0.05)),
        origin=Origin(xyz=(0.13, 0.17, 1.515)),
        material=satin_graphite,
        name="right_support_bracket",
    )
    tower.visual(
        Box((0.66, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.42, 1.515)),
        material=satin_graphite,
        name="front_deck_beam",
    )
    tower.visual(
        Box((0.62, 0.34, 0.025)),
        origin=Origin(xyz=(0.0, 0.28, 1.5525)),
        material=deck_gray,
        name="platform_deck",
    )
    tower.visual(
        Box((0.64, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, 0.44, 1.565)),
        material=coated_steel,
        name="front_trim",
    )
    tower.visual(upper_guard_mesh, material=satin_aluminum, name="guard_upper")
    tower.visual(mid_guard_mesh, material=coated_steel, name="guard_mid")
    tower.visual(
        Cylinder(radius=0.010, length=1.28),
        origin=Origin(xyz=(-0.18, 0.146, 1.00)),
        material=satin_aluminum,
        name="ladder_left_rail",
    )
    tower.visual(
        Cylinder(radius=0.010, length=1.28),
        origin=Origin(xyz=(-0.09, 0.146, 1.00)),
        material=satin_aluminum,
        name="ladder_right_rail",
    )
    for index in range(10):
        z = 0.48 + index * 0.105
        tower.visual(
            Cylinder(radius=0.0045, length=0.09),
            origin=Origin(xyz=(-0.135, 0.146, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=coated_steel,
            name=f"ladder_rung_{index:02d}",
        )
    for index, z in enumerate((0.62, 0.98, 1.34)):
        tower.visual(
            Box((0.11, 0.036, 0.03)),
            origin=Origin(xyz=(-0.135, 0.128, z)),
            material=satin_graphite,
            name=f"ladder_standoff_{index:02d}",
        )
    tower.visual(
        Box((0.12, 0.10, 0.18)),
        origin=Origin(xyz=(0.25, 0.13, 1.655)),
        material=satin_graphite,
        name="service_cabinet",
    )
    tower.visual(
        Box((0.010, 0.078, 0.14)),
        origin=Origin(xyz=(0.305, 0.13, 1.66)),
        material=coated_steel,
        name="service_cabinet_door",
    )
    tower.visual(
        Box((0.09, 0.08, 0.09)),
        origin=Origin(xyz=(-0.245, 0.13, 1.61)),
        material=satin_graphite,
        name="junction_box",
    )
    tower.visual(
        Box((0.06, 0.05, 0.05)),
        origin=Origin(xyz=(-0.245, 0.132, 1.68)),
        material=coated_steel,
        name="junction_box_cap",
    )
    tower.visual(conduit_mesh, material=coated_steel, name="service_conduit")
    tower.inertial = Inertial.from_geometry(
        Box((0.70, 0.70, 1.82)),
        mass=140.0,
        origin=Origin(xyz=(0.0, 0.0, 0.91)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.10, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=satin_graphite,
        name="lower_bearing",
    )
    pan_yoke.visual(
        Cylinder(radius=0.112, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=satin_aluminum,
        name="bearing_spacer",
    )
    pan_yoke.visual(
        Cylinder(radius=0.13, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=matte_charcoal,
        name="rotating_deck",
    )
    pan_yoke.visual(yoke_pedestal_mesh, material=matte_charcoal, name="yoke_pedestal")
    pan_yoke.visual(
        Box((0.22, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, -0.01, 0.21)),
        material=satin_graphite,
        name="lower_yoke_bridge",
    )
    pan_yoke.visual(
        Box((0.14, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, -0.03, 0.13)),
        material=satin_graphite,
        name="drive_housing",
    )
    pan_yoke.visual(left_yoke_arm, material=matte_charcoal, name="left_yoke_arm")
    pan_yoke.visual(right_yoke_arm, material=matte_charcoal, name="right_yoke_arm")
    pan_yoke.visual(
        Cylinder(radius=0.045, length=0.032),
        origin=Origin(xyz=(-0.160, 0.0, 0.49), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="left_tilt_collar",
    )
    pan_yoke.visual(
        Cylinder(radius=0.045, length=0.032),
        origin=Origin(xyz=(0.160, 0.0, 0.49), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="right_tilt_collar",
    )
    pan_yoke.visual(
        Box((0.020, 0.060, 0.062)),
        origin=Origin(xyz=(-0.160, 0.0, 0.415)),
        material=satin_graphite,
        name="left_tilt_web",
    )
    pan_yoke.visual(
        Box((0.020, 0.060, 0.062)),
        origin=Origin(xyz=(0.160, 0.0, 0.415)),
        material=satin_graphite,
        name="right_tilt_web",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.38, 0.24, 0.58)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
    )

    spotlight_head = model.part("spotlight_head")
    spotlight_head.visual(
        Cylinder(radius=0.090, length=0.05),
        origin=Origin(xyz=(0.0, -0.165, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_graphite,
        name="rear_cap",
    )
    spotlight_head.visual(
        Cylinder(radius=0.050, length=0.03),
        origin=Origin(xyz=(0.0, -0.205, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=coated_steel,
        name="rear_service_stub",
    )
    spotlight_head.visual(head_shell_mesh, material=matte_charcoal, name="main_body")
    for index, y in enumerate((-0.09, -0.05, -0.01)):
        spotlight_head.visual(
            Cylinder(radius=0.111, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=anti_glare_black,
            name=f"cooling_fin_{index:02d}",
        )
    spotlight_head.visual(
        Cylinder(radius=0.125, length=0.06),
        origin=Origin(xyz=(0.0, 0.20, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="front_bezel",
    )
    spotlight_head.visual(
        Cylinder(radius=0.104, length=0.008),
        origin=Origin(xyz=(0.0, 0.234, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    spotlight_head.visual(
        Cylinder(radius=0.128, length=0.018),
        origin=Origin(xyz=(0.0, 0.247, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anti_glare_black,
        name="sunshade_ring",
    )
    spotlight_head.visual(
        Box((0.12, 0.09, 0.016)),
        origin=Origin(xyz=(0.0, -0.015, 0.104)),
        material=coated_steel,
        name="top_access_cover",
    )
    spotlight_head.visual(
        Box((0.14, 0.10, 0.018)),
        origin=Origin(xyz=(0.0, -0.03, -0.104)),
        material=satin_graphite,
        name="lower_service_cover",
    )
    spotlight_head.visual(
        Cylinder(radius=0.036, length=0.036),
        origin=Origin(xyz=(-0.126, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="left_trunnion",
    )
    spotlight_head.visual(
        Cylinder(radius=0.036, length=0.036),
        origin=Origin(xyz=(0.126, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="right_trunnion",
    )
    spotlight_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.13, length=0.49),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.02, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "pan_rotate",
        ArticulationType.CONTINUOUS,
        parent="tower",
        child="pan_yoke",
        origin=Origin(xyz=(0.0, 0.28, 1.565)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.4),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent="pan_yoke",
        child="spotlight_head",
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.2, lower=-0.55, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("tower")
    pan_yoke = object_model.get_part("pan_yoke")
    spotlight_head = object_model.get_part("spotlight_head")
    pan_rotate = object_model.get_articulation("pan_rotate")
    head_tilt = object_model.get_articulation("head_tilt")

    mast_shell = tower.get_visual("mast_shell")
    platform_deck = tower.get_visual("platform_deck")
    lower_bearing = pan_yoke.get_visual("lower_bearing")
    left_tilt_collar = pan_yoke.get_visual("left_tilt_collar")
    right_tilt_collar = pan_yoke.get_visual("right_tilt_collar")
    front_lens = spotlight_head.get_visual("front_lens")
    left_trunnion = spotlight_head.get_visual("left_trunnion")
    right_trunnion = spotlight_head.get_visual("right_trunnion")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    tower_size = _aabb_size(ctx.part_world_aabb(tower))
    head_size = _aabb_size(ctx.part_world_aabb(spotlight_head))

    ctx.check(
        "pan_axis_is_vertical",
        getattr(pan_rotate, "axis", None) == (0.0, 0.0, 1.0),
        f"expected pan axis (0, 0, 1), got {getattr(pan_rotate, 'axis', None)}",
    )
    ctx.check(
        "tilt_axis_is_cross_tower",
        getattr(head_tilt, "axis", None) == (1.0, 0.0, 0.0),
        f"expected tilt axis (1, 0, 0), got {getattr(head_tilt, 'axis', None)}",
    )
    ctx.check(
        "tower_proportions_read_as_searchlight_tower",
        tower_size is not None
        and 0.65 <= tower_size[0] <= 0.90
        and 0.65 <= tower_size[1] <= 0.90
        and 1.75 <= tower_size[2] <= 2.10,
        f"unexpected tower overall size: {tower_size}",
    )
    ctx.check(
        "head_proportions_read_as_large_searchlight",
        head_size is not None
        and 0.24 <= head_size[0] <= 0.32
        and 0.46 <= head_size[1] <= 0.58
        and 0.20 <= head_size[2] <= 0.30,
        f"unexpected spotlight head size: {head_size}",
    )
    ctx.expect_gap(
        pan_yoke,
        tower,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=lower_bearing,
        negative_elem=platform_deck,
        name="pan_stage_seats_cleanly_on_platform",
    )
    ctx.expect_overlap(
        pan_yoke,
        tower,
        axes="xy",
        min_overlap=0.18,
        elem_a=lower_bearing,
        elem_b=platform_deck,
        name="pan_stage_has_believable_deck_footprint",
    )
    ctx.expect_contact(
        spotlight_head,
        pan_yoke,
        elem_a=left_trunnion,
        elem_b=left_tilt_collar,
        name="left_trunnion_contacts_left_bearing_collar",
    )
    ctx.expect_contact(
        spotlight_head,
        pan_yoke,
        elem_a=right_trunnion,
        elem_b=right_tilt_collar,
        name="right_trunnion_contacts_right_bearing_collar",
    )
    ctx.expect_gap(
        spotlight_head,
        tower,
        axis="y",
        min_gap=0.06,
        positive_elem=front_lens,
        negative_elem=mast_shell,
        name="lens_projects_forward_of_mast",
    )
    ctx.expect_gap(
        spotlight_head,
        pan_yoke,
        axis="z",
        min_gap=0.28,
        positive_elem=front_lens,
        negative_elem=lower_bearing,
        name="lens_clears_pan_stage",
    )

    with ctx.pose({pan_rotate: 1.15}):
        ctx.expect_gap(
            pan_yoke,
            tower,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=lower_bearing,
            negative_elem=platform_deck,
            name="pan_stage_remains_seated_while_rotated",
        )
        ctx.expect_overlap(
            pan_yoke,
            tower,
            axes="xy",
            min_overlap=0.18,
            elem_a=lower_bearing,
            elem_b=platform_deck,
            name="pan_stage_keeps_centered_footprint_while_rotated",
        )

    with ctx.pose({head_tilt: 0.80}):
        ctx.expect_contact(
            spotlight_head,
            pan_yoke,
            elem_a=left_trunnion,
            elem_b=left_tilt_collar,
            name="left_trunnion_stays_supported_at_up_tilt",
        )
        ctx.expect_contact(
            spotlight_head,
            pan_yoke,
            elem_a=right_trunnion,
            elem_b=right_tilt_collar,
            name="right_trunnion_stays_supported_at_up_tilt",
        )
        ctx.expect_gap(
            spotlight_head,
            tower,
            axis="z",
            min_gap=0.34,
            positive_elem=front_lens,
            negative_elem=platform_deck,
            name="up_tilt_lens_clears_platform",
        )

    with ctx.pose({head_tilt: -0.40}):
        ctx.expect_contact(
            spotlight_head,
            pan_yoke,
            elem_a=left_trunnion,
            elem_b=left_tilt_collar,
            name="left_trunnion_stays_supported_at_down_tilt",
        )
        ctx.expect_contact(
            spotlight_head,
            pan_yoke,
            elem_a=right_trunnion,
            elem_b=right_tilt_collar,
            name="right_trunnion_stays_supported_at_down_tilt",
        )
        ctx.expect_gap(
            spotlight_head,
            tower,
            axis="z",
            min_gap=0.22,
            positive_elem=front_lens,
            negative_elem=platform_deck,
            name="down_tilt_lens_still_clears_platform",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

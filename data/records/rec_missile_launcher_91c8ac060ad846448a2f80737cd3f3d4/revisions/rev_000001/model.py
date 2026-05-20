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


def _tube_shell_mesh(*, outer_radius: float, wall_thickness: float, length: float):
    half_length = 0.5 * length
    inner_radius = outer_radius - wall_thickness
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -half_length),
            (outer_radius, half_length),
        ],
        [
            (inner_radius, -half_length + wall_thickness),
            (inner_radius, half_length - wall_thickness),
        ],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)


def _aabb_center(aabb):
    return tuple(0.5 * (aabb[0][axis] + aabb[1][axis]) for axis in range(3))


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="truck_bed_missile_launcher_module")

    frame_green = model.material("frame_green", rgba=(0.34, 0.40, 0.28, 1.0))
    darker_green = model.material("darker_green", rgba=(0.25, 0.30, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.42, 0.45, 0.47, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.17, 0.18, 0.20, 1.0))
    tube_finish = model.material("tube_finish", rgba=(0.46, 0.49, 0.43, 1.0))

    tube_shell = mesh_from_geometry(
        _tube_shell_mesh(outer_radius=0.15, wall_thickness=0.012, length=2.30),
        "launch_tube_shell",
    )
    tube_muzzle_ring = mesh_from_geometry(
        _tube_shell_mesh(outer_radius=0.17, wall_thickness=0.016, length=0.10),
        "launch_tube_muzzle_ring",
    )
    left_upright_mesh = _save_mesh(
        "left_upright_sweep",
        sweep_profile_along_spline(
            [
                (-0.16, 0.80, 0.18),
                (-0.04, 0.80, 0.44),
                (0.08, 0.79, 0.68),
                (0.16, 0.78, 0.92),
            ],
            profile=rounded_rect_profile(0.12, 0.18, radius=0.028),
            samples_per_segment=14,
            cap_profile=True,
        ),
    )
    right_upright_mesh = _save_mesh(
        "right_upright_sweep",
        sweep_profile_along_spline(
            [
                (-0.16, -0.80, 0.18),
                (-0.04, -0.80, 0.44),
                (0.08, -0.79, 0.68),
                (0.16, -0.78, 0.92),
            ],
            profile=rounded_rect_profile(0.12, 0.18, radius=0.028),
            samples_per_segment=14,
            cap_profile=True,
        ),
    )
    rear_tie_bar_mesh = _save_mesh(
        "rear_tie_bar_tube",
        tube_from_spline_points(
            [
                (-0.28, 0.78, 0.91),
                (-0.36, 0.36, 0.97),
                (-0.38, 0.0, 1.00),
                (-0.36, -0.36, 0.97),
                (-0.28, -0.78, 0.91),
            ],
            radius=0.05,
            samples_per_segment=12,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    elevation_arm_mesh = _save_mesh(
        "elevation_arm_sweep",
        sweep_profile_along_spline(
            [
                (0.08, 0.0, 0.0),
                (0.46, 0.0, 0.02),
                (0.88, 0.0, 0.05),
                (1.18, 0.0, 0.08),
            ],
            profile=rounded_rect_profile(0.24, 0.18, radius=0.045),
            samples_per_segment=14,
            cap_profile=True,
        ),
    )

    base_frame = model.part("base_frame")
    for x_pos in (-1.85, 1.85):
        for y_pos in (-0.92, 0.92):
            base_frame.visual(
                Box((0.44, 0.26, 0.10)),
                origin=Origin(xyz=(x_pos, y_pos, 0.05)),
                material=darker_green,
                name=f"support_pad_{'f' if x_pos > 0.0 else 'r'}_{'l' if y_pos > 0.0 else 'r'}",
            )
            base_frame.visual(
                Box((0.18, 0.18, 0.22)),
                origin=Origin(xyz=(x_pos, y_pos, 0.17)),
                material=frame_green,
                name=f"support_post_{'f' if x_pos > 0.0 else 'r'}_{'l' if y_pos > 0.0 else 'r'}",
            )
    base_frame.visual(
        Box((4.40, 0.16, 0.24)),
        origin=Origin(xyz=(0.0, 0.92, 0.18)),
        material=frame_green,
        name="left_longitudinal_rail",
    )
    base_frame.visual(
        Box((4.40, 0.16, 0.24)),
        origin=Origin(xyz=(0.0, -0.92, 0.18)),
        material=frame_green,
        name="right_longitudinal_rail",
    )
    for index, x_pos in enumerate((-1.85, -0.55, 0.55, 1.85)):
        base_frame.visual(
            Box((0.18, 1.68, 0.20)),
            origin=Origin(xyz=(x_pos, 0.0, 0.20)),
            material=frame_green,
            name=f"crossmember_{index}",
        )
    base_frame.visual(
        Box((2.10, 1.42, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=frame_green,
        name="deck_plate",
    )
    base_frame.visual(
        Cylinder(radius=0.78, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=steel,
        name="slew_lower_ring",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((4.40, 2.10, 0.70)),
        mass=2200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
    )

    azimuth_turret = model.part("azimuth_turret")
    azimuth_turret.visual(
        Cylinder(radius=0.76, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=steel,
        name="slew_upper_ring",
    )
    azimuth_turret.visual(
        Box((2.00, 1.42, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=frame_green,
        name="turret_deck",
    )
    azimuth_turret.visual(
        Box((0.82, 1.06, 0.40)),
        origin=Origin(xyz=(-0.50, 0.0, 0.40)),
        material=darker_green,
        name="rear_equipment_pack",
    )
    azimuth_turret.visual(left_upright_mesh, material=frame_green, name="left_outer_upright")
    azimuth_turret.visual(right_upright_mesh, material=frame_green, name="right_outer_upright")
    azimuth_turret.visual(
        Box((0.24, 0.10, 0.26)),
        origin=Origin(xyz=(0.10, 0.74, 0.58)),
        material=dark_steel,
        name="left_yoke_arm",
    )
    azimuth_turret.visual(
        Box((0.24, 0.10, 0.26)),
        origin=Origin(xyz=(0.10, -0.74, 0.58)),
        material=dark_steel,
        name="right_yoke_arm",
    )
    azimuth_turret.visual(
        Box((0.34, 0.16, 0.56)),
        origin=Origin(xyz=(0.00, 0.77, 0.50)),
        material=frame_green,
        name="left_side_bracket",
    )
    azimuth_turret.visual(
        Box((0.34, 0.16, 0.56)),
        origin=Origin(xyz=(0.00, -0.77, 0.50)),
        material=frame_green,
        name="right_side_bracket",
    )
    azimuth_turret.visual(rear_tie_bar_mesh, material=frame_green, name="rear_tie_bar")
    azimuth_turret.visual(
        Box((0.30, 0.06, 0.14)),
        origin=Origin(xyz=(-0.17, 0.76, 0.84)),
        material=frame_green,
        name="left_tie_connector",
    )
    azimuth_turret.visual(
        Box((0.30, 0.06, 0.14)),
        origin=Origin(xyz=(-0.17, -0.76, 0.84)),
        material=frame_green,
        name="right_tie_connector",
    )
    azimuth_turret.inertial = Inertial.from_geometry(
        Box((1.90, 1.70, 1.00)),
        mass=950.0,
        origin=Origin(xyz=(-0.05, 0.0, 0.50)),
    )

    tube_bank = model.part("tube_bank")
    tube_bank.visual(
        Cylinder(radius=0.10, length=1.38),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    tube_bank.visual(elevation_arm_mesh, material=frame_green, name="elevation_arm")
    tube_bank.visual(
        Box((1.52, 1.20, 0.06)),
        origin=Origin(xyz=(1.70, 0.0, 0.0)),
        material=frame_green,
        name="center_shelf",
    )
    tube_bank.visual(
        Box((1.52, 1.20, 0.06)),
        origin=Origin(xyz=(1.70, 0.0, 0.36)),
        material=frame_green,
        name="top_beam",
    )
    tube_bank.visual(
        Box((1.52, 1.20, 0.06)),
        origin=Origin(xyz=(1.70, 0.0, -0.36)),
        material=frame_green,
        name="bottom_beam",
    )
    tube_bank.visual(
        Box((0.14, 0.08, 0.78)),
        origin=Origin(xyz=(1.00, 0.56, 0.0)),
        material=frame_green,
        name="left_rear_post",
    )
    tube_bank.visual(
        Box((0.14, 0.08, 0.78)),
        origin=Origin(xyz=(1.00, -0.56, 0.0)),
        material=frame_green,
        name="right_rear_post",
    )
    tube_bank.visual(
        Box((0.14, 0.08, 0.78)),
        origin=Origin(xyz=(2.40, 0.56, 0.0)),
        material=frame_green,
        name="left_front_post",
    )
    tube_bank.visual(
        Box((0.14, 0.08, 0.78)),
        origin=Origin(xyz=(2.40, -0.56, 0.0)),
        material=frame_green,
        name="right_front_post",
    )
    for row_index, z_pos in enumerate((-0.18, 0.18)):
        for col_index, y_pos in enumerate((-0.45, -0.15, 0.15, 0.45)):
            tube_bank.visual(
                tube_shell,
                origin=Origin(xyz=(1.70, y_pos, z_pos)),
                material=tube_finish,
                name=f"tube_{row_index}_{col_index}",
            )
            tube_bank.visual(
                tube_muzzle_ring,
                origin=Origin(xyz=(2.80, y_pos, z_pos)),
                material=dark_steel,
                name=f"tube_muzzle_{row_index}_{col_index}",
            )
    tube_bank.inertial = Inertial.from_geometry(
        Box((2.90, 1.38, 0.90)),
        mass=680.0,
        origin=Origin(xyz=(1.45, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=azimuth_turret,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30000.0,
            velocity=0.55,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=azimuth_turret,
        child=tube_bank,
        origin=Origin(xyz=(0.10, 0.0, 0.58)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24000.0,
            velocity=0.40,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    azimuth_turret = object_model.get_part("azimuth_turret")
    tube_bank = object_model.get_part("tube_bank")

    base_frame.get_visual("deck_plate")
    base_frame.get_visual("slew_lower_ring")
    azimuth_turret.get_visual("slew_upper_ring")
    azimuth_turret.get_visual("left_yoke_arm")
    azimuth_turret.get_visual("right_yoke_arm")
    tube_bank.get_visual("trunnion_shaft")
    tube_bank.get_visual("tube_0_0")
    tube_bank.get_visual("tube_1_3")

    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    elevation_axis = object_model.get_articulation("elevation_axis")

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
        azimuth_turret,
        base_frame,
        elem_a="slew_upper_ring",
        elem_b="slew_lower_ring",
        contact_tol=0.002,
        name="slew_ring_contact",
    )
    ctx.expect_contact(
        tube_bank,
        azimuth_turret,
        elem_a="trunnion_shaft",
        elem_b="left_yoke_arm",
        contact_tol=0.002,
        name="left_trunnion_bearing_contact",
    )
    ctx.expect_contact(
        tube_bank,
        azimuth_turret,
        elem_a="trunnion_shaft",
        elem_b="right_yoke_arm",
        contact_tol=0.002,
        name="right_trunnion_bearing_contact",
    )

    with ctx.pose({elevation_axis: 0.0}):
        ctx.expect_gap(
            tube_bank,
            base_frame,
            axis="z",
            min_gap=0.18,
            positive_elem="bottom_beam",
            negative_elem="deck_plate",
            name="horizontal_bank_clearance_above_deck",
        )

    rest_front_aabb = ctx.part_element_world_aabb(tube_bank, elem="top_beam")
    ctx.check("top_beam_rest_pose_available", rest_front_aabb is not None, "top_beam AABB missing at rest pose")
    rest_front_center = _aabb_center(rest_front_aabb) if rest_front_aabb is not None else (0.0, 0.0, 0.0)
    with ctx.pose({azimuth_rotation: math.pi / 2.0}):
        yawed_front_aabb = ctx.part_element_world_aabb(tube_bank, elem="top_beam")
        ctx.check(
            "top_beam_yawed_pose_available",
            yawed_front_aabb is not None,
            "top_beam AABB missing at 90 degree azimuth pose",
        )
        yawed_front_center = _aabb_center(yawed_front_aabb) if yawed_front_aabb is not None else rest_front_center
        ctx.check(
            "azimuth_rotation_swings_tube_bank_sideways",
            yawed_front_center[1] > rest_front_center[1] + 0.90,
            f"expected y shift > 0.90 m, got rest={rest_front_center} yawed={yawed_front_center}",
        )
        ctx.check(
            "azimuth_rotation_turns_launcher_away_from_x_axis",
            abs(yawed_front_center[0]) < 0.25,
            f"expected near-zero x center after 90 degree slew, got {yawed_front_center[0]:.3f}",
        )
        ctx.expect_contact(
            azimuth_turret,
            base_frame,
            elem_a="slew_upper_ring",
            elem_b="slew_lower_ring",
            contact_tol=0.002,
            name="slew_ring_contact_at_90deg",
        )

    level_top_beam = ctx.part_element_world_aabb(tube_bank, elem="top_beam")
    ctx.check("top_beam_level_pose_available", level_top_beam is not None, "top_beam AABB missing before elevation")
    level_top_center = _aabb_center(level_top_beam) if level_top_beam is not None else (0.0, 0.0, 0.0)
    with ctx.pose({elevation_axis: 1.10}):
        raised_top_beam = ctx.part_element_world_aabb(tube_bank, elem="top_beam")
        ctx.check(
            "top_beam_raised_pose_available",
            raised_top_beam is not None,
            "top_beam AABB missing at maximum elevation",
        )
        raised_top_center = _aabb_center(raised_top_beam) if raised_top_beam is not None else level_top_center
        ctx.check(
            "elevation_axis_raises_tube_bank",
            raised_top_center[2] > level_top_center[2] + 0.70,
            f"expected z rise > 0.70 m, got level={level_top_center} raised={raised_top_center}",
        )
        ctx.check(
            "elevation_axis_pulls_bank_back_toward_trunnion",
            raised_top_center[0] < level_top_center[0] - 0.70,
            f"expected x retreat > 0.70 m, got level={level_top_center} raised={raised_top_center}",
        )
        ctx.expect_gap(
            tube_bank,
            base_frame,
            axis="z",
            min_gap=0.18,
            positive_elem="bottom_beam",
            negative_elem="deck_plate",
            name="raised_bank_clearance_above_deck",
        )
        ctx.expect_contact(
            tube_bank,
            azimuth_turret,
            elem_a="trunnion_shaft",
            elem_b="left_yoke_arm",
            contact_tol=0.002,
            name="left_trunnion_contact_raised",
        )
        ctx.expect_contact(
            tube_bank,
            azimuth_turret,
            elem_a="trunnion_shaft",
            elem_b="right_yoke_arm",
            contact_tol=0.002,
            name="right_trunnion_contact_raised",
        )

    for joint in (azimuth_rotation, elevation_axis):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

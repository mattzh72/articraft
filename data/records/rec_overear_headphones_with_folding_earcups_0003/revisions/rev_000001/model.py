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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_headband_meshes():
    outer_profile = rounded_rect_profile(0.046, 0.014, radius=0.005, corner_segments=8)
    outer_path = [
        (-0.118, 0.0, 0.018),
        (-0.112, 0.0, 0.072),
        (-0.084, 0.0, 0.132),
        (-0.040, 0.0, 0.172),
        (0.000, 0.0, 0.186),
        (0.040, 0.0, 0.172),
        (0.084, 0.0, 0.132),
        (0.112, 0.0, 0.072),
        (0.118, 0.0, 0.018),
    ]
    pad_profile = rounded_rect_profile(0.030, 0.008, radius=0.003, corner_segments=6)
    pad_path = [
        (-0.082, 0.0, 0.058),
        (-0.060, 0.0, 0.110),
        (-0.028, 0.0, 0.145),
        (0.000, 0.0, 0.156),
        (0.028, 0.0, 0.145),
        (0.060, 0.0, 0.110),
        (0.082, 0.0, 0.058),
    ]
    return (
        _save_mesh(
            "dj_headband_outer.obj",
            sweep_profile_along_spline(
                outer_path,
                profile=outer_profile,
                samples_per_segment=18,
                cap_profile=True,
                up_hint=(0.0, 1.0, 0.0),
            ),
        ),
        _save_mesh(
            "dj_headband_pad.obj",
            sweep_profile_along_spline(
                pad_path,
                profile=pad_profile,
                samples_per_segment=18,
                cap_profile=True,
                up_hint=(0.0, 1.0, 0.0),
            ),
        ),
    )


def _build_earcup_meshes():
    shell_outer = superellipse_profile(0.078, 0.094, exponent=2.7, segments=48)
    shell_hole = [superellipse_profile(0.054, 0.068, exponent=2.5, segments=40)]
    shell_ring = _save_mesh(
        "dj_earcup_shell_ring.obj",
        ExtrudeWithHolesGeometry(
            shell_outer,
            shell_hole,
            height=0.028,
            cap=True,
            center=True,
            closed=True,
        ),
    )
    back_plate = _save_mesh(
        "dj_earcup_back_plate.obj",
        ExtrudeGeometry(
            superellipse_profile(0.060, 0.074, exponent=2.6, segments=40),
            height=0.004,
            cap=True,
            center=True,
            closed=True,
        ),
    )
    pad_outer = superellipse_profile(0.084, 0.100, exponent=2.4, segments=48)
    pad_hole = [superellipse_profile(0.036, 0.050, exponent=2.1, segments=36)]
    pad_ring = _save_mesh(
        "dj_earcup_pad_ring.obj",
        ExtrudeWithHolesGeometry(
            pad_outer,
            pad_hole,
            height=0.018,
            cap=True,
            center=True,
            closed=True,
        ),
    )
    outer_badge = _save_mesh(
        "dj_earcup_badge.obj",
        ExtrudeGeometry(
            superellipse_profile(0.040, 0.050, exponent=2.8, segments=32),
            height=0.0025,
            cap=True,
            center=True,
            closed=True,
        ),
    )
    return shell_ring, back_plate, pad_ring, outer_badge


def _build_yoke_frame_mesh():
    frame_profile = rounded_rect_profile(0.020, 0.012, radius=0.003, corner_segments=6)
    frame_path = [
        (0.0, 0.061, -0.060),
        (0.0, 0.058, -0.042),
        (0.0, 0.055, -0.022),
        (0.0, 0.050, -0.008),
        (0.0, 0.034, -0.002),
        (0.0, 0.000, 0.000),
        (0.0, -0.034, -0.002),
        (0.0, -0.050, -0.008),
        (0.0, -0.055, -0.022),
        (0.0, -0.058, -0.042),
        (0.0, -0.061, -0.060),
    ]
    return _save_mesh(
        "dj_yoke_frame.obj",
        sweep_profile_along_spline(
            frame_path,
            profile=frame_profile,
            samples_per_segment=14,
            cap_profile=True,
            up_hint=(1.0, 0.0, 0.0),
        ),
    )


def _add_yoke(part, *, frame_mesh, frame_material, hardware_material) -> None:
    y_axis_cyl = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=y_axis_cyl,
        material=hardware_material,
        name="top_barrel",
    )
    part.visual(
        frame_mesh,
        origin=Origin(),
        material=frame_material,
        name="yoke_frame",
    )
    part.visual(
        Box((0.020, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.061, -0.060)),
        material=hardware_material,
        name="front_pivot_block",
    )
    part.visual(
        Box((0.020, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, -0.061, -0.060)),
        material=hardware_material,
        name="rear_pivot_block",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.026, 0.126, 0.076)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )


def _add_earcup(
    part,
    *,
    shell_mesh,
    back_plate_mesh,
    pad_mesh,
    badge_mesh,
    shell_material,
    trim_material,
    pad_material,
    orientation_sign: float,
) -> None:
    x_axis_rot = math.pi / 2.0 if orientation_sign > 0.0 else -math.pi / 2.0
    cup_origin = Origin(rpy=(0.0, x_axis_rot, 0.0))
    part.visual(
        shell_mesh,
        origin=cup_origin,
        material=shell_material,
        name="cup_shell",
    )
    part.visual(
        back_plate_mesh,
        origin=Origin(xyz=(-0.012 * orientation_sign, 0.0, 0.0), rpy=(0.0, x_axis_rot, 0.0)),
        material=trim_material,
        name="back_plate",
    )
    part.visual(
        pad_mesh,
        origin=Origin(xyz=(0.011 * orientation_sign, 0.0, 0.0), rpy=(0.0, x_axis_rot, 0.0)),
        material=pad_material,
        name="inner_pad",
    )
    part.visual(
        badge_mesh,
        origin=Origin(xyz=(-0.014 * orientation_sign, 0.0, 0.0), rpy=(0.0, x_axis_rot, 0.0)),
        material=trim_material,
        name="outer_badge",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.049, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_material,
        name="front_trunnion",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, -0.049, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_material,
        name="rear_trunnion",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.032, 0.084, 0.100)),
        mass=0.23,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_headphones", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.16, 0.18, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    cushion_black = model.material("cushion_black", rgba=(0.08, 0.08, 0.09, 1.0))

    headband_outer_mesh, headband_pad_mesh = _build_headband_meshes()
    shell_ring_mesh, back_plate_mesh, pad_ring_mesh, badge_mesh = _build_earcup_meshes()
    yoke_frame_mesh = _build_yoke_frame_mesh()

    headband = model.part("headband")
    headband.visual(
        headband_outer_mesh,
        material=charcoal,
        name="band_shell",
    )
    headband.visual(
        headband_pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=cushion_black,
        name="band_pad",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        x_pos = 0.118 * side_sign
        headband.visual(
            Box((0.024, 0.012, 0.016)),
            origin=Origin(xyz=(x_pos, 0.016, 0.014)),
            material=charcoal,
            name=f"{side_name}_front_hinge_cheek",
        )
        headband.visual(
            Box((0.024, 0.012, 0.016)),
            origin=Origin(xyz=(x_pos, -0.016, 0.014)),
            material=charcoal,
            name=f"{side_name}_rear_hinge_cheek",
        )
        headband.visual(
            Box((0.016, 0.008, 0.016)),
            origin=Origin(xyz=(x_pos, 0.010, 0.012)),
            material=brushed_metal,
            name=f"{side_name}_front_hinge_plate",
        )
        headband.visual(
            Box((0.016, 0.008, 0.016)),
            origin=Origin(xyz=(x_pos, -0.010, 0.012)),
            material=brushed_metal,
            name=f"{side_name}_rear_hinge_plate",
        )
    headband.inertial = Inertial.from_geometry(
        Box((0.260, 0.046, 0.210)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    left_yoke = model.part("left_yoke")
    _add_yoke(
        left_yoke,
        frame_mesh=yoke_frame_mesh,
        frame_material=matte_black,
        hardware_material=brushed_metal,
    )

    right_yoke = model.part("right_yoke")
    _add_yoke(
        right_yoke,
        frame_mesh=yoke_frame_mesh,
        frame_material=matte_black,
        hardware_material=brushed_metal,
    )

    left_cup = model.part("left_cup")
    _add_earcup(
        left_cup,
        shell_mesh=shell_ring_mesh,
        back_plate_mesh=back_plate_mesh,
        pad_mesh=pad_ring_mesh,
        badge_mesh=badge_mesh,
        shell_material=matte_black,
        trim_material=dark_gray,
        pad_material=cushion_black,
        orientation_sign=1.0,
    )

    right_cup = model.part("right_cup")
    _add_earcup(
        right_cup,
        shell_mesh=shell_ring_mesh,
        back_plate_mesh=back_plate_mesh,
        pad_mesh=pad_ring_mesh,
        badge_mesh=badge_mesh,
        shell_material=matte_black,
        trim_material=dark_gray,
        pad_material=cushion_black,
        orientation_sign=-1.0,
    )

    model.articulation(
        "left_fold",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=left_yoke,
        origin=Origin(xyz=(-0.118, 0.0, -0.002)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    model.articulation(
        "right_fold",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=right_yoke,
        origin=Origin(xyz=(0.118, 0.0, -0.002)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    model.articulation(
        "left_swivel",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "right_swivel",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    headband = object_model.get_part("headband")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")

    left_fold = object_model.get_articulation("left_fold")
    right_fold = object_model.get_articulation("right_fold")
    left_swivel = object_model.get_articulation("left_swivel")
    right_swivel = object_model.get_articulation("right_swivel")

    left_front_hinge_plate = headband.get_visual("left_front_hinge_plate")
    left_rear_hinge_plate = headband.get_visual("left_rear_hinge_plate")
    right_front_hinge_plate = headband.get_visual("right_front_hinge_plate")
    right_rear_hinge_plate = headband.get_visual("right_rear_hinge_plate")
    left_top_barrel = left_yoke.get_visual("top_barrel")
    right_top_barrel = right_yoke.get_visual("top_barrel")
    left_yoke_frame = left_yoke.get_visual("yoke_frame")
    right_yoke_frame = right_yoke.get_visual("yoke_frame")
    left_front_arm = left_yoke.get_visual("front_pivot_block")
    left_rear_arm = left_yoke.get_visual("rear_pivot_block")
    right_front_arm = right_yoke.get_visual("front_pivot_block")
    right_rear_arm = right_yoke.get_visual("rear_pivot_block")
    left_front_trunnion = left_cup.get_visual("front_trunnion")
    left_rear_trunnion = left_cup.get_visual("rear_trunnion")
    right_front_trunnion = right_cup.get_visual("front_trunnion")
    right_rear_trunnion = right_cup.get_visual("rear_trunnion")

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
    ctx.allow_overlap(
        left_yoke,
        left_cup,
        elem_a=left_yoke_frame,
        elem_b=left_front_trunnion,
        reason="Left earcup trunnion seats inside the fork's hinge loop at the swivel joint.",
    )
    ctx.allow_overlap(
        left_yoke,
        left_cup,
        elem_a=left_yoke_frame,
        elem_b=left_rear_trunnion,
        reason="Left earcup rear trunnion seats inside the fork's hinge loop at the swivel joint.",
    )
    ctx.allow_overlap(
        right_yoke,
        right_cup,
        elem_a=right_yoke_frame,
        elem_b=right_front_trunnion,
        reason="Right earcup trunnion seats inside the fork's hinge loop at the swivel joint.",
    )
    ctx.allow_overlap(
        right_yoke,
        right_cup,
        elem_a=right_yoke_frame,
        elem_b=right_rear_trunnion,
        reason="Right earcup rear trunnion seats inside the fork's hinge loop at the swivel joint.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        headband,
        left_yoke,
        elem_a=left_front_hinge_plate,
        elem_b=left_top_barrel,
        contact_tol=1e-5,
        name="left_fold_front_plate_contact",
    )
    ctx.expect_contact(
        headband,
        left_yoke,
        elem_a=left_rear_hinge_plate,
        elem_b=left_top_barrel,
        name="left_fold_rear_plate_contact",
    )
    ctx.expect_contact(
        headband,
        right_yoke,
        elem_a=right_front_hinge_plate,
        elem_b=right_top_barrel,
        contact_tol=1e-5,
        name="right_fold_front_plate_contact",
    )
    ctx.expect_contact(
        headband,
        right_yoke,
        elem_a=right_rear_hinge_plate,
        elem_b=right_top_barrel,
        name="right_fold_rear_plate_contact",
    )
    ctx.expect_contact(
        left_yoke,
        left_cup,
        elem_a=left_front_arm,
        elem_b=left_front_trunnion,
        name="left_cup_front_trunnion_contact",
    )
    ctx.expect_contact(
        left_yoke,
        left_cup,
        elem_a=left_rear_arm,
        elem_b=left_rear_trunnion,
        name="left_cup_rear_trunnion_contact",
    )
    ctx.expect_contact(
        right_yoke,
        right_cup,
        elem_a=right_front_arm,
        elem_b=right_front_trunnion,
        name="right_cup_front_trunnion_contact",
    )
    ctx.expect_contact(
        right_yoke,
        right_cup,
        elem_a=right_rear_arm,
        elem_b=right_rear_trunnion,
        name="right_cup_rear_trunnion_contact",
    )
    ctx.expect_origin_distance(
        left_cup,
        right_cup,
        axes="x",
        min_dist=0.18,
        max_dist=0.28,
        name="earcup_span_is_headphone_sized",
    )

    headband_aabb = ctx.part_world_aabb(headband)
    left_cup_aabb = ctx.part_world_aabb(left_cup)
    right_cup_aabb = ctx.part_world_aabb(right_cup)
    if headband_aabb is None or left_cup_aabb is None or right_cup_aabb is None:
        ctx.fail("rest_pose_measurements_available", "Expected measurable AABBs for main headphone parts.")
        return ctx.report()

    headband_top = headband_aabb[1][2]
    cup_top = max(left_cup_aabb[1][2], right_cup_aabb[1][2])
    cup_bottom = min(left_cup_aabb[0][2], right_cup_aabb[0][2])
    ctx.check(
        "headband_arches_above_cups",
        headband_top > cup_top + 0.14,
        f"Headband top {headband_top:.4f} should sit well above cup top {cup_top:.4f}.",
    )
    ctx.check(
        "overall_headphone_height_plausible",
        headband_top - cup_bottom > 0.22,
        f"Overall height {headband_top - cup_bottom:.4f} is too short for an over-ear DJ headset.",
    )

    def _extents(aabb):
        return (
            aabb[1][0] - aabb[0][0],
            aabb[1][1] - aabb[0][1],
            aabb[1][2] - aabb[0][2],
        )

    left_rest_extents = _extents(left_cup_aabb)
    right_rest_extents = _extents(right_cup_aabb)
    ctx.check(
        "left_cup_reads_vertical_at_rest",
        left_rest_extents[2] > left_rest_extents[0] + 0.035,
        f"Left cup extents at rest are dx={left_rest_extents[0]:.4f}, dz={left_rest_extents[2]:.4f}.",
    )
    ctx.check(
        "right_cup_reads_vertical_at_rest",
        right_rest_extents[2] > right_rest_extents[0] + 0.035,
        f"Right cup extents at rest are dx={right_rest_extents[0]:.4f}, dz={right_rest_extents[2]:.4f}.",
    )

    left_rest_pos = ctx.part_world_position(left_cup)
    right_rest_pos = ctx.part_world_position(right_cup)
    if left_rest_pos is None or right_rest_pos is None:
        ctx.fail("cup_positions_available", "Expected measurable world positions for both earcups.")
        return ctx.report()

    with ctx.pose({left_swivel: math.pi / 2.0, right_swivel: math.pi / 2.0}):
        left_swiveled_aabb = ctx.part_world_aabb(left_cup)
        right_swiveled_aabb = ctx.part_world_aabb(right_cup)
        if left_swiveled_aabb is None or right_swiveled_aabb is None:
            ctx.fail("swiveled_cup_measurements_available", "Expected measurable AABBs for swiveled cups.")
        else:
            left_swiveled_extents = _extents(left_swiveled_aabb)
            right_swiveled_extents = _extents(right_swiveled_aabb)
            ctx.check(
                "left_cup_swivel_changes_orientation",
                left_swiveled_extents[0] > left_swiveled_extents[2] + 0.025,
                (
                    f"Left swiveled cup extents are dx={left_swiveled_extents[0]:.4f}, "
                    f"dz={left_swiveled_extents[2]:.4f}."
                ),
            )
            ctx.check(
                "right_cup_swivel_changes_orientation",
                right_swiveled_extents[0] > right_swiveled_extents[2] + 0.025,
                (
                    f"Right swiveled cup extents are dx={right_swiveled_extents[0]:.4f}, "
                    f"dz={right_swiveled_extents[2]:.4f}."
                ),
            )
        ctx.expect_contact(
            left_yoke,
            left_cup,
            elem_a=left_front_arm,
            elem_b=left_front_trunnion,
            name="left_swiveled_cup_stays_seated",
        )
        ctx.expect_contact(
            right_yoke,
            right_cup,
            elem_a=right_front_arm,
            elem_b=right_front_trunnion,
            name="right_swiveled_cup_stays_seated",
        )

    with ctx.pose({left_fold: math.radians(95.0), right_fold: math.radians(95.0)}):
        left_folded_pos = ctx.part_world_position(left_cup)
        right_folded_pos = ctx.part_world_position(right_cup)
        if left_folded_pos is None or right_folded_pos is None:
            ctx.fail("folded_cup_positions_available", "Expected measurable positions for folded cups.")
        else:
            ctx.check(
                "left_side_folds_inward",
                left_folded_pos[0] > left_rest_pos[0] + 0.045 and left_folded_pos[2] > left_rest_pos[2] + 0.035,
                (
                    f"Left folded cup moved from {left_rest_pos} to {left_folded_pos}; "
                    "it should move inward and upward."
                ),
            )
            ctx.check(
                "right_side_folds_inward",
                right_folded_pos[0] < right_rest_pos[0] - 0.045 and right_folded_pos[2] > right_rest_pos[2] + 0.035,
                (
                    f"Right folded cup moved from {right_rest_pos} to {right_folded_pos}; "
                    "it should move inward and upward."
                ),
            )
        ctx.expect_contact(
            headband,
            left_yoke,
            elem_a=left_front_hinge_plate,
            elem_b=left_top_barrel,
            contact_tol=1e-5,
            name="left_fold_joint_remains_mounted_when_folded",
        )
        ctx.expect_contact(
            headband,
            right_yoke,
            elem_a=right_front_hinge_plate,
            elem_b=right_top_barrel,
            contact_tol=1e-5,
            name="right_fold_joint_remains_mounted_when_folded",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

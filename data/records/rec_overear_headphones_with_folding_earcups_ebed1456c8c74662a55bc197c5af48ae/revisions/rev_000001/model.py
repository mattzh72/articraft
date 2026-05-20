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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_band_mesh(
    name: str,
    path: list[tuple[float, float, float]],
    *,
    width: float,
    thickness: float,
):
    profile = rounded_rect_profile(
        width,
        thickness,
        radius=min(width, thickness) * 0.35,
        corner_segments=8,
    )
    return _save_mesh(
        name,
        sweep_profile_along_spline(
            path,
            profile=profile,
            samples_per_segment=18,
            cap_profile=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
    )


def _build_ear_pad_mesh(name: str):
    outer_profile = [
        (0.030, -0.008),
        (0.040, -0.006),
        (0.047, -0.001),
        (0.049, 0.004),
        (0.045, 0.008),
    ]
    inner_profile = [
        (0.020, -0.008),
        (0.023, -0.004),
        (0.025, 0.001),
        (0.024, 0.006),
        (0.022, 0.008),
    ]
    ring = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(pi / 2.0)
    return _save_mesh(name, ring)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="over_ear_headphones")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    soft_black = model.material("soft_black", rgba=(0.15, 0.15, 0.16, 1.0))
    cushion_black = model.material("cushion_black", rgba=(0.06, 0.06, 0.07, 1.0))
    fabric_gray = model.material("fabric_gray", rgba=(0.24, 0.24, 0.26, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.33, 0.35, 0.38, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.62, 0.64, 0.67, 1.0))

    band_frame_mesh = _build_band_mesh(
        "headband_frame",
        [
            (-0.118, 0.0, 0.052),
            (-0.110, 0.0, 0.090),
            (-0.094, 0.0, 0.122),
            (-0.060, 0.0, 0.142),
            (0.000, 0.0, 0.150),
            (0.060, 0.0, 0.142),
            (0.094, 0.0, 0.122),
            (0.110, 0.0, 0.090),
            (0.118, 0.0, 0.052),
        ],
        width=0.032,
        thickness=0.010,
    )
    band_pad_mesh = _build_band_mesh(
        "headband_pad",
        [
            (-0.072, 0.0, 0.084),
            (-0.046, 0.0, 0.116),
            (0.000, 0.0, 0.132),
            (0.046, 0.0, 0.116),
            (0.072, 0.0, 0.084),
        ],
        width=0.042,
        thickness=0.018,
    )
    ear_pad_mesh = _build_ear_pad_mesh("ear_pad_ring")

    headband = model.part("headband")
    headband.visual(band_frame_mesh, material=matte_black, name="frame")
    headband.visual(band_pad_mesh, material=cushion_black, name="pad")
    headband.inertial = Inertial.from_geometry(
        Box((0.28, 0.05, 0.18)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    for side, x_pos in (("left", -0.118), ("right", 0.118)):
        headband.visual(
            Box((0.020, 0.012, 0.030)),
            origin=Origin(xyz=(x_pos, 0.006, 0.039)),
            material=matte_black,
            name=f"{side}_arm_front",
        )
        headband.visual(
            Box((0.020, 0.012, 0.030)),
            origin=Origin(xyz=(x_pos, -0.006, 0.039)),
            material=matte_black,
            name=f"{side}_arm_rear",
        )
        headband.visual(
            Cylinder(radius=0.005, length=0.004),
            origin=Origin(xyz=(x_pos, 0.008, 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"{side}_fold_ear_front",
        )
        headband.visual(
            Cylinder(radius=0.005, length=0.004),
            origin=Origin(xyz=(x_pos, -0.008, 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"{side}_fold_ear_rear",
        )

    left_yoke = model.part("left_yoke")
    right_yoke = model.part("right_yoke")
    for yoke in (left_yoke, right_yoke):
        yoke.visual(
            Cylinder(radius=0.0045, length=0.010),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name="fold_barrel",
        )
        yoke.visual(
            Box((0.010, 0.026, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.007)),
            material=gunmetal,
            name="top_block",
        )
        yoke.visual(
            Box((0.008, 0.004, 0.042)),
            origin=Origin(xyz=(0.0, 0.012, -0.024)),
            material=brushed_metal,
            name="front_arm",
        )
        yoke.visual(
            Box((0.008, 0.004, 0.042)),
            origin=Origin(xyz=(0.0, -0.012, -0.024)),
            material=brushed_metal,
            name="rear_arm",
        )
        yoke.inertial = Inertial.from_geometry(
            Box((0.03, 0.02, 0.05)),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.0, -0.022)),
        )

    left_cup = model.part("left_cup")
    right_cup = model.part("right_cup")
    for cup, inboard_sign in ((left_cup, 1.0), (right_cup, -1.0)):
        cup.visual(
            Cylinder(radius=0.0045, length=0.020),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name="swivel_barrel",
        )
        cup.visual(
            Box((0.010, 0.024, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.011)),
            material=brushed_metal,
            name="pivot_block",
        )
        cup.visual(
            Cylinder(radius=0.042, length=0.022),
            origin=Origin(xyz=(0.0, 0.0, -0.058), rpy=(0.0, pi / 2.0, 0.0)),
            material=soft_black,
            name="housing",
        )
        cup.visual(
            ear_pad_mesh,
            origin=Origin(xyz=(inboard_sign * 0.018, 0.0, -0.058)),
            material=cushion_black,
            name="pad",
        )
        cup.visual(
            Cylinder(radius=0.022, length=0.004),
            origin=Origin(xyz=(inboard_sign * 0.009, 0.0, -0.058), rpy=(0.0, pi / 2.0, 0.0)),
            material=fabric_gray,
            name="inner_grille",
        )
        cup.visual(
            Cylinder(radius=0.034, length=0.008),
            origin=Origin(xyz=(-inboard_sign * 0.015, 0.0, -0.058), rpy=(0.0, pi / 2.0, 0.0)),
            material=gunmetal,
            name="outer_cap",
        )
        cup.visual(
            Cylinder(radius=0.039, length=0.002),
            origin=Origin(xyz=(-inboard_sign * 0.010, 0.0, -0.058), rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed_metal,
            name="accent_ring",
        )
        cup.inertial = Inertial.from_geometry(
            Box((0.05, 0.10, 0.11)),
            mass=0.09,
            origin=Origin(xyz=(0.0, 0.0, -0.055)),
        )

    fold_limits = MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=0.75)
    swivel_limits = MotionLimits(effort=2.0, velocity=2.5, lower=-0.40, upper=0.40)

    model.articulation(
        "left_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=left_yoke,
        origin=Origin(xyz=(-0.118, 0.0, 0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=fold_limits,
    )
    model.articulation(
        "right_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=right_yoke,
        origin=Origin(xyz=(0.118, 0.0, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=fold_limits,
    )
    model.articulation(
        "left_cup_swivel",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=swivel_limits,
    )
    model.articulation(
        "right_cup_swivel",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=swivel_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    headband = object_model.get_part("headband")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")

    left_fold = object_model.get_articulation("left_fold_hinge")
    right_fold = object_model.get_articulation("right_fold_hinge")
    left_swivel = object_model.get_articulation("left_cup_swivel")
    right_swivel = object_model.get_articulation("right_cup_swivel")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=20)

    ctx.expect_contact(headband, left_yoke, name="left_fold_contact_rest")
    ctx.expect_contact(headband, right_yoke, name="right_fold_contact_rest")
    ctx.expect_contact(left_yoke, left_cup, name="left_swivel_contact_rest")
    ctx.expect_contact(right_yoke, right_cup, name="right_swivel_contact_rest")

    ctx.expect_origin_gap(
        right_yoke,
        left_yoke,
        axis="x",
        min_gap=0.20,
        max_gap=0.28,
        name="yoke_spacing_rest",
    )
    ctx.expect_origin_gap(
        right_cup,
        left_cup,
        axis="x",
        min_gap=0.20,
        max_gap=0.28,
        name="cup_spacing_rest",
    )

    def _union_aabb(parts):
        mins = [float("inf"), float("inf"), float("inf")]
        maxs = [float("-inf"), float("-inf"), float("-inf")]
        for part in parts:
            aabb = ctx.part_world_aabb(part)
            if aabb is None:
                return None
            low, high = aabb
            for index in range(3):
                mins[index] = min(mins[index], low[index])
                maxs[index] = max(maxs[index], high[index])
        return mins, maxs

    overall_aabb = _union_aabb((headband, left_yoke, right_yoke, left_cup, right_cup))
    if overall_aabb is None:
        ctx.fail("overall_aabb_present", "could not resolve world-space assembly bounds")
    else:
        mins, maxs = overall_aabb
        overall_width = maxs[0] - mins[0]
        overall_height = maxs[2] - mins[2]
        ctx.check(
            "overall_width_realistic",
            0.22 <= overall_width <= 0.30,
            f"overall width {overall_width:.4f} m outside realistic headphone range",
        )
        ctx.check(
            "overall_height_realistic",
            0.20 <= overall_height <= 0.32,
            f"overall height {overall_height:.4f} m outside realistic headphone range",
        )

    for side_name, cup in (("left", left_cup), ("right", right_cup)):
        aabb = ctx.part_world_aabb(cup)
        if aabb is None:
            ctx.fail(f"{side_name}_cup_aabb_present", f"{side_name} cup missing world AABB")
            continue
        low, high = aabb
        cup_height = high[2] - low[2]
        cup_depth = high[1] - low[1]
        ctx.check(
            f"{side_name}_cup_size_realistic",
            0.09 <= cup_height <= 0.115 and 0.09 <= cup_depth <= 0.11,
            f"{side_name} cup size {cup_depth:.4f} x {cup_height:.4f} m not over-ear sized",
        )

    for articulation in (left_fold, right_fold, left_swivel, right_swivel):
        limits = articulation.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        parent = object_model.get_part(articulation.parent)
        child = object_model.get_part(articulation.child)
        for label, value in (("lower", limits.lower), ("upper", limits.upper)):
            with ctx.pose({articulation: value}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_{label}_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{articulation.name}_{label}_no_floating")
                ctx.expect_contact(parent, child, name=f"{articulation.name}_{label}_contact")

    with ctx.pose(
        {
            left_fold: left_fold.motion_limits.upper,
            right_fold: right_fold.motion_limits.upper,
            left_swivel: 0.0,
            right_swivel: 0.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="storage_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="storage_pose_no_floating")
        ctx.expect_contact(headband, left_yoke, name="storage_left_fold_contact")
        ctx.expect_contact(headband, right_yoke, name="storage_right_fold_contact")
        ctx.expect_gap(
            right_cup,
            left_cup,
            axis="x",
            min_gap=0.0,
            max_gap=0.01,
            name="storage_cup_spacing",
        )

    with ctx.pose(
        {
            left_fold: 0.0,
            right_fold: 0.0,
            left_swivel: left_swivel.motion_limits.lower,
            right_swivel: right_swivel.motion_limits.upper,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="fit_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="fit_pose_no_floating")
        ctx.expect_contact(left_yoke, left_cup, name="fit_left_swivel_contact")
        ctx.expect_contact(right_yoke, right_cup, name="fit_right_swivel_contact")

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

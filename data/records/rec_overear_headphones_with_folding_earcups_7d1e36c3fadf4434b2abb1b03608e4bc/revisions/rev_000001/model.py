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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    sweep_profile_along_spline,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _oval_section(
    width: float,
    height: float,
    *,
    y: float,
    z_center: float,
    exponent: float = 2.6,
    segments: int = 44,
):
    return [(x, y, z + z_center) for x, z in superellipse_profile(width, height, exponent=exponent, segments=segments)]


def _build_headband_meshes():
    outer_path = [
        (0.0, -0.094, 0.131),
        (0.0, -0.080, 0.156),
        (0.0, -0.055, 0.188),
        (0.0, 0.000, 0.206),
        (0.0, 0.055, 0.188),
        (0.0, 0.080, 0.156),
        (0.0, 0.094, 0.131),
    ]
    outer = sweep_profile_along_spline(
        outer_path,
        profile=rounded_rect_profile(0.024, 0.010, 0.004, corner_segments=6),
        samples_per_segment=18,
        cap_profile=True,
    )
    cushion = sweep_profile_along_spline(
        outer_path,
        profile=rounded_rect_profile(0.016, 0.007, 0.003, corner_segments=6),
        samples_per_segment=18,
        cap_profile=True,
    )
    return (
        _mesh("headband_outer", outer),
        _mesh("headband_cushion", cushion),
    )


def _build_yoke_meshes():
    arm_profile = rounded_rect_profile(0.0040, 0.0080, 0.0014, corner_segments=4)
    front_arm = sweep_profile_along_spline(
        [
            (0.046, 0.012, -0.054),
            (0.041, 0.011, -0.046),
            (0.030, 0.007, -0.032),
            (0.016, 0.002, -0.015),
            (0.006, 0.000, -0.002),
        ],
        profile=arm_profile,
        samples_per_segment=12,
        cap_profile=True,
    )
    rear_arm = sweep_profile_along_spline(
        [
            (-0.046, 0.012, -0.054),
            (-0.041, 0.011, -0.046),
            (-0.030, 0.007, -0.032),
            (-0.016, 0.002, -0.015),
            (-0.006, 0.000, -0.002),
        ],
        profile=arm_profile,
        samples_per_segment=12,
        cap_profile=True,
    )
    return (
        _mesh("yoke_front_arm", front_arm),
        _mesh("yoke_rear_arm", rear_arm),
    )


def _build_cup_meshes():
    cup_outer = superellipse_profile(0.078, 0.094, exponent=2.7, segments=52)
    cup_inner = superellipse_profile(0.048, 0.070, exponent=2.6, segments=40)
    side_ring = (
        ExtrudeWithHolesGeometry(cup_outer, [cup_inner], 0.024, center=True)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.002, -0.015)
    )
    back_plate = (
        ExtrudeGeometry(superellipse_profile(0.074, 0.090, exponent=2.7, segments=52), 0.004, center=True)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -0.012, -0.015)
    )
    cushion = (
        ExtrudeWithHolesGeometry(
            superellipse_profile(0.082, 0.098, exponent=2.5, segments=52),
            [superellipse_profile(0.050, 0.072, exponent=2.4, segments=40)],
            0.018,
            center=True,
        )
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.018, -0.015)
    )
    outer_cap = section_loft(
        [
            _oval_section(0.076, 0.092, y=-0.010, z_center=-0.015, exponent=2.7, segments=52),
            _oval_section(0.070, 0.086, y=-0.020, z_center=-0.015, exponent=2.7, segments=52),
            _oval_section(0.058, 0.072, y=-0.032, z_center=-0.015, exponent=2.6, segments=52),
            _oval_section(0.038, 0.050, y=-0.042, z_center=-0.015, exponent=2.4, segments=52),
        ]
    )
    cup_shell = side_ring.copy().merge(back_plate).merge(outer_cap)
    return (
        _mesh("cup_shell", cup_shell),
        _mesh("cup_cushion", cushion),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="noise_canceling_headphones")

    graphite = model.material("graphite", rgba=(0.14, 0.15, 0.17, 1.0))
    satin_black = model.material("satin_black", rgba=(0.09, 0.09, 0.10, 1.0))
    cushion_black = model.material("cushion_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.30, 0.32, 0.35, 1.0))
    accent_gray = model.material("accent_gray", rgba=(0.46, 0.48, 0.50, 1.0))

    headband_outer_mesh, headband_cushion_mesh = _build_headband_meshes()
    cup_shell_mesh, cup_cushion_mesh = _build_cup_meshes()

    headband = model.part("headband")
    headband.visual(headband_outer_mesh, material=graphite, name="outer_band")
    headband.visual(
        headband_cushion_mesh,
        material=cushion_black,
        name="inner_pad",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        headband.visual(
            Box((0.020, 0.006, 0.012)),
            origin=Origin(xyz=(0.0, side_sign * 0.091, 0.131)),
            material=graphite,
            name=f"{side_name}_hinge_block",
        )
        headband.visual(
            Cylinder(radius=0.005, length=0.002),
            origin=Origin(xyz=(0.007, side_sign * 0.099, 0.131), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"{side_name}_hinge_washer_front",
        )
        headband.visual(
            Cylinder(radius=0.005, length=0.002),
            origin=Origin(xyz=(-0.007, side_sign * 0.099, 0.131), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"{side_name}_hinge_washer_rear",
        )

    for side_name, side_out, yaw in (("left", -1.0, 0.0), ("right", 1.0, math.pi)):
        yoke = model.part(f"{side_name}_yoke")
        cup = model.part(f"{side_name}_cup")

        yoke.visual(
            Cylinder(radius=0.005, length=0.012),
            origin=Origin(
                xyz=(0.0, side_out * 0.004, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_metal,
            name="fold_barrel",
        )
        yoke.visual(
            Box((0.004, 0.006, 0.052)),
            origin=Origin(xyz=(0.037, side_out * 0.004, -0.032)),
            material=dark_metal,
            name="front_arm",
        )
        yoke.visual(
            Box((0.004, 0.006, 0.052)),
            origin=Origin(xyz=(-0.037, side_out * 0.004, -0.032)),
            material=dark_metal,
            name="rear_arm",
        )
        yoke.visual(
            Box((0.078, 0.006, 0.004)),
            origin=Origin(xyz=(0.0, side_out * 0.004, -0.005)),
            material=accent_gray,
            name="top_bridge",
        )

        cup.visual(
            cup_shell_mesh,
            origin=Origin(xyz=(0.0, side_out * 0.036, 0.015), rpy=(0.0, 0.0, yaw)),
            material=graphite,
            name="cup_shell",
        )
        cup.visual(
            cup_cushion_mesh,
            origin=Origin(xyz=(0.0, side_out * 0.036, 0.015), rpy=(0.0, 0.0, yaw)),
            material=cushion_black,
            name="ear_cushion",
        )
        cup.visual(
            Cylinder(radius=0.005, length=0.070),
            origin=Origin(xyz=(0.0, side_out * 0.004, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=accent_gray,
            name="pivot_axle",
        )
        cup.visual(
            Box((0.010, 0.022, 0.016)),
            origin=Origin(xyz=(0.027, side_out * 0.020, 0.0)),
            material=accent_gray,
            name="front_boss",
        )
        cup.visual(
            Box((0.010, 0.022, 0.016)),
            origin=Origin(xyz=(-0.027, side_out * 0.020, 0.0)),
            material=accent_gray,
            name="rear_boss",
        )

    model.articulation(
        "headband_to_left_yoke",
        ArticulationType.REVOLUTE,
        parent="headband",
        child="left_yoke",
        origin=Origin(xyz=(0.0, -0.099, 0.131)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(65.0),
        ),
    )
    model.articulation(
        "headband_to_right_yoke",
        ArticulationType.REVOLUTE,
        parent="headband",
        child="right_yoke",
        origin=Origin(xyz=(0.0, 0.099, 0.131)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(65.0),
        ),
    )
    model.articulation(
        "left_yoke_to_cup",
        ArticulationType.REVOLUTE,
        parent="left_yoke",
        child="left_cup",
        origin=Origin(xyz=(0.0, -0.004, -0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "right_yoke_to_cup",
        ArticulationType.REVOLUTE,
        parent="right_yoke",
        child="right_cup",
        origin=Origin(xyz=(0.0, 0.004, -0.058)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    headband = object_model.get_part("headband")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")

    left_fold = object_model.get_articulation("headband_to_left_yoke")
    right_fold = object_model.get_articulation("headband_to_right_yoke")
    left_swivel = object_model.get_articulation("left_yoke_to_cup")
    right_swivel = object_model.get_articulation("right_yoke_to_cup")

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

    ctx.expect_contact(left_yoke, headband, contact_tol=0.0006, name="left_fold_hinge_attached")
    ctx.expect_contact(right_yoke, headband, contact_tol=0.0006, name="right_fold_hinge_attached")
    ctx.expect_contact(left_cup, left_yoke, contact_tol=0.0006, name="left_cup_clipped_in_yoke")
    ctx.expect_contact(right_cup, right_yoke, contact_tol=0.0006, name="right_cup_clipped_in_yoke")
    ctx.expect_origin_gap(right_cup, left_cup, axis="y", min_gap=0.14, name="cups_span_head_width")

    ctx.check(
        "left_fold_axis_is_fore_aft",
        tuple(round(v, 6) for v in left_fold.axis) == (1.0, 0.0, 0.0),
        details=f"axis={left_fold.axis}",
    )
    ctx.check(
        "right_fold_axis_is_fore_aft",
        tuple(round(v, 6) for v in right_fold.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={right_fold.axis}",
    )
    ctx.check(
        "left_swivel_axis_is_fore_aft",
        tuple(round(v, 6) for v in left_swivel.axis) == (1.0, 0.0, 0.0),
        details=f"axis={left_swivel.axis}",
    )
    ctx.check(
        "right_swivel_axis_is_fore_aft",
        tuple(round(v, 6) for v in right_swivel.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={right_swivel.axis}",
    )

    left_rest = ctx.part_world_position(left_cup)
    right_rest = ctx.part_world_position(right_cup)
    if left_rest is None or right_rest is None:
        ctx.fail("rest_cup_positions_available", "Could not resolve rest pose cup origins.")
    else:
        ctx.check(
            "cups_start_below_headband",
            left_rest[2] < 0.11 and right_rest[2] < 0.11,
            details=f"left_z={left_rest[2]:.4f}, right_z={right_rest[2]:.4f}",
        )
        ctx.check(
            "cups_start_outboard_of_headband_center",
            left_rest[1] < -0.06 and right_rest[1] > 0.06,
            details=f"left_y={left_rest[1]:.4f}, right_y={right_rest[1]:.4f}",
        )

    with ctx.pose({left_swivel: math.radians(88.0), right_swivel: math.radians(88.0)}):
        ctx.expect_contact(left_cup, left_yoke, contact_tol=0.0006, name="left_cup_stays_attached_when_flat")
        ctx.expect_contact(right_cup, right_yoke, contact_tol=0.0006, name="right_cup_stays_attached_when_flat")

    with ctx.pose(
        {
            left_fold: math.radians(46.0),
            right_fold: math.radians(46.0),
            left_swivel: math.radians(63.0),
            right_swivel: math.radians(63.0),
        }
    ):
        ctx.expect_contact(left_yoke, headband, contact_tol=0.0006, name="left_yoke_stays_attached_folded")
        ctx.expect_contact(right_yoke, headband, contact_tol=0.0006, name="right_yoke_stays_attached_folded")
        ctx.expect_contact(left_cup, left_yoke, contact_tol=0.0006, name="left_cup_stays_attached_folded")
        ctx.expect_contact(right_cup, right_yoke, contact_tol=0.0006, name="right_cup_stays_attached_folded")
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_folded_flat_pose")

        left_folded = ctx.part_world_position(left_cup)
        right_folded = ctx.part_world_position(right_cup)
        if left_rest is None or right_rest is None or left_folded is None or right_folded is None:
            ctx.fail("folded_cup_positions_available", "Could not resolve folded pose cup origins.")
        else:
            ctx.check(
                "left_cup_folds_inward",
                left_folded[1] > left_rest[1] + 0.030 and left_folded[2] > left_rest[2] + 0.010,
                details=(
                    f"rest={left_rest}, folded={left_folded}"
                ),
            )
            ctx.check(
                "right_cup_folds_inward",
                right_folded[1] < right_rest[1] - 0.030 and right_folded[2] > right_rest[2] + 0.010,
                details=(
                    f"rest={right_rest}, folded={right_folded}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

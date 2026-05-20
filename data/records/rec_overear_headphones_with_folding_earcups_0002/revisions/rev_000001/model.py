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
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_over_ear_headphones", assets=ASSETS)

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    plastic_black = model.material("plastic_black", rgba=(0.10, 0.11, 0.12, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.16, 0.17, 0.18, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    cushion_foam = model.material("cushion_foam", rgba=(0.18, 0.18, 0.19, 1.0))
    soft_pad = model.material("soft_pad", rgba=(0.24, 0.24, 0.26, 1.0))

    outer_band_mesh = save_mesh(
        "headband_outer.obj",
        sweep_profile_along_spline(
            [
                (-0.105, 0.0, 0.066),
                (-0.098, 0.0, 0.108),
                (-0.072, 0.0, 0.152),
                (-0.028, 0.0, 0.182),
                (0.0, 0.0, 0.188),
                (0.028, 0.0, 0.182),
                (0.072, 0.0, 0.152),
                (0.098, 0.0, 0.108),
                (0.105, 0.0, 0.066),
            ],
            profile=rounded_rect_profile(0.026, 0.007, radius=0.0025, corner_segments=6),
            samples_per_segment=16,
            cap_profile=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
    )
    inner_pad_mesh = save_mesh(
        "headband_pad.obj",
        sweep_profile_along_spline(
            [
                (-0.082, 0.0, 0.074),
                (-0.062, 0.0, 0.110),
                (-0.030, 0.0, 0.143),
                (0.0, 0.0, 0.154),
                (0.030, 0.0, 0.143),
                (0.062, 0.0, 0.110),
                (0.082, 0.0, 0.074),
            ],
            profile=rounded_rect_profile(0.018, 0.013, radius=0.0045, corner_segments=6),
            samples_per_segment=14,
            cap_profile=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
    )
    cup_body_geometry = BoxGeometry((0.014, 0.022, 0.016)).translate(0.0, 0.0, -0.008)
    cup_body_geometry.merge(BoxGeometry((0.024, 0.024, 0.020)).translate(0.0, 0.0, -0.022))
    cup_body_geometry.merge(
        CylinderGeometry(radius=0.038, height=0.022, radial_segments=48)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, -0.040)
    )
    cup_body_geometry.merge(
        CylinderGeometry(radius=0.025, height=0.008, radial_segments=36)
        .rotate_y(math.pi / 2.0)
        .translate(-0.006, 0.0, -0.040)
    )
    cup_body_geometry.merge(BoxGeometry((0.010, 0.014, 0.014)).translate(0.0, -0.018, -0.042))
    cup_body_mesh = save_mesh("earcup_body.obj", cup_body_geometry)
    ear_pad_mesh = save_mesh(
        "ear_pad_ring.obj",
        TorusGeometry(radius=0.028, tube=0.0065, radial_segments=18, tubular_segments=56)
        .rotate_y(math.pi / 2.0)
        .scale(0.60, 1.0, 1.0),
    )

    headband = model.part("headband")
    headband.visual(outer_band_mesh, material=plastic_black, name="outer_band")
    headband.visual(inner_pad_mesh, material=cushion_foam, name="inner_pad")
    headband.visual(
        Box((0.020, 0.012, 0.056)),
        origin=Origin(xyz=(-0.068, 0.0, 0.117)),
        material=cushion_foam,
        name="left_pad_anchor",
    )
    headband.visual(
        Box((0.020, 0.012, 0.056)),
        origin=Origin(xyz=(0.068, 0.0, 0.117)),
        material=cushion_foam,
        name="right_pad_anchor",
    )
    headband.visual(
        Box((0.014, 0.016, 0.014)),
        origin=Origin(xyz=(-0.106, 0.0, 0.068)),
        material=plastic_black,
        name="left_hinge_block",
    )
    headband.visual(
        Box((0.012, 0.006, 0.020)),
        origin=Origin(xyz=(-0.110, 0.008, 0.058)),
        material=dark_gray,
        name="left_clevis_front",
    )
    headband.visual(
        Box((0.012, 0.006, 0.020)),
        origin=Origin(xyz=(-0.110, -0.008, 0.058)),
        material=dark_gray,
        name="left_clevis_rear",
    )
    headband.visual(
        Box((0.014, 0.016, 0.014)),
        origin=Origin(xyz=(0.106, 0.0, 0.068)),
        material=plastic_black,
        name="right_hinge_block",
    )
    headband.visual(
        Box((0.012, 0.006, 0.020)),
        origin=Origin(xyz=(0.110, 0.008, 0.058)),
        material=dark_gray,
        name="right_clevis_front",
    )
    headband.visual(
        Box((0.012, 0.006, 0.020)),
        origin=Origin(xyz=(0.110, -0.008, 0.058)),
        material=dark_gray,
        name="right_clevis_rear",
    )
    headband.inertial = Inertial.from_geometry(
        Box((0.24, 0.05, 0.15)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )

    left_yoke = model.part("left_yoke")
    left_yoke.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hinge_barrel",
    )
    left_yoke.visual(
        Box((0.010, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=satin_metal,
        name="center_block",
    )
    left_yoke.visual(
        Box((0.006, 0.006, 0.072)),
        origin=Origin(xyz=(0.0, 0.009, -0.031)),
        material=satin_metal,
        name="front_arm",
    )
    left_yoke.visual(
        Box((0.006, 0.006, 0.072)),
        origin=Origin(xyz=(0.0, -0.009, -0.031)),
        material=satin_metal,
        name="rear_arm",
    )
    left_yoke.visual(
        Box((0.008, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=satin_metal,
        name="lower_bridge",
    )
    left_yoke.inertial = Inertial.from_geometry(
        Box((0.02, 0.03, 0.07)),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    right_yoke = model.part("right_yoke")
    right_yoke.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hinge_barrel",
    )
    right_yoke.visual(
        Box((0.010, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=satin_metal,
        name="center_block",
    )
    right_yoke.visual(
        Box((0.006, 0.006, 0.072)),
        origin=Origin(xyz=(0.0, 0.009, -0.031)),
        material=satin_metal,
        name="front_arm",
    )
    right_yoke.visual(
        Box((0.006, 0.006, 0.072)),
        origin=Origin(xyz=(0.0, -0.009, -0.031)),
        material=satin_metal,
        name="rear_arm",
    )
    right_yoke.visual(
        Box((0.008, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=satin_metal,
        name="lower_bridge",
    )
    right_yoke.inertial = Inertial.from_geometry(
        Box((0.02, 0.03, 0.07)),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    left_cup = model.part("left_cup")
    left_cup.visual(
        cup_body_mesh,
        origin=Origin(),
        material=plastic_black,
        name="cup_body",
    )
    left_cup.visual(
        ear_pad_mesh,
        origin=Origin(xyz=(0.012, 0.0, -0.040)),
        material=soft_pad,
        name="ear_pad",
    )
    left_cup.visual(
        Cylinder(radius=0.003, length=0.012),
        origin=Origin(xyz=(0.0, -0.020, -0.042), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="cable_gland",
    )
    left_cup.inertial = Inertial.from_geometry(
        Cylinder(radius=0.038, length=0.028),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, -0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_cup = model.part("right_cup")
    right_cup.visual(
        cup_body_mesh,
        origin=Origin(),
        material=plastic_black,
        name="cup_body",
    )
    right_cup.visual(
        ear_pad_mesh,
        origin=Origin(xyz=(-0.012, 0.0, -0.040)),
        material=soft_pad,
        name="ear_pad",
    )
    right_cup.visual(
        Cylinder(radius=0.003, length=0.012),
        origin=Origin(xyz=(0.0, -0.020, -0.042), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="cable_gland",
    )
    right_cup.inertial = Inertial.from_geometry(
        Cylinder(radius=0.038, length=0.028),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, -0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "left_fold",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=left_yoke,
        origin=Origin(xyz=(-0.110, 0.0, 0.058)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "right_fold",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=right_yoke,
        origin=Origin(xyz=(0.110, 0.0, 0.058)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "left_swivel",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-0.30,
            upper=0.30,
        ),
    )
    model.articulation(
        "right_swivel",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-0.30,
            upper=0.30,
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

    left_front_clevis = headband.get_visual("left_clevis_front")
    right_front_clevis = headband.get_visual("right_clevis_front")
    left_front_arm = left_yoke.get_visual("front_arm")
    left_rear_arm = left_yoke.get_visual("rear_arm")
    right_front_arm = right_yoke.get_visual("front_arm")
    right_rear_arm = right_yoke.get_visual("rear_arm")
    left_hinge_barrel = left_yoke.get_visual("hinge_barrel")
    right_hinge_barrel = right_yoke.get_visual("hinge_barrel")
    left_cup_body = left_cup.get_visual("cup_body")
    right_cup_body = right_cup.get_visual("cup_body")
    left_cable = left_cup.get_visual("cable_gland")
    right_cable = right_cup.get_visual("cable_gland")

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
        elem_a=left_front_arm,
        elem_b=left_cup_body,
        reason="Left cup pivot boss is intentionally nested into the yoke cheek at the swivel hinge.",
    )
    ctx.allow_overlap(
        right_yoke,
        right_cup,
        elem_a=right_front_arm,
        elem_b=right_cup_body,
        reason="Right cup pivot boss is intentionally nested into the yoke cheek at the swivel hinge.",
    )
    ctx.allow_overlap(
        left_yoke,
        left_cup,
        elem_a=left_rear_arm,
        elem_b=left_cup_body,
        reason="Left cup pivot boss is intentionally nested into the rear yoke cheek at the swivel hinge.",
    )
    ctx.allow_overlap(
        right_yoke,
        right_cup,
        elem_a=right_rear_arm,
        elem_b=right_cup_body,
        reason="Right cup pivot boss is intentionally nested into the rear yoke cheek at the swivel hinge.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(headband, left_yoke, elem_a=left_front_clevis, elem_b=left_hinge_barrel)
    ctx.expect_contact(headband, right_yoke, elem_a=right_front_clevis, elem_b=right_hinge_barrel)
    ctx.expect_contact(left_yoke, left_cup, elem_a=left_front_arm, elem_b=left_cup_body)
    ctx.expect_contact(right_yoke, right_cup, elem_a=right_front_arm, elem_b=right_cup_body)
    ctx.expect_gap(right_cup, left_cup, axis="x", min_gap=0.14)
    ctx.expect_origin_distance(left_cup, right_cup, axes="yz", max_dist=0.002)
    ctx.expect_gap(headband, left_cup, axis="z", min_gap=0.04)
    ctx.expect_gap(headband, right_cup, axis="z", min_gap=0.04)

    left_rest = ctx.part_world_position(left_cup)
    right_rest = ctx.part_world_position(right_cup)
    ctx.check("left_cup_present", left_rest is not None, "Left cup position unavailable.")
    ctx.check("right_cup_present", right_rest is not None, "Right cup position unavailable.")

    if left_rest is not None:
        with ctx.pose({left_fold: math.radians(80.0)}):
            left_folded = ctx.part_world_position(left_cup)
            ctx.expect_contact(left_yoke, left_cup, elem_a=left_front_arm, elem_b=left_cup_body)
            if left_folded is not None:
                ctx.check(
                    "left_fold_moves_cup_inward",
                    left_folded[0] > left_rest[0] + 0.045,
                    f"left cup x did not move inward enough: rest={left_rest}, folded={left_folded}",
                )
                ctx.check(
                    "left_fold_lifts_cup",
                    left_folded[2] > left_rest[2] + 0.045,
                    f"left cup z did not rise enough: rest={left_rest}, folded={left_folded}",
                )
            else:
                ctx.fail("left_fold_pose_position", "Left cup position unavailable in folded pose.")

    if right_rest is not None:
        with ctx.pose({right_fold: math.radians(80.0)}):
            right_folded = ctx.part_world_position(right_cup)
            ctx.expect_contact(right_yoke, right_cup, elem_a=right_front_arm, elem_b=right_cup_body)
            if right_folded is not None:
                ctx.check(
                    "right_fold_moves_cup_inward",
                    right_folded[0] < right_rest[0] - 0.045,
                    f"right cup x did not move inward enough: rest={right_rest}, folded={right_folded}",
                )
                ctx.check(
                    "right_fold_lifts_cup",
                    right_folded[2] > right_rest[2] + 0.045,
                    f"right cup z did not rise enough: rest={right_rest}, folded={right_folded}",
                )
            else:
                ctx.fail("right_fold_pose_position", "Right cup position unavailable in folded pose.")

    left_cable_rest = ctx.part_element_world_aabb(left_cup, elem=left_cable)
    if left_cable_rest is not None:
        left_cable_rest_center = 0.5 * (left_cable_rest[0][0] + left_cable_rest[1][0])
        with ctx.pose({left_swivel: 0.25}):
            left_cable_swiveled = ctx.part_element_world_aabb(left_cup, elem=left_cable)
            ctx.expect_contact(left_yoke, left_cup, elem_a=left_front_arm, elem_b=left_cup_body)
            if left_cable_swiveled is not None:
                left_cable_swivel_center = 0.5 * (
                    left_cable_swiveled[0][0] + left_cable_swiveled[1][0]
                )
                ctx.check(
                    "left_swivel_rotates_cup_about_local_axis",
                    left_cable_swivel_center > left_cable_rest_center + 0.002,
                    (
                        "Left cable gland did not shift laterally under swivel: "
                        f"rest_x={left_cable_rest_center:.4f}, swivel_x={left_cable_swivel_center:.4f}"
                    ),
                )
            else:
                ctx.fail("left_swivel_visual_aabb", "Left cable gland AABB unavailable in swivel pose.")
    else:
        ctx.fail("left_cable_gland_aabb", "Left cable gland AABB unavailable at rest.")

    right_cable_rest = ctx.part_element_world_aabb(right_cup, elem=right_cable)
    if right_cable_rest is not None:
        right_cable_rest_center = 0.5 * (right_cable_rest[0][0] + right_cable_rest[1][0])
        with ctx.pose({right_swivel: -0.25}):
            right_cable_swiveled = ctx.part_element_world_aabb(right_cup, elem=right_cable)
            ctx.expect_contact(right_yoke, right_cup, elem_a=right_front_arm, elem_b=right_cup_body)
            if right_cable_swiveled is not None:
                right_cable_swivel_center = 0.5 * (
                    right_cable_swiveled[0][0] + right_cable_swiveled[1][0]
                )
                ctx.check(
                    "right_swivel_rotates_cup_about_local_axis",
                    right_cable_swivel_center < right_cable_rest_center - 0.002,
                    (
                        "Right cable gland did not shift laterally under swivel: "
                        f"rest_x={right_cable_rest_center:.4f}, swivel_x={right_cable_swivel_center:.4f}"
                    ),
                )
            else:
                ctx.fail("right_swivel_visual_aabb", "Right cable gland AABB unavailable in swivel pose.")
    else:
        ctx.fail("right_cable_gland_aabb", "Right cable gland AABB unavailable at rest.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

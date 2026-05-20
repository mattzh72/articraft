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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medieval_bombard")

    oak = model.material("oak", rgba=(0.48, 0.33, 0.18, 1.0))
    dark_oak = model.material("dark_oak", rgba=(0.36, 0.24, 0.13, 1.0))
    wrought_iron = model.material("wrought_iron", rgba=(0.20, 0.20, 0.22, 1.0))
    iron_band = model.material("iron_band", rgba=(0.29, 0.29, 0.31, 1.0))

    def barrel_shell_mesh():
        outer_profile = [
            (0.18, -0.48),
            (0.30, -0.46),
            (0.33, -0.36),
            (0.32, -0.28),
            (0.30, -0.18),
            (0.28, -0.06),
            (0.27, 0.08),
            (0.26, 0.22),
            (0.24, 0.42),
            (0.23, 0.56),
        ]
        inner_profile = [
            (0.00, -0.42),
            (0.13, -0.35),
            (0.15, -0.10),
            (0.16, 0.16),
            (0.17, 0.40),
            (0.17, 0.54),
        ]
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile,
                inner_profile,
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
            "barrel_shell",
        )

    runner_length = 1.80
    runner_width = 0.18
    runner_height = 0.14
    runner_y = 0.40

    tie_length = 0.24
    tie_width = 0.98
    tie_x = 0.58

    bed_length = 1.38
    bed_width = 0.58
    bed_thickness = 0.04
    bed_z = runner_height + (0.5 * bed_thickness)

    cheek_length = 1.36
    cheek_height = 0.60
    cheek_thickness = 0.10
    cheek_y = 0.40
    cheek_base_z = runner_height
    lower_cheek_height = 0.445
    upper_cheek_height = cheek_height - lower_cheek_height
    cheek_post_length = 0.46
    cheek_front_x = 0.41
    cheek_rear_x = -0.41

    trunnion_radius = 0.055
    trunnion_length = 0.10
    trunnion_center_y = 0.40
    trunnion_axis_z = 0.64

    cradle = model.part("cradle")
    cradle.visual(
        Box((runner_length, runner_width, runner_height)),
        origin=Origin(xyz=(0.0, runner_y, 0.5 * runner_height)),
        material=dark_oak,
        name="right_runner",
    )
    cradle.visual(
        Box((runner_length, runner_width, runner_height)),
        origin=Origin(xyz=(0.0, -runner_y, 0.5 * runner_height)),
        material=dark_oak,
        name="left_runner",
    )
    cradle.visual(
        Box((tie_length, tie_width, runner_height)),
        origin=Origin(xyz=(tie_x, 0.0, 0.5 * runner_height)),
        material=oak,
        name="front_tie",
    )
    cradle.visual(
        Box((tie_length, tie_width, runner_height)),
        origin=Origin(xyz=(-tie_x, 0.0, 0.5 * runner_height)),
        material=oak,
        name="rear_tie",
    )
    cradle.visual(
        Box((bed_length, bed_width, bed_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bed_z)),
        material=oak,
        name="bed_floor",
    )
    cradle.visual(
        Box((cheek_length, cheek_thickness, lower_cheek_height)),
        origin=Origin(
            xyz=(0.0, -cheek_y, cheek_base_z + (0.5 * lower_cheek_height))
        ),
        material=oak,
        name="left_lower_side",
    )
    cradle.visual(
        Box((cheek_length, cheek_thickness, lower_cheek_height)),
        origin=Origin(
            xyz=(0.0, cheek_y, cheek_base_z + (0.5 * lower_cheek_height))
        ),
        material=oak,
        name="right_lower_side",
    )
    cradle.visual(
        Box((cheek_post_length, cheek_thickness, upper_cheek_height)),
        origin=Origin(
            xyz=(
                cheek_front_x,
                -cheek_y,
                cheek_base_z + lower_cheek_height + (0.5 * upper_cheek_height),
            )
        ),
        material=oak,
        name="left_front_post",
    )
    cradle.visual(
        Box((cheek_post_length, cheek_thickness, upper_cheek_height)),
        origin=Origin(
            xyz=(
                cheek_rear_x,
                -cheek_y,
                cheek_base_z + lower_cheek_height + (0.5 * upper_cheek_height),
            )
        ),
        material=oak,
        name="left_rear_post",
    )
    cradle.visual(
        Box((cheek_post_length, cheek_thickness, upper_cheek_height)),
        origin=Origin(
            xyz=(
                cheek_front_x,
                cheek_y,
                cheek_base_z + lower_cheek_height + (0.5 * upper_cheek_height),
            )
        ),
        material=oak,
        name="right_front_post",
    )
    cradle.visual(
        Box((cheek_post_length, cheek_thickness, upper_cheek_height)),
        origin=Origin(
            xyz=(
                cheek_rear_x,
                cheek_y,
                cheek_base_z + lower_cheek_height + (0.5 * upper_cheek_height),
            )
        ),
        material=oak,
        name="right_rear_post",
    )

    barrel = model.part("barrel")
    barrel.visual(
        barrel_shell_mesh(),
        origin=Origin(xyz=(0.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wrought_iron,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.345, length=0.05),
        origin=Origin(xyz=(-0.11, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_band,
        name="breech_band",
    )
    barrel.visual(
        Cylinder(radius=0.325, length=0.05),
        origin=Origin(xyz=(0.06, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_band,
        name="center_band",
    )
    barrel.visual(
        Cylinder(radius=0.285, length=0.045),
        origin=Origin(xyz=(0.28, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_band,
        name="forward_band",
    )
    barrel.visual(
        Cylinder(radius=0.245, length=0.04),
        origin=Origin(xyz=(0.60, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_band,
        name="muzzle_band",
    )
    barrel.visual(
        Cylinder(radius=trunnion_radius, length=trunnion_length),
        origin=Origin(
            xyz=(0.0, -trunnion_center_y, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=iron_band,
        name="left_trunnion",
    )
    barrel.visual(
        Cylinder(radius=trunnion_radius, length=trunnion_length),
        origin=Origin(
            xyz=(0.0, trunnion_center_y, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=iron_band,
        name="right_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.095, length=0.04),
        origin=Origin(
            xyz=(0.0, -0.33, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=iron_band,
        name="left_trunnion_boss",
    )
    barrel.visual(
        Cylinder(radius=0.095, length=0.04),
        origin=Origin(
            xyz=(0.0, 0.33, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=iron_band,
        name="right_trunnion_boss",
    )
    barrel.visual(
        Cylinder(radius=0.02, length=0.03),
        origin=Origin(xyz=(-0.02, 0.0, 0.305)),
        material=iron_band,
        name="touch_hole_boss",
    )

    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, trunnion_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8000.0,
            velocity=0.45,
            lower=0.0,
            upper=math.radians(18.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cradle = object_model.get_part("cradle")
    barrel = object_model.get_part("barrel")
    elevation = object_model.get_articulation("barrel_elevation")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=32)

    ctx.expect_contact(
        barrel,
        cradle,
        elem_a="left_trunnion",
        elem_b="left_lower_side",
        name="left_trunnion_bears_on_left_cheek",
    )
    ctx.expect_contact(
        barrel,
        cradle,
        elem_a="right_trunnion",
        elem_b="right_lower_side",
        name="right_trunnion_bears_on_right_cheek",
    )
    ctx.expect_gap(
        barrel,
        cradle,
        axis="z",
        positive_elem="barrel_shell",
        negative_elem="bed_floor",
        min_gap=0.02,
        name="barrel_body_clears_bed_floor",
    )
    ctx.expect_within(
        barrel,
        cradle,
        axes="y",
        inner_elem="barrel_shell",
        margin=0.02,
        name="barrel_body_stays_between_cheeks",
    )

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
    limits = elevation.motion_limits
    assert limits is not None
    assert limits.lower is not None
    assert limits.upper is not None

    rest_shell_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
    assert rest_shell_aabb is not None

    with ctx.pose({elevation: limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="barrel_lower_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="barrel_lower_pose_no_floating")
        ctx.expect_contact(
            barrel,
            cradle,
            elem_a="left_trunnion",
            elem_b="left_lower_side",
            name="barrel_lower_pose_left_trunnion_contact",
        )
        ctx.expect_contact(
            barrel,
            cradle,
            elem_a="right_trunnion",
            elem_b="right_lower_side",
            name="barrel_lower_pose_right_trunnion_contact",
        )
        ctx.expect_gap(
            barrel,
            cradle,
            axis="z",
            positive_elem="barrel_shell",
            negative_elem="bed_floor",
            min_gap=0.02,
            name="barrel_lower_pose_floor_clearance",
        )

    with ctx.pose({elevation: limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="barrel_upper_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="barrel_upper_pose_no_floating")
        ctx.expect_contact(
            barrel,
            cradle,
            elem_a="left_trunnion",
            elem_b="left_lower_side",
            name="barrel_upper_pose_left_trunnion_contact",
        )
        ctx.expect_contact(
            barrel,
            cradle,
            elem_a="right_trunnion",
            elem_b="right_lower_side",
            name="barrel_upper_pose_right_trunnion_contact",
        )
        ctx.expect_gap(
            barrel,
            cradle,
            axis="z",
            positive_elem="barrel_shell",
            negative_elem="bed_floor",
            min_gap=0.03,
            name="barrel_upper_pose_floor_clearance",
        )
        upper_shell_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
        assert upper_shell_aabb is not None
        assert upper_shell_aabb[1][2] > rest_shell_aabb[1][2] + 0.05

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_post_hydraulic_vehicle_lift")

    powder_coat = model.material("powder_coat", rgba=(0.22, 0.24, 0.27, 1.0))
    deck_gray = model.material("deck_gray", rgba=(0.30, 0.33, 0.36, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.88, 0.75, 0.15, 1.0))
    hydraulic_silver = model.material("hydraulic_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    cabinet_blue = model.material("cabinet_blue", rgba=(0.15, 0.34, 0.58, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))

    platform_length = 4.80
    platform_width = 2.16
    base_plate_length = 1.90
    base_plate_width = 1.50
    base_plate_thickness = 0.12

    post_height = 2.55
    post_outer = 0.56
    post_inner = 0.34
    post_base_z = base_plate_thickness

    guide_height = 0.92
    guide_outer_x = 0.90
    guide_outer_y = 0.76
    guide_inner_x = 0.62
    guide_inner_y = 0.62

    lift_origin_z = 0.22
    platform_top_z_local = 0.88
    arm_hinge_z_local = 0.86

    def hollow_tube_mesh(
        *,
        outer_x: float,
        outer_y: float,
        inner_x: float,
        inner_y: float,
        height: float,
        name: str,
    ):
        outer = rounded_rect_profile(
            outer_x,
            outer_y,
            radius=min(outer_x, outer_y) * 0.10,
            corner_segments=8,
        )
        inner = rounded_rect_profile(
            inner_x,
            inner_y,
            radius=min(inner_x, inner_y) * 0.08,
            corner_segments=8,
        )
        return mesh_from_geometry(
            ExtrudeWithHolesGeometry(outer, [inner], height=height, center=True),
            name,
        )

    base = model.part("base")
    base.visual(
        Box((base_plate_length, base_plate_width, base_plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_plate_thickness / 2.0)),
        material=powder_coat,
        name="base_plate",
    )
    base.visual(
        Box((1.56, 0.20, 0.14)),
        origin=Origin(xyz=(0.0, 0.48, 0.07)),
        material=powder_coat,
        name="front_anchor_skid",
    )
    base.visual(
        Box((1.56, 0.20, 0.14)),
        origin=Origin(xyz=(0.0, -0.48, 0.07)),
        material=powder_coat,
        name="rear_anchor_skid",
    )
    base.visual(
        Box((0.70, 0.16, 0.20)),
        origin=Origin(xyz=(0.0, 0.50, 0.16)),
        material=powder_coat,
        name="front_base_cheek",
    )
    base.visual(
        Box((0.70, 0.16, 0.20)),
        origin=Origin(xyz=(0.0, -0.50, 0.16)),
        material=powder_coat,
        name="rear_base_cheek",
    )
    base.visual(
        hollow_tube_mesh(
            outer_x=post_outer,
            outer_y=post_outer,
            inner_x=post_inner,
            inner_y=post_inner,
            height=post_height,
            name="lift_post_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, post_base_z + post_height / 2.0)),
        material=powder_coat,
        name="post_shell",
    )
    base.visual(
        hollow_tube_mesh(
            outer_x=0.66,
            outer_y=0.66,
            inner_x=0.26,
            inner_y=0.26,
            height=0.12,
            name="lift_post_head",
        ),
        origin=Origin(xyz=(0.0, 0.0, post_base_z + post_height + 0.06)),
        material=powder_coat,
        name="post_head_cap",
    )
    base.visual(
        Box((0.42, 0.24, 0.56)),
        origin=Origin(xyz=(-0.70, 0.58, 0.39)),
        material=cabinet_blue,
        name="power_pack_cabinet",
    )
    base.visual(
        Cylinder(radius=0.08, length=0.20),
        origin=Origin(
            xyz=(-0.73, 0.74, 0.50),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hydraulic_silver,
        name="pump_motor",
    )
    base.visual(
        Box((0.14, 0.14, 0.10)),
        origin=Origin(xyz=(-0.58, 0.66, 0.23)),
        material=rubber_black,
        name="hose_guard",
    )
    base.inertial = Inertial.from_geometry(
        Box((base_plate_length, base_plate_width, post_base_z + post_height + 0.30)),
        mass=860.0,
        origin=Origin(xyz=(0.0, 0.0, 1.48)),
    )

    platform = model.part("platform")
    platform.visual(
        hollow_tube_mesh(
            outer_x=guide_outer_x,
            outer_y=guide_outer_y,
            inner_x=guide_inner_x,
            inner_y=guide_inner_y,
            height=guide_height,
            name="platform_guide_sleeve",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=deck_gray,
        name="guide_sleeve",
    )
    platform.visual(
        Box((0.30, 0.84, 0.18)),
        origin=Origin(xyz=(0.59, 0.0, 0.71)),
        material=powder_coat,
        name="front_carriage_block",
    )
    platform.visual(
        Box((0.30, 0.84, 0.18)),
        origin=Origin(xyz=(-0.59, 0.0, 0.71)),
        material=powder_coat,
        name="rear_carriage_block",
    )
    platform.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(
                    platform_length,
                    platform_width,
                    radius=0.08,
                    corner_segments=10,
                ),
                [
                    rounded_rect_profile(
                        0.88,
                        0.78,
                        radius=0.10,
                        corner_segments=10,
                    )
                ],
                height=0.08,
                center=True,
            ),
            "platform_deck_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, platform_top_z_local)),
        material=deck_gray,
        name="deck_plate",
    )
    platform.visual(
        Box((platform_length, 0.16, 0.18)),
        origin=Origin(xyz=(0.0, platform_width / 2.0 - 0.08, 0.71)),
        material=powder_coat,
        name="left_side_beam",
    )
    platform.visual(
        Box((platform_length, 0.16, 0.18)),
        origin=Origin(xyz=(0.0, -platform_width / 2.0 + 0.08, 0.71)),
        material=powder_coat,
        name="right_side_beam",
    )
    platform.visual(
        Box((0.18, 1.86, 0.20)),
        origin=Origin(xyz=(platform_length / 2.0 - 0.09, 0.0, 0.74)),
        material=powder_coat,
        name="front_end_beam",
    )
    platform.visual(
        Box((0.18, 1.86, 0.20)),
        origin=Origin(xyz=(-platform_length / 2.0 + 0.09, 0.0, 0.74)),
        material=powder_coat,
        name="rear_end_beam",
    )
    for index, x_pos in enumerate((-1.45, 1.45)):
        platform.visual(
            Box((0.20, 1.86, 0.20)),
            origin=Origin(xyz=(x_pos, 0.0, 0.74)),
            material=powder_coat,
            name=f"crossmember_{index}",
        )
    platform.visual(
        Box((0.20, 0.60, 0.20)),
        origin=Origin(xyz=(0.0, 0.63, 0.74)),
        material=powder_coat,
        name="center_crossmember_left",
    )
    platform.visual(
        Box((0.20, 0.60, 0.20)),
        origin=Origin(xyz=(0.0, -0.63, 0.74)),
        material=powder_coat,
        name="center_crossmember_right",
    )
    platform.visual(
        Box((4.36, 0.42, 0.03)),
        origin=Origin(xyz=(0.0, 0.56, 0.935)),
        material=rubber_black,
        name="left_track_pad",
    )
    platform.visual(
        Box((4.36, 0.42, 0.03)),
        origin=Origin(xyz=(0.0, -0.56, 0.935)),
        material=rubber_black,
        name="right_track_pad",
    )
    platform.visual(
        Box((4.18, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, platform_width / 2.0 + 0.04, 0.82)),
        material=safety_yellow,
        name="left_hinge_mount",
    )
    platform.visual(
        Box((4.18, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -platform_width / 2.0 - 0.04, 0.82)),
        material=safety_yellow,
        name="right_hinge_mount",
    )
    platform.inertial = Inertial.from_geometry(
        Box((platform_length, platform_width, 1.05)),
        mass=1280.0,
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
    )

    def add_safety_arm(part_name: str, side_sign: float) -> None:
        arm = model.part(part_name)
        arm.visual(
            Box((4.15, 0.10, 0.05)),
            origin=Origin(xyz=(0.0, 0.05 * side_sign, 0.025)),
            material=safety_yellow,
            name="mount_flange",
        )
        arm.visual(
            Box((4.15, 0.05, 0.23)),
            origin=Origin(xyz=(0.0, 0.025 * side_sign, 0.165)),
            material=safety_yellow,
            name="arm_panel",
        )
        arm.visual(
            Box((4.15, 0.09, 0.05)),
            origin=Origin(xyz=(0.0, 0.045 * side_sign, 0.305)),
            material=powder_coat,
            name="top_rail",
        )
        arm.visual(
            Box((0.14, 0.08, 0.18)),
            origin=Origin(xyz=(2.01, 0.04 * side_sign, 0.14)),
            material=powder_coat,
            name="front_end_block",
        )
        arm.visual(
            Box((0.14, 0.08, 0.18)),
            origin=Origin(xyz=(-2.01, 0.04 * side_sign, 0.14)),
            material=powder_coat,
            name="rear_end_block",
        )
        arm.inertial = Inertial.from_geometry(
            Box((4.18, 0.12, 0.34)),
            mass=54.0,
            origin=Origin(xyz=(0.0, 0.05 * side_sign, 0.17)),
        )

    add_safety_arm("left_safety_arm", side_sign=1.0)
    add_safety_arm("right_safety_arm", side_sign=-1.0)

    model.articulation(
        "base_to_platform",
        ArticulationType.PRISMATIC,
        parent=base,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, lift_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24000.0,
            velocity=0.14,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "platform_to_left_safety_arm",
        ArticulationType.REVOLUTE,
        parent=platform,
        child="left_safety_arm",
        origin=Origin(xyz=(0.0, platform_width / 2.0, arm_hinge_z_local)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    model.articulation(
        "platform_to_right_safety_arm",
        ArticulationType.REVOLUTE,
        parent=platform,
        child="right_safety_arm",
        origin=Origin(xyz=(0.0, -platform_width / 2.0, arm_hinge_z_local)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    base = object_model.get_part("base")
    platform = object_model.get_part("platform")
    left_arm = object_model.get_part("left_safety_arm")
    right_arm = object_model.get_part("right_safety_arm")
    lift_joint = object_model.get_articulation("base_to_platform")
    left_hinge = object_model.get_articulation("platform_to_left_safety_arm")
    right_hinge = object_model.get_articulation("platform_to_right_safety_arm")

    ctx.expect_gap(
        platform,
        base,
        axis="z",
        positive_elem="deck_plate",
        negative_elem="base_plate",
        min_gap=0.70,
        name="platform deck clears the base plate at rest",
    )
    ctx.expect_overlap(
        platform,
        base,
        axes="z",
        elem_a="guide_sleeve",
        elem_b="post_shell",
        min_overlap=0.85,
        name="guide sleeve remains engaged on the lift post at rest",
    )
    ctx.expect_contact(
        left_arm,
        platform,
        elem_a="mount_flange",
        elem_b="left_hinge_mount",
        contact_tol=1e-4,
        name="left safety arm is seated on its hinge mount",
    )
    ctx.expect_contact(
        right_arm,
        platform,
        elem_a="mount_flange",
        elem_b="right_hinge_mount",
        contact_tol=1e-4,
        name="right safety arm is seated on its hinge mount",
    )

    rest_platform_pos = ctx.part_world_position(platform)
    rest_left_aabb = ctx.part_world_aabb(left_arm)
    rest_right_aabb = ctx.part_world_aabb(right_arm)
    lift_upper = lift_joint.motion_limits.upper if lift_joint.motion_limits else None
    arm_upper = left_hinge.motion_limits.upper if left_hinge.motion_limits else None

    with ctx.pose({lift_joint: lift_upper}):
        ctx.expect_overlap(
            platform,
            base,
            axes="z",
            elem_a="guide_sleeve",
            elem_b="post_shell",
            min_overlap=0.85,
            name="guide sleeve remains engaged on the lift post at full rise",
        )
        raised_platform_pos = ctx.part_world_position(platform)

    ctx.check(
        "platform rises upward along the central post",
        (
            rest_platform_pos is not None
            and raised_platform_pos is not None
            and raised_platform_pos[2] > rest_platform_pos[2] + 1.4
        ),
        details=f"rest={rest_platform_pos}, raised={raised_platform_pos}",
    )

    with ctx.pose(
        {
            left_hinge: arm_upper,
            right_hinge: arm_upper,
        }
    ):
        folded_left_aabb = ctx.part_world_aabb(left_arm)
        folded_right_aabb = ctx.part_world_aabb(right_arm)

    ctx.check(
        "left safety arm folds outward from the deck edge",
        (
            rest_left_aabb is not None
            and folded_left_aabb is not None
            and folded_left_aabb[1][1] > rest_left_aabb[1][1] + 0.20
            and folded_left_aabb[1][2] < rest_left_aabb[1][2] - 0.20
        ),
        details=f"rest={rest_left_aabb}, folded={folded_left_aabb}",
    )
    ctx.check(
        "right safety arm folds outward from the deck edge",
        (
            rest_right_aabb is not None
            and folded_right_aabb is not None
            and folded_right_aabb[0][1] < rest_right_aabb[0][1] - 0.20
            and folded_right_aabb[1][2] < rest_right_aabb[1][2] - 0.20
        ),
        details=f"rest={rest_right_aabb}, folded={folded_right_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

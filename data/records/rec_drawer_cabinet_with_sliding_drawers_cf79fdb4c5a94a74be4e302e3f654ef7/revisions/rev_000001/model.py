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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lateral_filing_cabinet")

    cabinet_steel = model.material("cabinet_steel", rgba=(0.73, 0.76, 0.79, 1.0))
    drawer_steel = model.material("drawer_steel", rgba=(0.69, 0.72, 0.75, 1.0))
    slide_dark = model.material("slide_dark", rgba=(0.24, 0.26, 0.29, 1.0))
    handle_chrome = model.material("handle_chrome", rgba=(0.88, 0.9, 0.92, 1.0))
    lock_dark = model.material("lock_dark", rgba=(0.18, 0.19, 0.21, 1.0))

    cabinet_width = 0.90
    cabinet_depth = 0.46
    cabinet_height = 1.38
    wall_thickness = 0.016
    top_thickness = 0.022
    bottom_thickness = 0.024
    back_thickness = 0.012
    front_frame_depth = 0.020
    front_stile_width = 0.026
    front_margin = 0.032
    drawer_reveal = 0.008

    drawer_front_height = (
        cabinet_height - 2.0 * front_margin - 3.0 * drawer_reveal
    ) / 4.0
    drawer_pitch = drawer_front_height + drawer_reveal
    drawer_centers = [
        front_margin + drawer_front_height / 2.0 + i * drawer_pitch for i in range(4)
    ]

    drawer_front_width = 0.842
    drawer_front_thickness = 0.020
    drawer_box_width = 0.832
    drawer_box_height = 0.252
    drawer_box_depth = 0.392
    drawer_side_thickness = 0.010
    drawer_bottom_thickness = 0.008

    handle_bar_length = 0.31
    handle_radius = 0.006
    handle_post_width = 0.012
    handle_post_depth = 0.028
    handle_post_height = 0.012

    moving_slide_width = 0.006
    moving_slide_length = 0.386
    moving_slide_height = 0.018
    moving_slide_z = -0.082

    fixed_slide_width = 0.008
    fixed_slide_length = 0.384
    fixed_slide_height = 0.022
    fixed_slide_clearance = 0.006

    lock_bar_x = 0.410
    lock_bar_y = -0.259
    lock_bar_base_z = 0.055
    lock_bar_width = 0.012
    lock_bar_depth = 0.008
    lock_bar_height = 1.24
    lock_travel = 0.060

    body = model.part("body")
    body.visual(
        Box((wall_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(
            xyz=(
                -cabinet_width / 2.0 + wall_thickness / 2.0,
                0.0,
                cabinet_height / 2.0,
            )
        ),
        material=cabinet_steel,
        name="left_side",
    )
    body.visual(
        Box((wall_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(
            xyz=(
                cabinet_width / 2.0 - wall_thickness / 2.0,
                0.0,
                cabinet_height / 2.0,
            )
        ),
        material=cabinet_steel,
        name="right_side",
    )
    body.visual(
        Box((cabinet_width - 2.0 * wall_thickness, cabinet_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - top_thickness / 2.0)),
        material=cabinet_steel,
        name="top_panel",
    )
    body.visual(
        Box((cabinet_width - 2.0 * wall_thickness, cabinet_depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness / 2.0)),
        material=cabinet_steel,
        name="bottom_panel",
    )
    body.visual(
        Box(
            (
                cabinet_width - 2.0 * wall_thickness,
                back_thickness,
                cabinet_height - top_thickness - bottom_thickness,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                cabinet_depth / 2.0 - back_thickness / 2.0,
                bottom_thickness
                + (cabinet_height - top_thickness - bottom_thickness) / 2.0,
            )
        ),
        material=cabinet_steel,
        name="back_panel",
    )
    body.visual(
        Box((front_stile_width, front_frame_depth, cabinet_height)),
        origin=Origin(
            xyz=(
                -cabinet_width / 2.0 + front_stile_width / 2.0,
                -cabinet_depth / 2.0 + front_frame_depth / 2.0,
                cabinet_height / 2.0,
            )
        ),
        material=cabinet_steel,
        name="left_front_stile",
    )
    body.visual(
        Box((front_stile_width, front_frame_depth, cabinet_height)),
        origin=Origin(
            xyz=(
                cabinet_width / 2.0 - front_stile_width / 2.0,
                -cabinet_depth / 2.0 + front_frame_depth / 2.0,
                cabinet_height / 2.0,
            )
        ),
        material=cabinet_steel,
        name="right_front_stile",
    )
    body.visual(
        Box(
            (
                cabinet_width - 2.0 * front_stile_width,
                front_frame_depth,
                front_margin,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -cabinet_depth / 2.0 + front_frame_depth / 2.0,
                front_margin / 2.0,
            )
        ),
        material=cabinet_steel,
        name="bottom_front_rail",
    )
    body.visual(
        Box(
            (
                cabinet_width - 2.0 * front_stile_width,
                front_frame_depth,
                front_margin,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -cabinet_depth / 2.0 + front_frame_depth / 2.0,
                cabinet_height - front_margin / 2.0,
            )
        ),
        material=cabinet_steel,
        name="top_front_rail",
    )

    inner_body_width = cabinet_width - 2.0 * wall_thickness
    inner_body_depth = cabinet_depth - back_thickness
    for index in range(3):
        divider_z = front_margin + drawer_front_height + drawer_reveal / 2.0 + index * drawer_pitch
        body.visual(
            Box((inner_body_width, inner_body_depth, drawer_reveal)),
            origin=Origin(xyz=(0.0, back_thickness / -2.0, divider_z)),
            material=cabinet_steel,
            name=f"drawer_gap_rail_{index}",
        )
        body.visual(
            Box((inner_body_width, cabinet_depth - back_thickness, 0.014)),
            origin=Origin(
                xyz=(
                    0.0,
                    (cabinet_depth - back_thickness) / 2.0 - back_thickness / 2.0,
                    divider_z,
                )
            ),
            material=cabinet_steel,
            name=f"drawer_divider_{index}",
        )

    guide_block_depth = 0.025
    guide_block_width = 0.018
    guide_block_front_y = lock_bar_y + lock_bar_depth / 2.0
    guide_block_center_y = guide_block_front_y + guide_block_depth / 2.0
    for name, center_z, height in (
        ("lock_bottom_guide", 0.016, 0.024),
        ("lock_mid_guide", 0.690, drawer_reveal),
        ("lock_top_guide", cabinet_height - front_margin / 2.0, 0.024),
    ):
        body.visual(
            Box((guide_block_width, guide_block_depth, height)),
            origin=Origin(
                xyz=(
                    lock_bar_x + guide_block_width / 2.0,
                    guide_block_center_y,
                    center_z,
                )
            ),
            material=lock_dark,
            name=name,
        )

    body.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=78.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height / 2.0)),
    )

    def add_drawer(index: int, center_z: float) -> tuple[str, str]:
        drawer_name = f"drawer_{index + 1}"
        joint_name = f"body_to_drawer_{index + 1}"
        drawer = model.part(drawer_name)

        drawer.visual(
            Box((drawer_front_width, drawer_front_thickness, drawer_front_height)),
            origin=Origin(xyz=(0.0, -drawer_front_thickness / 2.0, 0.0)),
            material=drawer_steel,
            name="front_panel",
        )
        drawer.visual(
            Box(
                (
                    drawer_box_width - 2.0 * drawer_side_thickness,
                    drawer_box_depth,
                    drawer_bottom_thickness,
                )
            ),
            origin=Origin(
                xyz=(
                    0.0,
                    drawer_box_depth / 2.0,
                    -drawer_box_height / 2.0 + drawer_bottom_thickness / 2.0,
                )
            ),
            material=drawer_steel,
            name="bottom_pan",
        )
        drawer.visual(
            Box((drawer_side_thickness, drawer_box_depth, drawer_box_height)),
            origin=Origin(
                xyz=(
                    -drawer_box_width / 2.0 + drawer_side_thickness / 2.0,
                    drawer_box_depth / 2.0,
                    0.0,
                )
            ),
            material=drawer_steel,
            name="left_wall",
        )
        drawer.visual(
            Box((drawer_side_thickness, drawer_box_depth, drawer_box_height)),
            origin=Origin(
                xyz=(
                    drawer_box_width / 2.0 - drawer_side_thickness / 2.0,
                    drawer_box_depth / 2.0,
                    0.0,
                )
            ),
            material=drawer_steel,
            name="right_wall",
        )
        drawer.visual(
            Box((drawer_box_width, drawer_side_thickness, drawer_box_height)),
            origin=Origin(
                xyz=(
                    0.0,
                    drawer_box_depth - drawer_side_thickness / 2.0,
                    0.0,
                )
            ),
            material=drawer_steel,
            name="rear_wall",
        )

        left_slide_x = -drawer_box_width / 2.0 - moving_slide_width / 2.0
        right_slide_x = drawer_box_width / 2.0 + moving_slide_width / 2.0
        for slide_name, slide_x in (
            ("left_slide", left_slide_x),
            ("right_slide", right_slide_x),
        ):
            drawer.visual(
                Box((moving_slide_width, moving_slide_length, moving_slide_height)),
                origin=Origin(
                    xyz=(slide_x, moving_slide_length / 2.0, moving_slide_z)
                ),
                material=slide_dark,
                name=slide_name,
            )

        post_y = -drawer_front_thickness - handle_post_depth / 2.0
        grip_y = -drawer_front_thickness - handle_post_depth - handle_radius
        for post_name, post_x in (
            ("handle_left_post", -0.135),
            ("handle_right_post", 0.135),
        ):
            drawer.visual(
                Box((handle_post_width, handle_post_depth, handle_post_height)),
                origin=Origin(xyz=(post_x, post_y, 0.0)),
                material=handle_chrome,
                name=post_name,
            )
        drawer.visual(
            Cylinder(radius=handle_radius, length=handle_bar_length),
            origin=Origin(xyz=(0.0, grip_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=handle_chrome,
            name="handle_bar",
        )

        keeper_x = drawer_front_width / 2.0 - 0.032
        drawer.visual(
            Box((0.018, 0.004, 0.024)),
            origin=Origin(xyz=(keeper_x, -0.022, 0.0)),
            material=lock_dark,
            name="keeper_back",
        )
        drawer.visual(
            Box((0.004, 0.018, 0.024)),
            origin=Origin(xyz=(keeper_x - 0.007, -0.029, 0.0)),
            material=lock_dark,
            name="keeper_left_cheek",
        )
        drawer.visual(
            Box((0.004, 0.018, 0.024)),
            origin=Origin(xyz=(keeper_x + 0.007, -0.029, 0.0)),
            material=lock_dark,
            name="keeper_right_cheek",
        )
        drawer.visual(
            Box((0.018, 0.018, 0.004)),
            origin=Origin(xyz=(keeper_x, -0.029, -0.010)),
            material=lock_dark,
            name="keeper_floor",
        )

        drawer.inertial = Inertial.from_geometry(
            Box(
                (
                    drawer_front_width,
                    drawer_front_thickness + drawer_box_depth,
                    drawer_front_height,
                )
            ),
            mass=8.5,
            origin=Origin(
                xyz=(
                    0.0,
                    (drawer_box_depth - drawer_front_thickness) / 2.0,
                    0.0,
                )
            ),
        )

        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(0.0, -cabinet_depth / 2.0, center_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=120.0,
                velocity=0.30,
                lower=0.0,
                upper=0.340,
            ),
        )

        rail_x = drawer_box_width / 2.0 + moving_slide_width + fixed_slide_clearance + fixed_slide_width / 2.0
        for fixed_name, x_sign in (
            (f"drawer_{index + 1}_left_slide_fixed", -1.0),
            (f"drawer_{index + 1}_right_slide_fixed", 1.0),
        ):
            body.visual(
                Box((fixed_slide_width, fixed_slide_length, fixed_slide_height)),
                origin=Origin(
                    xyz=(
                        x_sign * rail_x,
                        -cabinet_depth / 2.0 + fixed_slide_length / 2.0,
                        center_z + moving_slide_z,
                    )
                ),
                material=slide_dark,
                name=fixed_name,
            )

        return drawer_name, joint_name

    drawer_defs = [add_drawer(index, center_z) for index, center_z in enumerate(drawer_centers)]

    lock_bar = model.part("lock_bar")
    lock_bar.visual(
        Box((lock_bar_width, lock_bar_depth, lock_bar_height)),
        origin=Origin(xyz=(0.0, 0.0, lock_bar_height / 2.0)),
        material=lock_dark,
        name="bar_strip",
    )
    lock_bar.visual(
        Box((0.020, 0.010, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=lock_dark,
        name="lower_shoe",
    )
    lock_bar.visual(
        Box((0.022, 0.014, 0.084)),
        origin=Origin(xyz=(0.0, 0.0, lock_bar_height + 0.042)),
        material=lock_dark,
        name="top_cap",
    )
    lock_bar.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(
            xyz=(0.0, -0.012, lock_bar_height + 0.024),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=handle_chrome,
        name="lock_pull",
    )
    for index, center_z in enumerate(drawer_centers):
        lock_bar.visual(
            Box((0.018, 0.006, 0.012)),
            origin=Origin(
                xyz=(-0.014, 0.0, center_z - lock_bar_base_z)
            ),
            material=lock_dark,
            name=f"finger_{index + 1}",
        )
    lock_bar.inertial = Inertial.from_geometry(
        Box((0.030, 0.020, lock_bar_height + 0.050)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, (lock_bar_height + 0.050) / 2.0)),
    )

    model.articulation(
        "body_to_lock_bar",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lock_bar,
        origin=Origin(xyz=(lock_bar_x, lock_bar_y, lock_bar_base_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.10,
            lower=0.0,
            upper=lock_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lock_bar = object_model.get_part("lock_bar")
    lock_joint = object_model.get_articulation("body_to_lock_bar")
    drawer_parts = [object_model.get_part(f"drawer_{index}") for index in range(1, 5)]
    drawer_joints = [
        object_model.get_articulation(f"body_to_drawer_{index}") for index in range(1, 5)
    ]

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

    for index, drawer in enumerate(drawer_parts, start=1):
        ctx.expect_gap(
            body,
            drawer,
            axis="y",
            positive_elem="left_front_stile",
            negative_elem="front_panel",
            min_gap=0.0,
            max_gap=0.0005,
            name=f"drawer {index} front panel sits flush with cabinet face",
        )

    for index, (drawer, drawer_joint) in enumerate(zip(drawer_parts, drawer_joints), start=1):
        with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
            closed_pos = ctx.part_world_position(drawer)
        with ctx.pose(
            {
                lock_joint: lock_joint.motion_limits.upper,
                drawer_joint: drawer_joint.motion_limits.upper,
            }
        ):
            extended_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                elem_a="right_slide",
                elem_b=f"drawer_{index}_right_slide_fixed",
                min_overlap=0.045,
                name=f"drawer {index} retains slide engagement at full extension",
            )
        ctx.check(
            f"drawer {index} extends outward from cabinet front",
            closed_pos is not None
            and extended_pos is not None
            and extended_pos[1] < closed_pos[1] - 0.25,
            details=f"closed={closed_pos}, extended={extended_pos}",
        )

    ctx.expect_contact(
        lock_bar,
        body,
        elem_a="bar_strip",
        elem_b="lock_mid_guide",
        name="lock bar stays guided by the right-side housing",
    )

    for index, drawer in enumerate(drawer_parts, start=1):
        ctx.expect_overlap(
            lock_bar,
            drawer,
            axes="xz",
            elem_a=f"finger_{index}",
            elem_b="keeper_back",
            min_overlap=0.010,
            name=f"lock finger {index} aligns with drawer {index} keeper",
        )
        ctx.expect_gap(
            drawer,
            lock_bar,
            axis="y",
            positive_elem="keeper_back",
            negative_elem=f"finger_{index}",
            min_gap=0.001,
            max_gap=0.008,
            name=f"lock finger {index} sits inside drawer {index} keeper pocket",
        )

    locked_pos = ctx.part_world_position(lock_bar)
    with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
        unlocked_pos = ctx.part_world_position(lock_bar)
    ctx.check(
        "lock bar lifts vertically to unlock",
        locked_pos is not None
        and unlocked_pos is not None
        and unlocked_pos[2] > locked_pos[2] + 0.045,
        details=f"locked={locked_pos}, unlocked={unlocked_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    model = ArticulatedObject(name="four_door_refrigerator")

    stainless = model.material("stainless", rgba=(0.79, 0.80, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.20, 1.0))
    liner_white = model.material("liner_white", rgba=(0.92, 0.93, 0.94, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.72, 0.84, 0.90, 0.25))
    gasket_gray = model.material("gasket_gray", rgba=(0.60, 0.62, 0.64, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.56, 0.58, 0.60, 1.0))

    cabinet_width = 0.91
    cabinet_height = 1.86
    case_depth = 0.70
    wall = 0.032
    center_post = 0.026
    divider_thickness = 0.030
    toe_kick_height = 0.10
    toe_kick_recess = 0.09

    fresh_door_height = 1.08
    freezer_door_height = 0.60
    row_gap = 0.012
    door_thickness = 0.064
    hinge_radius = 0.013
    panel_overlap_to_axis = 0.002
    hinge_axis_y = case_depth + hinge_radius

    lower_door_z = toe_kick_height + 0.008
    upper_door_z = lower_door_z + freezer_door_height + row_gap

    half_opening_width = (cabinet_width - 2.0 * wall - center_post) / 2.0
    door_panel_width = half_opening_width - (hinge_radius - panel_overlap_to_axis)

    freezer_open_z = lower_door_z + 0.010
    freezer_open_height = freezer_door_height - 0.020
    fresh_open_z = upper_door_z + 0.010
    fresh_open_height = fresh_door_height - 0.020
    divider_center_z = lower_door_z + freezer_door_height + row_gap / 2.0

    left_open_inner_x = -cabinet_width / 2.0 + wall
    right_open_inner_x = cabinet_width / 2.0 - wall

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((wall, case_depth, cabinet_height)),
        origin=Origin(
            xyz=(-cabinet_width / 2.0 + wall / 2.0, case_depth / 2.0, cabinet_height / 2.0)
        ),
        material=dark_trim,
        name="left_wall",
    )
    cabinet.visual(
        Box((wall, case_depth, cabinet_height)),
        origin=Origin(
            xyz=(cabinet_width / 2.0 - wall / 2.0, case_depth / 2.0, cabinet_height / 2.0)
        ),
        material=dark_trim,
        name="right_wall",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, case_depth, wall)),
        origin=Origin(xyz=(0.0, case_depth / 2.0, wall / 2.0)),
        material=dark_trim,
        name="base_pan",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, case_depth, wall)),
        origin=Origin(xyz=(0.0, case_depth / 2.0, cabinet_height - wall / 2.0)),
        material=dark_trim,
        name="top_cap",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, wall, cabinet_height - 2.0 * wall)),
        origin=Origin(xyz=(0.0, wall / 2.0, cabinet_height / 2.0)),
        material=dark_trim,
        name="back_panel",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, case_depth - wall, divider_thickness)),
        origin=Origin(
            xyz=(0.0, wall + (case_depth - wall) / 2.0, divider_center_z)
        ),
        material=liner_white,
        name="middle_divider",
    )
    cabinet.visual(
        Box((center_post, case_depth - wall, fresh_open_height)),
        origin=Origin(
            xyz=(0.0, wall + (case_depth - wall) / 2.0, fresh_open_z + fresh_open_height / 2.0)
        ),
        material=dark_trim,
        name="upper_center_post",
    )
    cabinet.visual(
        Box((center_post, case_depth - wall, freezer_open_height)),
        origin=Origin(
            xyz=(0.0, wall + (case_depth - wall) / 2.0, freezer_open_z + freezer_open_height / 2.0)
        ),
        material=dark_trim,
        name="lower_center_post",
    )

    shelf_depth = case_depth - wall - 0.11
    shelf_center_y = wall + shelf_depth / 2.0
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, shelf_depth, 0.012)),
        origin=Origin(xyz=(0.0, shelf_center_y, upper_door_z + 0.30)),
        material=glass_tint,
        name="fresh_shelf_upper",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, shelf_depth, 0.012)),
        origin=Origin(xyz=(0.0, shelf_center_y, upper_door_z + 0.66)),
        material=glass_tint,
        name="fresh_shelf_lower",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, shelf_depth, 0.016)),
        origin=Origin(xyz=(0.0, shelf_center_y, lower_door_z + 0.28)),
        material=liner_white,
        name="freezer_shelf",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * wall, 0.016, toe_kick_height)),
        origin=Origin(
            xyz=(0.0, case_depth - toe_kick_recess - 0.008, toe_kick_height / 2.0)
        ),
        material=dark_trim,
        name="toe_kick_panel",
    )
    cabinet.visual(
        Box((cabinet_width, 0.004, 0.060)),
        origin=Origin(xyz=(0.0, case_depth - 0.002, cabinet_height - 0.030)),
        material=stainless,
        name="top_front_trim",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, case_depth, cabinet_height)),
        mass=135.0,
        origin=Origin(xyz=(0.0, case_depth / 2.0, cabinet_height / 2.0)),
    )

    def add_door(
        *,
        part_name: str,
        joint_name: str,
        hinge_x: float,
        bottom_z: float,
        height: float,
        positive_x_from_hinge: bool,
    ) -> None:
        door = model.part(part_name)
        x_sign = 1.0 if positive_x_from_hinge else -1.0

        shell_center_x = x_sign * (door_panel_width / 2.0 + 0.004)
        shell_center_y = 0.021
        front_skin_y = shell_center_y + door_thickness / 2.0 - 0.003
        inner_liner_y = -0.006
        gasket_y = shell_center_y - door_thickness / 2.0 + 0.006

        handle_height = min(height * 0.72, height - 0.20)
        handle_x = x_sign * (door_panel_width - 0.048)
        handle_center_y = shell_center_y + door_thickness / 2.0 + 0.010

        door.visual(
            Box((door_panel_width, door_thickness, height)),
            origin=Origin(xyz=(shell_center_x, shell_center_y, height / 2.0)),
            material=stainless,
            name="door_shell",
        )
        door.visual(
            Box((door_panel_width - 0.010, 0.006, height - 0.020)),
            origin=Origin(xyz=(shell_center_x, front_skin_y, height / 2.0)),
            material=stainless,
            name="front_skin",
        )
        door.visual(
            Box((door_panel_width - 0.060, 0.018, height - 0.090)),
            origin=Origin(xyz=(shell_center_x, inner_liner_y, height / 2.0)),
            material=liner_white,
            name="inner_liner",
        )
        door.visual(
            Box((door_panel_width - 0.032, 0.010, height - 0.040)),
            origin=Origin(xyz=(shell_center_x, gasket_y, height / 2.0)),
            material=gasket_gray,
            name="gasket_band",
        )
        door.visual(
            Box((0.022, 0.024, handle_height)),
            origin=Origin(xyz=(handle_x, handle_center_y, height / 2.0)),
            material=hinge_metal,
            name="handle",
        )
        door.visual(
            Box((door_panel_width - 0.10, 0.045, 0.060)),
            origin=Origin(xyz=(shell_center_x, -0.016, height * 0.28)),
            material=liner_white,
            name="inner_bin_lower",
        )
        door.visual(
            Box((door_panel_width - 0.12, 0.045, 0.055)),
            origin=Origin(xyz=(shell_center_x, -0.016, height * 0.60)),
            material=liner_white,
            name="inner_bin_upper",
        )
        door.visual(
            Cylinder(radius=hinge_radius, length=height - 0.020),
            origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
            material=hinge_metal,
            name="hinge_barrel",
        )
        door.inertial = Inertial.from_geometry(
            Box((door_panel_width, door_thickness, height)),
            mass=18.0 if height > 0.8 else 14.0,
            origin=Origin(xyz=(shell_center_x, shell_center_y, height / 2.0)),
        )

        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=door,
            origin=Origin(xyz=(hinge_x, hinge_axis_y, bottom_z)),
            axis=(0.0, 0.0, 1.0 if positive_x_from_hinge else -1.0),
            motion_limits=MotionLimits(
                effort=90.0,
                velocity=1.5,
                lower=0.0,
                upper=math.radians(115.0),
            ),
        )

    def add_hinge_clips(
        *,
        clip_prefix: str,
        hinge_x: float,
        bottom_z: float,
        height: float,
        positive_x_from_hinge: bool,
    ) -> None:
        x_sign = 1.0 if positive_x_from_hinge else -1.0
        clip_width = 0.018
        clip_depth = 0.014
        clip_height = min(0.12, max(0.10, height * 0.18))
        clip_center_x = hinge_x - x_sign * (clip_width / 2.0 - 0.001)
        clip_center_y = case_depth + 0.003

        cabinet.visual(
            Box((clip_width, clip_depth, clip_height)),
            origin=Origin(xyz=(clip_center_x, clip_center_y, bottom_z + 0.030 + clip_height / 2.0)),
            material=hinge_metal,
            name=f"{clip_prefix}_lower",
        )
        cabinet.visual(
            Box((clip_width, clip_depth, clip_height)),
            origin=Origin(
                xyz=(clip_center_x, clip_center_y, bottom_z + height - 0.030 - clip_height / 2.0)
            ),
            material=hinge_metal,
            name=f"{clip_prefix}_upper",
        )

    add_door(
        part_name="upper_left_door",
        joint_name="cabinet_to_upper_left_door",
        hinge_x=left_open_inner_x + hinge_radius,
        bottom_z=upper_door_z,
        height=fresh_door_height,
        positive_x_from_hinge=True,
    )
    add_hinge_clips(
        clip_prefix="upper_left_hinge_clip",
        hinge_x=left_open_inner_x + hinge_radius,
        bottom_z=upper_door_z,
        height=fresh_door_height,
        positive_x_from_hinge=True,
    )
    add_door(
        part_name="upper_right_door",
        joint_name="cabinet_to_upper_right_door",
        hinge_x=center_post / 2.0 + hinge_radius,
        bottom_z=upper_door_z,
        height=fresh_door_height,
        positive_x_from_hinge=True,
    )
    add_hinge_clips(
        clip_prefix="upper_right_hinge_clip",
        hinge_x=center_post / 2.0 + hinge_radius,
        bottom_z=upper_door_z,
        height=fresh_door_height,
        positive_x_from_hinge=True,
    )
    add_door(
        part_name="lower_left_door",
        joint_name="cabinet_to_lower_left_door",
        hinge_x=-(center_post / 2.0 + hinge_radius),
        bottom_z=lower_door_z,
        height=freezer_door_height,
        positive_x_from_hinge=False,
    )
    add_hinge_clips(
        clip_prefix="lower_left_hinge_clip",
        hinge_x=-(center_post / 2.0 + hinge_radius),
        bottom_z=lower_door_z,
        height=freezer_door_height,
        positive_x_from_hinge=False,
    )
    add_door(
        part_name="lower_right_door",
        joint_name="cabinet_to_lower_right_door",
        hinge_x=right_open_inner_x - hinge_radius,
        bottom_z=lower_door_z,
        height=freezer_door_height,
        positive_x_from_hinge=False,
    )
    add_hinge_clips(
        clip_prefix="lower_right_hinge_clip",
        hinge_x=right_open_inner_x - hinge_radius,
        bottom_z=lower_door_z,
        height=freezer_door_height,
        positive_x_from_hinge=False,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    upper_left = object_model.get_part("upper_left_door")
    upper_right = object_model.get_part("upper_right_door")
    lower_left = object_model.get_part("lower_left_door")
    lower_right = object_model.get_part("lower_right_door")

    upper_left_joint = object_model.get_articulation("cabinet_to_upper_left_door")
    upper_right_joint = object_model.get_articulation("cabinet_to_upper_right_door")
    lower_left_joint = object_model.get_articulation("cabinet_to_lower_left_door")
    lower_right_joint = object_model.get_articulation("cabinet_to_lower_right_door")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    door_specs = (
        (upper_left, "upper_left_hinge_clip_lower", "upper_left_hinge_clip_upper"),
        (upper_right, "upper_right_hinge_clip_lower", "upper_right_hinge_clip_upper"),
        (lower_left, "lower_left_hinge_clip_lower", "lower_left_hinge_clip_upper"),
        (lower_right, "lower_right_hinge_clip_lower", "lower_right_hinge_clip_upper"),
    )

    for door, lower_clip, upper_clip in door_specs:
        ctx.allow_overlap(
            cabinet,
            door,
            elem_a=lower_clip,
            elem_b="hinge_barrel",
            reason="Cabinet hinge keeper intentionally wraps the door barrel to keep the leaf captured.",
        )
        ctx.allow_overlap(
            cabinet,
            door,
            elem_a=upper_clip,
            elem_b="hinge_barrel",
            reason="Cabinet hinge keeper intentionally wraps the door barrel to keep the leaf captured.",
        )

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

    for joint, expected_sign in (
        (upper_left_joint, 1.0),
        (upper_right_joint, 1.0),
        (lower_left_joint, -1.0),
        (lower_right_joint, -1.0),
    ):
        axis = joint.axis
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_axis_vertical",
            axis is not None
            and abs(axis[0]) < 1e-9
            and abs(axis[1]) < 1e-9
            and axis[2] == expected_sign,
            details=f"axis={axis}",
        )
        ctx.check(
            f"{joint.name}_motion_limit",
            limits is not None and limits.lower == 0.0 and limits.upper is not None and limits.upper > 1.8,
            details=f"limits={limits}",
        )

    for door, lower_clip, upper_clip in door_specs:
        ctx.expect_overlap(
            cabinet,
            door,
            axes="xyz",
            min_overlap=0.008,
            elem_a=lower_clip,
            elem_b="hinge_barrel",
            name=f"{door.name}_lower_keeper_captures_barrel",
        )
        ctx.expect_overlap(
            cabinet,
            door,
            axes="xyz",
            min_overlap=0.008,
            elem_a=upper_clip,
            elem_b="hinge_barrel",
            name=f"{door.name}_upper_keeper_captures_barrel",
        )
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            min_gap=0.001,
            max_gap=0.004,
            positive_elem="door_shell",
            negative_elem="middle_divider",
            name=f"{door.name}_front_proud_gap",
        )

    ctx.expect_gap(
        upper_right,
        upper_left,
        axis="x",
        min_gap=0.020,
        max_gap=0.040,
        positive_elem="door_shell",
        negative_elem="door_shell",
        name="upper_center_split_gap",
    )
    ctx.expect_gap(
        lower_right,
        lower_left,
        axis="x",
        min_gap=0.020,
        max_gap=0.040,
        positive_elem="door_shell",
        negative_elem="door_shell",
        name="lower_center_split_gap",
    )
    ctx.expect_gap(
        upper_left,
        lower_left,
        axis="z",
        min_gap=0.010,
        max_gap=0.020,
        positive_elem="door_shell",
        negative_elem="door_shell",
        name="left_row_split_gap",
    )
    ctx.expect_gap(
        upper_right,
        lower_right,
        axis="z",
        min_gap=0.010,
        max_gap=0.020,
        positive_elem="door_shell",
        negative_elem="door_shell",
        name="right_row_split_gap",
    )

    def check_opening_motion(
        door,
        joint,
        min_extra_front: float,
        lower_clip: str,
        upper_clip: str,
    ) -> None:
        closed_aabb = ctx.part_element_world_aabb(door, elem="door_shell")
        assert closed_aabb is not None
        with ctx.pose({joint: math.radians(75.0)}):
            open_aabb = ctx.part_element_world_aabb(door, elem="door_shell")
            assert open_aabb is not None
            ctx.check(
                f"{door.name}_swings_forward",
                open_aabb[1][1] > closed_aabb[1][1] + min_extra_front,
                details=f"closed_max_y={closed_aabb[1][1]:.4f}, open_max_y={open_aabb[1][1]:.4f}",
            )
            ctx.expect_overlap(
                cabinet,
                door,
                axes="xyz",
                min_overlap=0.008,
                elem_a=lower_clip,
                elem_b="hinge_barrel",
                name=f"{door.name}_lower_keeper_captures_barrel_open",
            )
            ctx.expect_overlap(
                cabinet,
                door,
                axes="xyz",
                min_overlap=0.008,
                elem_a=upper_clip,
                elem_b="hinge_barrel",
                name=f"{door.name}_upper_keeper_captures_barrel_open",
            )

    check_opening_motion(
        upper_left,
        upper_left_joint,
        0.22,
        "upper_left_hinge_clip_lower",
        "upper_left_hinge_clip_upper",
    )
    check_opening_motion(
        upper_right,
        upper_right_joint,
        0.22,
        "upper_right_hinge_clip_lower",
        "upper_right_hinge_clip_upper",
    )
    check_opening_motion(
        lower_left,
        lower_left_joint,
        0.18,
        "lower_left_hinge_clip_lower",
        "lower_left_hinge_clip_upper",
    )
    check_opening_motion(
        lower_right,
        lower_right_joint,
        0.18,
        "lower_right_hinge_clip_lower",
        "lower_right_hinge_clip_upper",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_mixer_case")

    shell_black = model.material("shell_black", rgba=(0.13, 0.14, 0.16, 1.0))
    liner_black = model.material("liner_black", rgba=(0.06, 0.06, 0.07, 1.0))
    hardware = model.material("hardware", rgba=(0.62, 0.64, 0.68, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.19, 0.20, 0.22, 1.0))

    width = 0.62
    depth = 0.40
    corner_r = 0.045
    wall_t = 0.012
    floor_t = 0.012
    lower_h = 0.18
    lid_h = 0.048
    lid_roof_t = 0.010
    hinge_radius = 0.009
    hinge_axis_y = -(depth * 0.5) - 0.006
    hinge_axis_z = lower_h + 0.026
    lid_open_angle = math.radians(63.0)

    def rounded_slab(name: str, sx: float, sy: float, sz: float, radius: float):
        return mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(sx, sy, radius), sz),
            name,
        )

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        rounded_slab("lower_floor", width, depth, floor_t, corner_r),
        origin=Origin(xyz=(0.0, 0.0, floor_t * 0.5)),
        material=shell_black,
        name="floor",
    )
    lower_shell.visual(
        rounded_slab(
            "lower_liner",
            width - 0.040,
            depth - 0.040,
            0.003,
            corner_r - 0.015,
        ),
        origin=Origin(xyz=(0.0, 0.0, floor_t + 0.0015)),
        material=liner_black,
        name="liner",
    )

    wall_h = lower_h - floor_t
    wall_z = floor_t + wall_h * 0.5
    lower_shell.visual(
        Box((width - 2.0 * corner_r, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, depth * 0.5 - wall_t * 0.5, wall_z)),
        material=shell_black,
        name="front_wall",
    )
    lower_shell.visual(
        Box((width - 2.0 * corner_r, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -depth * 0.5 + wall_t * 0.5, wall_z)),
        material=shell_black,
        name="rear_wall",
    )
    lower_shell.visual(
        Box((wall_t, depth - 2.0 * corner_r, wall_h)),
        origin=Origin(xyz=(width * 0.5 - wall_t * 0.5, 0.0, wall_z)),
        material=shell_black,
        name="right_wall",
    )
    lower_shell.visual(
        Box((wall_t, depth - 2.0 * corner_r, wall_h)),
        origin=Origin(xyz=(-width * 0.5 + wall_t * 0.5, 0.0, wall_z)),
        material=shell_black,
        name="left_wall",
    )

    for ix, sx in enumerate((-1.0, 1.0)):
        for iy, sy in enumerate((-1.0, 1.0)):
            lower_shell.visual(
                Cylinder(radius=corner_r, length=wall_h),
                origin=Origin(
                    xyz=(
                        sx * (width * 0.5 - corner_r),
                        sy * (depth * 0.5 - corner_r),
                        wall_z,
                    )
                ),
                material=shell_black,
                name=f"corner_{ix}_{iy}",
            )

    lower_shell.visual(
        Box((width - 0.040, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, hinge_axis_y + 0.004, hinge_axis_z - 0.021)),
        material=trim_dark,
        name="rear_leaf",
    )

    shell_knuckles = [
        (-0.210, 0.120, "shell_knuckle_left"),
        (0.000, 0.100, "shell_knuckle_center"),
        (0.210, 0.120, "shell_knuckle_right"),
    ]
    for x_pos, length, name in shell_knuckles:
        lower_shell.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(
                xyz=(x_pos, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=hardware,
            name=name,
        )

    stay_x = 0.210
    stay_lower_y = -0.118
    stay_lower_z = 0.120
    stay_upper_local_y = 0.082
    stay_upper_local_z = -0.028
    stay_upper_world_y = (
        hinge_axis_y
        + stay_upper_local_y * math.cos(lid_open_angle)
        - stay_upper_local_z * math.sin(lid_open_angle)
    )
    stay_upper_world_z = (
        hinge_axis_z
        + stay_upper_local_y * math.sin(lid_open_angle)
        + stay_upper_local_z * math.cos(lid_open_angle)
    )
    stay_len = math.hypot(
        stay_upper_world_y - stay_lower_y,
        stay_upper_world_z - stay_lower_z,
    )
    stay_open_angle = math.atan2(
        stay_upper_world_z - stay_lower_z,
        stay_upper_world_y - stay_lower_y,
    )

    bracket_plate_t = 0.003
    stay_pad_x = 0.012
    plate_x = stay_pad_x * 0.5 + bracket_plate_t * 0.5
    lower_side_inner_x = width * 0.5 - wall_t
    for side, sx in (("left", -1.0), ("right", 1.0)):
        lower_shell.visual(
            Box(
                (
                    (lower_side_inner_x - stay_x) + stay_pad_x + 0.006,
                    0.004,
                    0.014,
                )
            ),
            origin=Origin(
                xyz=(
                    sx * ((lower_side_inner_x + stay_x) * 0.5),
                    stay_lower_y - 0.010,
                    stay_lower_z - 0.004,
                ),
            ),
            material=hardware,
            name=f"{side}_stay_bracket_web",
        )
        for plate, px in (("inner", -plate_x), ("outer", plate_x)):
            lower_shell.visual(
                Box((bracket_plate_t, 0.016, 0.022)),
                origin=Origin(
                    xyz=(sx * stay_x + px, stay_lower_y, stay_lower_z),
                ),
                material=hardware,
                name=f"{side}_stay_{plate}_plate",
            )

    lower_shell.inertial = Inertial.from_geometry(
        Box((width, depth, lower_h)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, lower_h * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        rounded_slab("lid_roof", width, depth, lid_roof_t, corner_r),
        origin=Origin(xyz=(0.0, depth * 0.5, 0.020)),
        material=shell_black,
        name="roof",
    )

    lid_wall_h = lid_h - lid_roof_t
    lid_wall_z = 0.020 - lid_roof_t * 0.5 - lid_wall_h * 0.5
    lid.visual(
        Box((width - 2.0 * corner_r, wall_t, lid_wall_h)),
        origin=Origin(xyz=(0.0, depth - wall_t * 0.5, lid_wall_z)),
        material=shell_black,
        name="lid_front_wall",
    )
    lid.visual(
        Box((wall_t, depth - 2.0 * corner_r, lid_wall_h)),
        origin=Origin(xyz=(width * 0.5 - wall_t * 0.5, depth * 0.5, lid_wall_z)),
        material=shell_black,
        name="lid_right_wall",
    )
    lid.visual(
        Box((wall_t, depth - 2.0 * corner_r, lid_wall_h)),
        origin=Origin(xyz=(-width * 0.5 + wall_t * 0.5, depth * 0.5, lid_wall_z)),
        material=shell_black,
        name="lid_left_wall",
    )
    for side, sx in (("left", -1.0), ("right", 1.0)):
        lid.visual(
            Cylinder(radius=corner_r, length=lid_wall_h),
            origin=Origin(
                xyz=(sx * (width * 0.5 - corner_r), depth - corner_r, lid_wall_z),
            ),
            material=shell_black,
            name=f"lid_front_corner_{side}",
        )

    lid.visual(
        Box((width - 0.040, 0.016, 0.016)),
        origin=Origin(xyz=(0.0, 0.008, 0.017)),
        material=trim_dark,
        name="lid_rear_rail",
    )

    lid_knuckles = [
        (-0.105, 0.090, "lid_knuckle_left"),
        (0.105, 0.090, "lid_knuckle_right"),
    ]
    for x_pos, length, name in lid_knuckles:
        lid.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(
                xyz=(x_pos, 0.0, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=hardware,
            name=name,
        )

    for side, sx in (("left", -1.0), ("right", 1.0)):
        lid.visual(
            Box(
                (
                    (lower_side_inner_x - stay_x) + stay_pad_x + 0.006,
                    0.014,
                    0.006,
                )
            ),
            origin=Origin(
                xyz=(
                    sx * ((lower_side_inner_x + stay_x) * 0.5),
                    stay_upper_local_y - 0.020,
                    stay_upper_local_z + 0.012,
                )
            ),
            material=hardware,
            name=f"{side}_upper_bridge",
        )
        for plate, px in (("inner", -plate_x), ("outer", plate_x)):
            lid.visual(
                Box((bracket_plate_t, 0.016, 0.018)),
                origin=Origin(
                    xyz=(
                        sx * stay_x + px,
                        stay_upper_local_y,
                        stay_upper_local_z,
                    )
                ),
                material=hardware,
                name=f"{side}_upper_{plate}_plate",
            )

    lid.inertial = Inertial.from_geometry(
        Box((width, depth, lid_h)),
        mass=2.3,
        origin=Origin(xyz=(0.0, depth * 0.5, 0.0)),
    )

    left_stay = model.part("left_stay")
    right_stay = model.part("right_stay")
    for stay_part in (left_stay, right_stay):
        stay_part.visual(
            Box((stay_pad_x, 0.016, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=hardware,
            name="lower_pad",
        )
        stay_part.visual(
            Box((0.006, stay_len - 0.016, 0.004)),
            origin=Origin(xyz=(0.0, stay_len * 0.5, 0.0)),
            material=hardware,
            name="link_bar",
        )
        stay_part.visual(
            Box((stay_pad_x, 0.016, 0.010)),
            origin=Origin(xyz=(0.0, stay_len, 0.0)),
            material=hardware,
            name="upper_pad",
        )
        stay_part.inertial = Inertial.from_geometry(
            Box((stay_pad_x, stay_len, 0.012)),
            mass=0.16,
            origin=Origin(xyz=(0.0, stay_len * 0.5, 0.0)),
        )

    model.articulation(
        "lower_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(
            xyz=(0.0, hinge_axis_y, hinge_axis_z),
            rpy=(lid_open_angle, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-lid_open_angle,
            upper=math.radians(12.0),
        ),
    )
    model.articulation(
        "lower_to_left_stay",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=left_stay,
        origin=Origin(
            xyz=(-stay_x, stay_lower_y, stay_lower_z),
            rpy=(stay_open_angle, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=-1.10,
            upper=0.25,
        ),
    )
    model.articulation(
        "lower_to_right_stay",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=right_stay,
        origin=Origin(
            xyz=(stay_x, stay_lower_y, stay_lower_z),
            rpy=(stay_open_angle, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=-1.10,
            upper=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    left_stay = object_model.get_part("left_stay")
    right_stay = object_model.get_part("right_stay")
    lid_hinge = object_model.get_articulation("lower_to_lid")
    left_stay_pivot = object_model.get_articulation("lower_to_left_stay")
    right_stay_pivot = object_model.get_articulation("lower_to_right_stay")

    ctx.check(
        "lid_hinge_axis_is_horizontal",
        tuple(lid_hinge.axis) == (1.0, 0.0, 0.0),
        f"Unexpected lid hinge axis: {lid_hinge.axis}",
    )
    ctx.check(
        "stay_axes_are_horizontal",
        tuple(left_stay_pivot.axis) == (1.0, 0.0, 0.0)
        and tuple(right_stay_pivot.axis) == (1.0, 0.0, 0.0),
        f"Stay axes: left={left_stay_pivot.axis}, right={right_stay_pivot.axis}",
    )

    ctx.expect_contact(lower_shell, lid, contact_tol=0.0025, name="lid_connected_at_hinge")
    ctx.expect_contact(
        left_stay,
        lower_shell,
        contact_tol=0.0015,
        name="left_stay_connected_to_shell",
    )
    ctx.expect_contact(
        right_stay,
        lower_shell,
        contact_tol=0.0015,
        name="right_stay_connected_to_shell",
    )
    ctx.expect_contact(
        left_stay,
        lid,
        contact_tol=0.0015,
        name="left_stay_supports_lid",
    )
    ctx.expect_contact(
        right_stay,
        lid,
        contact_tol=0.0015,
        name="right_stay_supports_lid",
    )
    ctx.expect_overlap(
        lid,
        lower_shell,
        axes="x",
        min_overlap=0.55,
        name="lid_spans_case_width",
    )

    def aabb_center(aabb):
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    open_roof_aabb = ctx.part_element_world_aabb(lid, elem="roof")
    left_open_upper = ctx.part_element_world_aabb(left_stay, elem="upper_pad")
    right_open_upper = ctx.part_element_world_aabb(right_stay, elem="upper_pad")

    if open_roof_aabb is None or left_open_upper is None or right_open_upper is None:
        ctx.fail("named_visuals_resolve", "Expected roof and stay upper pad visuals to resolve.")
        return ctx.report()

    open_roof_center = aabb_center(open_roof_aabb)
    left_open_center = aabb_center(left_open_upper)
    right_open_center = aabb_center(right_open_upper)

    with ctx.pose({lid_hinge: -math.radians(55.0)}):
        closed_roof_aabb = ctx.part_element_world_aabb(lid, elem="roof")
        if closed_roof_aabb is None:
            ctx.fail("lid_roof_pose_query", "Roof AABB unavailable in near-closed pose.")
        else:
            closed_roof_center = aabb_center(closed_roof_aabb)
            ctx.check(
                "lid_rotates_down_toward_case",
                closed_roof_center[2] < open_roof_center[2] - 0.12
                and closed_roof_center[1] > open_roof_center[1] + 0.09,
                f"Open roof center={open_roof_center}, near-closed roof center={closed_roof_center}",
            )
            ctx.expect_overlap(
                lid,
                lower_shell,
                axes="xy",
                min_overlap=0.22,
                name="near_closed_lid_covers_lower_shell",
            )

    with ctx.pose({left_stay_pivot: -0.85}):
        left_folded_upper = ctx.part_element_world_aabb(left_stay, elem="upper_pad")
        if left_folded_upper is None:
            ctx.fail("left_stay_pose_query", "Left stay upper pad AABB unavailable.")
        else:
            left_folded_center = aabb_center(left_folded_upper)
            ctx.check(
                "left_stay_rotates_about_shell_pivot",
                left_folded_center[2] < left_open_center[2] - 0.02
                and (left_folded_center[1] - left_open_center[1]) > 0.08,
                f"Open={left_open_center}, folded={left_folded_center}",
            )

    with ctx.pose({right_stay_pivot: -0.85}):
        right_folded_upper = ctx.part_element_world_aabb(right_stay, elem="upper_pad")
        if right_folded_upper is None:
            ctx.fail("right_stay_pose_query", "Right stay upper pad AABB unavailable.")
        else:
            right_folded_center = aabb_center(right_folded_upper)
            ctx.check(
                "right_stay_rotates_about_shell_pivot",
                right_folded_center[2] < right_open_center[2] - 0.02
                and (right_folded_center[1] - right_open_center[1]) > 0.08,
                f"Open={right_open_center}, folded={right_folded_center}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="violin_case")

    shell_black = model.material("shell_black", rgba=(0.10, 0.11, 0.12, 1.0))
    trim_black = model.material("trim_black", rgba=(0.05, 0.05, 0.06, 1.0))
    hardware = model.material("hardware", rgba=(0.74, 0.75, 0.78, 1.0))
    plush = model.material("plush", rgba=(0.38, 0.10, 0.12, 1.0))
    plush_dark = model.material("plush_dark", rgba=(0.28, 0.06, 0.08, 1.0))
    leather = model.material("leather", rgba=(0.20, 0.12, 0.08, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    case_length = 0.82
    lower_depth = 0.075
    upper_depth = 0.055
    skin_thickness = 0.004
    hinge_x = -0.148
    front_x = 0.149

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _case_outline() -> list[tuple[float, float]]:
        right_stations = [
            (0.032, -0.410),
            (0.070, -0.372),
            (0.128, -0.310),
            (0.146, -0.230),
            (0.124, -0.145),
            (0.098, -0.070),
            (0.100, 0.000),
            (0.128, 0.095),
            (0.121, 0.165),
            (0.090, 0.245),
            (0.064, 0.328),
            (0.052, 0.390),
            (0.028, 0.410),
        ]
        right = sample_catmull_rom_spline_2d(right_stations, samples_per_segment=8, closed=False)
        left = [(-x, y) for x, y in reversed(right[1:-1])]
        return right + left

    def _scale_profile(
        profile: list[tuple[float, float]],
        *,
        sx: float = 1.0,
        sy: float = 1.0,
        dx: float = 0.0,
        dy: float = 0.0,
    ) -> list[tuple[float, float]]:
        return [(x * sx + dx, y * sy + dy) for x, y in profile]

    def _section(
        profile: list[tuple[float, float]],
        *,
        z: float,
        x_shift: float = 0.0,
        y_shift: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(x + x_shift, y + y_shift, z) for x, y in profile]

    outer_profile = _case_outline()
    inner_profile = _scale_profile(outer_profile, sx=0.86, sy=0.935)
    lid_top_profile = _scale_profile(outer_profile, sx=0.985, sy=0.992)
    lid_inner_ceiling = _scale_profile(inner_profile, sx=0.985, sy=0.992)

    lower_shell = model.part("lower_shell")
    lower_shell.inertial = Inertial.from_geometry(
        Box((0.31, case_length, lower_depth)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, lower_depth * 0.5)),
    )

    lower_bottom = ExtrudeGeometry.from_z0(outer_profile, skin_thickness, cap=True, closed=True)
    lower_walls = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        lower_depth - skin_thickness,
        cap=False,
        center=False,
        closed=True,
    ).translate(0.0, 0.0, skin_thickness)
    lower_floor = ExtrudeGeometry.from_z0(inner_profile, 0.003, cap=True, closed=True).translate(
        0.0,
        0.0,
        skin_thickness,
    )

    lower_shell.visual(_save_mesh("lower_shell_bottom", lower_bottom), material=shell_black, name="lower_bottom")
    lower_shell.visual(_save_mesh("lower_shell_walls", lower_walls), material=shell_black, name="lower_wall_ring")
    lower_shell.visual(_save_mesh("lower_shell_floor", lower_floor), material=plush, name="lower_floor")
    lower_shell.visual(
        Box((0.020, case_length * 0.76, 0.018)),
        origin=Origin(xyz=(hinge_x + 0.014, 0.0, lower_depth - 0.009)),
        material=trim_black,
        name="rear_hinge_mount",
    )
    lower_shell.visual(
        Cylinder(radius=0.005, length=case_length * 0.74),
        origin=Origin(xyz=(hinge_x, 0.0, lower_depth), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="rear_hinge_spine",
    )

    lower_shell.visual(
        Box((0.060, 0.250, 0.018)),
        origin=Origin(xyz=(0.112, 0.0, 0.046)),
        material=trim_black,
        name="front_mount_backing",
    )
    for name, y_pos in [
        ("left_latch_ear_outer", -0.139),
        ("left_latch_ear_inner", -0.105),
        ("right_latch_ear_inner", 0.105),
        ("right_latch_ear_outer", 0.139),
    ]:
        lower_shell.visual(
            Box((0.014, 0.007, 0.018)),
            origin=Origin(xyz=(0.138, y_pos, 0.055)),
            material=trim_black,
            name=name,
        )
    for name, y_pos in [
        ("handle_ear_left", -0.095),
        ("handle_ear_right", 0.095),
    ]:
        lower_shell.visual(
            Box((0.014, 0.014, 0.016)),
            origin=Origin(xyz=(0.138, y_pos, 0.043)),
            material=trim_black,
            name=name,
        )

    lower_shell.visual(
        Cylinder(radius=0.0045, length=0.030),
        origin=Origin(xyz=(front_x, -0.122, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_latch_axle",
    )
    lower_shell.visual(
        Cylinder(radius=0.0045, length=0.030),
        origin=Origin(xyz=(front_x, 0.122, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_latch_axle",
    )
    lower_shell.visual(
        Cylinder(radius=0.0045, length=0.185),
        origin=Origin(xyz=(front_x, 0.0, 0.043), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="handle_axle",
    )

    lower_shell.visual(
        Box((0.180, 0.135, 0.022)),
        origin=Origin(xyz=(0.0, -0.190, 0.015)),
        material=plush_dark,
        name="lower_body_pad",
    )
    lower_shell.visual(
        Box((0.156, 0.120, 0.020)),
        origin=Origin(xyz=(0.0, 0.000, 0.014)),
        material=plush_dark,
        name="upper_body_pad",
    )
    lower_shell.visual(
        Box((0.092, 0.072, 0.032)),
        origin=Origin(xyz=(0.0, 0.205, 0.020)),
        material=plush_dark,
        name="neck_cradle",
    )
    lower_shell.visual(
        Box((0.074, 0.085, 0.026)),
        origin=Origin(xyz=(0.0, 0.330, 0.017)),
        material=plush_dark,
        name="scroll_support",
    )
    lower_shell.visual(
        Box((0.092, 0.095, 0.034)),
        origin=Origin(xyz=(-0.050, -0.100, 0.021)),
        material=plush_dark,
        name="accessory_box",
    )
    for foot_name, x_pos, y_pos in [
        ("foot_tail_left", -0.085, -0.350),
        ("foot_tail_right", 0.085, -0.350),
        ("foot_mid_left", -0.090, 0.040),
        ("foot_mid_right", 0.090, 0.040),
    ]:
        lower_shell.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, -0.003)),
            material=rubber,
            name=foot_name,
        )

    upper_lid = model.part("upper_lid")
    upper_lid.inertial = Inertial.from_geometry(
        Box((0.31, case_length, upper_depth)),
        mass=3.1,
        origin=Origin(xyz=(-hinge_x + 0.145, 0.0, upper_depth * 0.5)),
    )

    lid_profile_local = _scale_profile(outer_profile, dx=-hinge_x)
    lid_inner_local = _scale_profile(inner_profile, dx=-hinge_x)
    lid_top_local = _scale_profile(lid_top_profile, dx=-hinge_x)
    lid_ceiling_local = _scale_profile(lid_inner_ceiling, dx=-hinge_x)

    lid_walls = ExtrudeWithHolesGeometry(
        lid_profile_local,
        [lid_inner_local],
        upper_depth - skin_thickness,
        cap=False,
        center=False,
        closed=True,
    )
    lid_top_skin = section_loft(
        [
            _section(lid_profile_local, z=upper_depth - 0.008),
            _section(_scale_profile(lid_profile_local, sx=0.992, sy=0.994), z=upper_depth - 0.004),
            _section(lid_top_local, z=upper_depth),
        ]
    )
    lid_lining = ExtrudeGeometry.from_z0(lid_ceiling_local, 0.003, cap=True, closed=True).translate(
        0.0,
        0.0,
        upper_depth - 0.010,
    )

    upper_lid.visual(_save_mesh("upper_lid_walls", lid_walls), material=shell_black, name="lid_wall_ring")
    upper_lid.visual(_save_mesh("upper_lid_top", lid_top_skin), material=shell_black, name="lid_top_skin")
    upper_lid.visual(_save_mesh("upper_lid_lining", lid_lining), material=plush, name="lid_lining")
    upper_lid.visual(
        Box((0.014, case_length * 0.76, 0.014)),
        origin=Origin(xyz=(0.015, 0.0, 0.007)),
        material=trim_black,
        name="lid_hinge_mount",
    )
    upper_lid.visual(
        Cylinder(radius=0.0055, length=case_length * 0.74),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="rear_hinge_knuckle",
    )

    left_strike_local_x = 0.145 - hinge_x
    right_strike_local_x = left_strike_local_x
    upper_lid.visual(
        Box((0.016, 0.270, 0.018)),
        origin=Origin(xyz=(left_strike_local_x - 0.009, 0.0, 0.014)),
        material=trim_black,
        name="front_strike_rail",
    )
    upper_lid.visual(
        Box((0.008, 0.028, 0.008)),
        origin=Origin(xyz=(left_strike_local_x, -0.122, 0.016)),
        material=hardware,
        name="left_strike_plate",
    )
    upper_lid.visual(
        Box((0.008, 0.028, 0.008)),
        origin=Origin(xyz=(right_strike_local_x, 0.122, 0.016)),
        material=hardware,
        name="right_strike_plate",
    )

    left_latch = model.part("left_latch")
    left_latch.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="pivot_barrel",
    )
    left_latch.visual(
        Box((0.014, 0.024, 0.028)),
        origin=Origin(xyz=(0.007, 0.0, 0.014)),
        material=hardware,
        name="toggle_body",
    )
    left_latch.visual(
        Box((0.010, 0.024, 0.008)),
        origin=Origin(xyz=(-0.003, 0.0, 0.028)),
        material=hardware,
        name="hook_tip",
    )
    left_latch.inertial = Inertial.from_geometry(
        Box((0.024, 0.024, 0.036)),
        mass=0.05,
        origin=Origin(xyz=(0.004, 0.0, 0.018)),
    )

    right_latch = model.part("right_latch")
    right_latch.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="pivot_barrel",
    )
    right_latch.visual(
        Box((0.014, 0.024, 0.028)),
        origin=Origin(xyz=(0.007, 0.0, 0.014)),
        material=hardware,
        name="toggle_body",
    )
    right_latch.visual(
        Box((0.010, 0.024, 0.008)),
        origin=Origin(xyz=(-0.003, 0.0, 0.028)),
        material=hardware,
        name="hook_tip",
    )
    right_latch.inertial = Inertial.from_geometry(
        Box((0.024, 0.024, 0.036)),
        mass=0.05,
        origin=Origin(xyz=(0.004, 0.0, 0.018)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0055, length=0.176),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="pivot_sleeve",
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.156),
        origin=Origin(xyz=(0.008, 0.0, -0.028), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=leather,
        name="grip",
    )
    handle.visual(
        Box((0.014, 0.014, 0.026)),
        origin=Origin(xyz=(0.006, -0.072, -0.014)),
        material=leather,
        name="left_connector",
    )
    handle.visual(
        Box((0.014, 0.014, 0.026)),
        origin=Origin(xyz=(0.006, 0.072, -0.014)),
        material=leather,
        name="right_connector",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.030, 0.176, 0.050)),
        mass=0.16,
        origin=Origin(xyz=(0.006, 0.0, -0.016)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=upper_lid,
        origin=Origin(xyz=(hinge_x, 0.0, lower_depth)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-1.40,
            upper=0.0,
        ),
    )
    model.articulation(
        "left_latch_joint",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=left_latch,
        origin=Origin(xyz=(front_x, -0.122, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "right_latch_joint",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=right_latch,
        origin=Origin(xyz=(front_x, 0.122, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "handle_joint",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=handle,
        origin=Origin(xyz=(front_x, 0.0, 0.043)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-1.20,
            upper=0.10,
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
    lower_shell = object_model.get_part("lower_shell")
    upper_lid = object_model.get_part("upper_lid")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")
    handle = object_model.get_part("handle")

    lid_hinge = object_model.get_articulation("lid_hinge")
    left_latch_joint = object_model.get_articulation("left_latch_joint")
    right_latch_joint = object_model.get_articulation("right_latch_joint")
    handle_joint = object_model.get_articulation("handle_joint")

    ctx.allow_overlap(
        lower_shell,
        upper_lid,
        elem_a="rear_hinge_spine",
        elem_b="rear_hinge_knuckle",
        reason="Rear piano-hinge barrels share the hinge axis.",
    )
    ctx.allow_overlap(
        lower_shell,
        left_latch,
        elem_a="left_latch_axle",
        elem_b="pivot_barrel",
        reason="Latch rotates around its fixed axle.",
    )
    ctx.allow_overlap(
        lower_shell,
        right_latch,
        elem_a="right_latch_axle",
        elem_b="pivot_barrel",
        reason="Latch rotates around its fixed axle.",
    )
    ctx.allow_overlap(
        lower_shell,
        handle,
        elem_a="handle_axle",
        elem_b="pivot_sleeve",
        reason="Handle sleeve wraps the carry-handle axle.",
    )

    ctx.fail_if_parts_overlap_in_current_pose()

    for joint_name, joint in [
        ("lid_hinge_axis", lid_hinge),
        ("left_latch_axis", left_latch_joint),
        ("right_latch_axis", right_latch_joint),
        ("handle_axis", handle_joint),
    ]:
        ctx.check(joint_name, tuple(joint.axis) == (0.0, 1.0, 0.0), details=f"unexpected axis {joint.axis}")

    with ctx.pose({lid_hinge: 0.0, left_latch_joint: 0.0, right_latch_joint: 0.0, handle_joint: 0.0}):
        ctx.expect_gap(
            upper_lid,
            lower_shell,
            axis="z",
            positive_elem="lid_wall_ring",
            negative_elem="lower_wall_ring",
            max_gap=0.001,
            max_penetration=0.0,
            name="lid_seats_on_lower_rim",
        )
        ctx.expect_overlap(
            upper_lid,
            lower_shell,
            axes="xy",
            elem_a="lid_wall_ring",
            elem_b="lower_wall_ring",
            min_overlap=0.12,
            name="lid_matches_lower_footprint",
        )
        ctx.expect_contact(
            left_latch,
            upper_lid,
            elem_a="hook_tip",
            elem_b="left_strike_plate",
            name="left_latch_hooks_lid",
        )
        ctx.expect_contact(
            right_latch,
            upper_lid,
            elem_a="hook_tip",
            elem_b="right_strike_plate",
            name="right_latch_hooks_lid",
        )
        ctx.expect_contact(handle, lower_shell, name="handle_is_mounted")
        ctx.expect_contact(left_latch, lower_shell, name="left_latch_is_mounted")
        ctx.expect_contact(right_latch, lower_shell, name="right_latch_is_mounted")

    lower_lid_aabb = ctx.part_world_aabb(upper_lid)
    ctx.check("lid_rest_aabb_exists", lower_lid_aabb is not None, details="upper lid AABB unavailable")

    with ctx.pose(
        {
            lid_hinge: lid_hinge.motion_limits.lower,
            left_latch_joint: left_latch_joint.motion_limits.upper,
            right_latch_joint: right_latch_joint.motion_limits.upper,
            handle_joint: 0.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_open_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_open_pose_no_floating")
        open_lid_aabb = ctx.part_world_aabb(upper_lid)
        ok = lower_lid_aabb is not None and open_lid_aabb is not None and open_lid_aabb[1][2] > lower_lid_aabb[1][2] + 0.14
        ctx.check(
            "lid_opens_upward",
            ok,
            details=f"rest={lower_lid_aabb}, open={open_lid_aabb}",
        )

    left_rest = ctx.part_element_world_aabb(left_latch, elem="hook_tip")
    with ctx.pose({left_latch_joint: left_latch_joint.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="left_latch_open_no_overlap")
        ctx.fail_if_isolated_parts(name="left_latch_open_no_floating")
        left_open = ctx.part_element_world_aabb(left_latch, elem="hook_tip")
        ok = (
            left_rest is not None
            and left_open is not None
            and left_open[1][0] > left_rest[1][0] + 0.010
        )
        ctx.check("left_latch_swings_outward", ok, details=f"rest={left_rest}, open={left_open}")

    right_rest = ctx.part_element_world_aabb(right_latch, elem="hook_tip")
    with ctx.pose({right_latch_joint: right_latch_joint.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="right_latch_open_no_overlap")
        ctx.fail_if_isolated_parts(name="right_latch_open_no_floating")
        right_open = ctx.part_element_world_aabb(right_latch, elem="hook_tip")
        ok = (
            right_rest is not None
            and right_open is not None
            and right_open[1][0] > right_rest[1][0] + 0.010
        )
        ctx.check("right_latch_swings_outward", ok, details=f"rest={right_rest}, open={right_open}")

    handle_rest = ctx.part_element_world_aabb(handle, elem="grip")
    with ctx.pose({handle_joint: handle_joint.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="handle_lifted_no_overlap")
        ctx.fail_if_isolated_parts(name="handle_lifted_no_floating")
        handle_open = ctx.part_element_world_aabb(handle, elem="grip")
        ok = (
            handle_rest is not None
            and handle_open is not None
            and handle_open[1][0] > handle_rest[1][0] + 0.015
        )
        ctx.check("handle_pivots_outward", ok, details=f"rest={handle_rest}, open={handle_open}")

    left_pos = ctx.part_world_position(left_latch)
    handle_pos = ctx.part_world_position(handle)
    right_pos = ctx.part_world_position(right_latch)
    ordered = (
        left_pos is not None
        and handle_pos is not None
        and right_pos is not None
        and left_pos[1] < handle_pos[1] < right_pos[1]
    )
    ctx.check(
        "front_hardware_layout",
        ordered,
        details=f"left={left_pos}, handle={handle_pos}, right={right_pos}",
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

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

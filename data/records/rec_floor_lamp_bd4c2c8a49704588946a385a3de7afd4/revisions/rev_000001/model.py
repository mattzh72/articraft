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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_floor_lamp")

    base_finish = model.material("base_finish", rgba=(0.17, 0.16, 0.15, 1.0))
    metal_finish = model.material("metal_finish", rgba=(0.67, 0.68, 0.70, 1.0))
    joint_finish = model.material("joint_finish", rgba=(0.26, 0.27, 0.29, 1.0))
    shade_finish = model.material("shade_finish", rgba=(0.92, 0.88, 0.76, 1.0))

    base_length = 0.34
    base_width = 0.24
    base_height = 0.035
    top_plate_height = 0.007
    socket_height = 0.012

    column_height = 1.22
    column_radius = 0.016
    sleeve_height = 0.02
    top_cap_height = 0.03

    boom_length = 0.70
    boom_radius = 0.013
    boom_hub_height = 0.028

    base = model.part("base")
    base.visual(
        Box((base_length, base_width, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=base_finish,
        name="weighted_block",
    )
    base.visual(
        Box((0.30, 0.20, top_plate_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height + (top_plate_height * 0.5))),
        material=base_finish,
        name="base_top",
    )
    base.visual(
        Cylinder(radius=0.03, length=socket_height),
        origin=Origin(
            xyz=(0.0, 0.0, base_height + top_plate_height + (socket_height * 0.5))
        ),
        material=joint_finish,
        name="column_socket",
    )
    base.inertial = Inertial.from_geometry(
        Box((base_length, base_width, base_height + top_plate_height + socket_height)),
        mass=8.5,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (base_height + top_plate_height + socket_height) * 0.5,
            )
        ),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.025, length=sleeve_height),
        origin=Origin(xyz=(0.0, 0.0, sleeve_height * 0.5)),
        material=joint_finish,
        name="lower_sleeve",
    )
    column.visual(
        Cylinder(radius=column_radius, length=column_height),
        origin=Origin(xyz=(0.0, 0.0, sleeve_height + (column_height * 0.5))),
        material=metal_finish,
        name="column_tube",
    )
    column.visual(
        Cylinder(radius=0.028, length=top_cap_height),
        origin=Origin(
            xyz=(0.0, 0.0, sleeve_height + column_height + (top_cap_height * 0.5))
        ),
        material=joint_finish,
        name="top_cap",
    )
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.03, length=sleeve_height + column_height + top_cap_height),
        mass=2.4,
        origin=Origin(
            xyz=(0.0, 0.0, (sleeve_height + column_height + top_cap_height) * 0.5)
        ),
    )

    boom = model.part("boom")
    boom_tube_length = 0.60
    boom.visual(
        Cylinder(radius=0.024, length=boom_hub_height),
        origin=Origin(xyz=(0.0, 0.0, boom_hub_height * 0.5)),
        material=joint_finish,
        name="pivot_hub",
    )
    boom.visual(
        Cylinder(radius=boom_radius, length=boom_tube_length),
        origin=Origin(
            xyz=(boom_tube_length * 0.5, 0.0, boom_hub_height * 0.5),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=metal_finish,
        name="boom_tube",
    )
    boom.visual(
        Box((0.186, 0.028, 0.024)),
        origin=Origin(xyz=(0.600, 0.0, boom_hub_height * 0.5)),
        material=joint_finish,
        name="hinge_block",
    )
    boom.inertial = Inertial.from_geometry(
        Box((boom_length + 0.03, 0.06, 0.06)),
        mass=0.85,
        origin=Origin(xyz=(boom_length * 0.5, 0.0, boom_hub_height * 0.5)),
    )

    shade_outer_profile = [
        (0.014, 0.0),
        (0.040, -0.040),
        (0.076, -0.190),
    ]
    shade_inner_profile = [
        (0.0, 0.0),
        (0.034, -0.038),
        (0.070, -0.186),
    ]
    shade_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            shade_outer_profile,
            shade_inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        "lamp_shade_shell",
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=joint_finish,
        name="tilt_barrel",
    )
    shade.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=joint_finish,
        name="support_arm",
    )
    shade.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(0.050, 0.0, -0.006)),
        material=joint_finish,
        name="top_collar",
    )
    shade.visual(
        shade_shell_mesh,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=shade_finish,
        name="shade_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.18, 0.16, 0.22)),
        mass=0.35,
        origin=Origin(xyz=(0.050, 0.0, -0.09)),
    )

    base_to_column_height = base_height + top_plate_height + socket_height
    column_to_boom_height = sleeve_height + column_height + top_cap_height

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, base_to_column_height)),
    )
    model.articulation(
        "column_to_boom_swing",
        ArticulationType.REVOLUTE,
        parent=column,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, column_to_boom_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=-1.35,
            upper=1.35,
        ),
    )
    model.articulation(
        "boom_to_shade_tilt",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=shade,
        origin=Origin(xyz=(boom_length, 0.0, boom_hub_height * 0.5)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=-1.0,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    boom = object_model.get_part("boom")
    shade = object_model.get_part("shade")

    boom_swing = object_model.get_articulation("column_to_boom_swing")
    shade_tilt = object_model.get_articulation("boom_to_shade_tilt")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return (
            (lower[0] + upper[0]) * 0.5,
            (lower[1] + upper[1]) * 0.5,
            (lower[2] + upper[2]) * 0.5,
        )

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

    ctx.expect_contact(base, column, elem_a="column_socket", elem_b="lower_sleeve")
    ctx.expect_contact(column, boom, elem_a="top_cap", elem_b="pivot_hub")
    ctx.expect_contact(boom, shade, elem_a="hinge_block", elem_b="tilt_barrel")

    ctx.expect_within(column, base, axes="xy", margin=0.0)
    ctx.expect_overlap(boom, column, axes="xy", min_overlap=0.04, elem_a="pivot_hub", elem_b="top_cap")
    ctx.expect_gap(shade, base, axis="z", min_gap=1.0, positive_elem="shade_shell", negative_elem="base_top")
    ctx.expect_origin_gap(column, base, axis="z", min_gap=0.053, max_gap=0.055)

    shade_rest_pos = ctx.part_world_position(shade)
    with ctx.pose({boom_swing: 1.10}):
        shade_left_pos = ctx.part_world_position(shade)
        ctx.check(
            "boom_swings_laterally_positive",
            shade_rest_pos is not None
            and shade_left_pos is not None
            and shade_left_pos[1] > 0.60
            and shade_left_pos[0] < shade_rest_pos[0] - 0.35,
            details=f"rest={shade_rest_pos}, swung={shade_left_pos}",
        )
        ctx.expect_contact(column, boom, elem_a="top_cap", elem_b="pivot_hub")
    with ctx.pose({boom_swing: -1.10}):
        shade_right_pos = ctx.part_world_position(shade)
        ctx.check(
            "boom_swings_laterally_negative",
            shade_rest_pos is not None
            and shade_right_pos is not None
            and shade_right_pos[1] < -0.60
            and shade_right_pos[0] < shade_rest_pos[0] - 0.35,
            details=f"rest={shade_rest_pos}, swung={shade_right_pos}",
        )
        ctx.expect_contact(column, boom, elem_a="top_cap", elem_b="pivot_hub")

    shade_rest_shell = ctx.part_element_world_aabb(shade, elem="shade_shell")
    shade_rest_center = _aabb_center(shade_rest_shell)
    with ctx.pose({shade_tilt: 0.45}):
        shade_up_shell = ctx.part_element_world_aabb(shade, elem="shade_shell")
        shade_up_center = _aabb_center(shade_up_shell)
        ctx.check(
            "shade_tilts_forward",
            shade_rest_center is not None
            and shade_up_center is not None
            and shade_up_center[0] < shade_rest_center[0] - 0.035
            and shade_up_center[2] < shade_rest_center[2] - 0.008,
            details=f"rest={shade_rest_shell}, tilted={shade_up_shell}",
        )
        ctx.expect_contact(boom, shade, elem_a="hinge_block", elem_b="tilt_barrel")
    with ctx.pose({shade_tilt: -0.75}):
        shade_down_shell = ctx.part_element_world_aabb(shade, elem="shade_shell")
        shade_down_center = _aabb_center(shade_down_shell)
        ctx.check(
            "shade_tilts_back",
            shade_rest_center is not None
            and shade_down_center is not None
            and shade_down_center[0] > shade_rest_center[0] + 0.05
            and shade_down_center[2] > shade_rest_center[2] + 0.02,
            details=f"rest={shade_rest_shell}, tilted={shade_down_shell}",
        )
        ctx.expect_contact(boom, shade, elem_a="hinge_block", elem_b="tilt_barrel")

    boom_limits = boom_swing.motion_limits
    if boom_limits is not None and boom_limits.lower is not None and boom_limits.upper is not None:
        with ctx.pose({boom_swing: boom_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="boom_swing_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="boom_swing_lower_no_floating")
        with ctx.pose({boom_swing: boom_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="boom_swing_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="boom_swing_upper_no_floating")

    shade_limits = shade_tilt.motion_limits
    if shade_limits is not None and shade_limits.lower is not None and shade_limits.upper is not None:
        with ctx.pose({shade_tilt: shade_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="shade_tilt_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="shade_tilt_lower_no_floating")
            ctx.expect_gap(
                shade,
                base,
                axis="z",
                min_gap=0.95,
                positive_elem="shade_shell",
                negative_elem="base_top",
                name="shade_tilt_lower_clear_of_base",
            )
        with ctx.pose({shade_tilt: shade_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="shade_tilt_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="shade_tilt_upper_no_floating")

    with ctx.pose({boom_swing: 1.10, shade_tilt: -0.60}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=False,
    )
    ctx.fail_if_isolated_parts(max_pose_samples=12, name="lamp_sampled_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

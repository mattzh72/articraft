from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Cylinder,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    shell = model.material("shell", rgba=(0.90, 0.90, 0.88, 1.0))
    trim = model.material("trim", rgba=(0.82, 0.82, 0.79, 1.0))
    display_tint = model.material("display_tint", rgba=(0.12, 0.18, 0.22, 1.0))
    rocker_finish = model.material("rocker_finish", rgba=(0.95, 0.95, 0.92, 1.0))
    cover_finish = model.material("cover_finish", rgba=(0.93, 0.93, 0.90, 1.0))
    button_finish = model.material("button_finish", rgba=(0.72, 0.77, 0.82, 1.0))
    shadow = model.material("shadow", rgba=(0.55, 0.57, 0.58, 1.0))

    width = 0.086
    height = 0.122
    depth = 0.024
    rear_plate_depth = 0.004
    shell_depth = depth - rear_plate_depth

    body = model.part("body")
    body.visual(
        Box((width, rear_plate_depth, height)),
        origin=Origin(xyz=(0.0, rear_plate_depth / 2.0, 0.0)),
        material=trim,
        name="rear_plate",
    )
    body.visual(
        Box((width, shell_depth, 0.016)),
        origin=Origin(xyz=(0.0, rear_plate_depth + shell_depth / 2.0, 0.053)),
        material=shell,
        name="top_shell",
    )
    body.visual(
        Box((width, shell_depth, 0.014)),
        origin=Origin(xyz=(0.0, rear_plate_depth + shell_depth / 2.0, -0.054)),
        material=shell,
        name="bottom_shell",
    )
    body.visual(
        Box((0.010, shell_depth, 0.092)),
        origin=Origin(xyz=(-0.038, rear_plate_depth + shell_depth / 2.0, -0.001)),
        material=shell,
        name="left_shell",
    )
    body.visual(
        Box((0.010, shell_depth, 0.092)),
        origin=Origin(xyz=(0.038, rear_plate_depth + shell_depth / 2.0, -0.001)),
        material=shell,
        name="right_shell",
    )
    body.visual(
        Box((0.062, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.009, 0.029)),
        material=shadow,
        name="display_recess",
    )
    body.visual(
        Box((0.050, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, 0.012, 0.029)),
        material=display_tint,
        name="display_window",
    )
    body.visual(
        Box((0.056, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, 0.008, -0.017)),
        material=trim,
        name="button_bed",
    )
    body.visual(
        Box((0.060, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.008, 0.002)),
        material=trim,
        name="switch_mount",
    )
    body.visual(
        Box((0.015, 0.004, 0.008)),
        origin=Origin(xyz=(-0.013, 0.014, -0.017)),
        material=button_finish,
        name="set_button_0",
    )
    body.visual(
        Box((0.015, 0.004, 0.008)),
        origin=Origin(xyz=(0.013, 0.014, -0.017)),
        material=button_finish,
        name="set_button_1",
    )

    rocker_width = 0.018
    rocker_height = 0.018
    rocker_depth = 0.005
    rocker_axis_y = 0.018
    rocker_z = 0.002
    rocker_positions = (("rocker_0", -0.021), ("rocker_1", 0.021))

    for rocker_name, rocker_x in rocker_positions:
        body.visual(
            Box((0.004, 0.008, 0.010)),
            origin=Origin(xyz=(rocker_x - 0.010, 0.012, rocker_z)),
            material=trim,
            name=f"{rocker_name}_hinge_post_0",
        )
        body.visual(
            Box((0.004, 0.008, 0.010)),
            origin=Origin(xyz=(rocker_x + 0.010, 0.012, rocker_z)),
            material=trim,
            name=f"{rocker_name}_hinge_post_1",
        )

        rocker = model.part(rocker_name)
        rocker.visual(
            Box((rocker_width, rocker_depth, rocker_height)),
            origin=Origin(xyz=(0.0, rocker_depth / 2.0, 0.0)),
            material=rocker_finish,
            name="rocker_body",
        )
        rocker.visual(
            Box((0.012, 0.0012, 0.005)),
            origin=Origin(xyz=(0.0, 0.0052, 0.0062)),
            material=shadow,
            name="rocker_band",
        )
        rocker.visual(
            Cylinder(radius=0.0014, length=0.014),
            origin=Origin(
                xyz=(0.0, 0.0010, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=trim,
            name="rocker_axle",
        )

        model.articulation(
            f"body_to_{rocker_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=rocker,
            origin=Origin(xyz=(rocker_x, rocker_axis_y, rocker_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=3.0,
                lower=-0.24,
                upper=0.24,
            ),
        )

    cover = model.part("cover")
    cover_width = 0.068
    cover_height = 0.044
    cover_depth = 0.004
    cover_hinge_y = 0.020
    cover_hinge_z = -0.036
    cover.visual(
        Box((cover_width, cover_depth, cover_height)),
        origin=Origin(xyz=(0.0, cover_depth / 2.0, cover_height / 2.0)),
        material=cover_finish,
        name="cover_panel",
    )
    cover.visual(
        Box((0.026, 0.0022, 0.004)),
        origin=Origin(xyz=(0.0, 0.0045, cover_height - 0.004)),
        material=shadow,
        name="pull_lip",
    )
    cover.visual(
        Cylinder(radius=0.0018, length=0.016),
        origin=Origin(
            xyz=(-0.020, cover_depth / 2.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim,
        name="hinge_barrel_0",
    )
    cover.visual(
        Cylinder(radius=0.0018, length=0.016),
        origin=Origin(
            xyz=(0.020, cover_depth / 2.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim,
        name="hinge_barrel_1",
    )

    body.visual(
        Box((0.018, 0.006, 0.008)),
        origin=Origin(xyz=(-0.025, cover_hinge_y, cover_hinge_z)),
        material=trim,
        name="cover_hinge_block_0",
    )
    body.visual(
        Box((0.018, 0.006, 0.008)),
        origin=Origin(xyz=(0.025, cover_hinge_y, cover_hinge_z)),
        material=trim,
        name="cover_hinge_block_1",
    )
    body.visual(
        Box((0.050, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.018, cover_hinge_z - 0.003)),
        material=trim,
        name="cover_sill",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, cover_hinge_y, cover_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.75,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[idx] + high[idx]) / 2.0 for idx in range(3))


def _aabbs_close(aabb_a, aabb_b, tol: float = 1e-6) -> bool:
    if aabb_a is None or aabb_b is None:
        return False
    return all(
        abs(aabb_a[bound][axis] - aabb_b[bound][axis]) <= tol
        for bound in range(2)
        for axis in range(3)
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    rocker_0 = object_model.get_part("rocker_0")
    rocker_1 = object_model.get_part("rocker_1")
    cover_hinge = object_model.get_articulation("body_to_cover")
    rocker_0_hinge = object_model.get_articulation("body_to_rocker_0")
    rocker_1_hinge = object_model.get_articulation("body_to_rocker_1")

    ctx.expect_gap(
        body,
        cover,
        axis="z",
        positive_elem="display_window",
        negative_elem="cover_panel",
        min_gap=0.006,
        name="display stays above the cover",
    )
    ctx.expect_gap(
        cover,
        body,
        axis="y",
        positive_elem="cover_panel",
        negative_elem="button_bed",
        min_gap=0.004,
        max_gap=0.020,
        name="cover closes in front of the setting buttons",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="x",
        elem_a="cover_panel",
        elem_b="button_bed",
        min_overlap=0.050,
        name="cover spans the full button area",
    )

    rocker_0_limits = rocker_0_hinge.motion_limits
    rocker_1_limits = rocker_1_hinge.motion_limits
    cover_limits = cover_hinge.motion_limits

    with ctx.pose({rocker_0_hinge: rocker_0_limits.upper}):
        rocker_0_upper = ctx.part_element_world_aabb(rocker_0, elem="rocker_band")
        rocker_1_rest_from_left_pose = ctx.part_element_world_aabb(rocker_1, elem="rocker_band")
    with ctx.pose({rocker_0_hinge: rocker_0_limits.lower}):
        rocker_0_lower = ctx.part_element_world_aabb(rocker_0, elem="rocker_band")
    rocker_1_rest = ctx.part_element_world_aabb(rocker_1, elem="rocker_band")

    rocker_0_upper_center = _aabb_center(rocker_0_upper)
    rocker_0_lower_center = _aabb_center(rocker_0_lower)
    ctx.check(
        "rocker_0 pivots on its local hinge",
        rocker_0_upper_center is not None
        and rocker_0_lower_center is not None
        and rocker_0_upper_center[1] < rocker_0_lower_center[1] - 0.001,
        details=f"upper={rocker_0_upper_center}, lower={rocker_0_lower_center}",
    )
    ctx.check(
        "rocker_1 stays still when rocker_0 moves",
        _aabbs_close(rocker_1_rest, rocker_1_rest_from_left_pose),
        details=f"rest={rocker_1_rest}, posed={rocker_1_rest_from_left_pose}",
    )

    with ctx.pose({rocker_1_hinge: rocker_1_limits.upper}):
        rocker_1_upper = ctx.part_element_world_aabb(rocker_1, elem="rocker_band")
    with ctx.pose({rocker_1_hinge: rocker_1_limits.lower}):
        rocker_1_lower = ctx.part_element_world_aabb(rocker_1, elem="rocker_band")
    rocker_1_upper_center = _aabb_center(rocker_1_upper)
    rocker_1_lower_center = _aabb_center(rocker_1_lower)
    ctx.check(
        "rocker_1 pivots on its local hinge",
        rocker_1_upper_center is not None
        and rocker_1_lower_center is not None
        and rocker_1_upper_center[1] < rocker_1_lower_center[1] - 0.001,
        details=f"upper={rocker_1_upper_center}, lower={rocker_1_lower_center}",
    )

    closed_lip = ctx.part_element_world_aabb(cover, elem="pull_lip")
    with ctx.pose({cover_hinge: cover_limits.upper}):
        open_lip = ctx.part_element_world_aabb(cover, elem="pull_lip")
    closed_lip_center = _aabb_center(closed_lip)
    open_lip_center = _aabb_center(open_lip)
    ctx.check(
        "cover flips downward below the display",
        closed_lip_center is not None
        and open_lip_center is not None
        and open_lip_center[2] < closed_lip_center[2] - 0.025
        and open_lip_center[1] > closed_lip_center[1] + 0.010,
        details=f"closed={closed_lip_center}, open={open_lip_center}",
    )

    return ctx.report()


object_model = build_object_model()

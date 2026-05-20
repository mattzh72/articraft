from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_X = 0.56
BASE_Y = 0.44
BASE_T = 0.03

MAST_W = 0.10
MAST_D = 0.08
MAST_H = 1.36
MAST_WALL = 0.012

UPPER_SHAFT_Y = 0.07
UPPER_SHAFT_Z = 1.25
LOWER_SHAFT_X = 0.12
LOWER_SHAFT_Z = 0.82

SHAFT_RADIUS = 0.012
SLEEVE_OUTER_RADIUS = 0.020
SLEEVE_BORE_RADIUS = 0.0135


def _combine(*solids):
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _rect_tube(size_x: float, size_y: float, size_z: float, wall: float):
    outer = cq.Workplane("XY").box(size_x, size_y, size_z)
    inner = cq.Workplane("XY").box(
        size_x - 2.0 * wall,
        size_y - 2.0 * wall,
        size_z + 0.004,
    )
    return outer.cut(inner)


def _tube_y(length: float, outer_radius: float, inner_radius: float):
    outer = cq.Workplane("XZ").circle(outer_radius).extrude(length / 2.0, both=True)
    inner = cq.Workplane("XZ").circle(inner_radius).extrude(length / 2.0 + 0.002, both=True)
    return outer.cut(inner)


def _tube_x(length: float, outer_radius: float, inner_radius: float):
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length / 2.0, both=True)
    inner = cq.Workplane("YZ").circle(inner_radius).extrude(length / 2.0 + 0.002, both=True)
    return outer.cut(inner)


def _rod_y(length: float, radius: float):
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def _rod_x(length: float, radius: float):
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True)


def _make_mast_body():
    base_plate = cq.Workplane("XY").box(BASE_X, BASE_Y, BASE_T).translate((0.0, 0.0, BASE_T / 2.0))
    foot_block = cq.Workplane("XY").box(0.18, 0.16, 0.09).translate((0.0, 0.0, BASE_T + 0.045))
    mast_tube = _rect_tube(MAST_W, MAST_D, MAST_H, MAST_WALL).translate(
        (0.0, 0.0, BASE_T + MAST_H / 2.0)
    )

    gusset_front = (
        cq.Workplane("XZ")
        .polyline([(0.0, BASE_T), (0.14, BASE_T), (0.0, BASE_T + 0.22)])
        .close()
        .extrude(0.018)
        .translate((0.0, BASE_Y / 2.0 - 0.009, 0.0))
    )
    gusset_back = gusset_front.mirror("XZ")

    gusset_left = (
        cq.Workplane("YZ")
        .polyline([(0.0, BASE_T), (0.12, BASE_T), (0.0, BASE_T + 0.20)])
        .close()
        .extrude(0.018)
        .translate((BASE_X / 2.0 - 0.009, 0.0, 0.0))
    )
    gusset_right = gusset_left.mirror("YZ")

    return _combine(base_plate, foot_block, mast_tube, gusset_front, gusset_back, gusset_left, gusset_right)


def _make_upper_support():
    mount_plate = cq.Workplane("XY").box(0.11, 0.016, 0.14).translate((0.0, -0.022, 0.0))
    left_cheek = cq.Workplane("XY").box(0.014, 0.058, 0.11).translate((-0.030, 0.0, 0.0))
    right_cheek = cq.Workplane("XY").box(0.014, 0.058, 0.11).translate((0.030, 0.0, 0.0))
    top_bridge = cq.Workplane("XY").box(0.074, 0.024, 0.014).translate((0.0, -0.002, 0.034))
    bottom_bridge = cq.Workplane("XY").box(0.074, 0.024, 0.014).translate((0.0, -0.002, -0.034))
    shaft = _rod_y(0.060, 0.0105)
    return _combine(mount_plate, left_cheek, right_cheek, top_bridge, bottom_bridge, shaft)


def _make_lower_support():
    mount_plate = cq.Workplane("XY").box(0.014, 0.12, 0.14).translate((-0.033, 0.0, 0.0))
    outer_plate = cq.Workplane("XY").box(0.014, 0.11, 0.12).translate((0.033, 0.0, 0.0))
    top_bridge = cq.Workplane("XY").box(0.048, 0.11, 0.014).translate((0.0, 0.0, 0.033))
    bottom_bridge = cq.Workplane("XY").box(0.048, 0.11, 0.014).translate((0.0, 0.0, -0.033))
    shaft = _rod_x(0.080, 0.0105)
    return _combine(mount_plate, outer_plate, top_bridge, bottom_bridge, shaft)


def _make_upper_arm_body():
    root_block = cq.Workplane("XY").box(0.034, 0.024, 0.09).translate((0.0, 0.042, 0.0))
    riser = cq.Workplane("XY").box(0.030, 0.034, 0.12).translate((0.0, 0.075, 0.06))
    beam = cq.Workplane("XY").box(0.032, 0.34, 0.028).translate((0.0, 0.23, 0.11))
    end_block = cq.Workplane("XY").box(0.042, 0.05, 0.042).translate((0.0, 0.405, 0.11))
    brace = (
        cq.Workplane("YZ")
        .polyline([(0.03, 0.0), (0.20, 0.0), (0.07, 0.10)])
        .close()
        .extrude(0.010, both=True)
    )
    return _combine(root_block, riser, beam, end_block, brace)


def _make_upper_tool_pad():
    pad = (
        cq.Workplane("XY")
        .box(0.09, 0.07, 0.016)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.490, 0.11))
    )
    puck = cq.Workplane("XY").circle(0.022).extrude(0.010).translate((0.0, 0.490, 0.118))
    stem = cq.Workplane("XY").box(0.030, 0.060, 0.024).translate((0.0, 0.450, 0.11))
    return _combine(pad, puck, stem)


def _make_side_arm_body():
    root_block = cq.Workplane("XY").box(0.026, 0.06, 0.08).translate((0.053, 0.0, 0.0))
    outboard_beam = cq.Workplane("XY").box(0.32, 0.04, 0.03).translate((0.22, 0.0, 0.0))
    forward_beam = cq.Workplane("XY").box(0.10, 0.34, 0.03).translate((0.38, 0.20, 0.03))
    riser = cq.Workplane("XY").box(0.06, 0.06, 0.08).translate((0.40, 0.37, 0.04))
    brace = (
        cq.Workplane("YZ")
        .polyline([(0.04, -0.01), (0.24, -0.01), (0.08, 0.07)])
        .close()
        .extrude(0.020)
        .translate((0.16, 0.0, 0.0))
    )
    return _combine(root_block, outboard_beam, forward_beam, riser, brace)


def _make_side_tool_pad():
    pad = (
        cq.Workplane("XY")
        .box(0.09, 0.08, 0.018)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.44, 0.47, 0.05))
    )
    puck = cq.Workplane("XY").circle(0.024).extrude(0.010).translate((0.44, 0.47, 0.059))
    neck = cq.Workplane("XY").box(0.060, 0.12, 0.028).translate((0.41, 0.41, 0.05))
    return _combine(pad, puck, neck)


def _aabb_center(aabb):
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((low + high) / 2.0 for low, high in zip(minimum, maximum))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_arm_positioning_tree")

    mast_steel = model.material("mast_steel", rgba=(0.27, 0.29, 0.32, 1.0))
    support_steel = model.material("support_steel", rgba=(0.52, 0.55, 0.58, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.76, 0.43, 0.19, 1.0))
    pad_rubber = model.material("pad_rubber", rgba=(0.11, 0.12, 0.13, 1.0))

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(_make_mast_body(), "mast_body"),
        material=mast_steel,
        name="mast_body",
    )
    mast.visual(
        Box((0.024, 0.12, 0.14)),
        material=support_steel,
        origin=Origin(xyz=(0.062, 0.0, LOWER_SHAFT_Z)),
        name="lower_mount_boss",
    )

    upper_support = model.part("upper_support")
    upper_support.visual(
        mesh_from_cadquery(_make_upper_support(), "upper_support_body"),
        material=support_steel,
        name="support_body",
    )

    lower_support = model.part("lower_support")
    lower_support.visual(
        Box((0.014, 0.12, 0.14)),
        material=support_steel,
        origin=Origin(xyz=(-0.039, 0.0, 0.0)),
        name="mast_mount",
    )
    lower_support.visual(
        Box((0.014, 0.11, 0.12)),
        material=support_steel,
        origin=Origin(xyz=(0.033, 0.0, 0.0)),
        name="outer_cheek",
    )
    lower_support.visual(
        Box((0.058, 0.11, 0.016)),
        material=support_steel,
        origin=Origin(xyz=(-0.003, 0.0, 0.034)),
        name="top_bridge",
    )
    lower_support.visual(
        Box((0.058, 0.11, 0.016)),
        material=support_steel,
        origin=Origin(xyz=(-0.003, 0.0, -0.034)),
        name="bottom_bridge",
    )
    lower_support.visual(
        Cylinder(radius=0.0105, length=0.080),
        material=support_steel,
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        name="support_shaft",
    )

    upper_branch = model.part("upper_branch")
    upper_branch.visual(
        mesh_from_cadquery(_make_upper_arm_body(), "upper_branch_body"),
        material=arm_paint,
        name="arm_body",
    )
    upper_branch.visual(
        mesh_from_cadquery(_make_upper_tool_pad(), "upper_tool_pad"),
        material=pad_rubber,
        name="tool_pad",
    )

    side_branch = model.part("side_branch")
    side_branch.visual(
        mesh_from_cadquery(_make_side_arm_body(), "side_branch_body"),
        material=arm_paint,
        name="arm_body",
    )
    side_branch.visual(
        mesh_from_cadquery(_make_side_tool_pad(), "side_tool_pad"),
        material=pad_rubber,
        name="tool_pad",
    )

    model.articulation(
        "mast_to_upper_support",
        ArticulationType.FIXED,
        parent=mast,
        child=upper_support,
        origin=Origin(xyz=(0.0, UPPER_SHAFT_Y, UPPER_SHAFT_Z)),
    )
    model.articulation(
        "mast_to_lower_support",
        ArticulationType.FIXED,
        parent=mast,
        child=lower_support,
        origin=Origin(xyz=(LOWER_SHAFT_X, 0.0, LOWER_SHAFT_Z)),
    )
    model.articulation(
        "upper_support_to_upper_branch",
        ArticulationType.REVOLUTE,
        parent=upper_support,
        child=upper_branch,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.2,
            lower=-0.6,
            upper=0.85,
        ),
    )
    model.articulation(
        "lower_support_to_side_branch",
        ArticulationType.REVOLUTE,
        parent=lower_support,
        child=side_branch,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=-0.45,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    upper_support = object_model.get_part("upper_support")
    lower_support = object_model.get_part("lower_support")
    upper_branch = object_model.get_part("upper_branch")
    side_branch = object_model.get_part("side_branch")

    mast_body = mast.get_visual("mast_body")
    mast_lower_boss = mast.get_visual("lower_mount_boss")
    upper_support_body = upper_support.get_visual("support_body")
    lower_support_mount = lower_support.get_visual("mast_mount")
    lower_support_shaft = lower_support.get_visual("support_shaft")
    upper_arm_body = upper_branch.get_visual("arm_body")
    upper_tool_pad = upper_branch.get_visual("tool_pad")
    side_arm_body = side_branch.get_visual("arm_body")
    side_tool_pad = side_branch.get_visual("tool_pad")

    upper_joint = object_model.get_articulation("upper_support_to_upper_branch")
    lower_joint = object_model.get_articulation("lower_support_to_side_branch")

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

    ctx.check(
        "upper_branch_axis_is_front_back",
        tuple(round(value, 6) for value in upper_joint.axis) == (0.0, 1.0, 0.0),
        f"expected upper axis (0, 1, 0), got {upper_joint.axis}",
    )
    ctx.check(
        "side_branch_axis_is_left_right",
        tuple(round(value, 6) for value in lower_joint.axis) == (1.0, 0.0, 0.0),
        f"expected side axis (1, 0, 0), got {lower_joint.axis}",
    )

    ctx.expect_contact(
        mast,
        upper_support,
        elem_a=mast_body,
        elem_b=upper_support_body,
        name="upper_support_clamped_to_mast",
    )
    ctx.expect_contact(
        mast,
        lower_support,
        elem_a=mast_lower_boss,
        elem_b=lower_support_mount,
        name="lower_support_clamped_to_mast",
    )
    ctx.expect_contact(
        upper_support,
        upper_branch,
        elem_a=upper_support_body,
        elem_b=upper_arm_body,
        name="upper_branch_supported_on_shaft_module",
    )
    ctx.expect_contact(
        lower_support,
        side_branch,
        elem_a=lower_support_shaft,
        elem_b=side_arm_body,
        name="side_branch_supported_on_shaft_module",
    )

    ctx.expect_gap(
        upper_branch,
        mast,
        axis="y",
        min_gap=0.22,
        positive_elem=upper_tool_pad,
        negative_elem=mast_body,
        name="upper_tool_pad_projects_forward",
    )
    ctx.expect_gap(
        side_branch,
        mast,
        axis="x",
        min_gap=0.16,
        positive_elem=side_tool_pad,
        negative_elem=mast_body,
        name="side_tool_pad_projects_outboard",
    )
    ctx.expect_origin_gap(
        upper_support,
        lower_support,
        axis="z",
        min_gap=0.35,
        name="two_modules_are_staggered_on_mast",
    )

    upper_rest_center = _aabb_center(ctx.part_element_world_aabb(upper_branch, elem=upper_tool_pad))
    side_rest_center = _aabb_center(ctx.part_element_world_aabb(side_branch, elem=side_tool_pad))

    with ctx.pose({upper_joint: 0.55}):
        upper_posed_center = _aabb_center(ctx.part_element_world_aabb(upper_branch, elem=upper_tool_pad))
    with ctx.pose({lower_joint: 0.65}):
        side_posed_center = _aabb_center(ctx.part_element_world_aabb(side_branch, elem=side_tool_pad))

    ctx.check(
        "upper_joint_changes_pad_lateral_position",
        upper_rest_center is not None
        and upper_posed_center is not None
        and abs(upper_posed_center[0] - upper_rest_center[0]) > 0.05,
        f"upper pad centers: rest={upper_rest_center}, posed={upper_posed_center}",
    )
    ctx.check(
        "side_joint_lifts_pad_in_z",
        side_rest_center is not None
        and side_posed_center is not None
        and (side_posed_center[2] - side_rest_center[2]) > 0.18,
        f"side pad centers: rest={side_rest_center}, posed={side_posed_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

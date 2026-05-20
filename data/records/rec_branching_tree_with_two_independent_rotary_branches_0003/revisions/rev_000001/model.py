from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_Z = 0.011
NODE_Z = 0.61
LEFT_AXIS = (-0.176, 0.0, 0.56)
RIGHT_AXIS = (0.186, 0.0, 0.69)
SHAFT_R = 0.0155
SUPPORT_BLOCK_Y = 0.045
SUPPORT_BLOCK_LEN_Y = 0.028
SPLIT_OFFSET_Z = 0.006


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(shape, filename, assets=ASSETS, tolerance=0.0008, angular_tolerance=0.08)


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _make_backbone() -> cq.Workplane:
    rail_a = cq.Workplane("XY").box(0.44, 0.055, 0.022).translate((0.0, 0.085, BASE_Z))
    rail_b = cq.Workplane("XY").box(0.44, 0.055, 0.022).translate((0.0, -0.085, BASE_Z))
    cross_member = cq.Workplane("XY").box(0.18, 0.16, 0.022).translate((0.0, 0.0, BASE_Z))
    center_pad = cq.Workplane("XY").box(0.14, 0.12, 0.016).translate((0.0, 0.0, 0.028))

    column_outer = cq.Workplane("XY").box(0.09, 0.11, 0.46).translate((0.0, 0.0, 0.25))
    column_inner = cq.Workplane("XY").box(0.054, 0.074, 0.34).translate((0.0, 0.0, 0.29))
    column = (
        column_outer
        .cut(column_inner)
        .cut(cq.Workplane("XY").box(0.052, 0.14, 0.10).translate((0.0, 0.0, 0.19)))
        .cut(cq.Workplane("XY").box(0.052, 0.14, 0.12).translate((0.0, 0.0, 0.35)))
    )

    node_outer = cq.Workplane("XY").box(0.24, 0.17, 0.28).translate((0.0, 0.0, NODE_Z))
    node_inner = cq.Workplane("XY").box(0.16, 0.11, 0.18).translate((0.0, 0.0, NODE_Z))
    node = (
        node_outer
        .cut(node_inner)
        .cut(cq.Workplane("XY").box(0.12, 0.055, 0.14).translate((0.0, 0.058, NODE_Z)))
        .cut(cq.Workplane("XY").box(0.12, 0.055, 0.14).translate((0.0, -0.058, NODE_Z)))
    )

    gusset = (
        cq.Workplane("XZ")
        .polyline([(0.018, 0.39), (0.018, 0.49), (0.082, 0.54), (0.082, 0.43)])
        .close()
        .extrude(0.012, both=True)
    )
    gusset_mirror = gusset.mirror("YZ")

    rear_spine = cq.Workplane("XY").box(0.028, 0.03, 0.23).translate((0.0, -0.058, 0.47))
    crown = cq.Workplane("XY").box(0.10, 0.10, 0.016).translate((0.0, 0.0, 0.758))

    frame = rail_a.union(rail_b).union(cross_member).union(center_pad).union(column).union(node).union(gusset).union(gusset_mirror).union(rear_spine).union(crown)

    for x in (-0.16, 0.16):
        for y in (-0.085, 0.085):
            frame = frame.cut(_cyl_z(0.007, 0.05, (x, y, BASE_Z)))

    return frame


def _make_support(sign: float, axis_center: tuple[float, float, float]) -> cq.Workplane:
    sx, _, sz = axis_center
    split_z = sz + SPLIT_OFFSET_Z
    flange_x = sign * 0.127

    flange = cq.Workplane("XY").box(0.014, 0.14, 0.18).translate((flange_x, 0.0, sz))
    front_arm = cq.Workplane("XY").box(0.050, 0.018, 0.036).translate((sign * 0.150, SUPPORT_BLOCK_Y, sz - 0.030))
    rear_arm = cq.Workplane("XY").box(0.050, 0.018, 0.036).translate((sign * 0.150, -SUPPORT_BLOCK_Y, sz - 0.030))
    front_block = cq.Workplane("XY").box(0.078, SUPPORT_BLOCK_LEN_Y, 0.052).translate((sx, SUPPORT_BLOCK_Y, split_z - 0.026))
    rear_block = cq.Workplane("XY").box(0.078, SUPPORT_BLOCK_LEN_Y, 0.052).translate((sx, -SUPPORT_BLOCK_Y, split_z - 0.026))
    lower_tie = cq.Workplane("XY").box(0.050, 0.040, 0.018).translate((sign * 0.170, 0.0, sz - 0.055))

    rib_front = cq.Workplane("XY").box(0.060, 0.010, 0.074).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -sign * 24.0).translate((sign * 0.152, SUPPORT_BLOCK_Y, sz - 0.024))
    rib_rear = cq.Workplane("XY").box(0.060, 0.010, 0.074).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -sign * 24.0).translate((sign * 0.152, -SUPPORT_BLOCK_Y, sz - 0.024))
    stop_block = cq.Workplane("XY").box(0.024, 0.032, 0.018).translate((sx + sign * 0.018, 0.0, sz - 0.055))

    support = flange.union(front_arm).union(rear_arm).union(front_block).union(rear_block).union(lower_tie).union(rib_front).union(rib_rear).union(stop_block)
    support = support.cut(_cyl_y(SHAFT_R, 0.042, (sx, SUPPORT_BLOCK_Y, sz))).cut(_cyl_y(SHAFT_R, 0.042, (sx, -SUPPORT_BLOCK_Y, sz)))

    for y in (-0.044, -0.018, 0.018, 0.044):
        for dz in (-0.048, 0.048):
            support = support.cut(_cyl_x(0.0035, 0.022, (flange_x, y, sz + dz)))

    return support


def _make_cap(axis_center: tuple[float, float, float]) -> cq.Workplane:
    sx, _, sz = axis_center
    split_z = sz + SPLIT_OFFSET_Z

    front_cap = cq.Workplane("XY").box(0.078, SUPPORT_BLOCK_LEN_Y, 0.034).translate((sx, SUPPORT_BLOCK_Y, split_z + 0.017))
    rear_cap = cq.Workplane("XY").box(0.078, SUPPORT_BLOCK_LEN_Y, 0.034).translate((sx, -SUPPORT_BLOCK_Y, split_z + 0.017))
    bridge = cq.Workplane("XY").box(0.050, 0.030, 0.010).translate((sx, 0.0, split_z + 0.046))
    cap = front_cap.union(rear_cap).union(bridge)

    cap = cap.cut(_cyl_y(SHAFT_R, 0.042, (sx, SUPPORT_BLOCK_Y, sz))).cut(_cyl_y(SHAFT_R, 0.042, (sx, -SUPPORT_BLOCK_Y, sz)))

    for y in (-SUPPORT_BLOCK_Y, SUPPORT_BLOCK_Y):
        for dx in (-0.024, 0.024):
            cap = cap.cut(_cyl_z(0.0035, 0.045, (sx + dx, y, split_z + 0.017)))
            cap = cap.union(_cyl_z(0.006, 0.006, (sx + dx, y, split_z + 0.037)))

    return cap


def _make_branch_arm(length: float, angle_deg: float, sign: float) -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    shaft = _cyl_y(SHAFT_R, 0.170, (0.0, 0.0, 0.0))
    hub = _cyl_y(0.036, 0.050, (0.0, 0.0, 0.0))
    collar_front = _cyl_y(0.022, 0.010, (0.0, 0.074, 0.0))
    collar_rear = _cyl_y(0.022, 0.010, (0.0, -0.074, 0.0))
    stop_tab = cq.Workplane("XY").box(0.026, 0.014, 0.012).translate((0.040, 0.0, -0.045))
    shaft_hub = shaft.union(hub).union(collar_front).union(collar_rear).union(stop_tab)

    beam_outer = cq.Workplane("XY").box(length, 0.050, 0.060).translate((length / 2.0, 0.0, 0.0))
    beam_inner = cq.Workplane("XY").box(length - 0.050, 0.028, 0.036).translate((length / 2.0 + 0.012, 0.0, 0.0))
    beam = beam_outer.cut(beam_inner)
    root_block = cq.Workplane("XY").box(0.072, 0.054, 0.046).translate((0.042, 0.0, 0.0))
    brace = (
        cq.Workplane("XZ")
        .polyline([(0.018, -0.018), (0.098, -0.018), (0.176, 0.048), (0.088, 0.018)])
        .close()
        .extrude(0.010, both=True)
    )
    arm_body = beam.union(root_block).union(brace).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg)

    pad_plate = cq.Workplane("XY").box(0.018, 0.076, 0.068).translate((length + 0.009, 0.0, 0.0))
    pad_spine = cq.Workplane("XY").box(0.030, 0.044, 0.032).translate((length - 0.010, 0.0, 0.0))
    gusset = (
        cq.Workplane("XZ")
        .polyline([(length - 0.070, -0.014), (length - 0.010, -0.014), (length + 0.006, 0.040), (length - 0.084, 0.010)])
        .close()
        .extrude(0.010, both=True)
    )
    mount_pad = pad_plate.union(pad_spine).union(gusset)
    for y in (-0.022, 0.022):
        for z in (-0.018, 0.018):
            mount_pad = mount_pad.cut(_cyl_x(0.004, 0.045, (length + 0.002, y, z)))
    mount_pad = mount_pad.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg)

    if sign < 0.0:
        shaft_hub = shaft_hub.mirror("YZ")
        arm_body = arm_body.mirror("YZ")
        mount_pad = mount_pad.mirror("YZ")

    return shaft_hub, arm_body, mount_pad


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((a + b) * 0.5 for a, b in zip(lo, hi))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="branching_tree_study", assets=ASSETS)

    steel_dark = model.material("steel_dark", rgba=(0.26, 0.29, 0.32, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.47, 0.50, 0.54, 1.0))
    steel_light = model.material("steel_light", rgba=(0.72, 0.74, 0.77, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.12, 0.12, 0.13, 1.0))

    backbone = model.part("backbone_frame")
    backbone.visual(_mesh(_make_backbone(), "backbone_frame.obj"), material=steel_dark, name="frame_shell")
    backbone.inertial = Inertial.from_geometry(Box((0.44, 0.17, 0.78)), mass=18.0, origin=Origin(xyz=(0.0, 0.0, 0.39)))

    left_support = model.part("left_support")
    left_support.visual(_mesh(_make_support(-1.0, LEFT_AXIS), "left_support.obj"), material=steel_mid, name="support_body")
    left_support.inertial = Inertial.from_geometry(Box((0.11, 0.14, 0.19)), mass=2.8, origin=Origin(xyz=(-0.15, 0.0, LEFT_AXIS[2] - 0.01)))

    right_support = model.part("right_support")
    right_support.visual(_mesh(_make_support(1.0, RIGHT_AXIS), "right_support.obj"), material=steel_mid, name="support_body")
    right_support.inertial = Inertial.from_geometry(Box((0.11, 0.14, 0.19)), mass=2.9, origin=Origin(xyz=(0.16, 0.0, RIGHT_AXIS[2] - 0.01)))

    left_cap = model.part("left_cap")
    left_cap.visual(_mesh(_make_cap(LEFT_AXIS), "left_cap.obj"), material=steel_light, name="cap_body")
    left_cap.inertial = Inertial.from_geometry(Box((0.08, 0.12, 0.06)), mass=0.7, origin=Origin(xyz=LEFT_AXIS))

    right_cap = model.part("right_cap")
    right_cap.visual(_mesh(_make_cap(RIGHT_AXIS), "right_cap.obj"), material=steel_light, name="cap_body")
    right_cap.inertial = Inertial.from_geometry(Box((0.08, 0.12, 0.06)), mass=0.7, origin=Origin(xyz=RIGHT_AXIS))

    left_shaft_hub, left_arm_body, left_mount_pad = _make_branch_arm(length=0.225, angle_deg=31.0, sign=-1.0)
    left_branch = model.part("left_branch")
    left_branch.visual(_mesh(left_shaft_hub, "left_branch_shaft_hub.obj"), material=bearing_black, name="shaft_hub")
    left_branch.visual(_mesh(left_arm_body, "left_branch_body.obj"), material=steel_mid, name="arm_body")
    left_branch.visual(_mesh(left_mount_pad, "left_branch_mount_pad.obj"), material=steel_light, name="mount_pad")
    left_branch.inertial = Inertial.from_geometry(Box((0.28, 0.16, 0.20)), mass=2.4, origin=Origin(xyz=(-0.13, 0.0, 0.08)))

    right_shaft_hub, right_arm_body, right_mount_pad = _make_branch_arm(length=0.275, angle_deg=39.0, sign=1.0)
    right_branch = model.part("right_branch")
    right_branch.visual(_mesh(right_shaft_hub, "right_branch_shaft_hub.obj"), material=bearing_black, name="shaft_hub")
    right_branch.visual(_mesh(right_arm_body, "right_branch_body.obj"), material=steel_mid, name="arm_body")
    right_branch.visual(_mesh(right_mount_pad, "right_branch_mount_pad.obj"), material=steel_light, name="mount_pad")
    right_branch.inertial = Inertial.from_geometry(Box((0.33, 0.16, 0.23)), mass=2.8, origin=Origin(xyz=(0.16, 0.0, 0.11)))

    model.articulation("backbone_to_left_support", ArticulationType.FIXED, parent=backbone, child=left_support, origin=Origin())
    model.articulation("backbone_to_right_support", ArticulationType.FIXED, parent=backbone, child=right_support, origin=Origin())
    model.articulation("left_support_to_left_cap", ArticulationType.FIXED, parent=left_support, child=left_cap, origin=Origin())
    model.articulation("right_support_to_right_cap", ArticulationType.FIXED, parent=right_support, child=right_cap, origin=Origin())
    model.articulation(
        "left_support_to_left_branch",
        ArticulationType.REVOLUTE,
        parent=left_support,
        child=left_branch,
        origin=Origin(xyz=LEFT_AXIS),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=32.0, velocity=1.6, lower=-0.60, upper=0.95),
    )
    model.articulation(
        "right_support_to_right_branch",
        ArticulationType.REVOLUTE,
        parent=right_support,
        child=right_branch,
        origin=Origin(xyz=RIGHT_AXIS),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=36.0, velocity=1.5, lower=-0.85, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    backbone = object_model.get_part("backbone_frame")
    left_support = object_model.get_part("left_support")
    right_support = object_model.get_part("right_support")
    left_cap = object_model.get_part("left_cap")
    right_cap = object_model.get_part("right_cap")
    left_branch = object_model.get_part("left_branch")
    right_branch = object_model.get_part("right_branch")

    left_joint = object_model.get_articulation("left_support_to_left_branch")
    right_joint = object_model.get_articulation("right_support_to_right_branch")
    left_support_joint = object_model.get_articulation("backbone_to_left_support")
    right_support_joint = object_model.get_articulation("backbone_to_right_support")
    left_cap_joint = object_model.get_articulation("left_support_to_left_cap")
    right_cap_joint = object_model.get_articulation("right_support_to_right_cap")

    left_shaft = left_branch.get_visual("shaft_hub")
    right_shaft = right_branch.get_visual("shaft_hub")
    left_pad = left_branch.get_visual("mount_pad")
    right_pad = right_branch.get_visual("mount_pad")
    frame_shell = backbone.get_visual("frame_shell")
    left_support_body = left_support.get_visual("support_body")
    right_support_body = right_support.get_visual("support_body")
    left_cap_body = left_cap.get_visual("cap_body")
    right_cap_body = right_cap.get_visual("cap_body")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "articulations present",
        all(j is not None for j in (left_joint, right_joint, left_support_joint, right_support_joint, left_cap_joint, right_cap_joint)),
        "missing one or more expected articulations",
    )

    ctx.expect_contact(left_support, backbone, elem_a=left_support_body, elem_b=frame_shell, name="left support mounts on backbone")
    ctx.expect_contact(right_support, backbone, elem_a=right_support_body, elem_b=frame_shell, name="right support mounts on backbone")
    ctx.expect_overlap(left_support, backbone, axes="yz", min_overlap=0.12, elem_a=left_support_body, elem_b=frame_shell, name="left support flange overlaps node face")
    ctx.expect_overlap(right_support, backbone, axes="yz", min_overlap=0.12, elem_a=right_support_body, elem_b=frame_shell, name="right support flange overlaps node face")

    ctx.expect_contact(left_cap, left_support, elem_a=left_cap_body, elem_b=left_support_body, name="left cap seats on split carrier")
    ctx.expect_contact(right_cap, right_support, elem_a=right_cap_body, elem_b=right_support_body, name="right cap seats on split carrier")
    ctx.expect_gap(left_cap, left_support, axis="z", max_gap=0.0, max_penetration=0.0, positive_elem=left_cap_body, negative_elem=left_support_body, name="left cap split line is flush")
    ctx.expect_gap(right_cap, right_support, axis="z", max_gap=0.0, max_penetration=0.0, positive_elem=right_cap_body, negative_elem=right_support_body, name="right cap split line is flush")
    ctx.expect_overlap(left_cap, left_support, axes="xy", min_overlap=0.05, elem_a=left_cap_body, elem_b=left_support_body, name="left cap spans both bearing blocks")
    ctx.expect_overlap(right_cap, right_support, axes="xy", min_overlap=0.05, elem_a=right_cap_body, elem_b=right_support_body, name="right cap spans both bearing blocks")

    ctx.expect_overlap(left_branch, left_support, axes="yz", min_overlap=0.028, elem_a=left_shaft, elem_b=left_support_body, name="left shaft aligns with support bores")
    ctx.expect_overlap(right_branch, right_support, axes="yz", min_overlap=0.028, elem_a=right_shaft, elem_b=right_support_body, name="right shaft aligns with support bores")
    ctx.expect_overlap(left_branch, left_cap, axes="yz", min_overlap=0.028, elem_a=left_shaft, elem_b=left_cap_body, name="left shaft aligns with cap bores")
    ctx.expect_overlap(right_branch, right_cap, axes="yz", min_overlap=0.028, elem_a=right_shaft, elem_b=right_cap_body, name="right shaft aligns with cap bores")

    with ctx.pose({left_joint: 0.0, right_joint: 0.0}):
        left_rest = _aabb_center(ctx.part_element_world_aabb(left_branch, elem=left_pad))
        right_rest = _aabb_center(ctx.part_element_world_aabb(right_branch, elem=right_pad))

    with ctx.pose({left_joint: 0.70, right_joint: 0.0}):
        left_raised = _aabb_center(ctx.part_element_world_aabb(left_branch, elem=left_pad))
        right_still = _aabb_center(ctx.part_element_world_aabb(right_branch, elem=right_pad))

    with ctx.pose({left_joint: 0.0, right_joint: -0.60}):
        left_still = _aabb_center(ctx.part_element_world_aabb(left_branch, elem=left_pad))
        right_moved = _aabb_center(ctx.part_element_world_aabb(right_branch, elem=right_pad))

    if None not in (left_rest, right_rest, left_raised, right_still, left_still, right_moved):
        left_move = tuple(abs(a - b) for a, b in zip(left_raised, left_rest))
        right_move = tuple(abs(a - b) for a, b in zip(right_moved, right_rest))
        right_cross_talk = tuple(abs(a - b) for a, b in zip(right_still, right_rest))
        left_cross_talk = tuple(abs(a - b) for a, b in zip(left_still, left_rest))

        ctx.check(
            "left branch rotates about its own y-axis support",
            left_move[1] < 0.010 and left_move[0] > 0.035 and left_move[2] > 0.035,
            f"left pad movement {left_move} did not stay mostly in the xz plane",
        )
        ctx.check(
            "right branch stays independent while left branch moves",
            max(right_cross_talk) < 0.004,
            f"right pad drifted {right_cross_talk} when only the left articulation was posed",
        )
        ctx.check(
            "right branch rotates about its own y-axis support",
            right_move[1] < 0.010 and right_move[0] > 0.040 and right_move[2] > 0.040,
            f"right pad movement {right_move} did not stay mostly in the xz plane",
        )
        ctx.check(
            "left branch stays independent while right branch moves",
            max(left_cross_talk) < 0.004,
            f"left pad drifted {left_cross_talk} when only the right articulation was posed",
        )
    else:
        ctx.fail("mount pad pose measurements available", "one or more mount pad AABBs could not be measured")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

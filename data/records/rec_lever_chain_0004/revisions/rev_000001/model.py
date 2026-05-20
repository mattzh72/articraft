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
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

PLATE_T = 0.004
UPPER_Z = 0.005
LOWER_Z = -0.005

PIN_RADIUS = 0.0032
PIN_LENGTH = 0.018
PIN_CENTER_Z = 0.0
PIN_HEAD_RADIUS = 0.0053
PIN_HEAD_T = 0.0016
HOLE_RADIUS = 0.0037

BOSS_RADIUS = 0.009
WEB_WIDTH = 0.011
TAB_RADIUS = 0.0058
TAB_WIDTH = 0.008
L1_DX = 0.066
L1_DY = 0.018
L2_DX = 0.061
L2_DY = -0.019
L3_DX = 0.032
L3_DY = 0.010


def _extruded_circle(x: float, y: float, radius: float, thickness: float) -> cq.Workplane:
    return cq.Workplane("XY").center(x, y).circle(radius).extrude(thickness)


def _extruded_bridge(
    start: tuple[float, float],
    end: tuple[float, float],
    width: float,
    thickness: float,
) -> cq.Workplane:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    span = math.hypot(dx, dy)
    angle_deg = math.degrees(math.atan2(dy, dx))
    return (
        cq.Workplane("XY")
        .transformed(offset=((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, 0.0), rotate=(0.0, 0.0, angle_deg))
        .rect(max(span - BOSS_RADIUS * 0.65, width), width)
        .extrude(thickness)
    )


def _circle_cutter(x: float, y: float, radius: float, height: float = 0.020) -> cq.Workplane:
    return cq.Workplane("XY").center(x, y).circle(radius).extrude(height).translate((0.0, 0.0, -height / 2.0))


def _translate_to_layer(shape: cq.Workplane, z_center: float) -> cq.Workplane:
    return shape.translate((0.0, 0.0, z_center - PLATE_T / 2.0))


def _base_lug_plate() -> cq.Workplane:
    mount_center = (-0.031, -0.002)
    brace_center = (-0.018, -0.019)
    plate = _extruded_circle(0.0, 0.0, BOSS_RADIUS, PLATE_T)
    plate = plate.union(_extruded_circle(mount_center[0], mount_center[1], 0.0105, PLATE_T))
    plate = plate.union(_extruded_circle(brace_center[0], brace_center[1], 0.0070, PLATE_T))
    plate = plate.union(_extruded_bridge((0.0, 0.0), mount_center, 0.015, PLATE_T))
    plate = plate.union(_extruded_bridge((-0.010, -0.010), brace_center, 0.010, PLATE_T))
    plate = plate.cut(_circle_cutter(mount_center[0], mount_center[1], 0.0032))
    plate = plate.cut(_circle_cutter(brace_center[0], brace_center[1], 0.0026))
    return _translate_to_layer(plate, LOWER_Z)


def _lever_plate(dx: float, dy: float, z_center: float) -> cq.Workplane:
    plate = _extruded_circle(0.0, 0.0, BOSS_RADIUS, PLATE_T)
    plate = plate.union(_extruded_circle(dx, dy, BOSS_RADIUS, PLATE_T))
    plate = plate.union(_extruded_bridge((0.0, 0.0), (dx, dy), WEB_WIDTH, PLATE_T))
    plate = plate.cut(_circle_cutter(0.0, 0.0, HOLE_RADIUS))
    return _translate_to_layer(plate, z_center)


def _terminal_lever_plate(dx: float, dy: float, z_center: float) -> cq.Workplane:
    plate = _extruded_circle(0.0, 0.0, BOSS_RADIUS, PLATE_T)
    plate = plate.union(_extruded_circle(dx, dy, 0.0074, PLATE_T))
    plate = plate.union(_extruded_bridge((0.0, 0.0), (dx, dy), 0.0086, PLATE_T))
    plate = plate.cut(_circle_cutter(0.0, 0.0, HOLE_RADIUS))
    return _translate_to_layer(plate, z_center)


def _lever_end_tab(dx: float, dy: float, z_center: float) -> cq.Workplane:
    span = math.hypot(dx, dy)
    px = -dy / span
    py = dx / span
    left = (dx - px * 0.0058, dy - py * 0.0058)
    right = (dx + px * 0.0058, dy + py * 0.0058)
    tab = _extruded_circle(left[0], left[1], 0.0036, PLATE_T)
    tab = tab.union(_extruded_circle(right[0], right[1], 0.0036, PLATE_T))
    tab = tab.union(_extruded_bridge(left, right, 0.0056, PLATE_T))
    return _translate_to_layer(tab, z_center)


def _head_center_z(side: str) -> float:
    if side == "up":
        return UPPER_Z + PLATE_T / 2.0 + PIN_HEAD_T / 2.0
    return LOWER_Z - PLATE_T / 2.0 - PIN_HEAD_T / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lever_chain", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.33, 0.36, 0.40, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.61, 0.65, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    tab_finish = model.material("tab_finish", rgba=(0.47, 0.49, 0.54, 1.0))

    base_lug = model.part("base_lug")
    base_lug.visual(
        mesh_from_cadquery(_base_lug_plate(), "base_lug_plate.obj", assets=ASSETS),
        material=dark_steel,
        name="lug_plate",
    )
    base_lug.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, PIN_CENTER_Z)),
        material=pin_steel,
        name="pin_shaft",
    )
    base_lug.visual(
        Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_T),
        origin=Origin(xyz=(0.0, 0.0, _head_center_z("up"))),
        material=pin_steel,
        name="pin_head",
    )

    lever_1 = model.part("lever_1")
    lever_1.visual(
        mesh_from_cadquery(_lever_plate(L1_DX, L1_DY, UPPER_Z), "lever_1_plate.obj", assets=ASSETS),
        material=steel,
        name="link_plate",
    )
    lever_1.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(L1_DX, L1_DY, PIN_CENTER_Z)),
        material=pin_steel,
        name="pin_shaft",
    )
    lever_1.visual(
        Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_T),
        origin=Origin(xyz=(L1_DX, L1_DY, _head_center_z("down"))),
        material=pin_steel,
        name="pin_head",
    )

    lever_2 = model.part("lever_2")
    lever_2.visual(
        mesh_from_cadquery(_lever_plate(L2_DX, L2_DY, LOWER_Z), "lever_2_plate.obj", assets=ASSETS),
        material=dark_steel,
        name="link_plate",
    )
    lever_2.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(L2_DX, L2_DY, PIN_CENTER_Z)),
        material=pin_steel,
        name="pin_shaft",
    )
    lever_2.visual(
        Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_T),
        origin=Origin(xyz=(L2_DX, L2_DY, _head_center_z("up"))),
        material=pin_steel,
        name="pin_head",
    )

    lever_3 = model.part("lever_3")
    lever_3.visual(
        mesh_from_cadquery(_terminal_lever_plate(L3_DX, L3_DY, UPPER_Z), "lever_3_plate.obj", assets=ASSETS),
        material=tab_finish,
        name="link_plate",
    )
    lever_3.visual(
        mesh_from_cadquery(_lever_end_tab(L3_DX, L3_DY, UPPER_Z), "lever_3_end_tab.obj", assets=ASSETS),
        material=tab_finish,
        name="end_tab",
    )

    model.articulation(
        "base_to_lever_1",
        ArticulationType.REVOLUTE,
        parent=base_lug,
        child=lever_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-2.30, upper=2.30),
    )
    model.articulation(
        "lever_1_to_lever_2",
        ArticulationType.REVOLUTE,
        parent=lever_1,
        child=lever_2,
        origin=Origin(xyz=(L1_DX, L1_DY, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.0, lower=-2.25, upper=2.25),
    )
    model.articulation(
        "lever_2_to_lever_3",
        ArticulationType.REVOLUTE,
        parent=lever_2,
        child=lever_3,
        origin=Origin(xyz=(L2_DX, L2_DY, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-2.20, upper=2.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_lug = object_model.get_part("base_lug")
    lever_1 = object_model.get_part("lever_1")
    lever_2 = object_model.get_part("lever_2")
    lever_3 = object_model.get_part("lever_3")

    base_to_lever_1 = object_model.get_articulation("base_to_lever_1")
    lever_1_to_lever_2 = object_model.get_articulation("lever_1_to_lever_2")
    lever_2_to_lever_3 = object_model.get_articulation("lever_2_to_lever_3")

    lug_plate = base_lug.get_visual("lug_plate")
    base_head = base_lug.get_visual("pin_head")
    lever_1_plate = lever_1.get_visual("link_plate")
    lever_1_head = lever_1.get_visual("pin_head")
    lever_2_plate = lever_2.get_visual("link_plate")
    lever_2_head = lever_2.get_visual("pin_head")
    lever_3_plate = lever_3.get_visual("link_plate")
    end_tab = lever_3.get_visual("end_tab")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo_i + hi_i) / 2.0 for lo_i, hi_i in zip(lo, hi))

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
        "lever_chain_part_count",
        len(object_model.parts) == 4,
        f"expected 4 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "parallel_revolute_axes",
        all(
            articulation.axis == (0.0, 0.0, 1.0)
            for articulation in (base_to_lever_1, lever_1_to_lever_2, lever_2_to_lever_3)
        ),
        "all three joints should revolve about parallel +Z pin axes",
    )
    ctx.check(
        "compact_fold_range",
        all(
            articulation.motion_limits is not None
            and articulation.motion_limits.lower <= -2.20
            and articulation.motion_limits.upper >= 2.20
            for articulation in (base_to_lever_1, lever_1_to_lever_2, lever_2_to_lever_3)
        ),
        "joint limits should support both compact folding and shallow extension",
    )

    ctx.expect_origin_distance(lever_1, base_lug, axes="xy", max_dist=1e-6, name="base_joint_origin_alignment")
    ctx.expect_gap(
        lever_1,
        base_lug,
        axis="z",
        min_gap=0.0035,
        max_gap=0.0060,
        positive_elem=lever_1_plate,
        negative_elem=lug_plate,
        name="lever_1_stacked_above_base_lug",
    )
    ctx.expect_contact(
        base_lug,
        lever_1,
        elem_a=base_head,
        elem_b=lever_1_plate,
        name="base_pin_head_retains_lever_1",
    )
    ctx.expect_overlap(
        base_lug,
        lever_1,
        axes="xy",
        min_overlap=0.006,
        elem_a=base_head,
        elem_b=lever_1_plate,
        name="base_joint_xy_overlap",
    )

    ctx.expect_gap(
        lever_1,
        lever_2,
        axis="z",
        min_gap=0.0058,
        max_gap=0.0140,
        positive_elem=lever_1_plate,
        negative_elem=lever_2_plate,
        name="lever_1_above_lever_2_stack_gap",
    )
    ctx.expect_contact(
        lever_1,
        lever_2,
        elem_a=lever_1_head,
        elem_b=lever_2_plate,
        name="lever_1_pin_head_retains_lever_2",
    )
    ctx.expect_overlap(
        lever_1,
        lever_2,
        axes="xy",
        min_overlap=0.006,
        elem_a=lever_1_head,
        elem_b=lever_2_plate,
        name="joint_2_xy_overlap",
    )

    ctx.expect_gap(
        lever_3,
        lever_2,
        axis="z",
        min_gap=0.0058,
        max_gap=0.0140,
        positive_elem=lever_3_plate,
        negative_elem=lever_2_plate,
        name="lever_3_above_lever_2_stack_gap",
    )
    ctx.expect_contact(
        lever_2,
        lever_3,
        elem_a=lever_2_head,
        elem_b=lever_3_plate,
        name="lever_2_pin_head_retains_lever_3",
    )
    ctx.expect_overlap(
        lever_2,
        lever_3,
        axes="xy",
        min_overlap=0.006,
        elem_a=lever_2_head,
        elem_b=lever_3_plate,
        name="joint_3_xy_overlap",
    )

    lever_2_pos = ctx.part_world_position(lever_2)
    lever_3_pos = ctx.part_world_position(lever_3)
    end_tab_center = _aabb_center(ctx.part_element_world_aabb(lever_3, elem=end_tab))
    ctx.check(
        "rest_pose_joint_2_location",
        lever_2_pos is not None
        and abs(lever_2_pos[0] - L1_DX) < 1e-6
        and abs(lever_2_pos[1] - L1_DY) < 1e-6,
        f"lever_2 origin should sit at lever_1 distal pin: {lever_2_pos}",
    )
    ctx.check(
        "rest_pose_joint_3_location",
        lever_3_pos is not None
        and abs(lever_3_pos[0] - (L1_DX + L2_DX)) < 1e-6
        and abs(lever_3_pos[1] - (L1_DY + L2_DY)) < 1e-6,
        f"lever_3 origin should sit at lever_2 distal pin: {lever_3_pos}",
    )
    ctx.check(
        "rest_pose_end_tab_reaches_forward",
        end_tab_center is not None
        and end_tab_center[0] > 0.155
        and 0.0 < end_tab_center[1] < 0.03,
        f"end tab should sit ahead of the chain in rest pose: {end_tab_center}",
    )

    with ctx.pose(
        {
            base_to_lever_1: -2.05,
            lever_1_to_lever_2: 2.10,
            lever_2_to_lever_3: -1.85,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="compact_fold_pose_no_overlaps")
        folded_tab_center = _aabb_center(ctx.part_element_world_aabb(lever_3, elem=end_tab))
        ctx.check(
            "compact_fold_pose",
            folded_tab_center is not None
            and folded_tab_center[0] < 0.09
            and abs(folded_tab_center[1]) < 0.12,
            f"folded pose should tuck the end tab close to the base: {folded_tab_center}",
        )

    with ctx.pose(
        {
            base_to_lever_1: 0.28,
            lever_1_to_lever_2: 0.23,
            lever_2_to_lever_3: 0.18,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="shallow_arc_pose_no_overlaps")
        shallow_tab_center = _aabb_center(ctx.part_element_world_aabb(lever_3, elem=end_tab))
        ctx.check(
            "shallow_arc_pose",
            shallow_tab_center is not None
            and shallow_tab_center[0] > 0.135
            and shallow_tab_center[1] > 0.03
            and shallow_tab_center[1] < 0.12,
            f"extended pose should read as a shallow arc: {shallow_tab_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

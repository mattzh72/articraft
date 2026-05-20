from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


BASE_WIDTH = 0.22
BASE_DEPTH = 0.12
BASE_THICKNESS = 0.018

INNER_ARM_GAP = 0.160
ARM_THICKNESS = 0.022
ARM_DEPTH = 0.082
ARM_HEIGHT = 0.132

BOSS_RADIUS = 0.026
BOSS_THICKNESS = 0.010
SHAFT_AXIS_Z = 0.118
BORE_RADIUS = 0.0104


def _make_fork_frame() -> cq.Workplane:
    arm_center_x = INNER_ARM_GAP / 2.0 + ARM_THICKNESS / 2.0
    arm_outer_x = INNER_ARM_GAP / 2.0 + ARM_THICKNESS
    bridge_depth = 0.022
    bridge_height = 0.074
    bridge_y = -0.5 * ARM_DEPTH + 0.5 * bridge_depth + 0.008

    base = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
    )
    left_arm = (
        cq.Workplane("XY")
        .box(ARM_THICKNESS, ARM_DEPTH, ARM_HEIGHT)
        .translate((-arm_center_x, 0.0, BASE_THICKNESS + ARM_HEIGHT / 2.0))
    )
    right_arm = (
        cq.Workplane("XY")
        .box(ARM_THICKNESS, ARM_DEPTH, ARM_HEIGHT)
        .translate((arm_center_x, 0.0, BASE_THICKNESS + ARM_HEIGHT / 2.0))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(INNER_ARM_GAP, bridge_depth, bridge_height)
        .translate((0.0, bridge_y, BASE_THICKNESS + bridge_height / 2.0))
    )

    left_boss = (
        cq.Workplane("YZ", origin=(-arm_outer_x - BOSS_THICKNESS, 0.0, SHAFT_AXIS_Z))
        .circle(BOSS_RADIUS)
        .extrude(BOSS_THICKNESS)
    )
    right_boss = (
        cq.Workplane("YZ", origin=(arm_outer_x, 0.0, SHAFT_AXIS_Z))
        .circle(BOSS_RADIUS)
        .extrude(BOSS_THICKNESS)
    )

    frame = (
        base.union(left_arm)
        .union(right_arm)
        .union(rear_bridge)
        .union(left_boss)
        .union(right_boss)
    )

    bore_start_x = -(arm_outer_x + BOSS_THICKNESS)
    bore_length = 2.0 * (arm_outer_x + BOSS_THICKNESS)
    bore = (
        cq.Workplane("YZ", origin=(bore_start_x, 0.0, SHAFT_AXIS_Z))
        .circle(BORE_RADIUS)
        .extrude(bore_length)
    )

    return frame.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_supported_rotary_shaft")

    model.material("frame_gray", rgba=(0.28, 0.31, 0.34, 1.0))
    model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("dark_steel", rgba=(0.42, 0.45, 0.48, 1.0))

    frame = model.part("fork_frame")
    frame.visual(
        mesh_from_cadquery(_make_fork_frame(), "fork_frame"),
        material="frame_gray",
        name="frame_body",
    )

    shaft = model.part("shaft_assembly")
    shaft.visual(
        Cylinder(radius=0.009, length=0.248),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="steel",
        name="shaft_core",
    )
    shaft.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(-0.115, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_steel",
        name="left_collar",
    )
    shaft.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.117, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_steel",
        name="right_hub",
    )
    shaft.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.127, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="steel",
        name="output_disk",
    )
    shaft.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(xyz=(0.136, 0.018, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_steel",
        name="disk_index_pin",
    )

    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=6.0),
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
    frame = object_model.get_part("fork_frame")
    shaft = object_model.get_part("shaft_assembly")
    spin = object_model.get_articulation("shaft_spin")

    ctx.check("fork frame exists", frame is not None)
    ctx.check("shaft assembly exists", shaft is not None)
    ctx.check(
        "shaft joint is continuous about x",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 4) for v in spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    ctx.expect_contact(
        shaft,
        frame,
        elem_a="left_collar",
        elem_b="frame_body",
        contact_tol=5e-4,
        name="left collar seats against the left fork support",
    )
    ctx.expect_contact(
        shaft,
        frame,
        elem_a="right_hub",
        elem_b="frame_body",
        contact_tol=5e-4,
        name="right hub seats against the right fork support",
    )
    ctx.expect_gap(
        shaft,
        frame,
        axis="x",
        positive_elem="disk_index_pin",
        negative_elem="frame_body",
        min_gap=0.020,
        name="output disk sits beyond the right support arm",
    )

    rest_pin = ctx.part_element_world_aabb(shaft, elem="disk_index_pin")
    with ctx.pose({spin: math.pi / 2.0}):
        quarter_turn_pin = ctx.part_element_world_aabb(shaft, elem="disk_index_pin")

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    rest_pin_center = aabb_center(rest_pin)
    quarter_turn_center = aabb_center(quarter_turn_pin)
    ctx.check(
        "index pin follows x-axis rotation",
        rest_pin_center is not None
        and quarter_turn_center is not None
        and abs(quarter_turn_center[0] - rest_pin_center[0]) < 0.002
        and rest_pin_center[1] > 0.012
        and abs(quarter_turn_center[1]) < 0.004
        and quarter_turn_center[2] > rest_pin_center[2] + 0.012
        and abs(rest_pin_center[2] - SHAFT_AXIS_Z) < 0.004
        and abs(quarter_turn_center[1]) < 0.004,
        details=f"rest={rest_pin_center}, quarter_turn={quarter_turn_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_trunnion_positioner")

    safety_blue = model.material("safety_blue", rgba=(0.05, 0.22, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    table_gray = model.material("table_gray", rgba=(0.30, 0.32, 0.33, 1.0))
    slot_black = model.material("slot_black", rgba=(0.01, 0.012, 0.014, 1.0))

    # Root frame: an overhead/top support with a central underslung yaw bearing.
    top_support = model.part("top_support")
    top_support.visual(
        Box((1.35, 0.34, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=safety_blue,
        name="top_beam",
    )
    top_support.visual(
        Box((0.92, 0.62, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=safety_blue,
        name="mounting_plate",
    )
    top_support.visual(
        Box((0.42, 0.36, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=safety_blue,
        name="drop_housing",
    )
    top_support.visual(
        Cylinder(radius=0.19, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_steel,
        name="yaw_bearing_socket",
    )
    top_support.visual(
        Cylinder(radius=0.245, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_steel,
        name="bearing_face",
    )
    top_support.visual(
        Box((0.08, 0.54, 0.24)),
        origin=Origin(xyz=(-0.31, 0.0, 0.25)),
        material=safety_blue,
        name="web_0",
    )
    top_support.visual(
        Box((0.08, 0.54, 0.24)),
        origin=Origin(xyz=(0.31, 0.0, 0.25)),
        material=safety_blue,
        name="web_1",
    )

    # First moving link: a lower rotary stage suspended below the top support.
    # It carries the trunnion yoke and its bearings, so it yaws as one assembly.
    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        Cylinder(radius=0.085, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=machined_steel,
        name="yaw_spindle",
    )
    lower_stage.visual(
        Cylinder(radius=0.24, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=dark_steel,
        name="rotary_flange",
    )
    lower_stage.visual(
        Cylinder(radius=0.12, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.14)),
        material=dark_steel,
        name="hanger_column",
    )
    lower_stage.visual(
        Box((1.12, 0.24, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, -0.23)),
        material=safety_blue,
        name="crosshead",
    )
    lower_stage.visual(
        Box((0.13, 0.22, 0.60)),
        origin=Origin(xyz=(-0.55, 0.0, -0.50)),
        material=safety_blue,
        name="yoke_arm_0",
    )
    lower_stage.visual(
        Box((0.13, 0.22, 0.60)),
        origin=Origin(xyz=(0.55, 0.0, -0.50)),
        material=safety_blue,
        name="yoke_arm_1",
    )
    lower_stage.visual(
        Box((0.13, 0.22, 0.11)),
        origin=Origin(xyz=(-0.55, 0.0, -1.075)),
        material=safety_blue,
        name="lower_lug_0",
    )
    lower_stage.visual(
        Box((0.13, 0.22, 0.11)),
        origin=Origin(xyz=(0.55, 0.0, -1.075)),
        material=safety_blue,
        name="lower_lug_1",
    )
    lower_stage.visual(
        Cylinder(radius=0.135, length=0.18),
        origin=Origin(xyz=(-0.55, 0.0, -0.92), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="bearing_0",
    )
    lower_stage.visual(
        Cylinder(radius=0.135, length=0.18),
        origin=Origin(xyz=(0.55, 0.0, -0.92), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="bearing_1",
    )
    lower_stage.visual(
        Box((0.24, 0.19, 0.18)),
        origin=Origin(xyz=(0.34, 0.0, -0.13)),
        material=dark_steel,
        name="slew_motor",
    )
    lower_stage.visual(
        Cylinder(radius=0.07, length=0.10),
        origin=Origin(xyz=(0.465, 0.0, -0.13), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="motor_cap",
    )

    # Second moving link: a fixture table hanging below the trunnion axis.
    # The part frame is exactly on the trunnion shaft centerline.
    work_table = model.part("work_table")
    work_table.visual(
        Cylinder(radius=0.045, length=1.16),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="trunnion_shaft",
    )
    work_table.visual(
        Box((0.72, 0.60, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.32)),
        material=table_gray,
        name="table_plate",
    )
    work_table.visual(
        Box((0.06, 0.48, 0.34)),
        origin=Origin(xyz=(-0.32, 0.0, -0.16)),
        material=table_gray,
        name="table_cheek_0",
    )
    work_table.visual(
        Box((0.06, 0.48, 0.34)),
        origin=Origin(xyz=(0.32, 0.0, -0.16)),
        material=table_gray,
        name="table_cheek_1",
    )
    for i, y in enumerate((-0.20, 0.0, 0.20)):
        work_table.visual(
            Box((0.58, 0.025, 0.012)),
            origin=Origin(xyz=(0.0, y, -0.273)),
            material=slot_black,
            name=f"fixture_slot_{i}",
        )
    work_table.visual(
        Box((0.78, 0.06, 0.09)),
        origin=Origin(xyz=(0.0, -0.33, -0.31)),
        material=dark_steel,
        name="front_lip",
    )
    work_table.visual(
        Box((0.78, 0.06, 0.09)),
        origin=Origin(xyz=(0.0, 0.33, -0.31)),
        material=dark_steel,
        name="rear_lip",
    )

    model.articulation(
        "top_support_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=top_support,
        child=lower_stage,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.75, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "lower_stage_to_table",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=work_table,
        origin=Origin(xyz=(0.0, 0.0, -0.92)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.65, lower=-1.75, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    lower_stage = object_model.get_part("lower_stage")
    work_table = object_model.get_part("work_table")
    yaw_joint = object_model.get_articulation("top_support_to_lower_stage")
    trunnion_joint = object_model.get_articulation("lower_stage_to_table")

    ctx.check(
        "lower stage is revolute",
        yaw_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={yaw_joint.articulation_type}",
    )
    ctx.check(
        "table is revolute",
        trunnion_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={trunnion_joint.articulation_type}",
    )
    ctx.check(
        "yaw axis is vertical",
        tuple(round(v, 6) for v in yaw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "trunnion axis is horizontal",
        tuple(round(v, 6) for v in trunnion_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={trunnion_joint.axis}",
    )

    ctx.allow_overlap(
        top_support,
        lower_stage,
        elem_a="yaw_bearing_socket",
        elem_b="yaw_spindle",
        reason="The yaw spindle is intentionally represented as captured inside the bearing socket.",
    )
    ctx.expect_within(
        lower_stage,
        top_support,
        axes="xy",
        inner_elem="yaw_spindle",
        outer_elem="yaw_bearing_socket",
        margin=0.002,
        name="yaw spindle is centered in socket",
    )
    ctx.expect_overlap(
        lower_stage,
        top_support,
        axes="z",
        elem_a="yaw_spindle",
        elem_b="yaw_bearing_socket",
        min_overlap=0.035,
        name="yaw spindle remains inserted",
    )
    ctx.allow_overlap(
        top_support,
        lower_stage,
        elem_a="bearing_face",
        elem_b="yaw_spindle",
        reason="The spindle passes through the represented solid bearing face plate.",
    )
    ctx.expect_within(
        lower_stage,
        top_support,
        axes="xy",
        inner_elem="yaw_spindle",
        outer_elem="bearing_face",
        margin=0.002,
        name="yaw spindle centered in bearing face",
    )
    ctx.expect_overlap(
        lower_stage,
        top_support,
        axes="z",
        elem_a="yaw_spindle",
        elem_b="bearing_face",
        min_overlap=0.02,
        name="yaw spindle passes through face plate",
    )

    for bearing_name in ("bearing_0", "bearing_1"):
        ctx.allow_overlap(
            lower_stage,
            work_table,
            elem_a=bearing_name,
            elem_b="trunnion_shaft",
            reason="The trunnion shaft is intentionally captured in the solid bearing proxy.",
        )
        ctx.expect_within(
            work_table,
            lower_stage,
            axes="yz",
            inner_elem="trunnion_shaft",
            outer_elem=bearing_name,
            margin=0.002,
            name=f"shaft centered in {bearing_name}",
        )
        ctx.expect_overlap(
            work_table,
            lower_stage,
            axes="x",
            elem_a="trunnion_shaft",
            elem_b=bearing_name,
            min_overlap=0.06,
            name=f"shaft retained by {bearing_name}",
        )

    plate_aabb = ctx.part_element_world_aabb(work_table, elem="table_plate")
    shaft_aabb = ctx.part_element_world_aabb(work_table, elem="trunnion_shaft")
    ctx.check(
        "work table hangs below trunnion axis",
        plate_aabb is not None
        and shaft_aabb is not None
        and plate_aabb[1][2] < shaft_aabb[0][2] - 0.10,
        details=f"plate_aabb={plate_aabb}, shaft_aabb={shaft_aabb}",
    )

    def _aabb_center_z(aabb):
        return None if aabb is None else 0.5 * (aabb[0][2] + aabb[1][2])

    rest_plate_z = _aabb_center_z(plate_aabb)
    with ctx.pose({trunnion_joint: 0.8}):
        tilted_plate_z = _aabb_center_z(ctx.part_element_world_aabb(work_table, elem="table_plate"))
    ctx.check(
        "trunnion tilt moves hanging table",
        rest_plate_z is not None and tilted_plate_z is not None and tilted_plate_z > rest_plate_z + 0.035,
        details=f"rest_z={rest_plate_z}, tilted_z={tilted_plate_z}",
    )

    def _aabb_center_xy(aabb):
        return None if aabb is None else (0.5 * (aabb[0][0] + aabb[1][0]), 0.5 * (aabb[0][1] + aabb[1][1]))

    rest_bearing_xy = _aabb_center_xy(ctx.part_element_world_aabb(lower_stage, elem="bearing_1"))
    with ctx.pose({yaw_joint: math.pi / 2.0}):
        turned_bearing_xy = _aabb_center_xy(ctx.part_element_world_aabb(lower_stage, elem="bearing_1"))
    ctx.check(
        "lower rotary stage yaws about top support",
        rest_bearing_xy is not None
        and turned_bearing_xy is not None
        and abs(turned_bearing_xy[1] - rest_bearing_xy[1]) > 0.45,
        details=f"rest_xy={rest_bearing_xy}, turned_xy={turned_bearing_xy}",
    )

    return ctx.report()


object_model = build_object_model()

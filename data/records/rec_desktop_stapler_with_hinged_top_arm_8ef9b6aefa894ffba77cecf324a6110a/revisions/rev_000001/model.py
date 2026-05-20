from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.160
BASE_WIDTH = 0.048
BASE_HEIGHT = 0.013
BASE_FRONT_PAD_LENGTH = 0.036
BASE_FRONT_PAD_WIDTH = 0.028
BASE_FRONT_PAD_HEIGHT = 0.004
HINGE_X = -0.062
HINGE_Z = 0.021
HINGE_TOWER_LENGTH = 0.018
HINGE_TOWER_WIDTH = 0.006
HINGE_TOWER_HEIGHT = 0.011
HINGE_TOWER_Y = 0.023

ARM_FRONT_X = 0.148
ARM_REAR_OVERHANG = 0.008
ARM_WIDTH = 0.036
ARM_HEIGHT = 0.020
ARM_BOTTOM_Z = -0.002
ARM_INNER_WIDTH = 0.028
ARM_INNER_HEIGHT = 0.017
ARM_FRONT_WALL = 0.010
ARM_REAR_WALL = 0.012
HINGE_LUG_LENGTH = 0.010
HINGE_LUG_WIDTH = 0.007
HINGE_LUG_HEIGHT = 0.006

TRAY_LENGTH = 0.122
TRAY_WIDTH = 0.024
TRAY_FLOOR_HEIGHT = 0.0016
TRAY_SIDE_THICKNESS = 0.002
TRAY_SIDE_HEIGHT = 0.006
TRAY_FRONT_BLOCK_LENGTH = 0.010
TRAY_FRONT_BLOCK_HEIGHT = 0.0075
TRAY_TRAVEL = 0.060
TRAY_JOINT_X = ARM_FRONT_X - ARM_FRONT_WALL

LATCH_PAD_LENGTH = 0.010
LATCH_PAD_WIDTH = 0.018
LATCH_PAD_HEIGHT = 0.0055
LATCH_STEM_LENGTH = 0.007
LATCH_STEM_WIDTH = 0.007
LATCH_STEM_HEIGHT = 0.008
LATCH_TRAVEL = 0.006


def _box_xy(length: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height, centered=(False, True, False))


def _base_shape() -> cq.Workplane:
    slab = cq.Workplane("XY").box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, centered=(True, True, False))

    front_pad = (
        cq.Workplane("XY")
        .box(BASE_FRONT_PAD_LENGTH, BASE_FRONT_PAD_WIDTH, BASE_FRONT_PAD_HEIGHT, centered=(True, True, False))
        .translate((0.050, 0.0, BASE_HEIGHT - 0.0005))
    )

    tower_z = BASE_HEIGHT - 0.0005
    tower_left = (
        cq.Workplane("XY")
        .box(HINGE_TOWER_LENGTH, HINGE_TOWER_WIDTH, HINGE_TOWER_HEIGHT, centered=(True, True, False))
        .translate((HINGE_X, HINGE_TOWER_Y, tower_z))
    )
    tower_right = (
        cq.Workplane("XY")
        .box(HINGE_TOWER_LENGTH, HINGE_TOWER_WIDTH, HINGE_TOWER_HEIGHT, centered=(True, True, False))
        .translate((HINGE_X, -HINGE_TOWER_Y, tower_z))
    )

    return slab.union(front_pad).union(tower_left).union(tower_right)


def _arm_shape() -> cq.Workplane:
    total_length = ARM_FRONT_X + ARM_REAR_OVERHANG
    outer = _box_xy(total_length, ARM_WIDTH, ARM_HEIGHT).translate((-
        ARM_REAR_OVERHANG,
        0.0,
        ARM_BOTTOM_Z,
    ))

    cavity_length = ARM_FRONT_X - ARM_FRONT_WALL - ARM_REAR_WALL
    cavity = _box_xy(cavity_length, ARM_INNER_WIDTH, ARM_INNER_HEIGHT).translate(
        (
            ARM_REAR_WALL,
            0.0,
            ARM_BOTTOM_Z - 0.001,
        )
    )

    front_mouth = _box_xy(ARM_FRONT_WALL + 0.004, ARM_INNER_WIDTH - 0.002, 0.009).translate(
        (
            ARM_FRONT_X - ARM_FRONT_WALL,
            0.0,
            ARM_BOTTOM_Z + 0.001,
        )
    )

    latch_slot = _box_xy(0.020, 0.022, 0.012).translate((-0.010, 0.0, 0.003))

    lug_positive = _box_xy(HINGE_LUG_LENGTH, HINGE_LUG_WIDTH, HINGE_LUG_HEIGHT).translate(
        (-0.002, 0.0145, -0.003)
    )
    lug_negative = _box_xy(HINGE_LUG_LENGTH, HINGE_LUG_WIDTH, HINGE_LUG_HEIGHT).translate(
        (-0.002, -0.0145, -0.003)
    )

    return outer.cut(cavity).cut(front_mouth).cut(latch_slot).union(lug_positive).union(lug_negative)


def _tray_shape() -> cq.Workplane:
    floor = _box_xy(TRAY_LENGTH, TRAY_WIDTH - (2.0 * TRAY_SIDE_THICKNESS), TRAY_FLOOR_HEIGHT).translate(
        (-TRAY_LENGTH, 0.0, -0.0015)
    )
    side_positive = _box_xy(TRAY_LENGTH, TRAY_SIDE_THICKNESS, TRAY_SIDE_HEIGHT).translate(
        (-TRAY_LENGTH, 0.011, -0.0015)
    )
    side_negative = _box_xy(TRAY_LENGTH, TRAY_SIDE_THICKNESS, TRAY_SIDE_HEIGHT).translate(
        (-TRAY_LENGTH, -0.011, -0.0015)
    )
    front_block = _box_xy(TRAY_FRONT_BLOCK_LENGTH, TRAY_WIDTH, TRAY_FRONT_BLOCK_HEIGHT).translate(
        (0.0, 0.0, -0.0005)
    )
    rail_positive = _box_xy(TRAY_LENGTH - 0.012, 0.002, 0.0018).translate((-TRAY_LENGTH + 0.006, 0.009, 0.003))
    rail_negative = _box_xy(TRAY_LENGTH - 0.012, 0.002, 0.0018).translate((-TRAY_LENGTH + 0.006, -0.009, 0.003))
    return floor.union(side_positive).union(side_negative).union(front_block).union(rail_positive).union(rail_negative)


def _latch_shape() -> cq.Workplane:
    pad = cq.Workplane("XY").box(LATCH_PAD_LENGTH, LATCH_PAD_WIDTH, LATCH_PAD_HEIGHT).translate((-0.005, 0.0, 0.0))
    stem = cq.Workplane("XY").box(LATCH_STEM_LENGTH, LATCH_STEM_WIDTH, LATCH_STEM_HEIGHT).translate((0.003, 0.0, 0.0))
    return pad.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_stapler")

    model.material("painted_steel", rgba=(0.45, 0.48, 0.52, 1.0))
    model.material("brushed_steel", rgba=(0.71, 0.73, 0.76, 1.0))
    model.material("polished_steel", rgba=(0.79, 0.80, 0.82, 1.0))
    model.material("dark_insert", rgba=(0.20, 0.21, 0.23, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "base_shell"), material="painted_steel", name="base_shell")
    base.visual(
        Box((0.026, 0.018, 0.003)),
        origin=Origin(xyz=(0.056, 0.0, BASE_HEIGHT + 0.0015)),
        material="dark_insert",
        name="anvil_plate",
    )

    arm = model.part("arm")
    arm.visual(mesh_from_cadquery(_arm_shape(), "arm_shell"), material="brushed_steel", name="arm_shell")

    tray = model.part("tray")
    tray.visual(mesh_from_cadquery(_tray_shape(), "tray_body"), material="polished_steel", name="tray_body")

    latch = model.part("latch")
    latch.visual(mesh_from_cadquery(_latch_shape(), "latch_body"), material="dark_insert", name="latch_body")

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.95, effort=25.0, velocity=2.0),
    )
    model.articulation(
        "arm_to_tray",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=tray,
        origin=Origin(xyz=(TRAY_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAY_TRAVEL, effort=12.0, velocity=0.20),
    )
    model.articulation(
        "arm_to_latch",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=latch,
        origin=Origin(xyz=(-ARM_REAR_OVERHANG, 0.0, 0.009)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=LATCH_TRAVEL, effort=8.0, velocity=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    tray = object_model.get_part("tray")
    latch = object_model.get_part("latch")
    hinge = object_model.get_articulation("base_to_arm")
    tray_slide = object_model.get_articulation("arm_to_tray")
    latch_slide = object_model.get_articulation("arm_to_latch")

    hinge_clearance_reason = (
        "The upper stapler assembly is visually carried by the rear hinge knuckles, "
        "with a small clearance left around the simplified hidden hinge pin."
    )
    ctx.allow_isolated_part(arm, reason=hinge_clearance_reason)
    ctx.allow_isolated_part(tray, reason=hinge_clearance_reason)
    ctx.allow_isolated_part(latch, reason=hinge_clearance_reason)

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            positive_elem="arm_shell",
            negative_elem="anvil_plate",
            min_gap=0.001,
            max_gap=0.010,
            name="closed arm hovers just above the anvil",
        )
        ctx.expect_overlap(
            arm,
            base,
            axes="xy",
            elem_a="arm_shell",
            elem_b="base_shell",
            min_overlap=0.020,
            name="closed arm stays over the base footprint",
        )
        ctx.expect_within(
            tray,
            arm,
            axes="yz",
            inner_elem="tray_body",
            outer_elem="arm_shell",
            margin=0.002,
            name="closed tray stays within the arm guide width and height",
        )
        ctx.expect_overlap(
            tray,
            arm,
            axes="x",
            elem_a="tray_body",
            elem_b="arm_shell",
            min_overlap=0.110,
            name="closed tray remains deeply inserted in the arm",
        )
        ctx.expect_contact(
            tray,
            arm,
            elem_a="tray_body",
            elem_b="arm_shell",
            name="tray front stop seats against the arm front opening",
        )
    limits = hinge.motion_limits
    if limits is not None and limits.upper is not None:
        rest_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")
        with ctx.pose({hinge: limits.upper}):
            open_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")
            ctx.check(
                "arm opens upward from the rear hinge",
                rest_aabb is not None and open_aabb is not None and open_aabb[1][2] > rest_aabb[1][2] + 0.045,
                details=f"rest={rest_aabb}, open={open_aabb}",
            )

    tray_limits = tray_slide.motion_limits
    if tray_limits is not None and tray_limits.upper is not None:
        tray_rest = ctx.part_world_position(tray)
        with ctx.pose({tray_slide: tray_limits.upper}):
            tray_open = ctx.part_world_position(tray)
            ctx.expect_within(
                tray,
                arm,
                axes="yz",
                inner_elem="tray_body",
                outer_elem="arm_shell",
                margin=0.002,
                name="extended tray stays aligned with the arm rails",
            )
            ctx.expect_overlap(
                tray,
                arm,
                axes="x",
                elem_a="tray_body",
                elem_b="arm_shell",
                min_overlap=0.050,
                name="extended tray retains magazine engagement",
            )
        ctx.check(
            "tray slides forward out of the arm",
            tray_rest is not None and tray_open is not None and tray_open[0] > tray_rest[0] + 0.050,
            details=f"rest={tray_rest}, open={tray_open}",
        )

    latch_limits = latch_slide.motion_limits
    if latch_limits is not None and latch_limits.upper is not None:
        latch_rest = ctx.part_world_position(latch)
        with ctx.pose({latch_slide: latch_limits.upper}):
            latch_pressed = ctx.part_world_position(latch)
        ctx.check(
            "rear latch presses inward toward the tray",
            latch_rest is not None and latch_pressed is not None and latch_pressed[0] > latch_rest[0] + 0.004,
            details=f"rest={latch_rest}, pressed={latch_pressed}",
        )

    return ctx.report()


object_model = build_object_model()

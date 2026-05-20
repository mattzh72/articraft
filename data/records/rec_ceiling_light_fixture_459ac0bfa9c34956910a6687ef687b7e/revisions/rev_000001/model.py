from __future__ import annotations

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


CANOPY_PLATE_RADIUS = 0.090
CANOPY_PLATE_THICKNESS = 0.004
CANOPY_BODY_RADIUS = 0.060
CANOPY_BODY_HEIGHT = 0.028
PAN_BOSS_RADIUS = 0.011
PAN_BOSS_HEIGHT = 0.008
PAN_OFFSET_X = 0.052
PAN_ORIGIN_Z = -0.032

YOKE_CAP_RADIUS = 0.013
YOKE_CAP_HEIGHT = 0.006
YOKE_STEM_RADIUS = 0.0065
YOKE_STEM_HEIGHT = 0.016
YOKE_BLOCK_SIZE = (0.022, 0.024, 0.020)
YOKE_BRIDGE_SIZE = (0.108, 0.028, 0.008)
YOKE_ARM_SIZE = (0.008, 0.032, 0.050)
YOKE_ARM_X = 0.050
YOKE_ARM_Y = 0.050
YOKE_ARM_Z = -0.049
TILT_ORIGIN = (0.0, 0.062, -0.052)

HEAD_OUTER_RADIUS = 0.031
HEAD_INNER_RADIUS = 0.0275
HEAD_BODY_LENGTH = 0.086
HEAD_REAR_WALL = 0.010
HEAD_BEZEL_RADIUS = 0.0345
HEAD_BEZEL_LENGTH = 0.008
HEAD_REAR_CAP_RADIUS = 0.022
HEAD_REAR_CAP_LENGTH = 0.012
HEAD_PIVOT_OFFSET = 0.028
TRUNNION_RADIUS = 0.0065
TRUNNION_LENGTH = 0.015
TRUNNION_CENTER_X = 0.0385


def _build_head_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XZ")
        .circle(HEAD_OUTER_RADIUS)
        .extrude(HEAD_BODY_LENGTH / 2.0, both=True)
    )
    shell = (
        shell.faces(">Y")
        .workplane()
        .circle(HEAD_INNER_RADIUS)
        .cutBlind(-(HEAD_BODY_LENGTH - HEAD_REAR_WALL))
    )
    shell = shell.faces(">Y").workplane().circle(HEAD_BEZEL_RADIUS).extrude(HEAD_BEZEL_LENGTH)
    shell = (
        shell.faces(">Y")
        .workplane()
        .circle(HEAD_INNER_RADIUS)
        .cutBlind(-(HEAD_BODY_LENGTH + HEAD_BEZEL_LENGTH - HEAD_REAR_WALL))
    )
    shell = (
        shell.faces("<Y")
        .workplane()
        .circle(HEAD_REAR_CAP_RADIUS)
        .extrude(-HEAD_REAR_CAP_LENGTH)
    )
    return shell.translate((0.0, HEAD_PIVOT_OFFSET, 0.0))


def _add_yoke(model: ArticulatedObject, name: str, material: str) -> None:
    yoke = model.part(name)
    yoke.visual(
        Cylinder(radius=YOKE_CAP_RADIUS, length=YOKE_CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, -(YOKE_CAP_HEIGHT / 2.0))),
        material=material,
        name="pan_cap",
    )
    yoke.visual(
        Cylinder(radius=YOKE_STEM_RADIUS, length=YOKE_STEM_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, -(YOKE_STEM_HEIGHT / 2.0) - 0.004)),
        material=material,
        name="pan_stem",
    )
    yoke.visual(
        Box(YOKE_BLOCK_SIZE),
        origin=Origin(xyz=(0.0, 0.011, -0.024)),
        material=material,
        name="pan_block",
    )
    yoke.visual(
        Box(YOKE_BRIDGE_SIZE),
        origin=Origin(xyz=(0.0, 0.026, -0.024)),
        material=material,
        name="bridge",
    )
    yoke.visual(
        Box(YOKE_ARM_SIZE),
        origin=Origin(xyz=(-YOKE_ARM_X, YOKE_ARM_Y, YOKE_ARM_Z)),
        material=material,
        name="arm_0",
    )
    yoke.visual(
        Box(YOKE_ARM_SIZE),
        origin=Origin(xyz=(YOKE_ARM_X, YOKE_ARM_Y, YOKE_ARM_Z)),
        material=material,
        name="arm_1",
    )


def _add_head(model: ArticulatedObject, name: str, material: str, trim_material: str) -> None:
    head = model.part(name)
    head.visual(
        mesh_from_cadquery(_build_head_shell(), f"{name}_shell"),
        material=material,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(
            xyz=(-TRUNNION_CENTER_X, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_material,
        name="trunnion_0",
    )
    head.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(
            xyz=(TRUNNION_CENTER_X, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_material,
        name="trunnion_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_spotlight_fixture")

    canopy_white = model.material("canopy_white", rgba=(0.95, 0.95, 0.93, 1.0))
    fixture_black = model.material("fixture_black", rgba=(0.15, 0.15, 0.16, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        Cylinder(radius=CANOPY_PLATE_RADIUS, length=CANOPY_PLATE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -(CANOPY_PLATE_THICKNESS / 2.0))),
        material=canopy_white,
        name="ceiling_plate",
    )
    canopy.visual(
        Cylinder(radius=CANOPY_BODY_RADIUS, length=CANOPY_BODY_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, -(CANOPY_PLATE_THICKNESS + (CANOPY_BODY_HEIGHT / 2.0))),
        ),
        material=canopy_white,
        name="canopy_body",
    )
    canopy.visual(
        Cylinder(radius=PAN_BOSS_RADIUS, length=PAN_BOSS_HEIGHT),
        origin=Origin(
            xyz=(-PAN_OFFSET_X, 0.0, PAN_ORIGIN_Z + (PAN_BOSS_HEIGHT / 2.0)),
        ),
        material=trim_black,
        name="pan_boss_0",
    )
    canopy.visual(
        Cylinder(radius=PAN_BOSS_RADIUS, length=PAN_BOSS_HEIGHT),
        origin=Origin(
            xyz=(PAN_OFFSET_X, 0.0, PAN_ORIGIN_Z + (PAN_BOSS_HEIGHT / 2.0)),
        ),
        material=trim_black,
        name="pan_boss_1",
    )

    _add_yoke(model, "yoke_0", "fixture_black")
    _add_yoke(model, "yoke_1", "fixture_black")
    _add_head(model, "head_0", "fixture_black", "trim_black")
    _add_head(model, "head_1", "fixture_black", "trim_black")

    model.articulation(
        "pan_0",
        ArticulationType.REVOLUTE,
        parent=canopy,
        child="yoke_0",
        origin=Origin(xyz=(-PAN_OFFSET_X, 0.0, PAN_ORIGIN_Z), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.2,
            upper=1.2,
            effort=8.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "pan_1",
        ArticulationType.REVOLUTE,
        parent=canopy,
        child="yoke_1",
        origin=Origin(xyz=(PAN_OFFSET_X, 0.0, PAN_ORIGIN_Z), rpy=(0.0, 0.0, -math.pi / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.2,
            upper=1.2,
            effort=8.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "tilt_0",
        ArticulationType.REVOLUTE,
        parent="yoke_0",
        child="head_0",
        origin=Origin(xyz=TILT_ORIGIN, rpy=(-0.25, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.45,
            upper=1.25,
            effort=6.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "tilt_1",
        ArticulationType.REVOLUTE,
        parent="yoke_1",
        child="head_1",
        origin=Origin(xyz=TILT_ORIGIN, rpy=(-0.25, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.45,
            upper=1.25,
            effort=6.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    yoke_0 = object_model.get_part("yoke_0")
    yoke_1 = object_model.get_part("yoke_1")
    head_0 = object_model.get_part("head_0")
    head_1 = object_model.get_part("head_1")

    ctx.expect_contact(
        yoke_0,
        canopy,
        elem_a="pan_cap",
        elem_b="pan_boss_0",
        name="yoke_0 seats against canopy",
    )
    ctx.expect_contact(
        yoke_1,
        canopy,
        elem_a="pan_cap",
        elem_b="pan_boss_1",
        name="yoke_1 seats against canopy",
    )
    ctx.expect_contact(
        head_0,
        yoke_0,
        elem_a="trunnion_0",
        elem_b="arm_0",
        name="head_0 is carried by yoke_0 arm_0",
    )
    ctx.expect_contact(
        head_0,
        yoke_0,
        elem_a="trunnion_1",
        elem_b="arm_1",
        name="head_0 is carried by yoke_0 arm_1",
    )
    ctx.expect_contact(
        head_1,
        yoke_1,
        elem_a="trunnion_0",
        elem_b="arm_0",
        name="head_1 is carried by yoke_1 arm_0",
    )
    ctx.expect_contact(
        head_1,
        yoke_1,
        elem_a="trunnion_1",
        elem_b="arm_1",
        name="head_1 is carried by yoke_1 arm_1",
    )
    ctx.expect_origin_gap(
        canopy,
        head_0,
        axis="z",
        min_gap=0.040,
        name="head_0 hangs below the canopy",
    )
    ctx.expect_origin_gap(
        canopy,
        head_1,
        axis="z",
        min_gap=0.040,
        name="head_1 hangs below the canopy",
    )

    rest_pan_pos = ctx.part_world_position(head_0)
    with ctx.pose(pan_0=1.0):
        panned_pos = ctx.part_world_position(head_0)
    ctx.check(
        "pan_0 swings head_0 around the canopy",
        rest_pan_pos is not None
        and panned_pos is not None
        and math.hypot(
            panned_pos[0] - rest_pan_pos[0],
            panned_pos[1] - rest_pan_pos[1],
        )
        > 0.018,
        details=f"rest={rest_pan_pos}, panned={panned_pos}",
    )

    shell_rest = ctx.part_element_world_aabb(head_0, elem="head_shell")
    with ctx.pose(tilt_0=1.0):
        shell_tilt = ctx.part_element_world_aabb(head_0, elem="head_shell")
    ctx.check(
        "tilt_0 pitches head_0 downward",
        shell_rest is not None
        and shell_tilt is not None
        and shell_tilt[0][2] < shell_rest[0][2] - 0.015,
        details=f"rest={shell_rest}, tilted={shell_tilt}",
    )

    return ctx.report()


object_model = build_object_model()

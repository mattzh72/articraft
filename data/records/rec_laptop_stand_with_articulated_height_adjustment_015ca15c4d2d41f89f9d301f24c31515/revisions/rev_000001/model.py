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


BASE_DEPTH = 0.220
BASE_WIDTH = 0.270
BASE_THICKNESS = 0.010

TRAY_DEPTH = 0.250
TRAY_WIDTH = 0.285
TRAY_THICKNESS = 0.010
TRAY_PANEL_OFFSET = 0.010
TRAY_NOMINAL_ANGLE = math.radians(30.0)
TRAY_TEST_DELTA = math.radians(12.0)

HINGE_X = -0.108
HINGE_Z = 0.017
LOWER_PIVOT_X = -0.030
LOWER_PIVOT_Z = 0.018

UPPER_PROP_MOUNT_X = 0.160
UPPER_PROP_MOUNT_Z = -0.012

UPPER_PIVOT_WORLD_X = HINGE_X + (
    math.cos(TRAY_NOMINAL_ANGLE) * UPPER_PROP_MOUNT_X
    - math.sin(TRAY_NOMINAL_ANGLE) * UPPER_PROP_MOUNT_Z
)
UPPER_PIVOT_WORLD_Z = HINGE_Z + (
    math.sin(TRAY_NOMINAL_ANGLE) * UPPER_PROP_MOUNT_X
    + math.cos(TRAY_NOMINAL_ANGLE) * UPPER_PROP_MOUNT_Z
)
PROP_SPAN_X = UPPER_PIVOT_WORLD_X - LOWER_PIVOT_X
PROP_SPAN_Z = UPPER_PIVOT_WORLD_Z - LOWER_PIVOT_Z
PROP_NOMINAL_ANGLE = math.atan2(PROP_SPAN_Z, PROP_SPAN_X)
PROP_SPAN = math.hypot(PROP_SPAN_X, PROP_SPAN_Z)

OUTER_SLEEVE_START = 0.012
STAGE_JOINT_OFFSET = -0.006
OUTER_SLEEVE_LENGTH = PROP_SPAN + STAGE_JOINT_OFFSET - OUTER_SLEEVE_START
STAGE_EXTENSION = 0.016
STAGE_LENGTH = PROP_SPAN - 0.020


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_laptop_stand")

    anodized = model.material("anodized", rgba=(0.48, 0.50, 0.54, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_DEPTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=graphite,
        name="base_plate",
    )
    base.visual(
        Box((0.170, 0.028, 0.002)),
        origin=Origin(xyz=(0.028, 0.0, BASE_THICKNESS + 0.001)),
        material=rubber,
        name="base_pad",
    )
    base.visual(
        Box((0.016, 0.096, 0.004)),
        origin=Origin(xyz=(HINGE_X + 0.010, 0.0, HINGE_Z - 0.004)),
        material=anodized,
        name="hinge_leaf",
    )
    for index, y in enumerate((-0.079, 0.079)):
        base.visual(
            Cylinder(radius=0.007, length=0.062),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=anodized,
            name=f"hinge_knuckle_{index}",
        )
    base.visual(
        Box((0.030, 0.026, 0.006)),
        origin=Origin(xyz=(LOWER_PIVOT_X, 0.0, BASE_THICKNESS + 0.003)),
        material=anodized,
        name="prop_pedestal",
    )
    for index, y in enumerate((-0.0105, 0.0105)):
        base.visual(
            Box((0.012, 0.005, 0.020)),
            origin=Origin(xyz=(LOWER_PIVOT_X, y, BASE_THICKNESS + 0.010)),
            material=anodized,
            name=f"lower_cheek_{index}",
        )

    tray = model.part("tray")
    tray.visual(
        Box((TRAY_DEPTH, TRAY_WIDTH, TRAY_THICKNESS)),
        origin=Origin(xyz=(TRAY_PANEL_OFFSET + TRAY_DEPTH / 2.0, 0.0, TRAY_THICKNESS / 2.0)),
        material=anodized,
        name="tray_panel",
    )
    tray.visual(
        Box((0.010, 0.255, 0.018)),
        origin=Origin(
            xyz=(TRAY_PANEL_OFFSET + TRAY_DEPTH - 0.005, 0.0, TRAY_THICKNESS + 0.009)
        ),
        material=anodized,
        name="front_lip",
    )
    tray.visual(
        Box((0.150, 0.028, 0.002)),
        origin=Origin(xyz=(TRAY_PANEL_OFFSET + 0.140, -0.072, TRAY_THICKNESS + 0.001)),
        material=rubber,
        name="pad_0",
    )
    tray.visual(
        Box((0.150, 0.028, 0.002)),
        origin=Origin(xyz=(TRAY_PANEL_OFFSET + 0.140, 0.072, TRAY_THICKNESS + 0.001)),
        material=rubber,
        name="pad_1",
    )
    tray.visual(
        Cylinder(radius=0.007, length=0.096),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="hinge_barrel",
    )
    tray.visual(
        Box((0.018, 0.096, 0.004)),
        origin=Origin(xyz=(0.010, 0.0, 0.004)),
        material=anodized,
        name="hinge_leaf",
    )
    tray.visual(
        Box((0.155, 0.034, 0.006)),
        origin=Origin(xyz=(0.088, 0.0, -0.003)),
        material=graphite,
        name="spine_rib",
    )
    tray.visual(
        Box((0.034, 0.024, 0.008)),
        origin=Origin(xyz=(UPPER_PROP_MOUNT_X, 0.0, -0.003)),
        material=graphite,
        name="upper_bridge",
    )
    for index, y in enumerate((-0.0105, 0.0105)):
        tray.visual(
            Box((0.012, 0.005, 0.018)),
            origin=Origin(xyz=(UPPER_PROP_MOUNT_X, y, -0.010)),
            material=graphite,
            name=f"upper_cheek_{index}",
        )

    prop_outer = model.part("prop_outer")
    prop_outer.visual(
        Cylinder(radius=0.0038, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="lower_barrel",
    )
    prop_outer.visual(
        Box((0.024, 0.016, 0.004)),
        origin=Origin(xyz=(0.012, 0.0, -0.007)),
        material=graphite,
        name="lower_bridge",
    )
    prop_outer.visual(
        Box((0.008, 0.003, 0.006)),
        origin=Origin(xyz=(0.004, -0.00575, -0.002)),
        material=graphite,
        name="lower_lug_0",
    )
    prop_outer.visual(
        Box((0.008, 0.003, 0.006)),
        origin=Origin(xyz=(0.004, 0.00575, -0.002)),
        material=graphite,
        name="lower_lug_1",
    )
    prop_outer.visual(
        Box((OUTER_SLEEVE_LENGTH, 0.0025, 0.010)),
        origin=Origin(xyz=(OUTER_SLEEVE_START + OUTER_SLEEVE_LENGTH / 2.0, -0.00575, 0.0)),
        material=graphite,
        name="sleeve_0",
    )
    prop_outer.visual(
        Box((OUTER_SLEEVE_LENGTH, 0.0025, 0.010)),
        origin=Origin(xyz=(OUTER_SLEEVE_START + OUTER_SLEEVE_LENGTH / 2.0, 0.00575, 0.0)),
        material=graphite,
        name="sleeve_1",
    )
    prop_outer.visual(
        Box((OUTER_SLEEVE_LENGTH, 0.0105, 0.002)),
        origin=Origin(xyz=(OUTER_SLEEVE_START + OUTER_SLEEVE_LENGTH / 2.0, 0.0, -0.0042)),
        material=graphite,
        name="sleeve_back",
    )

    prop_head = model.part("prop_head")
    prop_head.visual(
        Cylinder(radius=0.0038, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="upper_barrel",
    )

    prop_stage = model.part("prop_stage")
    prop_stage.visual(
        Box((STAGE_LENGTH, 0.009, 0.0046)),
        origin=Origin(xyz=(-STAGE_LENGTH / 2.0, 0.0, 0.0)),
        material=anodized,
        name="stage_rod",
    )

    model.articulation(
        "base_to_tray",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tray,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(0.0, -TRAY_NOMINAL_ANGLE, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=-math.radians(18.0),
            upper=math.radians(16.0),
        ),
    )
    model.articulation(
        "base_to_prop_outer",
        ArticulationType.REVOLUTE,
        parent=base,
        child=prop_outer,
        origin=Origin(
            xyz=(LOWER_PIVOT_X, 0.0, LOWER_PIVOT_Z),
            rpy=(0.0, -PROP_NOMINAL_ANGLE, 0.0),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=-math.radians(12.0),
            upper=math.radians(18.0),
        ),
    )
    model.articulation(
        "tray_to_prop_head",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=prop_head,
        origin=Origin(
            xyz=(UPPER_PROP_MOUNT_X, 0.0, UPPER_PROP_MOUNT_Z),
            rpy=(0.0, TRAY_NOMINAL_ANGLE - PROP_NOMINAL_ANGLE, 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=-math.radians(18.0),
            upper=math.radians(18.0),
        ),
    )
    model.articulation(
        "prop_head_to_prop_stage",
        ArticulationType.PRISMATIC,
        parent=prop_head,
        child=prop_stage,
        origin=Origin(xyz=(STAGE_JOINT_OFFSET, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.12,
            lower=0.0,
            upper=STAGE_EXTENSION,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    prop_outer = object_model.get_part("prop_outer")
    prop_stage = object_model.get_part("prop_stage")

    tray_hinge = object_model.get_articulation("base_to_tray")
    stage_slide = object_model.get_articulation("prop_head_to_prop_stage")

    ctx.expect_gap(
        tray,
        base,
        axis="z",
        positive_elem="tray_panel",
        negative_elem="base_plate",
        min_gap=0.006,
        name="tray panel clears the lower base",
    )
    ctx.expect_within(
        prop_stage,
        prop_outer,
        axes="y",
        inner_elem="stage_rod",
        margin=0.003,
        name="stage rod stays laterally centered in the sleeve",
    )
    ctx.expect_overlap(
        prop_stage,
        prop_outer,
        axes="x",
        elem_a="stage_rod",
        min_overlap=0.035,
        name="collapsed stage remains telescopically inserted",
    )

    rest_front = ctx.part_element_world_aabb(tray, elem="front_lip")
    with ctx.pose({tray_hinge: TRAY_TEST_DELTA}):
        raised_front = ctx.part_element_world_aabb(tray, elem="front_lip")
    ctx.check(
        "tray raises higher when the rear hinge opens",
        rest_front is not None
        and raised_front is not None
        and raised_front[1][2] > rest_front[1][2] + 0.020,
        details=f"rest={rest_front}, raised={raised_front}",
    )

    rest_stage_pos = ctx.part_world_position(prop_stage)
    with ctx.pose({stage_slide: STAGE_EXTENSION}):
        ctx.expect_within(
            prop_stage,
            prop_outer,
            axes="y",
            inner_elem="stage_rod",
            margin=0.003,
            name="extended stage stays laterally centered in the sleeve",
        )
        ctx.expect_overlap(
            prop_stage,
            prop_outer,
            axes="x",
            elem_a="stage_rod",
            min_overlap=0.026,
            name="extended stage retains insertion in the sleeve",
        )
        extended_stage_pos = ctx.part_world_position(prop_stage)
    ctx.check(
        "stage extends down the prop axis",
        rest_stage_pos is not None
        and extended_stage_pos is not None
        and extended_stage_pos[0] < rest_stage_pos[0] - 0.010
        and extended_stage_pos[2] < rest_stage_pos[2] - 0.010,
        details=f"rest={rest_stage_pos}, extended={extended_stage_pos}",
    )

    return ctx.report()


object_model = build_object_model()

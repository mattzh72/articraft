from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_WIDTH = 0.30
BASE_DEPTH = 0.24
BASE_THICK = 0.008
TRAY_WIDTH = 0.28
TRAY_DEPTH = 0.24
TRAY_THICK = 0.008

HINGE_Y = -0.096
HINGE_Z = 0.028
HINGE_RADIUS = 0.006

BASE_PIVOT_Y = 0.082
BASE_PIVOT_Z = 0.014
TRAY_PIVOT_LOCAL_Y = 0.160
TRAY_PIVOT_LOCAL_Z = -0.004

LOWER_ARM_LENGTH = 0.048
UPPER_ARM_LENGTH = 0.030
LOWER_ARM_REST = -0.200
UPPER_ARM_REST = 0.030


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laptop_stand")

    model.material("anodized_silver", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("graphite", rgba=(0.31, 0.33, 0.36, 1.0))
    model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("steel_pin", rgba=(0.67, 0.69, 0.72, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICK)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICK * 0.5)),
        material="anodized_silver",
        name="deck",
    )
    base.visual(
        Box((0.18, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, -0.085, BASE_THICK + 0.003)),
        material="rubber",
        name="front_pad",
    )
    base.visual(
        Box((0.18, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.108, BASE_THICK + 0.003)),
        material="rubber",
        name="rear_pad",
    )
    for x_pos, name in ((-0.084, "hinge_barrel_0"), (0.084, "hinge_barrel_1")):
        base.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.036),
            origin=Origin(
                xyz=(x_pos, HINGE_Y, HINGE_Z),
                rpy=(0.0, 1.57079632679 / 1.0, 0.0),
            ),
            material="steel_pin",
            name=name,
        )
        base.visual(
            Box((0.028, 0.024, 0.020)),
            origin=Origin(xyz=(x_pos, HINGE_Y + 0.004, HINGE_Z - 0.010)),
            material="graphite",
            name=f"{name}_cheek",
        )
    for x_pos, name in ((-0.065, "arm_pivot_0"), (0.065, "arm_pivot_1")):
        base.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(
                xyz=(x_pos, BASE_PIVOT_Y, BASE_PIVOT_Z),
                rpy=(0.0, 1.57079632679 / 1.0, 0.0),
            ),
            material="steel_pin",
            name=name,
        )
        base.visual(
            Box((0.026, 0.022, 0.022)),
            origin=Origin(xyz=(x_pos, BASE_PIVOT_Y, BASE_PIVOT_Z - 0.011)),
            material="graphite",
            name=f"{name}_cheek",
        )

    tray = model.part("tray")
    tray.visual(
        Box((TRAY_WIDTH, TRAY_DEPTH, TRAY_THICK)),
        origin=Origin(xyz=(0.0, 0.128, TRAY_THICK * 0.5)),
        material="anodized_silver",
        name="tray_panel",
    )
    tray.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.136),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, 1.57079632679 / 1.0, 0.0),
        ),
        material="steel_pin",
        name="hinge_barrel",
    )
    tray.visual(
        Box((0.150, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.008, 0.005)),
        material="graphite",
        name="hinge_bridge",
    )
    tray.visual(
        Box((0.25, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.012, 0.012)),
        material="graphite",
        name="front_stop",
    )
    tray.visual(
        Box((0.012, 0.180, 0.012)),
        origin=Origin(xyz=(-0.108, 0.125, 0.010)),
        material="graphite",
        name="rail_0",
    )
    tray.visual(
        Box((0.012, 0.180, 0.012)),
        origin=Origin(xyz=(0.108, 0.125, 0.010)),
        material="graphite",
        name="rail_1",
    )
    tray.visual(
        Box((0.16, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.198, 0.014)),
        material="graphite",
        name="rear_rib",
    )
    for x_pos, name in ((-0.065, "arm_pivot_0"), (0.065, "arm_pivot_1")):
        tray.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(
                xyz=(x_pos, TRAY_PIVOT_LOCAL_Y, TRAY_PIVOT_LOCAL_Z),
                rpy=(0.0, 1.57079632679 / 1.0, 0.0),
            ),
            material="steel_pin",
            name=name,
        )
        tray.visual(
            Box((0.026, 0.022, 0.022)),
            origin=Origin(
                xyz=(x_pos, TRAY_PIVOT_LOCAL_Y - 0.003, 0.000)
            ),
            material="graphite",
            name=f"{name}_cheek",
        )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.006, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679 / 1.0, 0.0)),
        material="steel_pin",
        name="pivot_barrel",
    )
    for x_pos, name in ((-0.040, "link_0"), (0.040, "link_1")):
        lower_arm.visual(
            Box((0.014, LOWER_ARM_LENGTH, 0.004)),
            origin=Origin(xyz=(x_pos, -LOWER_ARM_LENGTH * 0.5, 0.0)),
            material="graphite",
            name=name,
        )
    for x_pos, name in ((-0.040, "elbow_barrel_0"), (0.040, "elbow_barrel_1")):
        lower_arm.visual(
            Cylinder(radius=0.006, length=0.018),
            origin=Origin(
                xyz=(x_pos, -LOWER_ARM_LENGTH, 0.0),
                rpy=(0.0, 1.57079632679 / 1.0, 0.0),
            ),
            material="steel_pin",
            name=name,
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.006, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679 / 1.0, 0.0)),
        material="steel_pin",
        name="pivot_barrel",
    )
    for x_pos, name in ((-0.033, "link_0"), (0.033, "link_1")):
        upper_arm.visual(
            Box((0.014, UPPER_ARM_LENGTH, 0.004)),
            origin=Origin(xyz=(x_pos, -UPPER_ARM_LENGTH * 0.5, 0.0)),
            material="graphite",
            name=name,
        )
    upper_arm.visual(
        Cylinder(radius=0.006, length=0.052),
        origin=Origin(
            xyz=(0.0, -UPPER_ARM_LENGTH, 0.0),
            rpy=(0.0, 1.57079632679 / 1.0, 0.0),
        ),
        material="steel_pin",
        name="elbow_barrel",
    )

    tray_hinge = model.articulation(
        "base_to_tray",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.45, effort=20.0, velocity=1.6),
    )
    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, BASE_PIVOT_Y, BASE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-1.20, upper=0.20, effort=10.0, velocity=1.8),
        mimic=Mimic(joint=tray_hinge.name, multiplier=-0.72, offset=LOWER_ARM_REST),
    )
    model.articulation(
        "tray_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=upper_arm,
        origin=Origin(xyz=(0.0, TRAY_PIVOT_LOCAL_Y, TRAY_PIVOT_LOCAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.10, upper=0.80, effort=10.0, velocity=1.8),
        mimic=Mimic(joint=tray_hinge.name, multiplier=0.60, offset=UPPER_ARM_REST),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    tray_hinge = object_model.get_articulation("base_to_tray")

    ctx.expect_gap(
        tray,
        base,
        axis="z",
        positive_elem="tray_panel",
        negative_elem="deck",
        min_gap=0.010,
        max_gap=0.022,
        name="tray panel clears the flat base when collapsed",
    )
    ctx.expect_overlap(
        tray,
        base,
        axes="xy",
        elem_a="tray_panel",
        elem_b="deck",
        min_overlap=0.180,
        name="tray remains broadly aligned over the base footprint",
    )

    closed_aabb = ctx.part_element_world_aabb(tray, elem="tray_panel")
    with ctx.pose({tray_hinge: 0.35}):
        open_aabb = ctx.part_element_world_aabb(tray, elem="tray_panel")

    closed_top = None if closed_aabb is None else closed_aabb[1][2]
    open_top = None if open_aabb is None else open_aabb[1][2]
    ctx.check(
        "tray rear lifts when the front hinge opens",
        closed_top is not None and open_top is not None and open_top > closed_top + 0.08,
        details=f"closed_top={closed_top}, open_top={open_top}",
    )

    return ctx.report()


object_model = build_object_model()

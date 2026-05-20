from __future__ import annotations

from math import pi

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


BASE_L = 0.34
BASE_W = 0.062
BASE_T = 0.012

HINGE_X = 0.026
HINGE_Z = 0.086

ARM_OPEN = 0.95
TRAY_TRAVEL = 0.095


def _arm_shell_shape() -> cq.Workplane:
    rear = cq.Workplane("YZ", origin=(0.030, 0.0, -0.033)).rect(0.036, 0.006)
    front = rear.workplane(offset=0.242).rect(0.030, 0.010)
    return rear.add(front).loft(combine=True)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_desktop_stapler")

    model.material("powder_black", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("zinc", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("steel", rgba=(0.60, 0.63, 0.67, 1.0))

    base = model.part("base")
    base.visual(Box((BASE_L, BASE_W, BASE_T)), origin=Origin(xyz=(BASE_L / 2.0, 0.0, BASE_T / 2.0)), material="graphite", name="base_body")
    base.visual(
        Box((0.050, BASE_W, 0.026)),
        origin=Origin(xyz=(0.025, 0.0, BASE_T + 0.013)),
        material="graphite",
        name="rear_yoke",
    )
    base.visual(
        Box((0.030, 0.008, 0.056)),
        origin=Origin(xyz=(0.026, 0.024, 0.066)),
        material="graphite",
        name="tower_left",
    )
    base.visual(
        Box((0.030, 0.008, 0.056)),
        origin=Origin(xyz=(0.026, -0.024, 0.066)),
        material="graphite",
        name="tower_right",
    )
    base.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(xyz=(HINGE_X, 0.024, HINGE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="graphite",
        name="barrel_left",
    )
    base.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(xyz=(HINGE_X, -0.024, HINGE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="graphite",
        name="barrel_right",
    )
    base.visual(
        Box((0.032, 0.028, 0.006)),
        origin=Origin(xyz=(BASE_L - 0.020, 0.0, BASE_T + 0.003)),
        material="graphite",
        name="front_pad",
    )
    base.visual(
        Box((0.024, 0.014, 0.004)),
        origin=Origin(xyz=(BASE_L - 0.020, 0.0, BASE_T + 0.006)),
        material="steel",
        name="anvil",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_arm_shell_shape(), "arm_shell"),
        material="powder_black",
        name="arm_shell",
    )
    arm.visual(
        Box((0.272, 0.005, 0.032)),
        origin=Origin(xyz=(0.166, 0.0155, -0.052)),
        material="powder_black",
        name="side_left",
    )
    arm.visual(
        Box((0.272, 0.005, 0.032)),
        origin=Origin(xyz=(0.166, -0.0155, -0.052)),
        material="powder_black",
        name="side_right",
    )
    arm.visual(
        Box((0.030, 0.036, 0.016)),
        origin=Origin(xyz=(0.015, 0.0, -0.038)),
        material="powder_black",
        name="shoulder",
    )
    arm.visual(
        Box((0.032, 0.007, 0.056)),
        origin=Origin(xyz=(0.006, 0.0125, -0.017)),
        material="powder_black",
        name="cheek_left",
    )
    arm.visual(
        Box((0.032, 0.007, 0.056)),
        origin=Origin(xyz=(0.006, -0.0125, -0.017)),
        material="powder_black",
        name="cheek_right",
    )
    arm.visual(
        Cylinder(radius=0.0055, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="powder_black",
        name="hinge_barrel",
    )
    arm.visual(
        Box((0.020, 0.030, 0.010)),
        origin=Origin(xyz=(0.292, 0.0, -0.053)),
        material="powder_black",
        name="nose_tip",
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.238, 0.018, 0.004)),
        origin=Origin(xyz=(0.112, 0.0, -0.062)),
        material="zinc",
        name="tray_body",
    )
    tray.visual(
        Box((0.238, 0.002, 0.008)),
        origin=Origin(xyz=(0.112, 0.010, -0.056)),
        material="zinc",
        name="tray_wall_left",
    )
    tray.visual(
        Box((0.238, 0.002, 0.008)),
        origin=Origin(xyz=(0.112, -0.010, -0.056)),
        material="zinc",
        name="tray_wall_right",
    )
    tray.visual(
        Box((0.020, 0.026, 0.006)),
        origin=Origin(xyz=(-0.008, 0.0, -0.057)),
        material="zinc",
        name="rear_tab",
    )
    tray.visual(
        Box((0.005, 0.026, 0.014)),
        origin=Origin(xyz=(-0.0155, 0.0, -0.047)),
        material="zinc",
        name="thumb_grip",
    )

    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=ARM_OPEN, effort=18.0, velocity=1.6),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=tray,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAY_TRAVEL, effort=8.0, velocity=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    tray = object_model.get_part("tray")
    arm_hinge = object_model.get_articulation("arm_hinge")
    tray_slide = object_model.get_articulation("tray_slide")

    with ctx.pose({arm_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_overlap(arm, base, axes="xy", min_overlap=0.018, name="arm closes over the base")
        ctx.expect_overlap(
            arm,
            base,
            axes="xy",
            elem_a="nose_tip",
            elem_b="anvil",
            min_overlap=0.010,
            name="nose aligns above the anvil",
        )
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            elem_a="nose_tip",
            elem_b="anvil",
            min_gap=0.004,
            max_gap=0.018,
            name="closed nose floats just above the anvil",
        )
        ctx.expect_within(
            tray,
            arm,
            axes="yz",
            margin=0.003,
            name="closed tray stays guided in the arm channel",
        )
        ctx.expect_overlap(
            tray,
            arm,
            axes="x",
            min_overlap=0.220,
            name="closed tray remains substantially housed",
        )
        closed_nose = ctx.part_element_world_aabb(arm, elem="nose_tip")
        closed_tray = ctx.part_world_position(tray)

    with ctx.pose({arm_hinge: ARM_OPEN, tray_slide: 0.0}):
        open_nose = ctx.part_element_world_aabb(arm, elem="nose_tip")

    ctx.check(
        "arm opens upward from the rear hinge",
        closed_nose is not None
        and open_nose is not None
        and open_nose[0][2] > closed_nose[0][2] + 0.10
        and open_nose[0][0] < closed_nose[0][0] - 0.06,
        details=f"closed_nose={closed_nose}, open_nose={open_nose}",
    )

    with ctx.pose({arm_hinge: 0.0, tray_slide: TRAY_TRAVEL}):
        ctx.expect_within(
            tray,
            arm,
            axes="yz",
            margin=0.003,
            name="extended tray stays laterally guided",
        )
        ctx.expect_overlap(
            tray,
            arm,
            axes="x",
            min_overlap=0.120,
            name="extended tray retains insertion in the arm",
        )
        extended_tray = ctx.part_world_position(tray)

    ctx.check(
        "tray slides rearward for loading",
        closed_tray is not None and extended_tray is not None and extended_tray[0] < closed_tray[0] - 0.06,
        details=f"closed_tray={closed_tray}, extended_tray={extended_tray}",
    )

    return ctx.report()


object_model = build_object_model()

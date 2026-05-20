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
    model = ArticulatedObject(name="pedestal_cantilever_inspection_arm")

    cast_iron = model.material("cast_iron", color=(0.12, 0.13, 0.14, 1.0))
    dark_steel = model.material("dark_steel", color=(0.30, 0.32, 0.33, 1.0))
    blue = model.material("blue_powdercoat", color=(0.05, 0.23, 0.58, 1.0))
    orange = model.material("orange_joint_caps", color=(0.95, 0.42, 0.08, 1.0))
    black = model.material("black_rubber", color=(0.02, 0.02, 0.02, 1.0))
    lens = model.material("smoked_lens", color=(0.05, 0.10, 0.12, 0.85))

    cyl_y = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.30, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_iron,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.12, length=0.84),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=dark_steel,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.17, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.89)),
        material=cast_iron,
        name="top_plinth",
    )
    base.visual(
        Box((0.14, 0.22, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.91)),
        material=cast_iron,
        name="shoulder_yoke_bridge",
    )
    for side, y in enumerate((-0.095, 0.095)):
        base.visual(
            Box((0.13, 0.035, 0.26)),
            origin=Origin(xyz=(0.0, y, 1.05)),
            material=cast_iron,
            name=f"shoulder_cheek_{side}",
        )
        base.visual(
            Cylinder(radius=0.055, length=0.014),
            origin=Origin(xyz=(0.0, y + (0.024 if y > 0.0 else -0.024), 1.05), rpy=cyl_y.rpy),
            material=orange,
            name=f"shoulder_pin_cap_{side}",
        )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        Cylinder(radius=0.070, length=0.157),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y.rpy),
        material=dark_steel,
        name="shoulder_boss",
    )
    shoulder_link.visual(
        Box((0.64, 0.120, 0.180)),
        origin=Origin(xyz=(0.38, 0.0, 0.0)),
        material=blue,
        name="deep_box_beam",
    )
    shoulder_link.visual(
        Box((0.12, 0.22, 0.16)),
        origin=Origin(xyz=(0.64, 0.0, 0.0)),
        material=blue,
        name="elbow_yoke_bridge",
    )
    for side, y in enumerate((-0.082, 0.082)):
        shoulder_link.visual(
            Box((0.20, 0.032, 0.22)),
            origin=Origin(xyz=(0.78, y, 0.0)),
            material=blue,
            name=f"elbow_cheek_{side}",
        )
        shoulder_link.visual(
            Cylinder(radius=0.045, length=0.012),
            origin=Origin(xyz=(0.78, y + (0.021 if y > 0.0 else -0.021), 0.0), rpy=cyl_y.rpy),
            material=orange,
            name=f"elbow_pin_cap_{side}",
        )

    forelink = model.part("forelink")
    forelink.visual(
        Cylinder(radius=0.056, length=0.134),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y.rpy),
        material=dark_steel,
        name="elbow_boss",
    )
    forelink.visual(
        Box((0.58, 0.075, 0.105)),
        origin=Origin(xyz=(0.34, 0.0, 0.0)),
        material=dark_steel,
        name="slim_box_beam",
    )
    forelink.visual(
        Box((0.10, 0.16, 0.10)),
        origin=Origin(xyz=(0.58, 0.0, 0.0)),
        material=dark_steel,
        name="wrist_yoke_bridge",
    )
    for side, y in enumerate((-0.061, 0.061)):
        forelink.visual(
            Box((0.16, 0.026, 0.14)),
            origin=Origin(xyz=(0.68, y, 0.0)),
            material=dark_steel,
            name=f"wrist_cheek_{side}",
        )
        forelink.visual(
            Cylinder(radius=0.032, length=0.010),
            origin=Origin(xyz=(0.68, y + (0.018 if y > 0.0 else -0.018), 0.0), rpy=cyl_y.rpy),
            material=orange,
            name=f"wrist_pin_cap_{side}",
        )

    wrist_block = model.part("wrist_block")
    wrist_block.visual(
        Cylinder(radius=0.041, length=0.098),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y.rpy),
        material=dark_steel,
        name="wrist_boss",
    )
    wrist_block.visual(
        Box((0.16, 0.085, 0.095)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=cast_iron,
        name="compact_block",
    )
    wrist_block.visual(
        Box((0.030, 0.070, 0.070)),
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material=lens,
        name="inspection_window",
    )
    wrist_block.visual(
        Box((0.020, 0.095, 0.110)),
        origin=Origin(xyz=(0.142, 0.0, 0.0)),
        material=black,
        name="rubber_bumper",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder_link,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=-0.55, upper=0.95),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=forelink,
        origin=Origin(xyz=(0.78, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=-1.30, upper=1.30),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forelink,
        child=wrist_block,
        origin=Origin(xyz=(0.68, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.8, lower=-1.20, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    shoulder_link = object_model.get_part("shoulder_link")
    forelink = object_model.get_part("forelink")
    wrist_block = object_model.get_part("wrist_block")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    ctx.check(
        "serial revolute chain",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (shoulder, elbow, wrist)
        )
        and shoulder.parent == base.name
        and shoulder.child == shoulder_link.name
        and elbow.parent == shoulder_link.name
        and elbow.child == forelink.name
        and wrist.parent == forelink.name
        and wrist.child == wrist_block.name,
        details="The pedestal arm should be base -> shoulder -> forelink -> wrist through three revolute pins.",
    )

    ctx.expect_within(
        shoulder_link,
        base,
        axes="y",
        inner_elem="shoulder_boss",
        outer_elem="shoulder_yoke_bridge",
        margin=0.02,
        name="shoulder boss sits between yoke sides",
    )
    ctx.expect_within(
        forelink,
        shoulder_link,
        axes="y",
        inner_elem="elbow_boss",
        outer_elem="elbow_yoke_bridge",
        margin=0.02,
        name="elbow boss sits between shoulder fork sides",
    )
    ctx.expect_within(
        wrist_block,
        forelink,
        axes="y",
        inner_elem="wrist_boss",
        outer_elem="wrist_yoke_bridge",
        margin=0.02,
        name="wrist boss sits between forelink fork sides",
    )

    rest_tip = ctx.part_element_world_aabb(wrist_block, elem="inspection_window")
    with ctx.pose({shoulder: 0.60, elbow: -0.45, wrist: 0.35}):
        raised_tip = ctx.part_element_world_aabb(wrist_block, elem="inspection_window")
    ctx.check(
        "posed arm lifts inspection head",
        rest_tip is not None
        and raised_tip is not None
        and raised_tip[0][2] > rest_tip[0][2] + 0.12,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()

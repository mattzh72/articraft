from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


STEEL = Material("dark_blasted_steel", rgba=(0.09, 0.10, 0.11, 1.0))
CASTING = Material("warm_grey_casting", rgba=(0.58, 0.61, 0.61, 1.0))
BLUE = Material("painted_blue_arm", rgba=(0.05, 0.25, 0.55, 1.0))
BLACK = Material("black_rubber_seals", rgba=(0.015, 0.015, 0.014, 1.0))
BRIGHT = Material("machined_flange", rgba=(0.78, 0.76, 0.70, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_shoulder_wristed_arm")

    # A compact industrial-scale, table/floor-mounted 3R yaw arm.  The three
    # moving links stack on real rotary bearing faces so the mechanism is
    # visibly supported while avoiding broad interpenetrating joint solids.
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.30, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=STEEL,
        name="floor_plinth",
    )
    base.visual(
        Cylinder(radius=0.155, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=CASTING,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.23, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.69)),
        material=CASTING,
        name="shoulder_bearing",
    )
    base.visual(
        Cylinder(radius=0.09, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.7375)),
        material=BLACK,
        name="shoulder_seal",
    )

    upper_arm = model.part("upper_arm")
    upper_link_mesh = mesh_from_geometry(
        CapsuleGeometry(radius=0.045, length=0.50, radial_segments=32),
        "upper_arm_rounded_link",
    )
    upper_arm.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=BLUE,
        name="shoulder_hub",
    )
    upper_arm.visual(
        upper_link_mesh,
        origin=Origin(xyz=(0.36, 0.0, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLUE,
        name="upper_link",
    )
    upper_arm.visual(
        Box((0.34, 0.040, 0.026)),
        origin=Origin(xyz=(0.34, 0.0, 0.096)),
        material=CASTING,
        name="upper_rib",
    )
    upper_arm.visual(
        Cylinder(radius=0.13, length=0.10),
        origin=Origin(xyz=(0.68, 0.0, 0.05)),
        material=BLUE,
        name="elbow_bearing",
    )
    upper_arm.visual(
        Cylinder(radius=0.072, length=0.018),
        origin=Origin(xyz=(0.68, 0.0, 0.091)),
        material=BLACK,
        name="elbow_seal",
    )

    forearm = model.part("forearm")
    forearm_link_mesh = mesh_from_geometry(
        CapsuleGeometry(radius=0.045, length=0.38, radial_segments=32),
        "forearm_rounded_link",
    )
    forearm.visual(
        Cylinder(radius=0.12, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=BLUE,
        name="elbow_hub",
    )
    forearm.visual(
        forearm_link_mesh,
        origin=Origin(xyz=(0.28, 0.0, 0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLUE,
        name="forearm_link",
    )
    forearm.visual(
        Box((0.28, 0.034, 0.022)),
        origin=Origin(xyz=(0.28, 0.0, 0.089)),
        material=CASTING,
        name="forearm_rib",
    )
    forearm.visual(
        Cylinder(radius=0.095, length=0.09),
        origin=Origin(xyz=(0.53, 0.0, 0.045)),
        material=BLUE,
        name="wrist_bearing",
    )
    forearm.visual(
        Cylinder(radius=0.052, length=0.016),
        origin=Origin(xyz=(0.53, 0.0, 0.082)),
        material=BLACK,
        name="wrist_seal",
    )

    wrist_link = model.part("wrist_link")
    wrist_mesh = mesh_from_geometry(
        CapsuleGeometry(radius=0.032, length=0.16, radial_segments=24),
        "wrist_short_link",
    )
    wrist_link.visual(
        Cylinder(radius=0.085, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=BLUE,
        name="wrist_hub",
    )
    wrist_link.visual(
        wrist_mesh,
        origin=Origin(xyz=(0.15, 0.0, 0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLUE,
        name="wrist_link_shell",
    )
    wrist_link.visual(
        Cylinder(radius=0.075, length=0.035),
        origin=Origin(xyz=(0.265, 0.0, 0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BRIGHT,
        name="tool_flange",
    )
    wrist_link.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(0.290, 0.0, 0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="pilot_boss",
    )
    for i, (dy, dz) in enumerate(((0.045, 0.0), (-0.045, 0.0), (0.0, 0.045), (0.0, -0.045))):
        wrist_link.visual(
            Cylinder(radius=0.0085, length=0.012),
            origin=Origin(
                xyz=(0.2865, dy, 0.045 + dz),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=STEEL,
            name=f"flange_bolt_{i}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.75)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=150.0, velocity=1.6, lower=-2.95, upper=2.95),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.68, 0.0, 0.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.8, lower=-2.55, upper=2.55),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_link,
        origin=Origin(xyz=(0.53, 0.0, 0.09)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_link = object_model.get_part("wrist_link")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    joints = (shoulder, elbow, wrist)
    ctx.check(
        "three serial revolute joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joint types={[j.articulation_type for j in joints]}",
    )

    ctx.expect_gap(
        upper_arm,
        base,
        axis="z",
        positive_elem="shoulder_hub",
        negative_elem="shoulder_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="shoulder hub sits on pedestal bearing",
    )
    ctx.expect_overlap(
        upper_arm,
        base,
        axes="xy",
        elem_a="shoulder_hub",
        elem_b="shoulder_bearing",
        min_overlap=0.12,
        name="shoulder bearing footprint captures upper arm",
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="z",
        positive_elem="elbow_hub",
        negative_elem="elbow_bearing",
        max_gap=0.001,
        max_penetration=1e-6,
        name="forearm is carried on elbow bearing",
    )
    ctx.expect_gap(
        wrist_link,
        forearm,
        axis="z",
        positive_elem="wrist_hub",
        negative_elem="wrist_bearing",
        max_gap=0.001,
        max_penetration=1e-6,
        name="wrist link is carried at forearm tip",
    )

    rest_forearm_pos = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 0.70, elbow: 0.0, wrist: 0.0}):
        yawed_forearm_pos = ctx.part_world_position(forearm)
    ctx.check(
        "shoulder yaws upper arm train",
        rest_forearm_pos is not None
        and yawed_forearm_pos is not None
        and yawed_forearm_pos[1] > rest_forearm_pos[1] + 0.35,
        details=f"rest={rest_forearm_pos}, yawed={yawed_forearm_pos}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 1.0, wrist: 0.0}):
        forearm_pos = ctx.part_world_position(forearm)
        wrist_pos = ctx.part_world_position(wrist_link)
    ctx.check(
        "elbow swings wrist around forearm tip",
        forearm_pos is not None
        and wrist_pos is not None
        and wrist_pos[1] > forearm_pos[1] + 0.35,
        details=f"forearm={forearm_pos}, wrist={wrist_pos}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist: 0.0}):
        closed_flange_aabb = ctx.part_element_world_aabb(wrist_link, elem="tool_flange")
    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist: 1.0}):
        turned_flange_aabb = ctx.part_element_world_aabb(wrist_link, elem="tool_flange")
    if closed_flange_aabb is not None and turned_flange_aabb is not None:
        closed_center_y = (closed_flange_aabb[0][1] + closed_flange_aabb[1][1]) * 0.5
        turned_center_y = (turned_flange_aabb[0][1] + turned_flange_aabb[1][1]) * 0.5
    else:
        closed_center_y = turned_center_y = None
    ctx.check(
        "wrist rotates the tool flange",
        closed_center_y is not None
        and turned_center_y is not None
        and turned_center_y > closed_center_y + 0.18,
        details=f"closed_y={closed_center_y}, turned_y={turned_center_y}",
    )

    return ctx.report()


object_model = build_object_model()

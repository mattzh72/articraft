from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shoulder_elbow_wrist_arm")

    painted = Material("warm_industrial_orange", rgba=(0.92, 0.42, 0.12, 1.0))
    dark = Material("dark_anodized_joint_housings", rgba=(0.08, 0.09, 0.10, 1.0))
    steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    rubber = Material("matte_black_feet", rgba=(0.015, 0.015, 0.018, 1.0))

    # Grounded pedestal and the large shoulder yoke.  The shoulder axis is
    # horizontal along +Y at z=0.85 m.
    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.24, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark,
        name="floor_base",
    )
    pedestal.visual(
        Cylinder(radius=0.105, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=steel,
        name="column",
    )
    pedestal.visual(
        Box((0.25, 0.27, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.715)),
        material=dark,
        name="shoulder_saddle",
    )
    for y, name in ((0.126, "shoulder_cheek_0"), (-0.126, "shoulder_cheek_1")):
        pedestal.visual(
            Box((0.20, 0.045, 0.31)),
            origin=Origin(xyz=(0.0, y, 0.875)),
            material=painted,
            name=name,
        )
    for y, name in ((0.158, "shoulder_cap_0"), (-0.158, "shoulder_cap_1")):
        pedestal.visual(
            Cylinder(radius=0.095, length=0.020),
            origin=Origin(xyz=(0.0, y, 0.85), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )
    for x, y, name in (
        (0.17, 0.17, "foot_0"),
        (-0.17, 0.17, "foot_1"),
        (0.17, -0.17, "foot_2"),
        (-0.17, -0.17, "foot_3"),
    ):
        pedestal.visual(
            Cylinder(radius=0.032, length=0.012),
            origin=Origin(xyz=(x, y, 0.006)),
            material=rubber,
            name=name,
        )

    # Upper arm frame is at the shoulder axis.  Its distal end carries a
    # medium-size elbow yoke around the forearm hub.
    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.087, length=0.207),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.50, 0.080, 0.070)),
        origin=Origin(xyz=(0.275, 0.0, 0.0)),
        material=painted,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.054, 0.205, 0.074)),
        origin=Origin(xyz=(0.552, 0.0, 0.0)),
        material=painted,
        name="elbow_bridge",
    )
    for y, name in ((0.087, "elbow_cheek_0"), (-0.087, "elbow_cheek_1")):
        upper_arm.visual(
            Box((0.160, 0.028, 0.190)),
            origin=Origin(xyz=(0.650, y, 0.0)),
            material=painted,
            name=name,
        )
    for y, name in ((0.110, "elbow_cap_0"), (-0.110, "elbow_cap_1")):
        upper_arm.visual(
            Cylinder(radius=0.066, length=0.020),
            origin=Origin(xyz=(0.650, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )

    # Forearm frame is at the elbow axis.  The wrist support is intentionally
    # compact, visibly smaller than the shoulder and elbow supports.
    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.068, length=0.146),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.455, 0.064, 0.055)),
        origin=Origin(xyz=(0.285, 0.0, 0.0)),
        material=painted,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.065, 0.136, 0.050)),
        origin=Origin(xyz=(0.475, 0.0, 0.0)),
        material=painted,
        name="wrist_bridge",
    )
    for y, name in ((0.058, "wrist_cheek_0"), (-0.058, "wrist_cheek_1")):
        forearm.visual(
            Box((0.104, 0.020, 0.120)),
            origin=Origin(xyz=(0.550, y, 0.0)),
            material=painted,
            name=name,
        )
    for y, name in ((0.073, "wrist_cap_0"), (-0.073, "wrist_cap_1")):
        forearm.visual(
            Cylinder(radius=0.041, length=0.012),
            origin=Origin(xyz=(0.550, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )

    # Short wrist link, hub and circular tool flange.
    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.042, length=0.096),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="wrist_hub",
    )
    wrist.visual(
        Box((0.180, 0.045, 0.040)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=painted,
        name="wrist_link",
    )
    wrist.visual(
        Cylinder(radius=0.055, length=0.026),
        origin=Origin(xyz=(0.208, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tool_flange",
    )
    wrist.visual(
        Cylinder(radius=0.034, length=0.016),
        origin=Origin(xyz=(0.229, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="flange_pilot",
    )
    for i, (y, z) in enumerate(((0.033, 0.0), (-0.033, 0.0), (0.0, 0.033), (0.0, -0.033))):
        wrist.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(0.224, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"flange_bolt_{i}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.5, lower=-1.15, upper=1.35),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.65, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=110.0, velocity=1.8, lower=-1.65, upper=1.65),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.55, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.4, lower=-1.75, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")
    pedestal = object_model.get_part("pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_link = object_model.get_part("wrist")

    ctx.check(
        "three revolute joints with parallel horizontal axes",
        all(j.articulation_type == ArticulationType.REVOLUTE and tuple(j.axis) == (0.0, 1.0, 0.0) for j in (shoulder, elbow, wrist)),
        details=f"axes={[j.axis for j in (shoulder, elbow, wrist)]}",
    )

    def elem_dim(part, elem, axis_index: int) -> float:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return -1.0
        lo, hi = aabb
        return float(hi[axis_index] - lo[axis_index])

    shoulder_height = elem_dim(pedestal, "shoulder_cheek_0", 2)
    elbow_height = elem_dim(upper_arm, "elbow_cheek_0", 2)
    wrist_height = elem_dim(forearm, "wrist_cheek_0", 2)
    ctx.check(
        "wrist support is smaller than shoulder and elbow supports",
        0.0 < wrist_height < elbow_height < shoulder_height,
        details=f"shoulder={shoulder_height:.3f}, elbow={elbow_height:.3f}, wrist={wrist_height:.3f}",
    )

    ctx.expect_gap(
        upper_arm,
        pedestal,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="shoulder_hub",
        negative_elem="shoulder_cheek_1",
        name="shoulder hub seats against lower cheek",
    )
    ctx.expect_gap(
        pedestal,
        upper_arm,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="shoulder_cheek_0",
        negative_elem="shoulder_hub",
        name="shoulder hub seats against upper cheek",
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="elbow_hub",
        negative_elem="elbow_cheek_1",
        name="elbow hub seats against lower cheek",
    )
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="elbow_cheek_0",
        negative_elem="elbow_hub",
        name="elbow hub seats against upper cheek",
    )
    ctx.expect_gap(
        wrist_link,
        forearm,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="wrist_hub",
        negative_elem="wrist_cheek_1",
        name="wrist hub seats against lower cheek",
    )
    ctx.expect_gap(
        forearm,
        wrist_link,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="wrist_cheek_0",
        negative_elem="wrist_hub",
        name="wrist hub seats against upper cheek",
    )

    wrist_rest = ctx.part_world_position(wrist_link)
    with ctx.pose({shoulder: 0.55, elbow: -0.70, wrist: 0.65}):
        wrist_moved = ctx.part_world_position(wrist_link)
    ctx.check(
        "joint chain changes the wrist pose",
        wrist_rest is not None
        and wrist_moved is not None
        and abs(wrist_moved[0] - wrist_rest[0]) + abs(wrist_moved[2] - wrist_rest[2]) > 0.12,
        details=f"rest={wrist_rest}, moved={wrist_moved}",
    )

    return ctx.report()


object_model = build_object_model()

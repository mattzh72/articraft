from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHOULDER_Z = 0.58
UPPER_LEN = 0.55
FOREARM_LEN = 0.42


def _box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_z(radius: float, length: float, center: tuple[float, float, float]):
    return cq.Workplane("XY").cylinder(length, radius).translate(center)


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]):
    return cq.Workplane("XZ").cylinder(length, radius).translate(center)


def _base_shape():
    foot = _cylinder_z(0.26, 0.060, (0.0, 0.0, 0.030))
    column = _cylinder_z(0.135, 0.400, (0.0, 0.0, 0.250))
    neck = _cylinder_z(0.092, 0.110, (0.0, 0.0, 0.405))
    saddle = _box((0.145, 0.285, 0.046), (0.0, 0.0, 0.466))
    cheek_a = _box((0.135, 0.044, 0.210), (0.0, 0.118, SHOULDER_Z))
    cheek_b = _box((0.135, 0.044, 0.210), (0.0, -0.118, SHOULDER_Z))
    return foot.union(column).union(neck).union(saddle).union(cheek_a).union(cheek_b)


def _upper_arm_shape():
    proximal_hub = _cylinder_y(0.085, 0.145, (0.0, 0.0, 0.0))
    beam = _box((0.400, 0.095, 0.105), (0.270, 0.0, 0.0))
    side_rib_top = _box((0.360, 0.108, 0.022), (0.275, 0.0, 0.052))
    side_rib_bottom = _box((0.360, 0.108, 0.022), (0.275, 0.0, -0.052))
    tine_a = _box((0.130, 0.035, 0.145), (UPPER_LEN - 0.025, 0.060, 0.0))
    tine_b = _box((0.130, 0.035, 0.145), (UPPER_LEN - 0.025, -0.060, 0.0))
    return (
        proximal_hub.union(beam)
        .union(side_rib_top)
        .union(side_rib_bottom)
        .union(tine_a)
        .union(tine_b)
    )


def _forearm_shape():
    proximal_hub = _cylinder_y(0.060, 0.075, (0.0, 0.0, 0.0))
    beam = _box((0.310, 0.070, 0.075), (0.210, 0.0, 0.0))
    upper_rib = _box((0.280, 0.080, 0.018), (0.215, 0.0, 0.037))
    lower_rib = _box((0.280, 0.080, 0.018), (0.215, 0.0, -0.037))
    tine_a = _box((0.090, 0.024, 0.100), (FOREARM_LEN - 0.015, 0.046, 0.0))
    tine_b = _box((0.090, 0.024, 0.100), (FOREARM_LEN - 0.015, -0.046, 0.0))
    return proximal_hub.union(beam).union(upper_rib).union(lower_rib).union(tine_a).union(tine_b)


def _wrist_shape():
    hub = _cylinder_y(0.040, 0.060, (0.0, 0.0, 0.0))
    neck = _box((0.090, 0.050, 0.052), (0.066, 0.0, 0.0))
    face = cq.Workplane("YZ").cylinder(0.030, 0.044).translate((0.124, 0.0, 0.0))
    tool_nose = cq.Workplane("YZ").cylinder(0.035, 0.030).translate((0.154, 0.0, 0.0))
    return hub.union(neck).union(face).union(tool_nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_wristed_arm")

    base_paint = model.material("dark_cast_metal", rgba=(0.12, 0.13, 0.15, 1.0))
    upper_paint = model.material("safety_orange", rgba=(0.95, 0.38, 0.08, 1.0))
    forearm_paint = model.material("warm_yellow", rgba=(0.93, 0.62, 0.16, 1.0))
    wrist_paint = model.material("wrist_blue", rgba=(0.18, 0.32, 0.48, 1.0))
    black = model.material("black_bearing_caps", rgba=(0.02, 0.025, 0.03, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_structure", tolerance=0.002),
        material=base_paint,
        name="base_structure",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black,
        name="shoulder_pin",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm_shape(), "upper_arm_structure", tolerance=0.002),
        material=upper_paint,
        name="upper_arm_structure",
    )
    upper_arm.visual(
        Cylinder(radius=0.016, length=0.170),
        origin=Origin(xyz=(UPPER_LEN, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black,
        name="elbow_pin",
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_forearm_shape(), "forearm_structure", tolerance=0.002),
        material=forearm_paint,
        name="forearm_structure",
    )
    forearm.visual(
        Cylinder(radius=0.012, length=0.130),
        origin=Origin(xyz=(FOREARM_LEN, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black,
        name="wrist_pin",
    )

    wrist = model.part("wrist")
    wrist.visual(
        mesh_from_cadquery(_wrist_shape(), "wrist_stage", tolerance=0.002),
        material=wrist_paint,
        name="wrist_stage",
    )

    model.articulation(
        "base_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=0.0, upper=1.25),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=-1.35, upper=1.55),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(FOREARM_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=-1.70, upper=1.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    shoulder = object_model.get_articulation("base_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")
    wrist_joint = object_model.get_articulation("forearm_to_wrist")

    serial = [shoulder, elbow, wrist_joint]
    ctx.check(
        "three serial revolute joints",
        len(object_model.articulations) == 3
        and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in serial)
        and [(joint.parent, joint.child) for joint in serial]
        == [("base", "upper_arm"), ("upper_arm", "forearm"), ("forearm", "wrist")],
        details=f"joints={[(joint.name, joint.articulation_type, joint.parent, joint.child) for joint in object_model.articulations]}",
    )

    ctx.allow_overlap(
        base,
        upper_arm,
        elem_a="shoulder_pin",
        elem_b="upper_arm_structure",
        reason="The black shoulder shaft is intentionally captured through the upper-arm bearing hub.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_pin",
        elem_b="forearm_structure",
        reason="The elbow shaft is intentionally modeled passing through the forearm hub.",
    )
    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a="wrist_pin",
        elem_b="wrist_stage",
        reason="The small wrist shaft is intentionally captured through the terminal wrist hub.",
    )
    ctx.expect_overlap(
        base,
        upper_arm,
        axes="xyz",
        elem_a="shoulder_pin",
        elem_b="upper_arm_structure",
        min_overlap=0.030,
        name="shoulder pin passes through the upper arm hub",
    )
    ctx.expect_overlap(
        upper_arm,
        forearm,
        axes="xyz",
        elem_a="elbow_pin",
        elem_b="forearm_structure",
        min_overlap=0.020,
        name="elbow pin passes through the forearm hub",
    )
    ctx.expect_overlap(
        forearm,
        wrist,
        axes="xyz",
        elem_a="wrist_pin",
        elem_b="wrist_stage",
        min_overlap=0.016,
        name="wrist pin passes through the wrist hub",
    )

    ctx.expect_overlap(
        upper_arm,
        base,
        axes="yz",
        min_overlap=0.070,
        name="base yoke carries the upper arm joint",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="yz",
        min_overlap=0.055,
        name="upper arm fork carries the forearm joint",
    )
    ctx.expect_overlap(
        wrist,
        forearm,
        axes="yz",
        min_overlap=0.034,
        name="forearm fork carries the wrist joint",
    )

    upper_aabb = ctx.part_world_aabb(upper_arm)
    forearm_aabb = ctx.part_world_aabb(forearm)
    wrist_aabb = ctx.part_world_aabb(wrist)
    upper_height = upper_aabb[1][2] - upper_aabb[0][2] if upper_aabb else 0.0
    forearm_height = forearm_aabb[1][2] - forearm_aabb[0][2] if forearm_aabb else 0.0
    wrist_height = wrist_aabb[1][2] - wrist_aabb[0][2] if wrist_aabb else 0.0
    ctx.check(
        "sections step down from root to wrist",
        upper_height > forearm_height > wrist_height,
        details=f"heights upper={upper_height:.3f}, forearm={forearm_height:.3f}, wrist={wrist_height:.3f}",
    )

    rest_wrist_pos = ctx.part_world_position(wrist)
    with ctx.pose({shoulder: 0.70, elbow: 0.35, wrist_joint: 0.20}):
        raised_wrist_pos = ctx.part_world_position(wrist)
    ctx.check(
        "serial shoulder and elbow lift the wrist stage",
        rest_wrist_pos is not None
        and raised_wrist_pos is not None
        and raised_wrist_pos[2] > rest_wrist_pos[2] + 0.25,
        details=f"rest={rest_wrist_pos}, raised={raised_wrist_pos}",
    )

    return ctx.report()


object_model = build_object_model()

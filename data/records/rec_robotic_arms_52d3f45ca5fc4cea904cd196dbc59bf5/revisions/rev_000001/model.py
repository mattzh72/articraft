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
    model = ArticulatedObject(name="three_joint_pick_place_arm")

    dark_cast = model.material("dark_cast_metal", rgba=(0.08, 0.09, 0.10, 1.0))
    graphite = model.material("graphite_black", rgba=(0.015, 0.017, 0.018, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.42, 0.05, 1.0))
    warm_yellow = model.material("warm_yellow_link", rgba=(0.98, 0.70, 0.10, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.62, 0.64, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    pedestal_top_z = 0.62
    chain_offset_y = 0.28
    link_axis_z = 0.18
    elbow_x = 0.72
    wrist_x = 0.80

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.32, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_cast,
        name="floor_base",
    )
    pedestal.visual(
        Cylinder(radius=0.13, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material=dark_cast,
        name="fixed_column",
    )
    pedestal.visual(
        Cylinder(radius=0.19, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=brushed_steel,
        name="top_bearing",
    )
    for i, (x, y) in enumerate(((0.22, 0.22), (-0.22, 0.22), (-0.22, -0.22), (0.22, -0.22))):
        pedestal.visual(
            Cylinder(radius=0.035, length=0.014),
            origin=Origin(xyz=(x, y, 0.083)),
            material=brushed_steel,
            name=f"anchor_bolt_{i}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=brushed_steel,
        name="turntable",
    )
    upper_arm.visual(
        Box((0.24, 0.18, 0.10)),
        origin=Origin(xyz=(0.0, 0.13, 0.09)),
        material=safety_orange,
        name="side_bridge",
    )
    upper_arm.visual(
        Cylinder(radius=0.13, length=0.26),
        origin=Origin(xyz=(0.0, chain_offset_y, link_axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_orange,
        name="shoulder_housing",
    )
    upper_arm.visual(
        Box((0.51, 0.10, 0.10)),
        origin=Origin(xyz=(0.305, chain_offset_y, link_axis_z)),
        material=warm_yellow,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.09, 0.24, 0.09)),
        origin=Origin(xyz=(0.555, chain_offset_y, link_axis_z)),
        material=warm_yellow,
        name="fork_crossbar",
    )
    upper_arm.visual(
        Box((0.18, 0.055, 0.26)),
        origin=Origin(xyz=(0.685, 0.158, link_axis_z)),
        material=warm_yellow,
        name="fork_plate_inner",
    )
    upper_arm.visual(
        Box((0.18, 0.055, 0.26)),
        origin=Origin(xyz=(0.685, 0.402, link_axis_z)),
        material=warm_yellow,
        name="fork_plate_outer",
    )
    upper_arm.visual(
        Cylinder(radius=0.10, length=0.035),
        origin=Origin(xyz=(0.685, 0.118, link_axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="inner_bearing_cap",
    )
    upper_arm.visual(
        Cylinder(radius=0.10, length=0.035),
        origin=Origin(xyz=(0.685, 0.442, link_axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="outer_bearing_cap",
    )
    upper_arm.visual(
        Box((0.38, 0.038, 0.028)),
        origin=Origin(xyz=(0.34, chain_offset_y, link_axis_z + 0.064)),
        material=graphite,
        name="upper_cable_trough",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.095, length=0.189),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="elbow_hub",
    )
    forearm.visual(
        Cylinder(radius=0.055, length=0.66),
        origin=Origin(xyz=(0.37, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_yellow,
        name="forearm_tube",
    )
    forearm.visual(
        Box((0.48, 0.075, 0.030)),
        origin=Origin(xyz=(0.39, 0.0, 0.055)),
        material=safety_orange,
        name="forearm_cover",
    )
    forearm.visual(
        Cylinder(radius=0.075, length=0.14),
        origin=Origin(xyz=(0.73, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="distal_collar",
    )
    forearm.visual(
        Box((0.34, 0.034, 0.024)),
        origin=Origin(xyz=(0.38, 0.0, 0.079)),
        material=graphite,
        name="forearm_cable_trough",
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=Origin(xyz=(0.04, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="wrist_shaft",
    )
    wrist_head.visual(
        Cylinder(radius=0.10, length=0.12),
        origin=Origin(xyz=(0.14, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_cast,
        name="wrist_motor",
    )
    wrist_head.visual(
        Cylinder(radius=0.090, length=0.026),
        origin=Origin(xyz=(0.213, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="tool_flange",
    )
    wrist_head.visual(
        Box((0.032, 0.16, 0.12)),
        origin=Origin(xyz=(0.238, 0.0, 0.0)),
        material=dark_cast,
        name="tool_plate",
    )
    wrist_head.visual(
        Cylinder(radius=0.025, length=0.080),
        origin=Origin(xyz=(0.294, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="vacuum_nozzle",
    )
    wrist_head.visual(
        Cylinder(radius=0.045, length=0.028),
        origin=Origin(xyz=(0.348, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="suction_cup",
    )

    shoulder = model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, pedestal_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=-2.7, upper=2.7),
    )
    elbow = model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(elbow_x, chain_offset_y, link_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.70, upper=1.25),
    )
    wrist = model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(wrist_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )

    # Keep semantic handles useful for tests and probes.
    model.meta["primary_joints"] = (shoulder.name, elbow.name, wrist.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    pedestal = object_model.get_part("pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")
    shoulder = object_model.get_articulation("shoulder_yaw")
    elbow = object_model.get_articulation("elbow_pitch")
    wrist = object_model.get_articulation("wrist_roll")

    ctx.check(
        "three named revolute work joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in (shoulder, elbow, wrist)),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.check("shoulder axis is vertical", tuple(shoulder.axis) == (0.0, 0.0, 1.0), details=str(shoulder.axis))
    ctx.check("elbow axis is horizontal", abs(elbow.axis[1]) == 1.0 and elbow.axis[2] == 0.0, details=str(elbow.axis))
    ctx.check("wrist axis follows forearm", tuple(wrist.axis) == (1.0, 0.0, 0.0), details=str(wrist.axis))

    ctx.expect_gap(
        upper_arm,
        pedestal,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="turntable",
        negative_elem="top_bearing",
        name="turntable seats on pedestal bearing",
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem="elbow_hub",
        negative_elem="fork_plate_inner",
        name="elbow hub seats against inner fork cheek",
    )
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem="fork_plate_outer",
        negative_elem="elbow_hub",
        name="elbow hub seats against outer fork cheek",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="xz",
        min_overlap=0.08,
        elem_a="elbow_hub",
        elem_b="fork_plate_inner",
        name="elbow hub is captured between fork cheeks",
    )
    ctx.expect_gap(
        wrist_head,
        forearm,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="wrist_shaft",
        negative_elem="distal_collar",
        name="wrist shaft seats at distal collar",
    )

    forearm_rest = ctx.part_world_position(forearm)
    wrist_rest = ctx.part_world_position(wrist_head)
    with ctx.pose({elbow: elbow.motion_limits.upper}):
        wrist_lifted = ctx.part_world_position(wrist_head)
    ctx.check(
        "positive elbow pitch lifts wrist head",
        wrist_rest is not None and wrist_lifted is not None and wrist_lifted[2] > wrist_rest[2] + 0.45,
        details=f"rest={wrist_rest}, lifted={wrist_lifted}",
    )
    ctx.check(
        "moving chain is side-offset from support",
        forearm_rest is not None and forearm_rest[1] > 0.20,
        details=f"forearm origin={forearm_rest}",
    )

    return ctx.report()


object_model = build_object_model()

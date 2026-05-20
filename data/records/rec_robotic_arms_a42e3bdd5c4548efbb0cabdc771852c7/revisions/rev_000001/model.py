from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="three_joint_pick_place_arm")

    dark_steel = Material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.45, 0.48, 0.50, 1.0))
    safety_orange = Material("safety_orange", rgba=(0.95, 0.42, 0.08, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    blue_cover = Material("blue_cover", rgba=(0.05, 0.22, 0.55, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.32, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_steel,
        name="floor_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.15, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=satin_steel,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.20, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=dark_steel,
        name="top_bearing",
    )
    pedestal.visual(
        Box((0.07, 0.18, 0.18)),
        origin=Origin(xyz=(-0.155, 0.0, 0.22)),
        material=black_rubber,
        name="cable_box",
    )
    for i, (x, y) in enumerate(
        ((0.21, 0.21), (-0.21, 0.21), (-0.21, -0.21), (0.21, -0.21))
    ):
        pedestal.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(x, y, 0.088)),
            material=satin_steel,
            name=f"anchor_bolt_{i}",
        )

    shoulder = model.part("shoulder")
    shoulder.visual(
        Cylinder(radius=0.18, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_steel,
        name="turntable",
    )
    shoulder.visual(
        Cylinder(radius=0.12, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=safety_orange,
        name="waist",
    )
    shoulder.visual(
        Cylinder(radius=0.16, length=0.34),
        origin=Origin(xyz=(0.10, 0.0, 0.28), rpy=(pi / 2.0, 0.0, 0.0)),
        material=safety_orange,
        name="shoulder_motor",
    )
    shoulder.visual(
        Box((0.50, 0.045, 0.095)),
        origin=Origin(xyz=(0.395, 0.11, 0.28)),
        material=safety_orange,
        name="upper_rail_0",
    )
    shoulder.visual(
        Box((0.50, 0.045, 0.095)),
        origin=Origin(xyz=(0.395, -0.11, 0.28)),
        material=safety_orange,
        name="upper_rail_1",
    )
    shoulder.visual(
        Box((0.32, 0.24, 0.04)),
        origin=Origin(xyz=(0.25, 0.0, 0.39)),
        material=blue_cover,
        name="top_cover",
    )
    shoulder.visual(
        Box((0.16, 0.07, 0.24)),
        origin=Origin(xyz=(0.72, 0.16, 0.28)),
        material=dark_steel,
        name="elbow_lug_0",
    )
    shoulder.visual(
        Box((0.16, 0.07, 0.24)),
        origin=Origin(xyz=(0.72, -0.16, 0.28)),
        material=dark_steel,
        name="elbow_lug_1",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.105, length=0.18),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_hub",
    )
    forearm.visual(
        Cylinder(radius=0.035, length=0.42),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="elbow_pin",
    )
    forearm.visual(
        Box((0.62, 0.13, 0.12)),
        origin=Origin(xyz=(0.32, 0.0, 0.0)),
        material=safety_orange,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.50, 0.05, 0.04)),
        origin=Origin(xyz=(0.36, 0.0, 0.077)),
        material=blue_cover,
        name="rib_cover",
    )
    forearm.visual(
        Cylinder(radius=0.095, length=0.10),
        origin=Origin(xyz=(0.67, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="wrist_socket",
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.075, length=0.12),
        origin=Origin(xyz=(0.06, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="roll_barrel",
    )
    wrist_head.visual(
        Cylinder(radius=0.095, length=0.045),
        origin=Origin(xyz=(0.1425, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="tool_flange",
    )
    wrist_head.visual(
        Cylinder(radius=0.070, length=0.018),
        origin=Origin(xyz=(0.174, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="faceplate",
    )
    wrist_head.visual(
        Box((0.045, 0.035, 0.055)),
        origin=Origin(xyz=(0.115, 0.0, 0.09)),
        material=blue_cover,
        name="roll_marker",
    )
    for i, (y, z) in enumerate(((0.035, 0.035), (-0.035, 0.035), (-0.035, -0.035), (0.035, -0.035))):
        wrist_head.visual(
            Cylinder(radius=0.008, length=0.008),
            origin=Origin(xyz=(0.187, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=black_rubber,
            name=f"tool_bolt_{i}",
        )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.8, lower=-pi, upper=pi),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=forearm,
        origin=Origin(xyz=(0.72, 0.0, 0.28)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6, lower=-1.35, upper=1.15),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.72, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=3.0, lower=-pi, upper=pi),
    )
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
    shoulder = object_model.get_part("shoulder")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")
    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow = object_model.get_articulation("elbow")
    wrist_roll = object_model.get_articulation("wrist_roll")

    ctx.allow_overlap(
        shoulder,
        forearm,
        elem_a="elbow_lug_0",
        elem_b="elbow_pin",
        reason="The visible elbow pin is intentionally captured through the solid proxy yoke lug.",
    )
    ctx.allow_overlap(
        shoulder,
        forearm,
        elem_a="elbow_lug_1",
        elem_b="elbow_pin",
        reason="The visible elbow pin is intentionally captured through the solid proxy yoke lug.",
    )

    ctx.check(
        "exactly three revolute work joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.check("shoulder axis is vertical", shoulder_yaw.axis == (0.0, 0.0, 1.0), details=str(shoulder_yaw.axis))
    ctx.check("elbow axis is horizontal", elbow.axis == (0.0, 1.0, 0.0), details=str(elbow.axis))
    ctx.check("wrist axis follows forearm", wrist_roll.axis == (1.0, 0.0, 0.0), details=str(wrist_roll.axis))

    ctx.expect_gap(
        shoulder,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="turntable",
        negative_elem="top_bearing",
        name="shoulder turntable is seated on pedestal bearing",
    )
    ctx.expect_overlap(
        shoulder,
        forearm,
        axes="xyz",
        elem_a="elbow_lug_0",
        elem_b="elbow_pin",
        min_overlap=0.02,
        name="elbow pin passes through first yoke lug",
    )
    ctx.expect_overlap(
        shoulder,
        forearm,
        axes="xyz",
        elem_a="elbow_lug_1",
        elem_b="elbow_pin",
        min_overlap=0.02,
        name="elbow pin passes through second yoke lug",
    )
    ctx.expect_origin_distance(
        shoulder,
        forearm,
        axes="xz",
        min_dist=0.70,
        name="shoulder and elbow axes are widely separated",
    )
    ctx.expect_origin_distance(
        forearm,
        wrist_head,
        axes="x",
        min_dist=0.65,
        name="elbow and wrist axes are widely separated",
    )

    rest_wrist = ctx.part_world_position(wrist_head)
    with ctx.pose({shoulder_yaw: 0.8}):
        yawed_wrist = ctx.part_world_position(wrist_head)
    ctx.check(
        "shoulder yaw swings the arm around the pedestal",
        rest_wrist is not None
        and yawed_wrist is not None
        and abs(yawed_wrist[1] - rest_wrist[1]) > 0.55,
        details=f"rest={rest_wrist}, yawed={yawed_wrist}",
    )

    with ctx.pose({elbow: -0.7}):
        raised_wrist = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow pitch moves the wrist in profile",
        rest_wrist is not None
        and raised_wrist is not None
        and raised_wrist[2] > rest_wrist[2] + 0.25,
        details=f"rest={rest_wrist}, raised={raised_wrist}",
    )

    with ctx.pose({wrist_roll: pi / 2.0}):
        marker_aabb = ctx.part_element_world_aabb(wrist_head, elem="roll_marker")
    marker_center_y = None if marker_aabb is None else (marker_aabb[0][1] + marker_aabb[1][1]) / 2.0
    ctx.check(
        "wrist roll turns the offset head marker about the forearm axis",
        marker_center_y is not None and marker_center_y < -0.05,
        details=f"marker_aabb={marker_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

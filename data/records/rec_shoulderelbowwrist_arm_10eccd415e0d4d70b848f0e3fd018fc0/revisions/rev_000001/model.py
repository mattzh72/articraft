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
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_robotic_arm")

    dark = Material("mat_dark_cast_metal", rgba=(0.07, 0.075, 0.08, 1.0))
    graphite = Material("mat_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    orange = Material("mat_safety_orange", rgba=(0.95, 0.36, 0.08, 1.0))
    silver = Material("mat_brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    black = Material("mat_black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.165, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark,
        name="floor_plinth",
    )
    base.visual(
        Cylinder(radius=0.082, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=graphite,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.2975)),
        material=dark,
        name="top_bearing",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.096, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=graphite,
        name="shoulder_turntable",
    )
    upper_arm.visual(
        Box((0.115, 0.105, 0.088)),
        origin=Origin(xyz=(0.055, 0.0, 0.070)),
        material=orange,
        name="shoulder_riser",
    )
    upper_arm.visual(
        Cylinder(radius=0.035, length=0.335),
        origin=Origin(xyz=(0.205, 0.0, 0.096), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=orange,
        name="upper_tube",
    )
    upper_arm.visual(
        Box((0.165, 0.070, 0.052)),
        origin=Origin(xyz=(0.210, 0.0, 0.096)),
        material=orange,
        name="upper_spine",
    )
    upper_arm.visual(
        Box((0.075, 0.155, 0.072)),
        origin=Origin(xyz=(0.330, 0.0, 0.096)),
        material=orange,
        name="elbow_bridge",
    )
    upper_arm.visual(
        Box((0.118, 0.044, 0.070)),
        origin=Origin(xyz=(0.365, 0.064, 0.096)),
        material=orange,
        name="elbow_ear_0_web",
    )
    upper_arm.visual(
        Box((0.118, 0.044, 0.070)),
        origin=Origin(xyz=(0.365, -0.064, 0.096)),
        material=orange,
        name="elbow_ear_1_web",
    )
    upper_arm.visual(
        Cylinder(radius=0.058, length=0.042),
        origin=Origin(xyz=(0.420, 0.064, 0.096), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="elbow_ear_0",
    )
    upper_arm.visual(
        Cylinder(radius=0.058, length=0.042),
        origin=Origin(xyz=(0.420, -0.064, 0.096), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="elbow_ear_1",
    )
    upper_arm.visual(
        Cylinder(radius=0.012, length=0.180),
        origin=Origin(xyz=(0.420, 0.0, 0.096), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="elbow_pin",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.046, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="elbow_hub",
    )
    forearm.visual(
        Cylinder(radius=0.029, length=0.325),
        origin=Origin(xyz=(0.180, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=orange,
        name="forearm_tube",
    )
    forearm.visual(
        Box((0.205, 0.052, 0.040)),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=orange,
        name="forearm_flat",
    )
    forearm.visual(
        Cylinder(radius=0.056, length=0.070),
        origin=Origin(xyz=(0.345, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="wrist_socket",
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.050, length=0.070),
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="wrist_barrel",
    )
    wrist_head.visual(
        Cylinder(radius=0.067, length=0.026),
        origin=Origin(xyz=(0.083, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="tool_flange",
    )
    wrist_head.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.098, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="center_bore",
    )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        wrist_head.visual(
            Cylinder(radius=0.007, length=0.008),
            origin=Origin(
                xyz=(0.0995, 0.042 * math.cos(angle), 0.042 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=black,
            name=f"flange_bolt_{i}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.8, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.420, 0.0, 0.096)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.6, lower=-1.25, upper=1.45),
    )
    model.articulation(
        "wrist",
        ArticulationType.CONTINUOUS,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0),
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

    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        reason="The elbow shaft is intentionally captured through the rotating hub.",
    )

    ctx.check(
        "three user facing rotary joints",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and wrist.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"types: shoulder={shoulder.articulation_type}, "
            f"elbow={elbow.articulation_type}, wrist={wrist.articulation_type}"
        ),
    )
    ctx.check(
        "joint axes match robot architecture",
        shoulder.axis == (0.0, 0.0, 1.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and wrist.axis == (1.0, 0.0, 0.0),
        details=f"axes: shoulder={shoulder.axis}, elbow={elbow.axis}, wrist={wrist.axis}",
    )

    ctx.expect_contact(
        upper_arm,
        base,
        elem_a="shoulder_turntable",
        elem_b="top_bearing",
        contact_tol=0.001,
        name="turntable sits on pedestal bearing",
    )
    ctx.expect_overlap(
        upper_arm,
        base,
        axes="xy",
        elem_a="shoulder_turntable",
        elem_b="top_bearing",
        min_overlap=0.15,
        name="shoulder bearing footprint is supported",
    )
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        positive_elem="elbow_ear_0",
        negative_elem="elbow_hub",
        min_gap=0.002,
        max_gap=0.008,
        name="positive elbow fork clearance",
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_ear_1",
        min_gap=0.002,
        max_gap=0.008,
        name="negative elbow fork clearance",
    )
    ctx.expect_within(
        upper_arm,
        forearm,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="elbow_hub",
        margin=0.001,
        name="elbow pin runs through hub center",
    )
    ctx.expect_overlap(
        upper_arm,
        forearm,
        axes="y",
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        min_overlap=0.07,
        name="elbow pin spans the hub width",
    )
    ctx.expect_contact(
        wrist_head,
        forearm,
        elem_a="wrist_barrel",
        elem_b="wrist_socket",
        contact_tol=0.001,
        name="wrist barrel is seated in socket face",
    )

    rest_wrist = ctx.part_world_position(wrist_head)
    with ctx.pose({shoulder: 0.75}):
        yawed_wrist = ctx.part_world_position(wrist_head)
    ctx.check(
        "shoulder yaw moves arm around pedestal",
        rest_wrist is not None
        and yawed_wrist is not None
        and abs(yawed_wrist[1] - rest_wrist[1]) > 0.20,
        details=f"rest={rest_wrist}, yawed={yawed_wrist}",
    )

    with ctx.pose({elbow: 0.80}):
        raised_wrist = ctx.part_world_position(wrist_head)
    ctx.check(
        "positive elbow motion raises forearm",
        rest_wrist is not None
        and raised_wrist is not None
        and raised_wrist[2] > rest_wrist[2] + 0.20,
        details=f"rest={rest_wrist}, raised={raised_wrist}",
    )

    return ctx.report()


object_model = build_object_model()

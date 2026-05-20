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


BASE_H = 0.120
SHOULDER_Z = 0.255
UPPER_LEN = 0.480
FOREARM_LEN = 0.380
WRIST_LEN = 0.155


def _rounded_box(size: tuple[float, float, float], radius: float):
    """Small filleted hard-shell cover, centered on its local origin."""
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def _mesh(shape, name: str):
    return mesh_from_cadquery(shape, name, tolerance=0.0012, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_serial_robotic_arm")

    model.material("matte_graphite", rgba=(0.045, 0.047, 0.052, 1.0))
    model.material("satin_pearl", rgba=(0.78, 0.80, 0.78, 1.0))
    model.material("warm_titanium", rgba=(0.55, 0.52, 0.47, 1.0))
    model.material("black_rubber", rgba=(0.010, 0.011, 0.012, 1.0))
    model.material("dark_glass", rgba=(0.04, 0.06, 0.075, 1.0))
    model.material("polished_bolt", rgba=(0.82, 0.83, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.255, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="matte_graphite",
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.214, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material="warm_titanium",
        name="satin_base_bevel",
    )
    base.visual(
        Cylinder(radius=0.180, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material="matte_graphite",
        name="base_pedestal",
    )
    base.visual(
        Cylinder(radius=0.186, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, BASE_H - 0.003)),
        material="black_rubber",
        name="turntable_seam",
    )
    for i, (x, y) in enumerate(
        (
            (0.168, 0.168),
            (-0.168, 0.168),
            (-0.168, -0.168),
            (0.168, -0.168),
        )
    ):
        base.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, y, 0.033)),
            material="polished_bolt",
            name=f"anchor_bolt_{i}",
        )

    shoulder = model.part("shoulder")
    shoulder.visual(
        Cylinder(radius=0.170, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material="satin_pearl",
        name="yaw_turntable",
    )
    shoulder.visual(
        Cylinder(radius=0.138, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material="black_rubber",
        name="upper_yaw_seam",
    )
    shoulder.visual(
        _mesh(_rounded_box((0.170, 0.160, 0.130), 0.018), "shoulder_core"),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material="matte_graphite",
        name="shoulder_core",
    )
    shoulder.visual(
        _mesh(_rounded_box((0.105, 0.050, 0.195), 0.011), "shoulder_cheek_0"),
        origin=Origin(xyz=(0.0, 0.124, SHOULDER_Z)),
        material="satin_pearl",
        name="shoulder_cheek_0",
    )
    shoulder.visual(
        _mesh(_rounded_box((0.105, 0.050, 0.195), 0.011), "shoulder_cheek_1"),
        origin=Origin(xyz=(0.0, -0.124, SHOULDER_Z)),
        material="satin_pearl",
        name="shoulder_cheek_1",
    )
    shoulder.visual(
        Box((0.070, 0.286, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z - 0.110)),
        material="satin_pearl",
        name="shoulder_bridge",
    )
    shoulder.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, 0.124, SHOULDER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="warm_titanium",
        name="shoulder_bushing_0",
    )
    shoulder.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, -0.124, SHOULDER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="warm_titanium",
        name="shoulder_bushing_1",
    )
    shoulder.visual(
        Cylinder(radius=0.073, length=0.006),
        origin=Origin(xyz=(0.0, 0.151, SHOULDER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="black_rubber",
        name="shoulder_outer_seam_0",
    )
    shoulder.visual(
        Cylinder(radius=0.073, length=0.006),
        origin=Origin(xyz=(0.0, -0.151, SHOULDER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="black_rubber",
        name="shoulder_outer_seam_1",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.031, length=0.294),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="warm_titanium",
        name="shoulder_shaft",
    )
    upper_arm.visual(
        Cylinder(radius=0.082, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="satin_pearl",
        name="shoulder_collar",
    )
    upper_arm.visual(
        Cylinder(radius=0.090, length=0.011),
        origin=Origin(xyz=(0.0, 0.056, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="black_rubber",
        name="shoulder_inner_seam_0",
    )
    upper_arm.visual(
        Cylinder(radius=0.090, length=0.011),
        origin=Origin(xyz=(0.0, -0.056, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="black_rubber",
        name="shoulder_inner_seam_1",
    )
    upper_arm.visual(
        _mesh(_rounded_box((0.330, 0.110, 0.106), 0.019), "upper_arm_shell"),
        origin=Origin(xyz=(0.235, 0.0, 0.0)),
        material="satin_pearl",
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Box((0.106, 0.076, 0.062)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material="satin_pearl",
        name="shoulder_neck",
    )
    upper_arm.visual(
        Cylinder(radius=0.052, length=0.042),
        origin=Origin(xyz=(UPPER_LEN, 0.108, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="warm_titanium",
        name="elbow_bushing_0",
    )
    upper_arm.visual(
        Cylinder(radius=0.052, length=0.042),
        origin=Origin(xyz=(UPPER_LEN, -0.108, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="warm_titanium",
        name="elbow_bushing_1",
    )
    upper_arm.visual(
        Box((0.115, 0.034, 0.075)),
        origin=Origin(xyz=(0.432, 0.071, 0.0)),
        material="satin_pearl",
        name="elbow_yoke_lug_0",
    )
    upper_arm.visual(
        Box((0.115, 0.034, 0.075)),
        origin=Origin(xyz=(0.432, -0.071, 0.0)),
        material="satin_pearl",
        name="elbow_yoke_lug_1",
    )
    upper_arm.visual(
        Cylinder(radius=0.024, length=0.250),
        origin=Origin(xyz=(0.235, 0.0, -0.080), rpy=(0.0, pi / 2.0, 0.0)),
        material="matte_graphite",
        name="upper_actuator_housing",
    )
    upper_arm.visual(
        Cylinder(radius=0.027, length=0.035),
        origin=Origin(xyz=(0.105, 0.0, -0.080), rpy=(0.0, pi / 2.0, 0.0)),
        material="warm_titanium",
        name="upper_actuator_cap_0",
    )
    upper_arm.visual(
        Cylinder(radius=0.027, length=0.035),
        origin=Origin(xyz=(0.365, 0.0, -0.080), rpy=(0.0, pi / 2.0, 0.0)),
        material="warm_titanium",
        name="upper_actuator_cap_1",
    )
    upper_arm.visual(
        Box((0.040, 0.050, 0.035)),
        origin=Origin(xyz=(0.105, 0.0, -0.058)),
        material="matte_graphite",
        name="upper_actuator_mount_0",
    )
    upper_arm.visual(
        Box((0.040, 0.050, 0.035)),
        origin=Origin(xyz=(0.365, 0.0, -0.058)),
        material="matte_graphite",
        name="upper_actuator_mount_1",
    )
    for i, y in enumerate((-0.047, 0.047)):
        upper_arm.visual(
            Box((0.300, 0.006, 0.012)),
            origin=Origin(xyz=(0.250, y, 0.048)),
            material="black_rubber",
            name=f"upper_side_seam_{i}",
        )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.030, length=0.258),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="warm_titanium",
        name="elbow_shaft",
    )
    forearm.visual(
        Cylinder(radius=0.070, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="satin_pearl",
        name="elbow_collar",
    )
    forearm.visual(
        _mesh(_rounded_box((0.250, 0.100, 0.092), 0.016), "forearm_shell"),
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        material="satin_pearl",
        name="forearm_shell",
    )
    forearm.visual(
        Box((0.090, 0.065, 0.055)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material="satin_pearl",
        name="elbow_neck",
    )
    forearm.visual(
        Cylinder(radius=0.045, length=0.036),
        origin=Origin(xyz=(FOREARM_LEN, 0.082, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="warm_titanium",
        name="wrist_bushing_0",
    )
    forearm.visual(
        Cylinder(radius=0.045, length=0.036),
        origin=Origin(xyz=(FOREARM_LEN, -0.082, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="warm_titanium",
        name="wrist_bushing_1",
    )
    forearm.visual(
        Box((0.080, 0.028, 0.060)),
        origin=Origin(xyz=(0.337, 0.057, 0.0)),
        material="satin_pearl",
        name="wrist_yoke_lug_0",
    )
    forearm.visual(
        Box((0.080, 0.028, 0.060)),
        origin=Origin(xyz=(0.337, -0.057, 0.0)),
        material="satin_pearl",
        name="wrist_yoke_lug_1",
    )
    forearm.visual(
        Cylinder(radius=0.020, length=0.196),
        origin=Origin(xyz=(0.205, 0.0, -0.069), rpy=(0.0, pi / 2.0, 0.0)),
        material="matte_graphite",
        name="forearm_actuator_housing",
    )
    forearm.visual(
        Cylinder(radius=0.023, length=0.030),
        origin=Origin(xyz=(0.098, 0.0, -0.069), rpy=(0.0, pi / 2.0, 0.0)),
        material="warm_titanium",
        name="forearm_actuator_cap_0",
    )
    forearm.visual(
        Cylinder(radius=0.023, length=0.030),
        origin=Origin(xyz=(0.312, 0.0, -0.069), rpy=(0.0, pi / 2.0, 0.0)),
        material="warm_titanium",
        name="forearm_actuator_cap_1",
    )
    for i, y in enumerate((-0.043, 0.043)):
        forearm.visual(
            Box((0.235, 0.005, 0.010)),
            origin=Origin(xyz=(0.205, y, 0.043)),
            material="black_rubber",
            name=f"forearm_side_seam_{i}",
        )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.026, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="warm_titanium",
        name="wrist_pitch_shaft",
    )
    wrist.visual(
        Cylinder(radius=0.055, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="satin_pearl",
        name="wrist_pitch_collar",
    )
    wrist.visual(
        _mesh(_rounded_box((0.108, 0.065, 0.070), 0.012), "wrist_cradle"),
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        material="matte_graphite",
        name="wrist_cradle",
    )
    wrist.visual(
        Box((0.055, 0.052, 0.052)),
        origin=Origin(xyz=(0.122, 0.0, 0.0)),
        material="matte_graphite",
        name="roll_neck",
    )
    wrist.visual(
        Cylinder(radius=0.048, length=0.070),
        origin=Origin(xyz=(WRIST_LEN, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="satin_pearl",
        name="roll_bearing",
    )
    wrist.visual(
        Cylinder(radius=0.054, length=0.010),
        origin=Origin(xyz=(WRIST_LEN - 0.041, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_rubber",
        name="roll_seam",
    )

    tool_flange = model.part("tool_flange")
    tool_flange.visual(
        Cylinder(radius=0.028, length=0.070),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="warm_titanium",
        name="roll_spindle",
    )
    tool_flange.visual(
        Cylinder(radius=0.066, length=0.024),
        origin=Origin(xyz=(0.077, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="satin_pearl",
        name="mount_flange",
    )
    tool_flange.visual(
        Cylinder(radius=0.047, length=0.006),
        origin=Origin(xyz=(0.092, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_rubber",
        name="front_seal",
    )
    for i, (y, z) in enumerate(
        (
            (0.033, 0.033),
            (-0.033, 0.033),
            (-0.033, -0.033),
            (0.033, -0.033),
        )
    ):
        tool_flange.visual(
            Cylinder(radius=0.0065, length=0.006),
            origin=Origin(xyz=(0.098, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material="polished_bolt",
            name=f"flange_bolt_{i}",
        )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, BASE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-pi, upper=pi, effort=260.0, velocity=1.25),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.85, upper=1.55, effort=210.0, velocity=1.20),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.95, upper=1.15, effort=150.0, velocity=1.60),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(FOREARM_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.75, upper=1.75, effort=72.0, velocity=2.20),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=wrist,
        child=tool_flange,
        origin=Origin(xyz=(WRIST_LEN, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-pi, upper=pi, effort=42.0, velocity=3.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shoulder = object_model.get_part("shoulder")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    tool_flange = object_model.get_part("tool_flange")

    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

    ctx.allow_overlap(
        shoulder,
        upper_arm,
        elem_a="shoulder_bushing_0",
        elem_b="shoulder_shaft",
        reason="The shoulder shaft is intentionally captured through the side bushing.",
    )
    ctx.allow_overlap(
        shoulder,
        upper_arm,
        elem_a="shoulder_bushing_1",
        elem_b="shoulder_shaft",
        reason="The shoulder shaft is intentionally captured through the side bushing.",
    )
    ctx.allow_overlap(
        shoulder,
        upper_arm,
        elem_a="shoulder_cheek_0",
        elem_b="shoulder_shaft",
        reason="The shoulder shaft passes through the yoke cheek proxy at the bearing bore.",
    )
    ctx.allow_overlap(
        shoulder,
        upper_arm,
        elem_a="shoulder_cheek_1",
        elem_b="shoulder_shaft",
        reason="The shoulder shaft passes through the yoke cheek proxy at the bearing bore.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_bushing_0",
        elem_b="elbow_shaft",
        reason="The elbow shaft is intentionally seated in the yoke bushing.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_bushing_1",
        elem_b="elbow_shaft",
        reason="The elbow shaft is intentionally seated in the yoke bushing.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_yoke_lug_0",
        elem_b="elbow_shaft",
        reason="The elbow shaft passes through the side yoke cheek proxy at the bearing bore.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_yoke_lug_1",
        elem_b="elbow_shaft",
        reason="The elbow shaft passes through the side yoke cheek proxy at the bearing bore.",
    )
    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a="wrist_bushing_0",
        elem_b="wrist_pitch_shaft",
        reason="The wrist pitch shaft is intentionally captured through the side bushing.",
    )
    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a="wrist_bushing_1",
        elem_b="wrist_pitch_shaft",
        reason="The wrist pitch shaft is intentionally captured through the side bushing.",
    )
    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a="wrist_yoke_lug_0",
        elem_b="wrist_pitch_shaft",
        reason="The wrist pitch shaft passes through the side yoke cheek proxy at the bearing bore.",
    )
    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a="wrist_yoke_lug_1",
        elem_b="wrist_pitch_shaft",
        reason="The wrist pitch shaft passes through the side yoke cheek proxy at the bearing bore.",
    )
    ctx.allow_overlap(
        wrist,
        tool_flange,
        elem_a="roll_bearing",
        elem_b="roll_spindle",
        reason="The tool flange spindle is intentionally nested in the wrist roll bearing.",
    )

    ctx.expect_within(
        upper_arm,
        shoulder,
        axes="xz",
        inner_elem="shoulder_shaft",
        outer_elem="shoulder_bushing_0",
        margin=0.002,
        name="shoulder shaft centered in bushing",
    )
    ctx.expect_overlap(
        upper_arm,
        shoulder,
        axes="y",
        elem_a="shoulder_shaft",
        elem_b="shoulder_bushing_0",
        min_overlap=0.040,
        name="shoulder shaft retained through yoke",
    )
    ctx.expect_within(
        forearm,
        upper_arm,
        axes="xz",
        inner_elem="elbow_shaft",
        outer_elem="elbow_bushing_0",
        margin=0.002,
        name="elbow shaft centered in bushing",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="y",
        elem_a="elbow_shaft",
        elem_b="elbow_bushing_0",
        min_overlap=0.032,
        name="elbow shaft retained through yoke",
    )
    ctx.expect_within(
        wrist,
        forearm,
        axes="xz",
        inner_elem="wrist_pitch_shaft",
        outer_elem="wrist_bushing_0",
        margin=0.002,
        name="wrist pitch shaft centered in bushing",
    )
    ctx.expect_overlap(
        wrist,
        forearm,
        axes="y",
        elem_a="wrist_pitch_shaft",
        elem_b="wrist_bushing_0",
        min_overlap=0.028,
        name="wrist pitch shaft retained through yoke",
    )
    ctx.expect_within(
        tool_flange,
        wrist,
        axes="yz",
        inner_elem="roll_spindle",
        outer_elem="roll_bearing",
        margin=0.002,
        name="roll spindle centered in bearing",
    )
    ctx.expect_overlap(
        tool_flange,
        wrist,
        axes="x",
        elem_a="roll_spindle",
        elem_b="roll_bearing",
        min_overlap=0.020,
        name="roll spindle retained in bearing",
    )

    rest_elbow = ctx.part_world_position(forearm)
    rest_wrist = ctx.part_world_position(wrist)
    with ctx.pose({shoulder_pitch: 0.55, elbow_pitch: -0.75, wrist_pitch: 0.45, wrist_roll: 1.0}):
        raised_elbow = ctx.part_world_position(forearm)
        folded_wrist = ctx.part_world_position(wrist)
        ctx.expect_origin_distance(
            tool_flange,
            wrist,
            axes="xyz",
            min_dist=0.12,
            max_dist=0.18,
            name="tool flange remains on wrist roll axis",
        )

    ctx.check(
        "shoulder pitch moves elbow axis",
        rest_elbow is not None
        and raised_elbow is not None
        and raised_elbow[2] > rest_elbow[2] + 0.10,
        details=f"rest={rest_elbow}, raised={raised_elbow}",
    )
    ctx.check(
        "elbow joint changes wrist pose",
        rest_wrist is not None
        and folded_wrist is not None
        and abs(folded_wrist[0] - rest_wrist[0]) > 0.06,
        details=f"rest={rest_wrist}, folded={folded_wrist}",
    )

    return ctx.report()


object_model = build_object_model()

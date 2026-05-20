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
    TorusGeometry,
    mesh_from_geometry,
)


def _bearing_ring_mesh(name: str):
    """Create a thin washer-like bearing ring with its bore along local Y."""
    ring = TorusGeometry(0.025, 0.0045, radial_segments=18, tubular_segments=36)
    ring.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(ring, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_axis_gimbal_module")

    dark_metal = model.material("dark_anodized_metal", rgba=(0.06, 0.065, 0.07, 1.0))
    satin_metal = model.material("satin_aluminum", rgba=(0.55, 0.58, 0.60, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.012, 0.014, 1.0))
    face = model.material("brushed_faceplate", rgba=(0.30, 0.33, 0.36, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.165, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_metal,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.112, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=satin_metal,
        name="fixed_bearing_boss",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=black,
        name="center_cap",
    )

    turntable_fork = model.part("turntable_fork")
    turntable_fork.visual(
        Cylinder(radius=0.124, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=black,
        name="turntable_disk",
    )
    turntable_fork.visual(
        Box((0.086, 0.198, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=black,
        name="fork_bridge",
    )
    turntable_fork.visual(
        Box((0.055, 0.024, 0.178)),
        origin=Origin(xyz=(0.0, -0.088, 0.130)),
        material=black,
        name="arm_0",
    )
    turntable_fork.visual(
        _bearing_ring_mesh("bearing_ring_0"),
        origin=Origin(xyz=(0.0, -0.074, 0.145)),
        material=satin_metal,
        name="bearing_0",
    )
    turntable_fork.visual(
        Box((0.055, 0.024, 0.178)),
        origin=Origin(xyz=(0.0, 0.088, 0.130)),
        material=black,
        name="arm_1",
    )
    turntable_fork.visual(
        _bearing_ring_mesh("bearing_ring_1"),
        origin=Origin(xyz=(0.0, 0.074, 0.145)),
        material=satin_metal,
        name="bearing_1",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.012, length=0.152),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="axle",
    )
    pitch_cradle.visual(
        Box((0.032, 0.050, 0.028)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=satin_metal,
        name="cradle_web",
    )
    pitch_cradle.visual(
        Box((0.014, 0.092, 0.070)),
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        material=face,
        name="faceplate",
    )
    pitch_cradle.visual(
        Box((0.009, 0.058, 0.038)),
        origin=Origin(xyz=(0.0465, 0.0, 0.0)),
        material=dark_metal,
        name="raised_pad",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.052, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="center_socket",
    )

    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable_fork,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=turntable_fork,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.8, velocity=2.0, lower=-0.95, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    fork = object_model.get_part("turntable_fork")
    cradle = object_model.get_part("pitch_cradle")
    yaw = object_model.get_articulation("yaw_joint")
    pitch = object_model.get_articulation("pitch_joint")

    ctx.check(
        "yaw is vertical revolute",
        yaw.articulation_type == ArticulationType.REVOLUTE and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "pitch is horizontal revolute",
        pitch.articulation_type == ArticulationType.REVOLUTE and tuple(pitch.axis) == (0.0, 1.0, 0.0),
        details=f"type={pitch.articulation_type}, axis={pitch.axis}",
    )

    ctx.expect_gap(
        fork,
        base,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="fixed_bearing_boss",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable sits on fixed bearing boss",
    )
    ctx.expect_overlap(
        fork,
        base,
        axes="xy",
        elem_a="turntable_disk",
        elem_b="fixed_bearing_boss",
        min_overlap=0.08,
        name="yaw disk is centered over base bearing",
    )
    ctx.expect_gap(
        fork,
        cradle,
        axis="y",
        positive_elem="arm_1",
        negative_elem="axle",
        min_gap=0.0,
        max_gap=0.0005,
        name="axle clears positive fork arm",
    )
    ctx.expect_gap(
        cradle,
        fork,
        axis="y",
        positive_elem="axle",
        negative_elem="arm_0",
        min_gap=0.0,
        max_gap=0.0005,
        name="axle clears negative fork arm",
    )
    ctx.expect_within(
        cradle,
        fork,
        axes="y",
        inner_elem="faceplate",
        outer_elem="fork_bridge",
        margin=0.0,
        name="faceplate remains between fork arms",
    )

    rest_aabb = ctx.part_element_world_aabb(cradle, elem="faceplate")
    with ctx.pose({pitch: 0.75}):
        pitched_aabb = ctx.part_element_world_aabb(cradle, elem="faceplate")
    ctx.check(
        "pitch pose changes faceplate silhouette",
        rest_aabb is not None
        and pitched_aabb is not None
        and abs((pitched_aabb[1][2] - pitched_aabb[0][2]) - (rest_aabb[1][2] - rest_aabb[0][2])) > 0.005,
        details=f"rest_aabb={rest_aabb}, pitched_aabb={pitched_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _shade_shell(height: float, top_radius: float, bottom_radius: float, wall: float):
    """Return a hollow conical lampshade shell, open at both circular ends."""
    inner_top = top_radius - wall
    inner_bottom = bottom_radius - wall
    profile = (
        cq.Workplane("XZ")
        .moveTo(top_radius, 0.0)
        .lineTo(bottom_radius, -height)
        .lineTo(inner_bottom, -height)
        .lineTo(inner_top, 0.0)
        .close()
    )
    return profile.revolve(360.0, (0.0, 0.0, -1.0), (0.0, 0.0, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_link_reading_lamp")

    satin_black = model.material("satin_black", rgba=(0.02, 0.022, 0.024, 1.0))
    graphite = model.material("graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    warm_light = model.material("warm_light", rgba=(1.0, 0.78, 0.34, 0.72))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.60, 0.56, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.150, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=satin_black,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.115),
        origin=Origin(xyz=(-0.075, 0.0, 0.0925)),
        material=graphite,
        name="rear_pedestal",
    )
    base.visual(
        Box((0.050, 0.108, 0.075)),
        origin=Origin(xyz=(-0.061, 0.0, 0.1575)),
        material=graphite,
        name="shoulder_web",
    )
    for index, y in enumerate((-0.048, 0.048)):
        base.visual(
            Cylinder(radius=0.050, length=0.014),
            origin=Origin(xyz=(0.0, y, 0.160), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name=f"shoulder_cheek_{index}",
        )
    base.visual(
        Cylinder(radius=0.127, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=rubber,
        name="rubber_foot_ring",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.034, length=0.082),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="shoulder_boss",
    )
    lower_arm.visual(
        Cylinder(radius=0.012, length=0.414),
        origin=Origin(xyz=(0.0, 0.0, 0.207)),
        material=satin_black,
        name="lower_tube",
    )
    lower_arm.visual(
        Box((0.060, 0.024, 0.030)),
        origin=Origin(xyz=(-0.022, 0.0, 0.423)),
        material=graphite,
        name="elbow_neck",
    )
    lower_arm.visual(
        Box((0.020, 0.096, 0.035)),
        origin=Origin(xyz=(-0.050, 0.0, 0.445)),
        material=graphite,
        name="elbow_web",
    )
    for index, y in enumerate((-0.041, 0.041)):
        lower_arm.visual(
            Cylinder(radius=0.042, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.470), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name=f"elbow_cheek_{index}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="elbow_boss",
    )
    upper_arm.visual(
        Cylinder(radius=0.011, length=0.370),
        origin=Origin(xyz=(0.185, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="upper_tube",
    )
    upper_arm.visual(
        Cylinder(radius=0.014, length=0.086),
        origin=Origin(xyz=(0.382, 0.0, -0.037)),
        material=graphite,
        name="shade_stem",
    )
    upper_arm.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.382, 0.0, -0.047)),
        material=graphite,
        name="lamp_socket",
    )
    upper_arm.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.382, 0.0, -0.105)),
        material=warm_light,
        name="glowing_bulb",
    )
    upper_arm.visual(
        mesh_from_cadquery(
            _shade_shell(height=0.130, top_radius=0.044, bottom_radius=0.092, wall=0.004),
            "hollow_shade",
            tolerance=0.001,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.382, 0.0, -0.036)),
        material=satin_black,
        name="hollow_shade",
    )

    model.articulation(
        "shoulder_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.470)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=-1.10, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder = object_model.get_articulation("shoulder_hinge")
    elbow = object_model.get_articulation("elbow_hinge")
    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")

    ctx.check(
        "exactly two user hinges",
        len(object_model.articulations) == 2,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "lamp uses shoulder and elbow revolutes only",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE,
        details=f"types={(shoulder.articulation_type, elbow.articulation_type)}",
    )

    ctx.expect_within(
        lower_arm,
        base,
        axes="y",
        inner_elem="shoulder_boss",
        outer_elem="shoulder_web",
        margin=0.018,
        name="shoulder boss sits between base cheeks",
    )
    ctx.expect_within(
        upper_arm,
        lower_arm,
        axes="y",
        inner_elem="elbow_boss",
        outer_elem="elbow_web",
        margin=0.018,
        name="elbow boss sits between lower arm cheeks",
    )

    rest_elbow = ctx.part_world_position(upper_arm)
    with ctx.pose({shoulder: 0.55}):
        tilted_elbow = ctx.part_world_position(upper_arm)
    ctx.check(
        "shoulder hinge swings the two-link arm forward",
        rest_elbow is not None
        and tilted_elbow is not None
        and tilted_elbow[0] > rest_elbow[0] + 0.18
        and tilted_elbow[2] < rest_elbow[2] - 0.03,
        details=f"rest={rest_elbow}, tilted={tilted_elbow}",
    )

    def aabb_center_z(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return 0.5 * (low[2] + high[2])

    rest_shade_z = aabb_center_z(ctx.part_element_world_aabb(upper_arm, elem="hollow_shade"))
    with ctx.pose({elbow: 0.70}):
        bent_shade_z = aabb_center_z(ctx.part_element_world_aabb(upper_arm, elem="hollow_shade"))
    ctx.check(
        "elbow hinge aims the shade downward",
        rest_shade_z is not None and bent_shade_z is not None and bent_shade_z < rest_shade_z - 0.18,
        details=f"rest_shade_z={rest_shade_z}, bent_shade_z={bent_shade_z}",
    )

    return ctx.report()


object_model = build_object_model()

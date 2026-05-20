from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_lamp")

    powder_black = model.material("powder_black", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    brushed_brass = model.material("brushed_brass", rgba=(0.63, 0.55, 0.31, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.110, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=powder_black,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.097)),
        material=graphite,
        name="stem",
    )
    base.visual(
        Box((0.022, 0.074, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=graphite,
        name="shoulder_bridge",
    )
    for index, y in enumerate((-0.031, 0.031)):
        base.visual(
            Box((0.018, 0.012, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.190)),
            material=graphite,
            name=f"shoulder_cheek_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.220, 0.220, 0.210)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_brass,
        name="shoulder_collar",
    )
    for index, y in enumerate((-0.012, 0.012)):
        lower_arm.visual(
            Cylinder(radius=0.005, length=0.220),
            origin=Origin(
                xyz=(0.110, y, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brushed_brass,
            name=f"arm_rod_{index}",
        )
    lower_arm.visual(
        Box((0.027, 0.058, 0.016)),
        origin=Origin(xyz=(0.2115, 0.0, 0.0)),
        material=brushed_brass,
        name="elbow_bridge",
    )
    for index, y in enumerate((-0.022, 0.022)):
        lower_arm.visual(
            Box((0.018, 0.010, 0.050)),
            origin=Origin(xyz=(0.234, y, 0.0)),
            material=brushed_brass,
            name=f"elbow_cheek_{index}",
        )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.255, 0.050, 0.022)),
        mass=0.45,
        origin=Origin(xyz=(0.118, 0.0, 0.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.0095, length=0.034),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_brass,
        name="elbow_collar",
    )
    for index, y in enumerate((-0.0135, 0.0135)):
        upper_arm.visual(
            Cylinder(radius=0.0045, length=0.210),
            origin=Origin(
                xyz=(0.105, y, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brushed_brass,
            name=f"arm_rod_{index}",
        )
    for index, y in enumerate((-0.021, 0.021)):
        upper_arm.visual(
            Box((0.029, 0.008, 0.010)),
            origin=Origin(xyz=(0.198, y, -0.009)),
            material=brushed_brass,
            name=f"head_link_{index}",
        )
    for index, y in enumerate((-0.017, 0.017)):
        upper_arm.visual(
            Box((0.024, 0.006, 0.008)),
            origin=Origin(xyz=(0.188, y, -0.005)),
            material=brushed_brass,
            name=f"head_web_{index}",
        )
    for index, y in enumerate((-0.021, 0.021)):
        upper_arm.visual(
            Box((0.016, 0.010, 0.046)),
            origin=Origin(xyz=(0.220, y, 0.0)),
            material=brushed_brass,
            name=f"head_cheek_{index}",
        )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.235, 0.048, 0.022)),
        mass=0.34,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.15,
            upper=1.00,
            effort=18.0,
            velocity=1.4,
        ),
    )

    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.235, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.35,
            effort=14.0,
            velocity=1.8,
        ),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.009, length=0.032),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="tilt_collar",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(
            xyz=(0.025, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="neck_tube",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(
            xyz=(0.043, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="socket_housing",
    )
    head.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(
            xyz=(0.045, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=powder_black,
        name="shade_socket",
    )

    shade_length = 0.120
    rear_radius = 0.034
    front_radius = 0.068
    wall = 0.0025
    outer_shade = (
        cq.Workplane("XY")
        .circle(rear_radius)
        .workplane(offset=shade_length)
        .circle(front_radius)
        .loft(combine=True)
    )
    inner_shade = (
        cq.Workplane("XY")
        .workplane(offset=-0.003)
        .circle(rear_radius - wall)
        .workplane(offset=shade_length + 0.006)
        .circle(front_radius - wall)
        .loft(combine=True)
    )
    head.visual(
        mesh_from_cadquery(outer_shade.cut(inner_shade), "lamp_shade"),
        origin=Origin(
            xyz=(0.045, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=powder_black,
        name="shade",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(
            xyz=(0.077, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="bulb_neck",
    )
    head.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.101, 0.0, 0.0)),
        material=model.material("bulb_glass", rgba=(0.95, 0.92, 0.76, 0.70)),
        name="bulb",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.180, 0.140, 0.140)),
        mass=0.50,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
    )

    model.articulation(
        "upper_arm_to_head",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head,
        origin=Origin(xyz=(0.220, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.20,
            upper=0.40,
            effort=8.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")

    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    tilt = object_model.get_articulation("upper_arm_to_head")

    ctx.expect_origin_gap(
        head,
        base,
        axis="x",
        min_gap=0.40,
        name="rest pose projects the lamp head forward of the base",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({shoulder: 0.80, elbow: 0.95, tilt: 0.20}):
        raised_head_pos = ctx.part_world_position(head)
        ctx.check(
            "serial arm pose lifts the head substantially",
            rest_head_pos is not None
            and raised_head_pos is not None
            and raised_head_pos[2] > rest_head_pos[2] + 0.35,
            details=f"rest={rest_head_pos}, raised={raised_head_pos}",
        )
        ctx.check(
            "serial arm pose folds the reach back toward the base",
            rest_head_pos is not None
            and raised_head_pos is not None
            and raised_head_pos[0] < rest_head_pos[0] - 0.20,
            details=f"rest={rest_head_pos}, raised={raised_head_pos}",
        )

    with ctx.pose({shoulder: -0.15, elbow: 0.0, tilt: -0.20}):
        low_shade_aabb = ctx.part_element_world_aabb(head, elem="shade")
        ctx.check(
            "lowest allowed aiming pose keeps the shade above the tabletop plane",
            low_shade_aabb is not None and low_shade_aabb[0][2] > 0.0,
            details=f"shade_aabb={low_shade_aabb}",
        )

    with ctx.pose({shoulder: 0.40, elbow: 0.45, tilt: -0.20}):
        low_tilt_shade_aabb = ctx.part_element_world_aabb(head, elem="shade")
    with ctx.pose({shoulder: 0.40, elbow: 0.45, tilt: 0.40}):
        high_tilt_shade_aabb = ctx.part_element_world_aabb(head, elem="shade")
    ctx.check(
        "positive head tilt raises the shade opening",
        low_tilt_shade_aabb is not None
        and high_tilt_shade_aabb is not None
        and high_tilt_shade_aabb[0][2] > low_tilt_shade_aabb[0][2] + 0.03
        and high_tilt_shade_aabb[1][0] < low_tilt_shade_aabb[1][0] - 0.03,
        details=(
            f"low_tilt_shade_aabb={low_tilt_shade_aabb}, "
            f"high_tilt_shade_aabb={high_tilt_shade_aabb}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

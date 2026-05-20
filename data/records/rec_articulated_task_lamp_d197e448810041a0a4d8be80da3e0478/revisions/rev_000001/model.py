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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.105
BASE_THICKNESS = 0.016
COLLAR_RADIUS = 0.024
COLLAR_HEIGHT = 0.030
COLLAR_CAP_RADIUS = 0.030
COLLAR_CAP_HEIGHT = 0.008
SHOULDER_Z = 0.058

LOWER_ARM_LENGTH = 0.330
LOWER_TUBE_RADIUS = 0.009
LOWER_REST_PITCH = 0.88

UPPER_ARM_LENGTH = 0.240
UPPER_TUBE_RADIUS = 0.008
UPPER_REST_PITCH = 0.28

KNUCKLE_RADIUS = 0.0105
SHOULDER_KNUCKLE_LENGTH = 0.027
ELBOW_KNUCKLE_LENGTH = 0.027

FORK_CHEEK_LENGTH = 0.032
FORK_CHEEK_THICKNESS = 0.005
FORK_CHEEK_HEIGHT = 0.024
FORK_CHEEK_OFFSET = 0.016
FORK_BRIDGE_LENGTH = 0.014
FORK_BRIDGE_WIDTH = 0.039
FORK_BRIDGE_HEIGHT = 0.018
FORK_BRIDGE_BACKSET = 0.020

YOKE_CHEEK_LENGTH = 0.072
YOKE_CHEEK_THICKNESS = 0.005
YOKE_CHEEK_HEIGHT = 0.070
YOKE_CHEEK_OFFSET = 0.060
YOKE_BRIDGE_LENGTH = 0.028
YOKE_BRIDGE_WIDTH = 0.122
YOKE_BRIDGE_HEIGHT = 0.010
YOKE_BRIDGE_BACKSET = 0.044

SHADE_RADIUS = 0.045
SHADE_LENGTH = 0.100
SHADE_WALL = 0.003
SHADE_PIVOT_FROM_REAR = 0.015
SHADE_AXLE_RADIUS = 0.0045
SHADE_AXLE_HALF_SPAN = 0.0575
SHADE_REST_TILT = 1.00


def _add_x_cylinder(
    part,
    *,
    radius: float,
    length: float,
    x_start: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=(x_start + length / 2.0, y, z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_y_cylinder(
    part,
    *,
    radius: float,
    length: float,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=(x, y, z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def _build_shade_shape():
    outer_shell = cq.Workplane("YZ").circle(SHADE_RADIUS).extrude(SHADE_LENGTH)
    inner_void = (
        cq.Workplane("YZ")
        .circle(SHADE_RADIUS - SHADE_WALL)
        .extrude(SHADE_LENGTH)
    )
    drum = outer_shell.cut(inner_void).translate((-SHADE_PIVOT_FROM_REAR, 0.0, 0.0))
    axle = (
        cq.Workplane("XZ")
        .workplane(offset=-SHADE_AXLE_HALF_SPAN)
        .circle(SHADE_AXLE_RADIUS)
        .extrude(2.0 * SHADE_AXLE_HALF_SPAN)
    )
    return drum.union(axle)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((low + high) / 2.0 for low, high in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_desk_lamp")

    base_color = model.material("base_color", rgba=(0.14, 0.14, 0.15, 1.0))
    arm_color = model.material("arm_color", rgba=(0.60, 0.53, 0.34, 1.0))
    shade_color = model.material("shade_color", rgba=(0.90, 0.89, 0.84, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=base_color,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_THICKNESS + COLLAR_HEIGHT / 2.0 - 0.001),
        ),
        material=base_color,
        name="collar_stem",
    )
    base.visual(
        Box((FORK_CHEEK_LENGTH, FORK_CHEEK_THICKNESS, FORK_CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.001, FORK_CHEEK_OFFSET, SHOULDER_Z)),
        material=base_color,
        name="shoulder_cheek_0",
    )
    base.visual(
        Box((FORK_CHEEK_LENGTH, FORK_CHEEK_THICKNESS, FORK_CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.001, -FORK_CHEEK_OFFSET, SHOULDER_Z)),
        material=base_color,
        name="shoulder_cheek_1",
    )
    base.visual(
        Box((FORK_BRIDGE_LENGTH, FORK_BRIDGE_WIDTH, FORK_BRIDGE_HEIGHT)),
        origin=Origin(xyz=(-FORK_BRIDGE_BACKSET, 0.0, SHOULDER_Z)),
        material=base_color,
        name="shoulder_bridge",
    )
    base.visual(
        Box((0.010, 0.024, 0.012)),
        origin=Origin(xyz=(-0.020, 0.0, 0.046)),
        material=base_color,
        name="shoulder_neck",
    )

    lower_arm = model.part("lower_arm")
    _add_y_cylinder(
        lower_arm,
        radius=KNUCKLE_RADIUS,
        length=SHOULDER_KNUCKLE_LENGTH,
        material=arm_color,
        name="shoulder_knuckle",
    )
    _add_x_cylinder(
        lower_arm,
        radius=LOWER_TUBE_RADIUS,
        length=LOWER_ARM_LENGTH - 0.018,
        material=arm_color,
        name="lower_tube",
    )
    lower_arm.visual(
        Box((FORK_CHEEK_LENGTH, FORK_CHEEK_THICKNESS, FORK_CHEEK_HEIGHT)),
        origin=Origin(xyz=(LOWER_ARM_LENGTH + 0.001, FORK_CHEEK_OFFSET, 0.0)),
        material=arm_color,
        name="elbow_cheek_0",
    )
    lower_arm.visual(
        Box((FORK_CHEEK_LENGTH, FORK_CHEEK_THICKNESS, FORK_CHEEK_HEIGHT)),
        origin=Origin(xyz=(LOWER_ARM_LENGTH + 0.001, -FORK_CHEEK_OFFSET, 0.0)),
        material=arm_color,
        name="elbow_cheek_1",
    )
    lower_arm.visual(
        Box((FORK_BRIDGE_LENGTH, FORK_BRIDGE_WIDTH, FORK_BRIDGE_HEIGHT)),
        origin=Origin(xyz=(LOWER_ARM_LENGTH - FORK_BRIDGE_BACKSET, 0.0, 0.0)),
        material=arm_color,
        name="elbow_bridge",
    )

    upper_arm = model.part("upper_arm")
    _add_y_cylinder(
        upper_arm,
        radius=KNUCKLE_RADIUS,
        length=ELBOW_KNUCKLE_LENGTH,
        material=arm_color,
        name="shoulder_knuckle",
    )
    _add_x_cylinder(
        upper_arm,
        radius=UPPER_TUBE_RADIUS,
        length=UPPER_ARM_LENGTH - 0.026,
        material=arm_color,
        name="upper_tube",
    )
    upper_arm.visual(
        Box((YOKE_CHEEK_LENGTH, YOKE_CHEEK_THICKNESS, YOKE_CHEEK_HEIGHT)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH - 0.020, YOKE_CHEEK_OFFSET, -0.010)),
        material=arm_color,
        name="yoke_cheek_0",
    )
    upper_arm.visual(
        Box((YOKE_CHEEK_LENGTH, YOKE_CHEEK_THICKNESS, YOKE_CHEEK_HEIGHT)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH - 0.020, -YOKE_CHEEK_OFFSET, -0.010)),
        material=arm_color,
        name="yoke_cheek_1",
    )
    upper_arm.visual(
        Box((YOKE_BRIDGE_LENGTH, YOKE_BRIDGE_WIDTH, YOKE_BRIDGE_HEIGHT)),
        origin=Origin(
            xyz=(UPPER_ARM_LENGTH - YOKE_BRIDGE_BACKSET, 0.0, -0.050),
        ),
        material=arm_color,
        name="yoke_bridge",
    )
    upper_arm.visual(
        Box((0.032, 0.022, 0.040)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH - 0.040, 0.0, -0.028)),
        material=arm_color,
        name="yoke_neck",
    )

    shade = model.part("shade")
    shade.visual(
        mesh_from_cadquery(_build_shade_shape(), "shade_shell"),
        material=shade_color,
        name="shade_shell",
    )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z), rpy=(0.0, -LOWER_REST_PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.45,
            upper=0.55,
            effort=18.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(
            xyz=(LOWER_ARM_LENGTH, 0.0, 0.0),
            rpy=(0.0, -UPPER_REST_PITCH, 0.0),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.70,
            upper=0.60,
            effort=12.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "shade_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(
            xyz=(UPPER_ARM_LENGTH, 0.0, 0.0),
            rpy=(0.0, SHADE_REST_TILT, 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.80,
            upper=0.45,
            effort=6.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")

    base_hinge = object_model.get_articulation("base_hinge")
    elbow_hinge = object_model.get_articulation("elbow_hinge")
    shade_hinge = object_model.get_articulation("shade_hinge")

    ctx.allow_overlap(
        shade,
        upper_arm,
        elem_a="shade_shell",
        elem_b="yoke_neck",
        reason=(
            "The rear socket post is intentionally simplified as a solid neck that "
            "passes into the hollow lamp head."
        ),
    )

    ctx.expect_origin_gap(
        lower_arm,
        base,
        axis="z",
        min_gap=SHOULDER_Z - 0.001,
        max_gap=SHOULDER_Z + 0.001,
        name="lower arm pivots from the base collar height",
    )
    ctx.expect_origin_gap(
        shade,
        base,
        axis="z",
        min_gap=0.22,
        name="shade pivot sits well above the base",
    )

    with ctx.pose({base_hinge: base_hinge.motion_limits.lower}):
        lowered_elbow = ctx.part_world_position(upper_arm)
    with ctx.pose({base_hinge: base_hinge.motion_limits.upper}):
        raised_elbow = ctx.part_world_position(upper_arm)
    ctx.check(
        "base hinge raises the elbow",
        lowered_elbow is not None
        and raised_elbow is not None
        and raised_elbow[2] > lowered_elbow[2] + 0.15,
        details=f"lowered={lowered_elbow}, raised={raised_elbow}",
    )

    with ctx.pose({elbow_hinge: elbow_hinge.motion_limits.lower}):
        lowered_shade = ctx.part_world_position(shade)
    with ctx.pose({elbow_hinge: elbow_hinge.motion_limits.upper}):
        raised_shade = ctx.part_world_position(shade)
    ctx.check(
        "elbow hinge raises the shade support point",
        lowered_shade is not None
        and raised_shade is not None
        and raised_shade[2] > lowered_shade[2] + 0.12,
        details=f"lowered={lowered_shade}, raised={raised_shade}",
    )

    with ctx.pose({shade_hinge: shade_hinge.motion_limits.lower}):
        lifted_shell_center = _aabb_center(
            ctx.part_element_world_aabb(shade, elem="shade_shell")
        )
    with ctx.pose({shade_hinge: shade_hinge.motion_limits.upper}):
        dipped_shell_center = _aabb_center(
            ctx.part_element_world_aabb(shade, elem="shade_shell")
        )
    ctx.check(
        "shade hinge changes the lamp aim",
        lifted_shell_center is not None
        and dipped_shell_center is not None
        and dipped_shell_center[2] < lifted_shell_center[2] - 0.02,
        details=f"lifted={lifted_shell_center}, dipped={dipped_shell_center}",
    )

    return ctx.report()


object_model = build_object_model()

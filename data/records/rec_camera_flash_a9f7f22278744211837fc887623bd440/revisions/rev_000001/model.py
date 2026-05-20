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


def _rounded_box(size: tuple[float, float, float], fillet: float) -> cq.Workplane:
    sx, sy, sz = size
    radius = min(fillet, 0.4 * sx, 0.4 * sy, 0.4 * sz)
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(radius)


def _build_body_shell() -> cq.Workplane:
    lower = _rounded_box((0.050, 0.038, 0.094), 0.004).translate((0.0, 0.0, 0.047))
    upper = _rounded_box((0.042, 0.034, 0.022), 0.003).translate((0.003, 0.0, 0.105))
    collar = _rounded_box((0.030, 0.030, 0.008), 0.002).translate((0.0, 0.0, 0.120))
    shoe_pedestal = _rounded_box((0.020, 0.022, 0.010), 0.0015).translate((0.0, 0.0, -0.005))
    shoe_foot = _rounded_box((0.018, 0.032, 0.004), 0.001).translate((0.0, 0.0, -0.012))

    shell = lower.union(upper).union(collar).union(shoe_pedestal).union(shoe_foot)

    door_pocket = cq.Workplane("XY").box(0.0368, 0.0032, 0.069).translate((0.0005, 0.0172, 0.052))
    shell = shell.cut(door_pocket)

    return shell


def _build_head_shell() -> cq.Workplane:
    main = _rounded_box((0.074, 0.054, 0.043), 0.004).translate((0.027, 0.0, 0.005))
    rear_cap = _rounded_box((0.020, 0.044, 0.030), 0.003).translate((-0.006, 0.0, 0.001))
    return main.union(rear_cap)


def _build_door_panel() -> cq.Workplane:
    return _rounded_box((0.0355, 0.0024, 0.067), 0.0012).translate((0.01775, -0.0018, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="speedlight_flash")

    shell_black = model.material("shell_black", rgba=(0.10, 0.10, 0.11, 1.0))
    mount_black = model.material("mount_black", rgba=(0.14, 0.14, 0.15, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.92, 0.92, 0.88, 1.0))
    metal = model.material("metal", rgba=(0.72, 0.72, 0.74, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "body_shell"),
        material=shell_black,
        name="body_shell",
    )
    for index, z_center in enumerate((0.024, 0.052, 0.080)):
        body.visual(
            Cylinder(radius=0.0018, length=0.012),
            origin=Origin(xyz=(-0.019, 0.0186, z_center)),
            material=shell_black,
            name=f"body_hinge_{index}",
        )
    for index, y_center in enumerate((-0.0105, 0.0105)):
        body.visual(
            Box((0.016, 0.003, 0.004)),
            origin=Origin(xyz=(0.0, y_center, -0.012)),
            material=metal,
            name=f"shoe_rail_{index}",
        )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=mount_black,
        name="swivel_base",
    )
    swivel.visual(
        Box((0.016, 0.020, 0.024)),
        origin=Origin(xyz=(0.004, 0.0, 0.022)),
        material=mount_black,
        name="neck_block",
    )
    swivel.visual(
        Box((0.018, 0.060, 0.008)),
        origin=Origin(xyz=(0.004, 0.0, 0.018)),
        material=mount_black,
        name="yoke_bridge",
    )
    swivel.visual(
        Box((0.020, 0.004, 0.034)),
        origin=Origin(xyz=(0.012, -0.030, 0.037)),
        material=mount_black,
        name="yoke_arm_0",
    )
    swivel.visual(
        Box((0.020, 0.004, 0.034)),
        origin=Origin(xyz=(0.012, 0.030, 0.037)),
        material=mount_black,
        name="yoke_arm_1",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_build_head_shell(), "head_shell"),
        material=shell_black,
        name="head_shell",
    )
    head.visual(
        Box((0.004, 0.044, 0.032)),
        origin=Origin(xyz=(0.062, 0.0, 0.006)),
        material=diffuser_white,
        name="lamp_window",
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        mesh_from_cadquery(_build_door_panel(), "battery_door_panel"),
        material=shell_black,
        name="door_panel",
    )
    for index, z_center in enumerate((-0.014, 0.014)):
        battery_door.visual(
            Cylinder(radius=0.0018, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=shell_black,
            name=f"door_hinge_{index}",
        )

    model.articulation(
        "body_to_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-math.radians(180.0),
            upper=math.radians(180.0),
        ),
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.018, 0.0, 0.050)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=math.radians(-7.0),
            upper=math.radians(90.0),
        ),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(-0.019, 0.0186, 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    swivel = object_model.get_part("swivel")
    head = object_model.get_part("head")
    battery_door = object_model.get_part("battery_door")

    swivel_joint = object_model.get_articulation("body_to_swivel")
    tilt_joint = object_model.get_articulation("swivel_to_head")
    door_joint = object_model.get_articulation("body_to_battery_door")

    ctx.expect_overlap(
        battery_door,
        body,
        axes="xz",
        elem_a="door_panel",
        elem_b="body_shell",
        min_overlap=0.030,
        name="battery door covers the body side opening",
    )
    ctx.expect_gap(
        battery_door,
        body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="body_shell",
        min_gap=-0.0038,
        max_gap=-0.0002,
        name="battery door stays seated in the side recess",
    )
    ctx.expect_overlap(
        swivel,
        head,
        axes="xz",
        elem_a="yoke_arm_0",
        elem_b="head_shell",
        min_overlap=0.008,
        name="first yoke arm reaches the head side",
    )
    ctx.expect_overlap(
        swivel,
        head,
        axes="xz",
        elem_a="yoke_arm_1",
        elem_b="head_shell",
        min_overlap=0.008,
        name="second yoke arm reaches the head side",
    )

    closed_door_aabb = ctx.part_world_aabb(battery_door)
    with ctx.pose({door_joint: math.radians(80.0)}):
        opened_door_aabb = ctx.part_world_aabb(battery_door)

    ctx.check(
        "battery door swings outward",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[1][1] > closed_door_aabb[1][1] + 0.012,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )

    rest_head_aabb = ctx.part_world_aabb(head)
    with ctx.pose({tilt_joint: math.radians(90.0)}):
        raised_head_aabb = ctx.part_world_aabb(head)

    ctx.check(
        "head tilts upward",
        rest_head_aabb is not None
        and raised_head_aabb is not None
        and raised_head_aabb[1][2] > rest_head_aabb[1][2] + 0.020,
        details=f"rest={rest_head_aabb}, raised={raised_head_aabb}",
    )

    rest_swivel_aabb = ctx.part_world_aabb(head)
    with ctx.pose({swivel_joint: math.radians(85.0)}):
        side_swivel_aabb = ctx.part_world_aabb(head)

    ctx.check(
        "head swivels sideways",
        rest_swivel_aabb is not None
        and side_swivel_aabb is not None
        and side_swivel_aabb[1][1] > rest_swivel_aabb[1][1] + 0.020,
        details=f"rest={rest_swivel_aabb}, turned={side_swivel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

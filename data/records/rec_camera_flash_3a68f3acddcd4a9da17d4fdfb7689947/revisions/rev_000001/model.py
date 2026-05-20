from __future__ import annotations

from math import pi

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


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((low + high) * 0.5 for low, high in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hot_shoe_flash")

    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark = model.material("dark", rgba=(0.18, 0.18, 0.20, 1.0))
    mid = model.material("mid", rgba=(0.28, 0.29, 0.31, 1.0))
    screen = model.material("screen", rgba=(0.08, 0.10, 0.13, 1.0))
    diffuser = model.material("diffuser", rgba=(0.92, 0.94, 0.96, 1.0))
    metal = model.material("metal", rgba=(0.64, 0.66, 0.69, 1.0))

    body_w = 0.058
    body_d = 0.036
    body_h = 0.068
    body_bottom = 0.016
    body_top = body_bottom + body_h

    head_w = 0.074
    head_d = 0.040
    head_h = 0.046

    body = model.part("body")
    body.visual(
        Box((0.020, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=metal,
        name="shoe_rail",
    )
    body.visual(
        Box((0.030, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark,
        name="shoe_block",
    )
    body.visual(
        Box((0.036, 0.028, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=black,
        name="body_stem",
    )
    body.visual(
        Box((body_w, body_d, body_h)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + body_h * 0.5)),
        material=black,
        name="body_shell",
    )
    body.visual(
        Box((0.042, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, -body_d * 0.5 - 0.003, 0.066)),
        material=screen,
        name="display",
    )
    body.visual(
        Cylinder(radius=0.0105, length=0.0035),
        origin=Origin(xyz=(0.0, -body_d * 0.5 - 0.00175, 0.038), rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark,
        name="dial",
    )
    body.visual(
        Cylinder(radius=0.0042, length=0.0030),
        origin=Origin(xyz=(-0.014, -body_d * 0.5 - 0.0015, 0.030), rpy=(pi * 0.5, 0.0, 0.0)),
        material=mid,
        name="button_0",
    )
    body.visual(
        Cylinder(radius=0.0042, length=0.0030),
        origin=Origin(xyz=(0.014, -body_d * 0.5 - 0.0015, 0.030), rpy=(pi * 0.5, 0.0, 0.0)),
        material=mid,
        name="button_1",
    )
    body.visual(
        Box((0.050, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, body_d * 0.5 + 0.003, 0.056)),
        material=dark,
        name="front_sensor",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, body_top + 0.003)),
        material=dark,
        name="swivel_plinth",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.0135, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=mid,
        name="swivel_collar",
    )
    neck.visual(
        Box((0.018, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, -0.001, 0.015)),
        material=dark,
        name="neck_core",
    )
    neck.visual(
        Box((0.030, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.004, 0.026)),
        material=dark,
        name="yoke_bridge",
    )
    neck.visual(
        Box((0.004, 0.006, 0.020)),
        origin=Origin(xyz=(-0.017, -0.004, 0.016)),
        material=dark,
        name="yoke_arm_0",
    )
    neck.visual(
        Box((0.004, 0.006, 0.020)),
        origin=Origin(xyz=(0.017, -0.004, 0.016)),
        material=dark,
        name="yoke_arm_1",
    )

    head = model.part("head")
    head.visual(
        Box((head_w, head_d, head_h)),
        origin=Origin(xyz=(0.0, 0.021, 0.021)),
        material=black,
        name="head_shell",
    )
    head.visual(
        Box((head_w - 0.010, 0.006, head_h - 0.010)),
        origin=Origin(xyz=(0.0, 0.021 + head_d * 0.5 + 0.003, 0.021)),
        material=diffuser,
        name="flash_window",
    )
    head.visual(
        Box((head_w - 0.012, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.001, 0.037)),
        material=mid,
        name="head_top_cap",
    )
    head.visual(
        Box((0.010, 0.026, 0.004)),
        origin=Origin(xyz=(-0.028, 0.004, 0.010)),
        material=mid,
        name="head_side_trim_0",
    )
    head.visual(
        Box((0.010, 0.026, 0.004)),
        origin=Origin(xyz=(0.028, 0.004, 0.010)),
        material=mid,
        name="head_side_trim_1",
    )
    head.visual(
        Box((0.004, 0.004, 0.010)),
        origin=Origin(xyz=(-0.017, 0.001, -0.001)),
        material=mid,
        name="head_boss_0",
    )
    head.visual(
        Box((0.004, 0.004, 0.010)),
        origin=Origin(xyz=(0.017, 0.001, -0.001)),
        material=mid,
        name="head_boss_1",
    )

    door = model.part("battery_door")
    door.visual(
        Box((0.0022, 0.026, 0.044)),
        origin=Origin(xyz=(0.0011, 0.013, 0.0)),
        material=mid,
        name="door_panel",
    )
    door.visual(
        Box((0.0012, 0.018, 0.036)),
        origin=Origin(xyz=(0.0019, 0.013, 0.0)),
        material=dark,
        name="door_inset",
    )
    door.visual(
        Box((0.0035, 0.004, 0.012)),
        origin=Origin(xyz=(0.00175, 0.004, 0.014)),
        material=mid,
        name="door_leaf_0",
    )
    door.visual(
        Box((0.0035, 0.004, 0.012)),
        origin=Origin(xyz=(0.00175, 0.004, -0.014)),
        material=mid,
        name="door_leaf_1",
    )

    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, body_top + 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(body_w * 0.5, -0.013, 0.049)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("head")
    door = object_model.get_part("battery_door")

    swivel = object_model.get_articulation("body_to_neck")
    tilt = object_model.get_articulation("neck_to_head")
    door_hinge = object_model.get_articulation("body_to_battery_door")

    ctx.expect_gap(
        door,
        body,
        axis="x",
        positive_elem="door_panel",
        negative_elem="body_shell",
        min_gap=0.0,
        max_gap=0.001,
        name="battery door sits flush on the body side",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        elem_a="door_panel",
        elem_b="body_shell",
        min_overlap=0.020,
        name="battery door covers a tall side opening",
    )
    ctx.expect_gap(
        head,
        body,
        axis="z",
        positive_elem="head_shell",
        negative_elem="body_shell",
        min_gap=0.012,
        name="flash head stays clearly above the battery body",
    )
    ctx.expect_gap(
        head,
        neck,
        axis="y",
        positive_elem="head_shell",
        negative_elem="yoke_bridge",
        min_gap=0.0015,
        name="flash head stays visibly separate from the neck bridge",
    )

    flash_window_rest = _aabb_center(ctx.part_element_world_aabb(head, elem="flash_window"))
    with ctx.pose({tilt: 1.20}):
        flash_window_tilted = _aabb_center(ctx.part_element_world_aabb(head, elem="flash_window"))
    ctx.check(
        "flash head tilts upward",
        flash_window_rest is not None
        and flash_window_tilted is not None
        and flash_window_tilted[2] > flash_window_rest[2] + 0.020,
        details=f"rest={flash_window_rest}, tilted={flash_window_tilted}",
    )

    flash_window_forward = _aabb_center(ctx.part_element_world_aabb(head, elem="flash_window"))
    with ctx.pose({swivel: 1.20}):
        flash_window_swiveled = _aabb_center(ctx.part_element_world_aabb(head, elem="flash_window"))
    ctx.check(
        "neck swivel turns the head sideways",
        flash_window_forward is not None
        and flash_window_swiveled is not None
        and abs(flash_window_swiveled[0] - flash_window_forward[0]) > 0.020,
        details=f"rest={flash_window_forward}, swiveled={flash_window_swiveled}",
    )

    door_closed = _aabb_center(ctx.part_element_world_aabb(door, elem="door_panel"))
    with ctx.pose({door_hinge: 1.00}):
        door_open = _aabb_center(ctx.part_element_world_aabb(door, elem="door_panel"))
    ctx.check(
        "battery door swings outward",
        door_closed is not None
        and door_open is not None
        and door_open[0] > door_closed[0] + 0.008,
        details=f"closed={door_closed}, open={door_open}",
    )

    return ctx.report()


object_model = build_object_model()

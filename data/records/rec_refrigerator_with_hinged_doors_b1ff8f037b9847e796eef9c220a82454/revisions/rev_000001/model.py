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
    model = ArticulatedObject(name="side_by_side_refrigerator")

    stainless = model.material("brushed_warm_stainless", color=(0.78, 0.76, 0.70, 1.0))
    side_shadow = model.material("side_shadow_gray", color=(0.36, 0.37, 0.37, 1.0))
    black = model.material("soft_black_plastic", color=(0.015, 0.017, 0.018, 1.0))
    gasket = model.material("dark_magnetic_gasket", color=(0.02, 0.022, 0.024, 1.0))
    recess_black = model.material("dispenser_recess_black", color=(0.005, 0.007, 0.009, 1.0))
    paddle_gray = model.material("matte_paddle_gray", color=(0.45, 0.47, 0.48, 1.0))
    chrome = model.material("soft_chrome", color=(0.86, 0.84, 0.78, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((1.08, 0.66, 1.80)),
        origin=Origin(xyz=(0.0, 0.03, 0.90)),
        material=stainless,
        name="outer_case",
    )
    cabinet.visual(
        Box((1.02, 0.035, 0.08)),
        origin=Origin(xyz=(0.0, -0.285, 0.04)),
        material=black,
        name="toe_kick",
    )
    cabinet.visual(
        Box((0.012, 0.020, 1.70)),
        origin=Origin(xyz=(0.0, -0.310, 0.90)),
        material=gasket,
        name="center_mullion_shadow",
    )
    for x, name in ((-0.526, "freezer_hinge_barrel"), (0.526, "fresh_hinge_barrel")):
        cabinet.visual(
            Cylinder(radius=0.016, length=1.62),
            origin=Origin(xyz=(x, -0.277, 0.91)),
            material=side_shadow,
            name=name,
        )

    door_width = 0.501
    door_height = 1.72
    door_thickness = 0.045

    freezer_door = model.part("freezer_door")
    freezer_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width / 2.0, 0.0, 0.90)),
        material=stainless,
        name="freezer_slab",
    )
    freezer_door.visual(
        Box((0.020, 0.030, 1.44)),
        origin=Origin(xyz=(-0.009, 0.020, 0.90)),
        material=side_shadow,
        name="hinge_leaf",
    )
    freezer_door.visual(
        Box((0.006, 0.012, 1.64)),
        origin=Origin(xyz=(door_width - 0.003, -0.026, 0.91)),
        material=gasket,
        name="center_gasket",
    )
    freezer_door.visual(
        Cylinder(radius=0.017, length=1.16),
        origin=Origin(xyz=(door_width - 0.070, -0.063, 0.96)),
        material=chrome,
        name="vertical_handle",
    )
    for z, name in ((0.45, "handle_lower_standoff"), (1.47, "handle_upper_standoff")):
        freezer_door.visual(
            Box((0.040, 0.052, 0.070)),
            origin=Origin(xyz=(door_width - 0.070, -0.040, z)),
            material=chrome,
            name=name,
        )

    # The freezer-side ice/water dispenser is a shallow, dark inset built into
    # the moving door.  The lip pieces overlap the main slab slightly so the
    # door reads as one molded assembly rather than floating trim.
    recess_x = 0.250
    recess_z = 1.06
    freezer_door.visual(
        Box((0.250, 0.008, 0.460)),
        origin=Origin(xyz=(recess_x, -0.028, recess_z)),
        material=recess_black,
        name="dispenser_back",
    )
    freezer_door.visual(
        Box((0.315, 0.032, 0.040)),
        origin=Origin(xyz=(recess_x, -0.043, recess_z + 0.250)),
        material=stainless,
        name="dispenser_top_lip",
    )
    freezer_door.visual(
        Box((0.315, 0.032, 0.048)),
        origin=Origin(xyz=(recess_x, -0.043, recess_z - 0.250)),
        material=stainless,
        name="dispenser_drip_lip",
    )
    freezer_door.visual(
        Box((0.040, 0.032, 0.540)),
        origin=Origin(xyz=(recess_x - 0.158, -0.043, recess_z)),
        material=stainless,
        name="dispenser_side_lip_0",
    )
    freezer_door.visual(
        Box((0.040, 0.032, 0.540)),
        origin=Origin(xyz=(recess_x + 0.158, -0.043, recess_z)),
        material=stainless,
        name="dispenser_side_lip_1",
    )
    freezer_door.visual(
        Box((0.180, 0.018, 0.030)),
        origin=Origin(xyz=(recess_x, -0.055, recess_z - 0.220)),
        material=black,
        name="drip_tray_slot",
    )
    for x, name in (
        (recess_x - 0.093, "paddle_socket_0"),
        (recess_x + 0.093, "paddle_socket_1"),
    ):
        freezer_door.visual(
            Box((0.016, 0.060, 0.036)),
            origin=Origin(xyz=(x, -0.049, recess_z + 0.090)),
            material=black,
            name=name,
        )

    fresh_door = model.part("fresh_door")
    fresh_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(-door_width / 2.0, 0.0, 0.90)),
        material=stainless,
        name="fresh_slab",
    )
    fresh_door.visual(
        Box((0.020, 0.030, 1.44)),
        origin=Origin(xyz=(0.009, 0.020, 0.90)),
        material=side_shadow,
        name="hinge_leaf",
    )
    fresh_door.visual(
        Box((0.006, 0.012, 1.64)),
        origin=Origin(xyz=(-(door_width - 0.003), -0.026, 0.91)),
        material=gasket,
        name="center_gasket",
    )
    fresh_door.visual(
        Cylinder(radius=0.017, length=1.16),
        origin=Origin(xyz=(-(door_width - 0.070), -0.063, 0.96)),
        material=chrome,
        name="vertical_handle",
    )
    for z, name in ((0.45, "handle_lower_standoff"), (1.47, "handle_upper_standoff")):
        fresh_door.visual(
            Box((0.040, 0.052, 0.070)),
            origin=Origin(xyz=(-(door_width - 0.070), -0.040, z)),
            material=chrome,
            name=name,
        )

    paddle = model.part("paddle")
    paddle.visual(
        Box((0.150, 0.014, 0.240)),
        origin=Origin(xyz=(0.0, -0.004, -0.120)),
        material=paddle_gray,
        name="paddle_plate",
    )
    paddle.visual(
        Cylinder(radius=0.010, length=0.170),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="pivot_pin",
    )

    model.articulation(
        "freezer_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=freezer_door,
        origin=Origin(xyz=(-0.510, -0.335, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.95),
    )
    model.articulation(
        "fresh_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=fresh_door,
        origin=Origin(xyz=(0.510, -0.335, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.95),
    )
    model.articulation(
        "paddle_pivot",
        ArticulationType.REVOLUTE,
        parent=freezer_door,
        child=paddle,
        origin=Origin(xyz=(recess_x, -0.063, recess_z + 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=-0.08, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    freezer = object_model.get_part("freezer_door")
    fresh = object_model.get_part("fresh_door")
    cabinet = object_model.get_part("cabinet")
    paddle = object_model.get_part("paddle")
    freezer_hinge = object_model.get_articulation("freezer_hinge")
    fresh_hinge = object_model.get_articulation("fresh_hinge")
    paddle_pivot = object_model.get_articulation("paddle_pivot")

    ctx.expect_gap(
        fresh,
        freezer,
        axis="x",
        min_gap=0.010,
        max_gap=0.024,
        name="closed doors meet with a narrow center seam",
    )
    ctx.expect_overlap(
        freezer,
        fresh,
        axes="z",
        min_overlap=1.60,
        name="two doors are both full height",
    )
    ctx.expect_overlap(
        freezer,
        fresh,
        axes="y",
        min_overlap=0.030,
        name="two doors share the same front plane",
    )
    ctx.expect_gap(
        cabinet,
        freezer,
        axis="y",
        min_gap=-0.001,
        max_gap=0.030,
        positive_elem="outer_case",
        negative_elem="freezer_slab",
        name="freezer door sits just in front of cabinet",
    )
    ctx.expect_gap(
        cabinet,
        fresh,
        axis="y",
        min_gap=-0.001,
        max_gap=0.030,
        positive_elem="outer_case",
        negative_elem="fresh_slab",
        name="fresh door sits just in front of cabinet",
    )
    ctx.expect_within(
        paddle,
        freezer,
        axes="xz",
        inner_elem="paddle_plate",
        outer_elem="dispenser_back",
        margin=0.020,
        name="paddle is contained in the dispenser recess outline",
    )

    freezer_closed_aabb = ctx.part_world_aabb(freezer)
    with ctx.pose({freezer_hinge: 1.1}):
        freezer_open_aabb = ctx.part_world_aabb(freezer)
    ctx.check(
        "freezer door swings outward about its left vertical hinge",
        freezer_closed_aabb is not None
        and freezer_open_aabb is not None
        and freezer_open_aabb[0][1] < freezer_closed_aabb[0][1] - 0.20,
        details=f"closed={freezer_closed_aabb}, open={freezer_open_aabb}",
    )

    fresh_closed_aabb = ctx.part_world_aabb(fresh)
    with ctx.pose({fresh_hinge: 1.1}):
        fresh_open_aabb = ctx.part_world_aabb(fresh)
    ctx.check(
        "fresh door swings outward about its right vertical hinge",
        fresh_closed_aabb is not None
        and fresh_open_aabb is not None
        and fresh_open_aabb[0][1] < fresh_closed_aabb[0][1] - 0.20,
        details=f"closed={fresh_closed_aabb}, open={fresh_open_aabb}",
    )

    paddle_closed_aabb = ctx.part_world_aabb(paddle)
    with ctx.pose({paddle_pivot: 0.35}):
        paddle_pressed_aabb = ctx.part_world_aabb(paddle)
    ctx.check(
        "dispenser paddle rocks inward on a local horizontal pivot",
        paddle_closed_aabb is not None
        and paddle_pressed_aabb is not None
        and paddle_pressed_aabb[1][1] > paddle_closed_aabb[1][1] + 0.010,
        details=f"closed={paddle_closed_aabb}, pressed={paddle_pressed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

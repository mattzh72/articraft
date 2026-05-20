from __future__ import annotations

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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _filleted_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    x, y, z = size
    return cq.Workplane("XY").box(x, y, z).edges("|Z").fillet(radius)


def _base_shape() -> cq.Workplane:
    body = _filleted_box((0.285, 0.225, 0.096), 0.020).translate((0.0, 0.0, 0.048))
    collar = cq.Workplane("XY").circle(0.086).extrude(0.021).translate((0.0, 0.0, 0.096))
    return body.union(collar)


def _bowl_shell_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.095).extrude(0.145)
    inner = cq.Workplane("XY").circle(0.0915).extrude(0.142).translate((0.0, 0.0, 0.003))
    return outer.cut(inner)


def _lid_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").circle(0.101).extrude(0.012)
    feed_hole = (
        cq.Workplane("XY")
        .box(0.066, 0.050, 0.016)
        .translate((0.035, -0.010, 0.006))
    )
    return plate.cut(feed_hole)


def _lid_skirt_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.101).extrude(0.024).translate((0.0, 0.0, -0.020))
    inner = cq.Workplane("XY").circle(0.096).extrude(0.024).translate((0.0, 0.0, -0.020))
    return outer.cut(inner)


def _feed_tube_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(0.074, 0.058, 0.117)
        .translate((0.035, -0.010, 0.0565))
    )
    inner = (
        cq.Workplane("XY")
        .box(0.066, 0.050, 0.123)
        .translate((0.035, -0.010, 0.0565))
    )
    return outer.cut(inner)


def _pusher_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.062, 0.046, 0.205).translate((0.0, 0.0, 0.0075))
    return body.edges("|Z").fillet(0.006)


def _pusher_cap_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.080, 0.064, 0.020)
        .translate((0.0, 0.0, 0.112))
        .edges("|Z")
        .fillet(0.006)
    )


def _basket_ring_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.068).extrude(0.010).translate((0.0, 0.0, 0.072))
    inner = cq.Workplane("XY").circle(0.061).extrude(0.010).translate((0.0, 0.0, 0.072))
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_food_processor")

    stainless = model.material("stainless", rgba=(0.78, 0.79, 0.80, 1.0))
    black = model.material("black", rgba=(0.12, 0.12, 0.13, 1.0))
    smoke = model.material("smoke", rgba=(0.78, 0.80, 0.83, 0.65))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "food_processor_base"),
        material=stainless,
        name="base_shell",
    )
    for x_pos in (-0.090, 0.090):
        for y_pos in (-0.070, 0.070):
            base.visual(
                Cylinder(radius=0.016, length=0.008),
                origin=Origin(xyz=(x_pos, y_pos, 0.004)),
                material=black,
                name=f"foot_{int((x_pos > 0) * 2 + (y_pos > 0))}",
            )
    base.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=black,
        name="drive_collar",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.285, 0.225, 0.117)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0585)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_bowl_shell_shape(), "food_processor_bowl_shell"),
        material=smoke,
        name="bowl_shell",
    )
    for index, x_pos in enumerate((-0.104, 0.104)):
        bowl.visual(
            Box((0.022, 0.030, 0.026)),
            origin=Origin(xyz=(x_pos, 0.0, 0.112)),
            material=smoke,
            name=f"shoulder_{index}",
        )
    bowl.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=black,
        name="center_post",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=0.145),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.117)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_plate_shape(), "food_processor_lid_plate"),
        material=smoke,
        name="lid_plate",
    )
    lid.visual(
        mesh_from_cadquery(_lid_skirt_shape(), "food_processor_lid_skirt"),
        material=smoke,
        name="lid_skirt",
    )
    lid.visual(
        mesh_from_cadquery(_feed_tube_shape(), "food_processor_feed_tube"),
        material=smoke,
        name="feed_tube",
    )
    for index, x_pos in enumerate((-0.103, 0.103)):
        lid.visual(
            Box((0.020, 0.012, 0.020)),
            origin=Origin(xyz=(x_pos, 0.0, 0.014)),
            material=smoke,
            name=f"latch_ear_{index}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((0.205, 0.160, 0.135)),
        mass=0.5,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    model.articulation(
        "bowl_to_lid",
        ArticulationType.FIXED,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
    )

    pusher = model.part("pusher")
    pusher.visual(
        mesh_from_cadquery(_pusher_body_shape(), "food_processor_pusher_body"),
        material=smoke,
        name="pusher_body",
    )
    pusher.visual(
        mesh_from_cadquery(_pusher_cap_shape(), "food_processor_pusher_cap"),
        material=black,
        name="pusher_cap",
    )
    pusher.visual(
        Box((0.004, 0.010, 0.110)),
        origin=Origin(xyz=(0.031, 0.0, 0.030)),
        material=smoke,
        name="guide_x_pos",
    )
    pusher.visual(
        Box((0.004, 0.010, 0.110)),
        origin=Origin(xyz=(-0.031, 0.0, 0.030)),
        material=smoke,
        name="guide_x_neg",
    )
    pusher.visual(
        Box((0.010, 0.004, 0.110)),
        origin=Origin(xyz=(0.0, 0.023, 0.030)),
        material=smoke,
        name="guide_y_pos",
    )
    pusher.visual(
        Box((0.010, 0.004, 0.110)),
        origin=Origin(xyz=(0.0, -0.023, 0.030)),
        material=smoke,
        name="guide_y_neg",
    )
    pusher.inertial = Inertial.from_geometry(
        Box((0.080, 0.064, 0.225)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    model.articulation(
        "pusher_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.035, -0.010, 0.115)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.18, lower=0.0, upper=0.075),
    )

    cutter_basket = model.part("cutter_basket")
    cutter_basket.visual(
        Cylinder(radius=0.020, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=black,
        name="hub",
    )
    cutter_basket.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=stainless,
        name="shaft",
    )
    cutter_basket.visual(
        mesh_from_cadquery(_basket_ring_shape(), "food_processor_basket_ring"),
        material=stainless,
        name="basket_ring",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        cutter_basket.visual(
            Box((0.082, 0.010, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.031), rpy=(0.0, 0.18, angle)),
            material=stainless,
            name=f"blade_{index}",
        )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        cutter_basket.visual(
            Box((0.128, 0.010, 0.054)),
            origin=Origin(
                xyz=(0.0, 0.0, 0.055),
                rpy=(0.0, 0.0, angle),
            ),
            material=stainless,
            name=f"basket_rib_{index}",
        )
    cutter_basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.095),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
    )

    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cutter_basket,
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    for index, side in enumerate((-1.0, 1.0)):
        arm = model.part(f"clamp_arm_{index}")
        arm.visual(
            Cylinder(radius=0.008, length=0.030),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name="pivot_barrel",
        )
        arm.visual(
            Box((0.018, 0.016, 0.028)),
            origin=Origin(xyz=(0.0, 0.0, 0.020)),
            material=black,
            name="base_block",
        )
        arm.visual(
            Box((0.016, 0.014, 0.126)),
            origin=Origin(xyz=(0.006 * side, 0.0, 0.079)),
            material=black,
            name="arm_web",
        )
        arm.visual(
            Box((0.018, 0.022, 0.052)),
            origin=Origin(xyz=(0.012 * side, 0.0, 0.090)),
            material=black,
            name="grip",
        )
        arm.visual(
            Box((0.028, 0.016, 0.016)),
            origin=Origin(xyz=(-0.012 * side, 0.0, 0.146)),
            material=black,
            name="hook",
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.038, 0.030, 0.160)),
            mass=0.18,
            origin=Origin(xyz=(0.0, 0.0, 0.080)),
        )

        model.articulation(
            f"clamp_arm_{index}_swing",
            ArticulationType.REVOLUTE,
            parent=bowl,
            child=arm,
            origin=Origin(xyz=(0.123 * side, 0.0, 0.113)),
            axis=(0.0, side, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=0.0, upper=0.95),
        )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        Cylinder(radius=0.024, length=0.019),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="dial_body",
    )
    timer_dial.visual(
        Box((0.004, 0.002, 0.014)),
        origin=Origin(xyz=(0.0, -0.010, 0.011)),
        material=stainless,
        name="pointer",
    )
    timer_dial.inertial = Inertial.from_geometry(Box((0.048, 0.020, 0.048)), mass=0.08)

    model.articulation(
        "timer_dial_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=timer_dial,
        origin=Origin(xyz=(0.0, -0.122, 0.050)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    start_button = model.part("start_button")
    start_button.visual(
        Box((0.020, 0.011, 0.014)),
        material=black,
        name="button_cap",
    )
    start_button.inertial = Inertial.from_geometry(Box((0.020, 0.011, 0.014)), mass=0.04)

    model.articulation(
        "start_button_press",
        ArticulationType.PRISMATIC,
        parent=base,
        child=start_button,
        origin=Origin(xyz=(0.060, -0.118, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.004),
    )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    cutter_basket = object_model.get_part("cutter_basket")
    clamp_arm_0 = object_model.get_part("clamp_arm_0")
    clamp_arm_1 = object_model.get_part("clamp_arm_1")
    timer_dial = object_model.get_part("timer_dial")
    start_button = object_model.get_part("start_button")
    pusher_slide = object_model.get_articulation("pusher_slide")
    clamp_swing_0 = object_model.get_articulation("clamp_arm_0_swing")
    clamp_swing_1 = object_model.get_articulation("clamp_arm_1_swing")
    timer_spin = object_model.get_articulation("timer_dial_spin")
    button_press = object_model.get_articulation("start_button_press")

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        name="bowl seats on the motor collar",
    )
    ctx.expect_gap(
        lid,
        bowl,
        axis="z",
        positive_elem="lid_plate",
        negative_elem="bowl_shell",
        max_gap=0.0015,
        max_penetration=0.0,
        name="lid plate sits on the bowl rim",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_body",
        outer_elem="feed_tube",
        margin=0.001,
        name="pusher stays centered in the feed tube at rest",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_body",
        elem_b="feed_tube",
        min_overlap=0.090,
        name="pusher body remains guided by the feed tube at rest",
    )

    if pusher_slide.motion_limits is not None and pusher_slide.motion_limits.upper is not None:
        with ctx.pose({pusher_slide: pusher_slide.motion_limits.upper}):
            ctx.expect_within(
                pusher,
                lid,
                axes="xy",
                inner_elem="pusher_body",
                outer_elem="feed_tube",
                margin=0.001,
                name="pusher stays centered in the feed tube when pushed",
            )
            ctx.expect_overlap(
                pusher,
                lid,
                axes="z",
                elem_a="pusher_body",
                elem_b="feed_tube",
                min_overlap=0.020,
                name="pusher retains insertion at full stroke",
            )

    ctx.expect_within(
        cutter_basket,
        bowl,
        axes="xy",
        margin=0.025,
        name="cutter basket stays inside the bowl footprint",
    )
    ctx.expect_gap(
        lid,
        cutter_basket,
        axis="z",
        positive_elem="lid_plate",
        negative_elem="basket_ring",
        min_gap=0.030,
        name="cutter basket clears the lid underside",
    )

    left_hook_rest = aabb_center(ctx.part_element_world_aabb(clamp_arm_0, elem="hook"))
    right_hook_rest = aabb_center(ctx.part_element_world_aabb(clamp_arm_1, elem="hook"))
    if (
        clamp_swing_0.motion_limits is not None
        and clamp_swing_0.motion_limits.upper is not None
        and clamp_swing_1.motion_limits is not None
        and clamp_swing_1.motion_limits.upper is not None
    ):
        with ctx.pose(
            {
                clamp_swing_0: clamp_swing_0.motion_limits.upper,
                clamp_swing_1: clamp_swing_1.motion_limits.upper,
            }
        ):
            left_hook_open = aabb_center(ctx.part_element_world_aabb(clamp_arm_0, elem="hook"))
            right_hook_open = aabb_center(ctx.part_element_world_aabb(clamp_arm_1, elem="hook"))

        ctx.check(
            "clamp arms swing outward from the lid shoulders",
            left_hook_rest is not None
            and right_hook_rest is not None
            and left_hook_open is not None
            and right_hook_open is not None
            and left_hook_open[0] < left_hook_rest[0] - 0.025
            and right_hook_open[0] > right_hook_rest[0] + 0.025,
            details=(
                f"left_rest={left_hook_rest}, left_open={left_hook_open}, "
                f"right_rest={right_hook_rest}, right_open={right_hook_open}"
            ),
        )

    pointer_rest = aabb_center(ctx.part_element_world_aabb(timer_dial, elem="pointer"))
    with ctx.pose({timer_spin: 1.2}):
        pointer_rotated = aabb_center(ctx.part_element_world_aabb(timer_dial, elem="pointer"))
    ctx.check(
        "timer dial rotates its pointer around the front axis",
        pointer_rest is not None
        and pointer_rotated is not None
        and abs(pointer_rotated[0] - pointer_rest[0]) > 0.004
        and abs(pointer_rotated[2] - pointer_rest[2]) > 0.004,
        details=f"rest={pointer_rest}, rotated={pointer_rotated}",
    )

    button_rest = ctx.part_world_position(start_button)
    if button_press.motion_limits is not None and button_press.motion_limits.upper is not None:
        with ctx.pose({button_press: button_press.motion_limits.upper}):
            button_pressed = ctx.part_world_position(start_button)
        ctx.check(
            "start button presses inward",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[1] > button_rest[1] + 0.002,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    return ctx.report()


object_model = build_object_model()

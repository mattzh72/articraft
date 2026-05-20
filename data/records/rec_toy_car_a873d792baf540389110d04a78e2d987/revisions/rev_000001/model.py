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


WHEEL_RADIUS = 0.012
WHEEL_WIDTH = 0.009
TRACK_HALF = 0.030
FRONT_WHEEL_X = 0.040
REAR_WHEEL_X = -0.038

STEER_PIVOT_X = 0.037
STEER_PIVOT_Z = 0.014

DOOR_HINGE_X = 0.010
DOOR_HINGE_Y = 0.0265
DOOR_HINGE_Z = 0.026
DOOR_LENGTH = 0.032
DOOR_THICKNESS = 0.0025
DOOR_HEIGHT = 0.018
DOOR_OPEN_LIMIT = 1.12
STEER_LIMIT = 0.42


def _add_wheel(part, *, tire_material, wheel_material, hub_material) -> None:
    spin_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=spin_origin,
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.72, length=WHEEL_WIDTH * 0.72),
        origin=spin_origin,
        material=wheel_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.34, length=WHEEL_WIDTH * 1.05),
        origin=spin_origin,
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.18, length=WHEEL_WIDTH * 1.18),
        origin=spin_origin,
        material=hub_material,
        name="axle_cap",
    )


def _add_door(part, *, body_material, glass_material, trim_material) -> None:
    part.visual(
        Cylinder(radius=0.0012, length=0.014),
        material=trim_material,
        name="hinge_barrel",
    )
    part.visual(
        Box((DOOR_LENGTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(-DOOR_LENGTH / 2.0, 0.0, 0.0)),
        material=body_material,
        name="door_panel",
    )
    part.visual(
        Box((DOOR_LENGTH * 0.46, DOOR_THICKNESS * 1.02, DOOR_HEIGHT * 0.28)),
        origin=Origin(xyz=(-DOOR_LENGTH * 0.42, 0.0, DOOR_HEIGHT * 0.42)),
        material=glass_material,
        name="window_band",
    )
    part.visual(
        Box((0.0050, 0.0013, 0.0016)),
        origin=Origin(xyz=(-DOOR_LENGTH * 0.62, 0.0, 0.001)),
        material=trim_material,
        name="handle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_coupe")

    body_red = model.material("body_red", rgba=(0.78, 0.12, 0.12, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.20, 0.24, 0.29, 1.0))
    chassis_gray = model.material("chassis_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    tire_black = model.material("tire_black", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_silver = model.material("wheel_silver", rgba=(0.78, 0.79, 0.81, 1.0))
    hub_steel = model.material("hub_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    seam_black = model.material("seam_black", rgba=(0.09, 0.09, 0.10, 1.0))
    lamp_silver = model.material("lamp_silver", rgba=(0.83, 0.84, 0.86, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.110, 0.040, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, 0.014)),
        material=chassis_gray,
        name="undertray",
    )
    body.visual(
        Box((0.078, 0.004, 0.009)),
        origin=Origin(xyz=(-0.004, 0.023, 0.018)),
        material=body_red,
        name="left_rocker",
    )
    body.visual(
        Box((0.078, 0.004, 0.009)),
        origin=Origin(xyz=(-0.004, -0.023, 0.018)),
        material=body_red,
        name="right_rocker",
    )
    body.visual(
        Box((0.046, 0.042, 0.014)),
        origin=Origin(xyz=(0.029, 0.000, 0.023)),
        material=body_red,
        name="hood",
    )
    body.visual(
        Box((0.026, 0.036, 0.009)),
        origin=Origin(xyz=(0.038, 0.000, 0.029), rpy=(0.0, 0.28, 0.0)),
        material=body_red,
        name="hood_slope",
    )
    body.visual(
        Box((0.010, 0.034, 0.009)),
        origin=Origin(xyz=(0.053, 0.000, 0.022), rpy=(0.0, -0.40, 0.0)),
        material=body_red,
        name="nose_cap",
    )
    body.visual(
        Box((0.028, 0.008, 0.016)),
        origin=Origin(xyz=(0.028, 0.021, 0.021)),
        material=body_red,
        name="left_front_fender",
    )
    body.visual(
        Box((0.028, 0.008, 0.016)),
        origin=Origin(xyz=(0.028, -0.021, 0.021)),
        material=body_red,
        name="right_front_fender",
    )
    body.visual(
        Box((0.018, 0.032, 0.008)),
        origin=Origin(xyz=(0.016, 0.000, 0.021)),
        material=body_red,
        name="cowl",
    )
    body.visual(
        Box((0.004, 0.004, 0.024)),
        origin=Origin(xyz=(0.010, 0.0205, 0.028)),
        material=body_red,
        name="left_a_pillar",
    )
    body.visual(
        Box((0.004, 0.004, 0.024)),
        origin=Origin(xyz=(0.010, -0.0205, 0.028)),
        material=body_red,
        name="right_a_pillar",
    )
    body.visual(
        Box((0.004, 0.005, 0.004)),
        origin=Origin(xyz=(DOOR_HINGE_X, 0.0245, 0.018)),
        material=body_red,
        name="left_lower_hinge_bracket",
    )
    body.visual(
        Box((0.004, 0.005, 0.004)),
        origin=Origin(xyz=(DOOR_HINGE_X, 0.0245, 0.034)),
        material=body_red,
        name="left_upper_hinge_bracket",
    )
    body.visual(
        Box((0.004, 0.005, 0.004)),
        origin=Origin(xyz=(DOOR_HINGE_X, -0.0245, 0.018)),
        material=body_red,
        name="right_lower_hinge_bracket",
    )
    body.visual(
        Box((0.004, 0.005, 0.004)),
        origin=Origin(xyz=(DOOR_HINGE_X, -0.0245, 0.034)),
        material=body_red,
        name="right_upper_hinge_bracket",
    )
    body.visual(
        Cylinder(radius=0.0012, length=0.004),
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, 0.017)),
        material=hub_steel,
        name="left_lower_hinge",
    )
    body.visual(
        Cylinder(radius=0.0012, length=0.004),
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, 0.035)),
        material=hub_steel,
        name="left_upper_hinge",
    )
    body.visual(
        Cylinder(radius=0.0012, length=0.004),
        origin=Origin(xyz=(DOOR_HINGE_X, -DOOR_HINGE_Y, 0.017)),
        material=hub_steel,
        name="right_lower_hinge",
    )
    body.visual(
        Cylinder(radius=0.0012, length=0.004),
        origin=Origin(xyz=(DOOR_HINGE_X, -DOOR_HINGE_Y, 0.035)),
        material=hub_steel,
        name="right_upper_hinge",
    )
    body.visual(
        Box((0.042, 0.038, 0.008)),
        origin=Origin(xyz=(-0.007, 0.000, 0.040)),
        material=body_red,
        name="roof",
    )
    body.visual(
        Box((0.010, 0.004, 0.020)),
        origin=Origin(xyz=(-0.024, 0.0200, 0.029)),
        material=body_red,
        name="left_c_pillar",
    )
    body.visual(
        Box((0.010, 0.004, 0.020)),
        origin=Origin(xyz=(-0.024, -0.0200, 0.029)),
        material=body_red,
        name="right_c_pillar",
    )
    body.visual(
        Box((0.028, 0.008, 0.016)),
        origin=Origin(xyz=(-0.039, 0.021, 0.022)),
        material=body_red,
        name="left_rear_quarter",
    )
    body.visual(
        Box((0.028, 0.008, 0.016)),
        origin=Origin(xyz=(-0.039, -0.021, 0.022)),
        material=body_red,
        name="right_rear_quarter",
    )
    body.visual(
        Box((0.035, 0.040, 0.013)),
        origin=Origin(xyz=(-0.043, 0.000, 0.0275)),
        material=body_red,
        name="trunk",
    )
    body.visual(
        Box((0.021, 0.034, 0.008)),
        origin=Origin(xyz=(-0.026, 0.000, 0.037), rpy=(0.0, -0.36, 0.0)),
        material=body_red,
        name="rear_deck_slope",
    )
    body.visual(
        Box((0.010, 0.032, 0.008)),
        origin=Origin(xyz=(-0.058, 0.000, 0.023), rpy=(0.0, 0.32, 0.0)),
        material=body_red,
        name="tail_taper",
    )
    body.visual(
        Box((0.0025, 0.036, 0.018)),
        origin=Origin(xyz=(0.011, 0.000, 0.032), rpy=(0.0, 0.88, 0.0)),
        material=dark_glass,
        name="windshield",
    )
    body.visual(
        Box((0.0025, 0.032, 0.015)),
        origin=Origin(xyz=(-0.024, 0.000, 0.033), rpy=(0.0, -0.80, 0.0)),
        material=dark_glass,
        name="rear_window",
    )
    body.visual(
        Box((0.014, 0.012, 0.006)),
        origin=Origin(xyz=(STEER_PIVOT_X - 0.002, 0.000, 0.018)),
        material=chassis_gray,
        name="steer_bracket",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(STEER_PIVOT_X, 0.000, 0.016)),
        material=chassis_gray,
        name="steer_boss",
    )
    body.visual(
        Box((0.012, 0.006, 0.010)),
        origin=Origin(xyz=(REAR_WHEEL_X, 0.022, 0.015)),
        material=chassis_gray,
        name="left_rear_carrier",
    )
    body.visual(
        Box((0.012, 0.006, 0.010)),
        origin=Origin(xyz=(REAR_WHEEL_X, -0.022, 0.015)),
        material=chassis_gray,
        name="right_rear_carrier",
    )
    body.visual(
        Box((0.010, 0.040, 0.008)),
        origin=Origin(xyz=(0.058, 0.000, 0.019)),
        material=lamp_silver,
        name="front_bumper",
    )
    body.visual(
        Box((0.008, 0.040, 0.007)),
        origin=Origin(xyz=(-0.061, 0.000, 0.018)),
        material=lamp_silver,
        name="rear_bumper",
    )
    body.visual(
        Box((0.001, 0.026, 0.0015)),
        origin=Origin(xyz=(-0.031, 0.000, 0.0335)),
        material=seam_black,
        name="trunk_seam_front",
    )
    body.visual(
        Box((0.020, 0.001, 0.0015)),
        origin=Origin(xyz=(-0.043, 0.013, 0.0335)),
        material=seam_black,
        name="trunk_seam_left",
    )
    body.visual(
        Box((0.020, 0.001, 0.0015)),
        origin=Origin(xyz=(-0.043, -0.013, 0.0335)),
        material=seam_black,
        name="trunk_seam_right",
    )
    body.visual(
        Box((0.001, 0.022, 0.0015)),
        origin=Origin(xyz=(-0.055, 0.000, 0.0320)),
        material=seam_black,
        name="trunk_seam_rear",
    )

    left_door = model.part("left_door")
    _add_door(left_door, body_material=body_red, glass_material=dark_glass, trim_material=hub_steel)

    right_door = model.part("right_door")
    _add_door(right_door, body_material=body_red, glass_material=dark_glass, trim_material=hub_steel)

    front_axle = model.part("front_axle")
    front_axle.visual(
        Cylinder(radius=0.0045, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, -0.002)),
        material=hub_steel,
        name="pivot_plate",
    )
    front_axle.visual(
        Box((0.009, 0.007, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, -0.004)),
        material=hub_steel,
        name="pivot_web",
    )
    front_axle.visual(
        Box((0.014, 0.048, 0.005)),
        origin=Origin(xyz=(0.000, 0.000, -0.0075)),
        material=chassis_gray,
        name="cross_beam",
    )
    front_axle.visual(
        Box((0.030, 0.004, 0.003)),
        origin=Origin(xyz=(-0.002, 0.000, -0.0035)),
        material=hub_steel,
        name="tie_bar",
    )
    front_axle.visual(
        Box((0.008, 0.006, 0.010)),
        origin=Origin(xyz=(0.000, 0.022, -0.006)),
        material=chassis_gray,
        name="left_carrier",
    )
    front_axle.visual(
        Box((0.008, 0.006, 0.010)),
        origin=Origin(xyz=(0.000, -0.022, -0.006)),
        material=chassis_gray,
        name="right_carrier",
    )

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel(front_left_wheel, tire_material=tire_black, wheel_material=wheel_silver, hub_material=hub_steel)

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel(front_right_wheel, tire_material=tire_black, wheel_material=wheel_silver, hub_material=hub_steel)

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel(rear_left_wheel, tire_material=tire_black, wheel_material=wheel_silver, hub_material=hub_steel)

    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel(rear_right_wheel, tire_material=tire_black, wheel_material=wheel_silver, hub_material=hub_steel)

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=0.0, upper=DOOR_OPEN_LIMIT),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(DOOR_HINGE_X, -DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=0.0, upper=DOOR_OPEN_LIMIT),
    )
    model.articulation(
        "front_steer",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_axle,
        origin=Origin(xyz=(STEER_PIVOT_X, 0.000, STEER_PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=-STEER_LIMIT, upper=STEER_LIMIT),
    )
    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=front_axle,
        child=front_left_wheel,
        origin=Origin(xyz=(0.000, TRACK_HALF, -0.002)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=20.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=front_axle,
        child=front_right_wheel,
        origin=Origin(xyz=(0.000, -TRACK_HALF, -0.002)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=20.0),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_left_wheel,
        origin=Origin(xyz=(REAR_WHEEL_X, TRACK_HALF, WHEEL_RADIUS)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=20.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_right_wheel,
        origin=Origin(xyz=(REAR_WHEEL_X, -TRACK_HALF, WHEEL_RADIUS)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    front_axle = object_model.get_part("front_axle")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    steer_joint = object_model.get_articulation("front_steer")

    ctx.expect_gap(
        left_door,
        body,
        axis="y",
        min_gap=0.0001,
        max_gap=0.0020,
        positive_elem="door_panel",
        negative_elem="left_rocker",
        name="left door sits just outside the body side",
    )
    ctx.expect_gap(
        body,
        right_door,
        axis="y",
        min_gap=0.0001,
        max_gap=0.0020,
        positive_elem="right_rocker",
        negative_elem="door_panel",
        name="right door sits just outside the body side",
    )
    ctx.expect_gap(
        body,
        front_axle,
        axis="z",
        min_gap=0.0,
        max_gap=0.0010,
        positive_elem="steer_boss",
        negative_elem="pivot_plate",
        name="front axle pivot stays tucked under the nose",
    )
    ctx.expect_gap(
        front_left_wheel,
        front_axle,
        axis="y",
        min_gap=0.0002,
        max_gap=0.0020,
        positive_elem="hub",
        negative_elem="left_carrier",
        name="left front wheel clears the steer carrier",
    )
    ctx.expect_gap(
        front_axle,
        front_right_wheel,
        axis="y",
        min_gap=0.0002,
        max_gap=0.0020,
        positive_elem="right_carrier",
        negative_elem="hub",
        name="right front wheel clears the steer carrier",
    )
    ctx.expect_gap(
        rear_left_wheel,
        body,
        axis="y",
        min_gap=0.0002,
        max_gap=0.0020,
        positive_elem="hub",
        negative_elem="left_rear_carrier",
        name="left rear wheel clears the body carrier",
    )
    ctx.expect_gap(
        body,
        rear_right_wheel,
        axis="y",
        min_gap=0.0002,
        max_gap=0.0020,
        positive_elem="right_rear_carrier",
        negative_elem="hub",
        name="right rear wheel clears the body carrier",
    )

    left_closed = ctx.part_element_world_aabb(left_door, elem="door_panel")
    right_closed = ctx.part_element_world_aabb(right_door, elem="door_panel")
    with ctx.pose({left_hinge: DOOR_OPEN_LIMIT, right_hinge: DOOR_OPEN_LIMIT}):
        left_open = ctx.part_element_world_aabb(left_door, elem="door_panel")
        right_open = ctx.part_element_world_aabb(right_door, elem="door_panel")
    ctx.check(
        "doors swing outward from the A-pillars",
        left_closed is not None
        and right_closed is not None
        and left_open is not None
        and right_open is not None
        and left_open[1][1] > left_closed[1][1] + 0.010
        and right_open[0][1] < right_closed[0][1] - 0.010,
        details=f"left_closed={left_closed}, left_open={left_open}, right_closed={right_closed}, right_open={right_open}",
    )

    rest_left = ctx.part_world_position(front_left_wheel)
    rest_right = ctx.part_world_position(front_right_wheel)
    with ctx.pose({steer_joint: STEER_LIMIT}):
        steer_left = ctx.part_world_position(front_left_wheel)
        steer_right = ctx.part_world_position(front_right_wheel)
    ctx.check(
        "front axle steers as a paired assembly",
        rest_left is not None
        and rest_right is not None
        and steer_left is not None
        and steer_right is not None
        and steer_left[0] < rest_left[0] - 0.006
        and steer_right[0] > rest_right[0] + 0.006,
        details=f"rest_left={rest_left}, steer_left={steer_left}, rest_right={rest_right}, steer_right={steer_right}",
    )

    return ctx.report()


object_model = build_object_model()

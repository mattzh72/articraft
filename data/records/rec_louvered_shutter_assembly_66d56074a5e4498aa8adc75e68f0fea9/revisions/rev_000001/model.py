from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


DOOR_WIDTH = 0.78
DOOR_HEIGHT = 2.00
DOOR_THICKNESS = 0.035
HINGE_AXIS_Y = 0.0
DOOR_Y = -0.025
HINGE_STILE_W = 0.075
FREE_STILE_W = 0.075
RAIL_H = 0.085
BOTTOM_Z = 0.04
KICK_TOP_Z = 0.66
TOP_Z = BOTTOM_Z + DOOR_HEIGHT
LOUVER_COUNT = 12
LOUVER_PITCH = 0.095
LOUVER_FIRST_Z = 0.80


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="louvered_closet_shutter_door")

    painted_wood = model.material("painted_wood", rgba=(0.88, 0.86, 0.78, 1.0))
    inset_panel = model.material("inset_panel", rgba=(0.78, 0.76, 0.68, 1.0))
    hinge_metal = model.material("dark_hinge_metal", rgba=(0.09, 0.075, 0.055, 1.0))
    shadow = model.material("shadow_gap", rgba=(0.10, 0.09, 0.075, 1.0))

    jamb = model.part("jamb")
    jamb.visual(
        Box((0.063, 0.085, 2.12)),
        origin=Origin(xyz=(-0.0435, -0.0275, 1.06)),
        material=painted_wood,
        name="hinge_jamb",
    )
    jamb.visual(
        Box((0.075, 0.085, 2.12)),
        origin=Origin(xyz=(DOOR_WIDTH + 0.0775, -0.0275, 1.06)),
        material=painted_wood,
        name="strike_jamb",
    )
    jamb.visual(
        Box((DOOR_WIDTH + 0.15, 0.085, 0.070)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 + 0.02, -0.0275, 2.105)),
        material=painted_wood,
        name="head_jamb",
    )
    jamb.visual(
        Box((DOOR_WIDTH + 0.15, 0.085, 0.030)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 + 0.02, -0.0275, 0.015)),
        material=painted_wood,
        name="threshold",
    )

    door = model.part("door")
    door.visual(
        Box((HINGE_STILE_W, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.012 + HINGE_STILE_W / 2.0, DOOR_Y, BOTTOM_Z + DOOR_HEIGHT / 2.0)),
        material=painted_wood,
        name="hinge_stile",
    )
    door.visual(
        Box((FREE_STILE_W, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH - FREE_STILE_W / 2.0, DOOR_Y, BOTTOM_Z + DOOR_HEIGHT / 2.0)),
        material=painted_wood,
        name="free_stile",
    )
    door.visual(
        Box((DOOR_WIDTH - 0.012, DOOR_THICKNESS, RAIL_H)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 + 0.006, DOOR_Y, TOP_Z - RAIL_H / 2.0)),
        material=painted_wood,
        name="top_rail",
    )
    door.visual(
        Box((DOOR_WIDTH - 0.012, DOOR_THICKNESS, RAIL_H)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 + 0.006, DOOR_Y, BOTTOM_Z + RAIL_H / 2.0)),
        material=painted_wood,
        name="bottom_rail",
    )
    door.visual(
        Box((DOOR_WIDTH - 0.012, DOOR_THICKNESS, 0.095)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 + 0.006, DOOR_Y, KICK_TOP_Z)),
        material=painted_wood,
        name="lock_rail",
    )

    # Recessed solid lower kick panel, intentionally fixed into the door frame.
    door.visual(
        Box((DOOR_WIDTH - HINGE_STILE_W - FREE_STILE_W + 0.020, 0.018, 0.505)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 + 0.006, DOOR_Y - 0.002, 0.365)),
        material=inset_panel,
        name="kick_panel",
    )
    door.visual(
        Box((DOOR_WIDTH - HINGE_STILE_W - FREE_STILE_W - 0.045, 0.006, 0.040)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 + 0.006, DOOR_Y + 0.017, 0.585)),
        material=painted_wood,
        name="kick_top_trim",
    )
    door.visual(
        Box((DOOR_WIDTH - HINGE_STILE_W - FREE_STILE_W - 0.045, 0.006, 0.040)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 + 0.006, DOOR_Y + 0.017, 0.145)),
        material=painted_wood,
        name="kick_bottom_trim",
    )
    door.visual(
        Box((0.035, 0.006, 0.430)),
        origin=Origin(xyz=(0.110, DOOR_Y + 0.017, 0.365)),
        material=painted_wood,
        name="kick_side_trim_0",
    )
    door.visual(
        Box((0.035, 0.006, 0.430)),
        origin=Origin(xyz=(DOOR_WIDTH - 0.100, DOOR_Y + 0.017, 0.365)),
        material=painted_wood,
        name="kick_side_trim_1",
    )

    hinge_zs = (0.38, 1.02, 1.66)
    for i, z in enumerate(hinge_zs):
        # Jamb-side top and bottom knuckles are separated from the door-side
        # middle knuckle, matching the alternating barrel construction.
        for suffix, dz in (("lower", -0.050), ("upper", 0.050)):
            jamb.visual(
                Box((0.045, 0.021, 0.040)),
                origin=Origin(xyz=(-0.025, 0.007, z + dz)),
                material=hinge_metal,
                name=f"hinge_leaf_{i}_{suffix}",
            )
            jamb.visual(
                Cylinder(radius=0.010, length=0.040),
                origin=Origin(xyz=(0.0, HINGE_AXIS_Y, z + dz)),
                material=hinge_metal,
                name=f"hinge_knuckle_{i}_{suffix}",
            )
        door.visual(
            Box((0.045, 0.006, 0.060)),
            origin=Origin(xyz=(0.030, -0.0045, z)),
            material=hinge_metal,
            name=f"hinge_leaf_{i}",
        )
        door.visual(
            Cylinder(radius=0.010, length=0.060),
            origin=Origin(xyz=(0.0, HINGE_AXIS_Y, z)),
            material=hinge_metal,
            name=f"hinge_knuckle_{i}",
        )

    model.articulation(
        "jamb_to_door",
        ArticulationType.REVOLUTE,
        parent=jamb,
        child=door,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=0.0, upper=1.75),
    )

    louver_limits = MotionLimits(effort=1.0, velocity=2.0, lower=-0.65, upper=0.65)
    driver_name = "door_to_louver_0"
    for i in range(LOUVER_COUNT):
        louver = model.part(f"louver_{i}")
        louver.visual(
            Box((0.600, 0.068, 0.012)),
            origin=Origin(rpy=(-0.42, 0.0, 0.0)),
            material=painted_wood,
            name="slat",
        )
        louver.visual(
            Box((0.590, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, 0.031, -0.014), rpy=(-0.42, 0.0, 0.0)),
            material=painted_wood,
            name="front_edge",
        )
        louver.visual(
            Cylinder(radius=0.005, length=0.618),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_metal,
            name="pivot_pin",
        )
        joint_name = f"door_to_louver_{i}"
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=door,
            child=louver,
            origin=Origin(xyz=(DOOR_WIDTH / 2.0 + 0.006, DOOR_Y - 0.002, LOUVER_FIRST_Z + i * LOUVER_PITCH)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=louver_limits,
            mimic=None if i == 0 else Mimic(joint=driver_name, multiplier=1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    jamb = object_model.get_part("jamb")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("jamb_to_door")
    louver_0 = object_model.get_part("louver_0")
    louver_11 = object_model.get_part("louver_11")
    louver_driver = object_model.get_articulation("door_to_louver_0")

    ctx.expect_gap(
        door,
        jamb,
        axis="x",
        positive_elem="hinge_stile",
        negative_elem="hinge_jamb",
        min_gap=0.010,
        max_gap=0.026,
        name="closed door has hinge-side clearance at the jamb",
    )
    ctx.expect_gap(
        jamb,
        door,
        axis="x",
        positive_elem="strike_jamb",
        negative_elem="free_stile",
        min_gap=0.035,
        max_gap=0.045,
        name="closed door clears the strike jamb",
    )
    ctx.expect_overlap(
        door,
        jamb,
        axes="z",
        elem_a="hinge_stile",
        elem_b="hinge_jamb",
        min_overlap=1.8,
        name="door height is carried by the side jamb",
    )
    ctx.expect_within(
        louver_0,
        door,
        axes="x",
        inner_elem="pivot_pin",
        outer_elem="top_rail",
        margin=0.005,
        name="louver pivot pin spans within the upper frame width",
    )
    ctx.expect_gap(
        louver_0,
        door,
        axis="z",
        positive_elem="slat",
        negative_elem="lock_rail",
        min_gap=0.070,
        name="lowest louver stays above the fixed lower kick panel",
    )
    ctx.expect_gap(
        door,
        louver_11,
        axis="z",
        positive_elem="top_rail",
        negative_elem="slat",
        min_gap=0.050,
        name="top louver clears the top rail",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="free_stile")
    with ctx.pose({door_hinge: 1.0}):
        open_aabb = ctx.part_element_world_aabb(door, elem="free_stile")
    closed_y = None if closed_aabb is None else (closed_aabb[0][1] + closed_aabb[1][1]) / 2.0
    open_y = None if open_aabb is None else (open_aabb[0][1] + open_aabb[1][1]) / 2.0
    ctx.check(
        "door swings outward on the vertical side hinge",
        closed_y is not None and open_y is not None and open_y > closed_y + 0.40,
        details=f"closed_y={closed_y}, open_y={open_y}",
    )

    with ctx.pose({louver_driver: louver_driver.motion_limits.upper}):
        raised_aabb = ctx.part_element_world_aabb(louver_0, elem="front_edge")
    with ctx.pose({louver_driver: louver_driver.motion_limits.lower}):
        lowered_aabb = ctx.part_element_world_aabb(louver_0, elem="front_edge")
    raised_z = None if raised_aabb is None else (raised_aabb[0][2] + raised_aabb[1][2]) / 2.0
    lowered_z = None if lowered_aabb is None else (lowered_aabb[0][2] + lowered_aabb[1][2]) / 2.0
    ctx.check(
        "louvers rotate about their long horizontal pivots",
        raised_z is not None
        and lowered_z is not None
        and abs(raised_z - lowered_z) > 0.025
        and tuple(louver_driver.axis) == (1.0, 0.0, 0.0),
        details=f"raised_z={raised_z}, lowered_z={lowered_z}, axis={louver_driver.axis}",
    )

    return ctx.report()


object_model = build_object_model()

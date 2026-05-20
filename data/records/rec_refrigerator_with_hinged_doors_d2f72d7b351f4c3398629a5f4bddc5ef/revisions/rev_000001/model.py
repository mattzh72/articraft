from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="back_bar_beverage_refrigerator")

    black = model.material("black_powder_coat", rgba=(0.015, 0.016, 0.018, 1.0))
    dark = model.material("dark_interior", rgba=(0.035, 0.038, 0.042, 1.0))
    rubber = model.material("black_rubber", rgba=(0.008, 0.008, 0.008, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.45, 0.72, 0.92, 0.34))
    shelf_glass = model.material("pale_shelf_glass", rgba=(0.65, 0.82, 0.94, 0.30))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.70, 0.66, 1.0))
    white = model.material("white_print", rgba=(0.92, 0.92, 0.86, 1.0))
    red = model.material("red_can", rgba=(0.8, 0.05, 0.04, 1.0))
    blue = model.material("blue_can", rgba=(0.05, 0.18, 0.85, 1.0))
    green = model.material("green_can", rgba=(0.03, 0.48, 0.18, 1.0))
    amber = model.material("amber_bottle", rgba=(0.45, 0.24, 0.07, 0.92))

    cabinet = model.part("cabinet")

    # Short, wide under-counter proportions: black steel case, dark interior, and a fixed top control trim.
    cabinet.visual(Box((1.16, 0.56, 0.080)), origin=Origin(xyz=(0.0, 0.0, 0.040)), material=black, name="base_pan")
    cabinet.visual(Box((1.16, 0.56, 0.060)), origin=Origin(xyz=(0.0, 0.0, 0.820)), material=black, name="top_cap")
    cabinet.visual(Box((0.040, 0.56, 0.720)), origin=Origin(xyz=(-0.580, 0.0, 0.430)), material=black, name="side_wall_0")
    cabinet.visual(Box((0.040, 0.56, 0.720)), origin=Origin(xyz=(0.580, 0.0, 0.430)), material=black, name="side_wall_1")
    cabinet.visual(Box((1.16, 0.035, 0.720)), origin=Origin(xyz=(0.0, 0.2625, 0.430)), material=dark, name="back_wall")
    cabinet.visual(Box((1.16, 0.055, 0.135)), origin=Origin(xyz=(0.0, -0.3025, 0.7675)), material=black, name="top_trim_panel")
    cabinet.visual(Box((1.16, 0.055, 0.110)), origin=Origin(xyz=(0.0, -0.3025, 0.055)), material=black, name="toe_rail")
    cabinet.visual(Box((0.030, 0.045, 0.590)), origin=Origin(xyz=(0.0, -0.2875, 0.405)), material=black, name="center_mullion")

    # Interior shelves and a few visible beverages make the appliance read as a beverage refrigerator through the glass.
    for idx, z in enumerate((0.315, 0.500)):
        cabinet.visual(Box((1.12, 0.445, 0.012)), origin=Origin(xyz=(0.0, 0.010, z)), material=shelf_glass, name=f"shelf_{idx}")

    can_positions = (
        (-0.39, -0.045, 0.315 + 0.006 + 0.0575, red, "can_0"),
        (-0.30, -0.045, 0.315 + 0.006 + 0.0575, blue, "can_1"),
        (-0.21, -0.045, 0.315 + 0.006 + 0.0575, green, "can_2"),
        (0.21, -0.045, 0.500 + 0.006 + 0.0575, blue, "can_3"),
        (0.30, -0.045, 0.500 + 0.006 + 0.0575, red, "can_4"),
        (0.39, -0.045, 0.500 + 0.006 + 0.0575, green, "can_5"),
    )
    for x, y, z, mat, name in can_positions:
        cabinet.visual(Cylinder(radius=0.031, length=0.115), origin=Origin(xyz=(x, y, z)), material=mat, name=name)

    for x, z, name in ((-0.055, 0.383, "bottle_0"), (0.055, 0.383, "bottle_1")):
        cabinet.visual(Cylinder(radius=0.027, length=0.135), origin=Origin(xyz=(x, 0.060, z)), material=amber, name=f"{name}_body")
        cabinet.visual(Cylinder(radius=0.012, length=0.045), origin=Origin(xyz=(x, 0.060, z + 0.090)), material=amber, name=f"{name}_neck")

    # Exposed side hinge barrels are fixed to the cabinet; the doors rotate about their pin centerlines.
    for side, x in enumerate((-0.552, 0.552)):
        plate_x = -0.570 if x < 0.0 else 0.570
        cabinet.visual(Cylinder(radius=0.010, length=0.570), origin=Origin(xyz=(x, -0.315, 0.405)), material=brushed_steel, name=f"hinge_pin_{side}")
        for plate_idx, z in enumerate((0.175, 0.405, 0.635)):
            cabinet.visual(
                Box((0.050, 0.040, 0.055)),
                origin=Origin(xyz=(plate_x, -0.295, z)),
                material=brushed_steel,
                name=f"hinge_plate_{side}_{plate_idx}",
            )

    for x in (-0.46, 0.46):
        for y in (-0.20, 0.20):
            cabinet.visual(Cylinder(radius=0.030, length=0.040), origin=Origin(xyz=(x, y, -0.020)), material=rubber, name=f"foot_{x}_{y}")

    # Fixed thermostat tick marks printed on the upper trim around the knob.
    for idx, (x, z, angle) in enumerate(((-0.070, 0.765, 0.0), (-0.045, 0.800, 0.55), (0.0, 0.815, 1.57), (0.045, 0.800, -0.55), (0.070, 0.765, 0.0))):
        cabinet.visual(
            Box((0.004, 0.003, 0.030)),
            origin=Origin(xyz=(x, -0.3315, z), rpy=(0.0, angle, 0.0)),
            material=white,
            name=f"thermostat_tick_{idx}",
        )

    left_door = model.part("door_0")
    right_door = model.part("door_1")

    def add_door(part, sign: float) -> None:
        # Door frame local origin is the vertical hinge axis.  sign=+1 extends from the left hinge toward center;
        # sign=-1 extends from the right hinge toward center.
        sx = sign
        part.visual(Box((0.500, 0.040, 0.045)), origin=Origin(xyz=(sx * 0.260, 0.0, 0.2575)), material=black, name="top_rail")
        part.visual(Box((0.500, 0.040, 0.045)), origin=Origin(xyz=(sx * 0.260, 0.0, -0.2575)), material=black, name="bottom_rail")
        part.visual(Box((0.035, 0.040, 0.560)), origin=Origin(xyz=(sx * 0.035, 0.0, 0.0)), material=black, name="hinge_rail")
        part.visual(Box((0.035, 0.040, 0.560)), origin=Origin(xyz=(sx * 0.500, 0.0, 0.0)), material=black, name="inner_rail")
        part.visual(Box((0.430, 0.012, 0.475)), origin=Origin(xyz=(sx * 0.265, 0.004, 0.0)), material=smoked_glass, name="glass_pane")

        # Front pull handle with two standoffs mechanically tied back to the frame.
        handle_x = sx * 0.500
        part.visual(Cylinder(radius=0.009, length=0.350), origin=Origin(xyz=(handle_x, -0.058, 0.0)), material=brushed_steel, name="handle_bar")
        for idx, z in enumerate((-0.125, 0.125)):
            part.visual(
                Cylinder(radius=0.006, length=0.050),
                origin=Origin(xyz=(handle_x, -0.039, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=brushed_steel,
                name=f"handle_standoff_{idx}",
            )

    add_door(left_door, 1.0)
    add_door(right_door, -1.0)

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.058,
            0.027,
            body_style="skirted",
            top_diameter=0.047,
            skirt=KnobSkirt(0.066, 0.006, flare=0.05, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=22, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "thermostat_knob",
    )
    knob = model.part("thermostat_knob")
    knob.visual(knob_mesh, origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=black, name="knob_body")

    model.articulation(
        "door_hinge_0",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(xyz=(-0.552, -0.315, 0.405)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "door_hinge_1",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(xyz=(0.552, -0.315, 0.405)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "thermostat_axis",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(0.0, -0.330, 0.765)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=4.0, lower=-2.35, upper=2.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("door_0")
    right_door = object_model.get_part("door_1")
    knob = object_model.get_part("thermostat_knob")
    left_hinge = object_model.get_articulation("door_hinge_0")
    right_hinge = object_model.get_articulation("door_hinge_1")
    thermostat = object_model.get_articulation("thermostat_axis")

    ctx.check("two vertical door hinges", left_hinge.axis == (0.0, 0.0, -1.0) and right_hinge.axis == (0.0, 0.0, 1.0))
    ctx.check("thermostat axis is front to back", thermostat.axis == (0.0, -1.0, 0.0))
    ctx.check(
        "thermostat has rotary travel",
        thermostat.motion_limits is not None
        and thermostat.motion_limits.lower is not None
        and thermostat.motion_limits.upper is not None
        and thermostat.motion_limits.upper - thermostat.motion_limits.lower > 4.0,
    )

    ctx.expect_gap(
        cabinet,
        left_door,
        axis="z",
        positive_elem="top_trim_panel",
        negative_elem="top_rail",
        min_gap=0.010,
        max_gap=0.030,
        name="left door clears fixed upper trim",
    )
    ctx.expect_gap(
        cabinet,
        right_door,
        axis="z",
        positive_elem="top_trim_panel",
        negative_elem="top_rail",
        min_gap=0.010,
        max_gap=0.030,
        name="right door clears fixed upper trim",
    )
    ctx.expect_gap(
        cabinet,
        knob,
        axis="y",
        positive_elem="top_trim_panel",
        negative_elem="knob_body",
        max_gap=0.004,
        max_penetration=0.001,
        name="thermostat knob seats on front trim",
    )

    def _coord(vec, idx):
        try:
            return vec[idx]
        except TypeError:
            return (vec.x, vec.y, vec.z)[idx]

    closed_left = ctx.part_element_world_aabb(left_door, elem="inner_rail")
    closed_right = ctx.part_element_world_aabb(right_door, elem="inner_rail")
    with ctx.pose({left_hinge: 1.20, right_hinge: 1.20}):
        open_left = ctx.part_element_world_aabb(left_door, elem="inner_rail")
        open_right = ctx.part_element_world_aabb(right_door, elem="inner_rail")

    ctx.check(
        "left door opens outward from side hinge",
        closed_left is not None
        and open_left is not None
        and _coord(open_left[0], 1) < _coord(closed_left[0], 1) - 0.15,
    )
    ctx.check(
        "right door opens outward from side hinge",
        closed_right is not None
        and open_right is not None
        and _coord(open_right[0], 1) < _coord(closed_right[0], 1) - 0.15,
    )

    return ctx.report()


object_model = build_object_model()

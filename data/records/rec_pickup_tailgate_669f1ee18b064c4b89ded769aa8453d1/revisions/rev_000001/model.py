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
    model = ArticulatedObject(name="pickup_tailgate_pass_door")

    body_blue = model.material("body_blue", rgba=(0.06, 0.18, 0.34, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.015, 0.017, 0.020, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.48, 0.50, 0.52, 1.0))
    latch_black = model.material("latch_black", rgba=(0.02, 0.02, 0.018, 1.0))
    weather_strip = model.material("weather_strip", rgba=(0.01, 0.01, 0.012, 1.0))

    bed = model.part("bed_sides")
    bed.visual(
        Box((0.68, 1.80, 0.06)),
        origin=Origin(xyz=(-0.24, 0.0, -0.07)),
        material=dark_trim,
        name="bed_floor",
    )
    for y, name in [(-0.86, "side_wall_0"), (0.86, "side_wall_1")]:
        bed.visual(
            Box((0.64, 0.08, 0.69)),
            origin=Origin(xyz=(-0.22, y, 0.305)),
            material=body_blue,
            name=name,
        )
    bed.visual(
        Box((0.10, 1.80, 0.10)),
        origin=Origin(xyz=(-0.02, 0.0, -0.095)),
        material=dark_trim,
        name="rear_sill",
    )
    for y, length, name in [
        (-0.66, 0.22, "bed_hinge_0"),
        (0.0, 0.30, "bed_hinge_1"),
        (0.66, 0.22, "bed_hinge_2"),
    ]:
        bed.visual(
            Cylinder(radius=0.025, length=length),
            origin=Origin(xyz=(0.035, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=hinge_steel,
            name=name,
        )
        bed.visual(
            Box((0.050, length, 0.030)),
            origin=Origin(xyz=(0.035, y, -0.030)),
            material=hinge_steel,
            name=f"{name}_saddle",
        )

    tailgate = model.part("tailgate")
    # The tailgate frame is built as a continuous stamped panel with a central
    # pass-through opening rather than a solid block hiding the small door.
    tailgate.visual(
        Box((0.075, 1.62, 0.115)),
        origin=Origin(xyz=(0.0375, 0.0, 0.0825)),
        material=body_blue,
        name="lower_rail",
    )
    tailgate.visual(
        Box((0.075, 1.62, 0.09)),
        origin=Origin(xyz=(0.0375, 0.0, 0.515)),
        material=body_blue,
        name="upper_rail",
    )
    for y, name in [(-0.53, "side_panel_0"), (0.53, "side_panel_1")]:
        tailgate.visual(
            Box((0.075, 0.56, 0.33)),
            origin=Origin(xyz=(0.0375, y, 0.305)),
            material=body_blue,
            name=name,
        )
    # Pressed strengthening ribs and an inner weather seal around the access cutout.
    for z, zname in [(0.20, "lower"), (0.41, "upper")]:
        for y, yname in [(-0.515, "side_0"), (0.515, "side_1")]:
            tailgate.visual(
                Box((0.012, 0.520, 0.018)),
                origin=Origin(xyz=(-0.006, y, z)),
                material=dark_trim,
                name=f"rib_{zname}_{yname}",
            )
    tailgate.visual(
        Box((0.010, 0.465, 0.015)),
        origin=Origin(xyz=(-0.004, 0.0, 0.1325)),
        material=weather_strip,
        name="seal_bottom",
    )
    tailgate.visual(
        Box((0.010, 0.465, 0.015)),
        origin=Origin(xyz=(-0.004, 0.0, 0.4675)),
        material=weather_strip,
        name="seal_top",
    )
    for y, name in [(-0.2325, "seal_side_0"), (0.2325, "seal_side_1")]:
        tailgate.visual(
            Box((0.010, 0.015, 0.335)),
            origin=Origin(xyz=(-0.004, y, 0.300)),
            material=weather_strip,
            name=name,
        )
    tailgate.visual(
        Box((0.026, 0.030, 0.310)),
        origin=Origin(xyz=(0.025, -0.260, 0.300)),
        material=hinge_steel,
        name="access_hinge_leaf",
    )
    tailgate.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(xyz=(0.025, -0.235, 0.300)),
        material=hinge_steel,
        name="access_hinge_knuckle",
    )
    tailgate.visual(
        Box((0.014, 0.060, 0.060)),
        origin=Origin(xyz=(0.003, 0.285, 0.320)),
        material=hinge_steel,
        name="striker_plate",
    )
    for y, length, name in [
        (-0.39, 0.30, "gate_hinge_0"),
        (0.39, 0.30, "gate_hinge_1"),
    ]:
        tailgate.visual(
            Cylinder(radius=0.025, length=length),
            origin=Origin(xyz=(0.0375, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=hinge_steel,
            name=name,
        )

    pass_door = model.part("pass_door")
    pass_door.visual(
        Box((0.036, 0.420, 0.310)),
        origin=Origin(xyz=(0.022, 0.240, 0.0)),
        material=body_blue,
        name="door_panel",
    )
    pass_door.visual(
        Box((0.010, 0.360, 0.018)),
        origin=Origin(xyz=(0.045, 0.240, -0.092)),
        material=dark_trim,
        name="door_rib_lower",
    )
    pass_door.visual(
        Box((0.010, 0.360, 0.018)),
        origin=Origin(xyz=(0.045, 0.240, 0.092)),
        material=dark_trim,
        name="door_rib_upper",
    )
    pass_door.visual(
        Box((0.025, 0.025, 0.310)),
        origin=Origin(xyz=(0.022, 0.030, 0.0)),
        material=hinge_steel,
        name="door_hinge_leaf",
    )
    for z, length, name in [(-0.100, 0.100, "door_hinge_0"), (0.100, 0.100, "door_hinge_1")]:
        pass_door.visual(
            Cylinder(radius=0.010, length=length),
            origin=Origin(xyz=(0.025, 0.010, z)),
            material=hinge_steel,
            name=name,
        )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.025, length=0.012),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=latch_black,
        name="pivot_button",
    )
    latch.visual(
        Box((0.012, 0.035, 0.120)),
        origin=Origin(xyz=(-0.011, 0.0, 0.040)),
        material=latch_black,
        name="turn_tab",
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.2, lower=0.0, upper=1.45),
    )
    model.articulation(
        "tailgate_to_pass_door",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=pass_door,
        origin=Origin(xyz=(0.0, -0.240, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=0.0, upper=1.25),
    )
    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=pass_door,
        child=latch,
        origin=Origin(xyz=(0.004, 0.390, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-0.25, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bed = object_model.get_part("bed_sides")
    tailgate = object_model.get_part("tailgate")
    pass_door = object_model.get_part("pass_door")
    latch = object_model.get_part("latch")
    gate_joint = object_model.get_articulation("bed_to_tailgate")
    door_joint = object_model.get_articulation("tailgate_to_pass_door")
    latch_joint = object_model.get_articulation("door_to_latch")

    ctx.expect_within(
        pass_door,
        tailgate,
        axes="yz",
        inner_elem="door_panel",
        margin=0.010,
        name="pass door sits inside tailgate width and height",
    )
    ctx.expect_gap(
        pass_door,
        tailgate,
        axis="y",
        positive_elem="door_hinge_leaf",
        negative_elem="access_hinge_leaf",
        min_gap=0.010,
        max_gap=0.025,
        name="pass door hinge leaf is clipped close to fixed leaf",
    )
    ctx.expect_overlap(
        pass_door,
        tailgate,
        axes="z",
        elem_a="door_hinge_leaf",
        elem_b="access_hinge_leaf",
        min_overlap=0.25,
        name="pass door hinge leaves share the vertical hinge span",
    )

    closed_gate_aabb = ctx.part_world_aabb(tailgate)
    with ctx.pose({gate_joint: 1.20}):
        open_gate_aabb = ctx.part_world_aabb(tailgate)
    ctx.check(
        "main tailgate rotates downward about lower hinge",
        closed_gate_aabb is not None
        and open_gate_aabb is not None
        and open_gate_aabb[1][2] < closed_gate_aabb[1][2] - 0.15
        and open_gate_aabb[1][0] > closed_gate_aabb[1][0] + 0.30,
        details=f"closed={closed_gate_aabb}, open={open_gate_aabb}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(pass_door, elem="door_panel")
    with ctx.pose({door_joint: 1.00}):
        open_door_aabb = ctx.part_element_world_aabb(pass_door, elem="door_panel")
    ctx.check(
        "pass door swings inward on its vertical side hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][0] < closed_door_aabb[0][0] - 0.10,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    locked_tab_aabb = ctx.part_element_world_aabb(latch, elem="turn_tab")
    with ctx.pose({latch_joint: 1.25}):
        turned_tab_aabb = ctx.part_element_world_aabb(latch, elem="turn_tab")
    ctx.check(
        "small latch rotates on local pivot",
        locked_tab_aabb is not None
        and turned_tab_aabb is not None
        and (turned_tab_aabb[1][1] - turned_tab_aabb[0][1])
        > (locked_tab_aabb[1][1] - locked_tab_aabb[0][1]) + 0.035,
        details=f"locked={locked_tab_aabb}, turned={turned_tab_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

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
    model = ArticulatedObject(name="conventional_oven")

    stainless = model.material("brushed_stainless", rgba=(0.68, 0.70, 0.70, 1.0))
    dark_stainless = model.material("dark_stainless", rgba=(0.28, 0.30, 0.31, 1.0))
    black_enamel = model.material("black_enamel", rgba=(0.035, 0.037, 0.038, 1.0))
    cavity_enamel = model.material("cavity_enamel", rgba=(0.11, 0.115, 0.12, 1.0))
    black_glass = model.material("smoked_glass", rgba=(0.02, 0.025, 0.03, 0.48))
    chrome = model.material("polished_chrome", rgba=(0.86, 0.88, 0.88, 1.0))
    rubber_black = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    blue_glow = model.material("blue_display", rgba=(0.10, 0.55, 1.0, 1.0))
    red_glow = model.material("red_indicator", rgba=(1.0, 0.08, 0.03, 1.0))

    body = model.part("body")

    # Substantial insulated shell: a connected stainless cabinet with a real
    # front opening, top control fascia, and a black-enamel oven liner.
    body.visual(Box((0.04, 0.62, 0.72)), origin=Origin(xyz=(-0.36, 0.0, 0.36)), material=stainless, name="side_panel_0")
    body.visual(Box((0.04, 0.62, 0.72)), origin=Origin(xyz=(0.36, 0.0, 0.36)), material=stainless, name="side_panel_1")
    body.visual(Box((0.76, 0.62, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.70)), material=stainless, name="top_skin")
    body.visual(Box((0.76, 0.62, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.02)), material=stainless, name="bottom_skin")
    body.visual(Box((0.76, 0.04, 0.72)), origin=Origin(xyz=(0.0, 0.29, 0.36)), material=dark_stainless, name="rear_skin")

    body.visual(Box((0.76, 0.035, 0.125)), origin=Origin(xyz=(0.0, -0.3275, 0.6575)), material=stainless, name="control_fascia")
    body.visual(Box((0.76, 0.035, 0.105)), origin=Origin(xyz=(0.0, -0.3275, 0.0525)), material=stainless, name="toe_rail")
    body.visual(Box((0.12, 0.035, 0.50)), origin=Origin(xyz=(-0.32, -0.3275, 0.35)), material=stainless, name="front_jamb_0")
    body.visual(Box((0.12, 0.035, 0.50)), origin=Origin(xyz=(0.32, -0.3275, 0.35)), material=stainless, name="front_jamb_1")
    body.visual(Box((0.58, 0.035, 0.035)), origin=Origin(xyz=(0.0, -0.3275, 0.5925)), material=dark_stainless, name="upper_lip")
    body.visual(Box((0.58, 0.035, 0.035)), origin=Origin(xyz=(0.0, -0.3275, 0.1075)), material=dark_stainless, name="lower_lip")

    body.visual(Box((0.58, 0.52, 0.024)), origin=Origin(xyz=(0.0, -0.005, 0.124)), material=cavity_enamel, name="cavity_floor")
    body.visual(Box((0.58, 0.52, 0.024)), origin=Origin(xyz=(0.0, -0.005, 0.586)), material=cavity_enamel, name="cavity_ceiling")
    body.visual(Box((0.024, 0.52, 0.47)), origin=Origin(xyz=(-0.300, -0.005, 0.355)), material=cavity_enamel, name="cavity_wall_0")
    body.visual(Box((0.024, 0.52, 0.47)), origin=Origin(xyz=(0.300, -0.005, 0.355)), material=cavity_enamel, name="cavity_wall_1")
    body.visual(Box((0.58, 0.024, 0.47)), origin=Origin(xyz=(0.0, 0.258, 0.355)), material=cavity_enamel, name="cavity_back")

    # Embossed convection fan cover, welded to the rear liner.
    body.visual(
        Cylinder(radius=0.105, length=0.012),
        origin=Origin(xyz=(0.0, 0.240, 0.365), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_enamel,
        name="fan_cover",
    )
    for i, angle in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0)):
        body.visual(
            Box((0.170, 0.014, 0.008)),
            origin=Origin(xyz=(0.0, 0.232, 0.365), rpy=(0.0, angle, 0.0)),
            material=chrome,
            name=f"fan_slot_{i}",
        )

    # Display, indicator lights, and fixed panel graphics seated into the fascia.
    body.visual(Box((0.18, 0.004, 0.045)), origin=Origin(xyz=(0.0, -0.346, 0.665)), material=black_enamel, name="clock_display")
    for i, x in enumerate((-0.055, -0.030, 0.010, 0.035)):
        body.visual(Box((0.015, 0.002, 0.006)), origin=Origin(xyz=(x, -0.349, 0.672)), material=blue_glow, name=f"display_segment_{i}")
        body.visual(Box((0.015, 0.002, 0.006)), origin=Origin(xyz=(x, -0.349, 0.658)), material=blue_glow, name=f"display_segment_{i + 4}")
    body.visual(Box((0.010, 0.002, 0.010)), origin=Origin(xyz=(0.072, -0.349, 0.666)), material=red_glow, name="heat_indicator")

    knob_xs = (-0.285, -0.185, 0.185, 0.285)
    for i, x in enumerate(knob_xs):
        body.visual(Box((0.086, 0.004, 0.070)), origin=Origin(xyz=(x, -0.344, 0.665)), material=black_enamel, name=f"knob_recess_{i}")

    body.visual(
        Cylinder(radius=0.006, length=0.660),
        origin=Origin(xyz=(0.0, -0.378, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_pin",
    )
    for i, x in enumerate((-0.115, 0.115, 0.345, -0.345)):
        body.visual(Box((0.050, 0.050, 0.016)), origin=Origin(xyz=(x, -0.367, 0.073)), material=dark_stainless, name=f"hinge_shelf_{i}")
        body.visual(Box((0.050, 0.010, 0.016)), origin=Origin(xyz=(x, -0.345, 0.088)), material=dark_stainless, name=f"hinge_web_{i}")

    # Two supported chrome racks inside the enamel cavity.  The small side tabs
    # tie the rack wires back into the liner so the rods read as installed rails.
    for level, z in enumerate((0.285, 0.435)):
        body.visual(Box((0.012, 0.430, 0.012)), origin=Origin(xyz=(-0.242, -0.020, z)), material=chrome, name=f"rack_{level}_rail_0")
        body.visual(Box((0.012, 0.430, 0.012)), origin=Origin(xyz=(0.242, -0.020, z)), material=chrome, name=f"rack_{level}_rail_1")
        for j, y in enumerate((-0.205, -0.105, -0.005, 0.095, 0.195)):
            body.visual(Box((0.510, 0.008, 0.008)), origin=Origin(xyz=(0.0, y, z)), material=chrome, name=f"rack_{level}_wire_{j}")
        for j, y in enumerate((-0.185, 0.000, 0.185)):
            body.visual(Box((0.070, 0.010, 0.010)), origin=Origin(xyz=(-0.275, y, z)), material=chrome, name=f"rack_{level}_tab_0_{j}")
            body.visual(Box((0.070, 0.010, 0.010)), origin=Origin(xyz=(0.275, y, z)), material=chrome, name=f"rack_{level}_tab_1_{j}")

    door = model.part("door")
    # The door part frame is the bottom hinge line.  At q=0 it is a vertical
    # insulated frame; positive hinge rotation drops the free edge forward/down.
    door.visual(Box((0.72, 0.045, 0.090)), origin=Origin(xyz=(0.0, 0.0, 0.045)), material=stainless, name="bottom_stile")
    door.visual(Box((0.72, 0.045, 0.100)), origin=Origin(xyz=(0.0, 0.0, 0.450)), material=stainless, name="top_stile")
    door.visual(Box((0.085, 0.045, 0.500)), origin=Origin(xyz=(-0.3175, 0.0, 0.250)), material=stainless, name="side_stile_0")
    door.visual(Box((0.085, 0.045, 0.500)), origin=Origin(xyz=(0.3175, 0.0, 0.250)), material=stainless, name="side_stile_1")
    door.visual(Box((0.585, 0.014, 0.340)), origin=Origin(xyz=(0.0, -0.030, 0.255)), material=black_glass, name="glass_panel")
    door.visual(Box((0.620, 0.010, 0.030)), origin=Origin(xyz=(0.0, -0.042, 0.098)), material=rubber_black, name="window_gasket_0")
    door.visual(Box((0.620, 0.010, 0.030)), origin=Origin(xyz=(0.0, -0.042, 0.415)), material=rubber_black, name="window_gasket_1")
    door.visual(Box((0.032, 0.010, 0.330)), origin=Origin(xyz=(-0.296, -0.042, 0.255)), material=rubber_black, name="window_gasket_2")
    door.visual(Box((0.032, 0.010, 0.330)), origin=Origin(xyz=(0.296, -0.042, 0.255)), material=rubber_black, name="window_gasket_3")
    door.visual(
        Cylinder(radius=0.019, length=0.560),
        origin=Origin(xyz=(0.0, -0.076, 0.425), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="handle_bar",
    )
    for i, x in enumerate((-0.235, 0.235)):
        door.visual(
            Cylinder(radius=0.012, length=0.060),
            origin=Origin(xyz=(x, -0.049, 0.425), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"handle_post_{i}",
        )
    for i, x in enumerate((-0.230, 0.0, 0.230)):
        door.visual(
            Cylinder(radius=0.022, length=0.135),
            origin=Origin(xyz=(x, -0.006, -0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_stainless,
            name=f"hinge_barrel_{i}",
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -0.372, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.55),
    )

    for i, x in enumerate(knob_xs):
        knob = model.part(f"knob_{i}")
        knob.visual(
            Cylinder(radius=0.034, length=0.034),
            origin=Origin(xyz=(0.0, -0.017, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_stainless,
            name="knob_cap",
        )
        knob.visual(Box((0.008, 0.004, 0.024)), origin=Origin(xyz=(0.0, -0.035, 0.015)), material=chrome, name="pointer_ridge")
        knob.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(0.0, -0.0025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber_black,
            name="shaft_collar",
        )
        model.articulation(
            f"knob_{i}_turn",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x, -0.346, 0.665)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=3.0, lower=-2.35, upper=2.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")

    for i in range(3):
        ctx.allow_overlap(
            body,
            door,
            elem_a="hinge_pin",
            elem_b=f"hinge_barrel_{i}",
            reason="The continuous hinge pin is intentionally captured inside the door hinge barrel.",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="xyz",
            elem_a="hinge_pin",
            elem_b=f"hinge_barrel_{i}",
            min_overlap=0.004,
            name=f"hinge pin captured in barrel {i}",
        )

    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_jamb_0",
        negative_elem="side_stile_0",
        min_gap=0.002,
        max_gap=0.010,
        name="closed door sits just proud of front face",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.40,
        name="door covers the oven opening",
    )
    ctx.expect_within(
        "door",
        "body",
        axes="x",
        margin=0.03,
        inner_elem="glass_panel",
        outer_elem="cavity_back",
        name="window aligns with interior cavity width",
    )

    closed_top = ctx.part_element_world_aabb(door, elem="top_stile")
    with ctx.pose({hinge: 1.20}):
        opened_top = ctx.part_element_world_aabb(door, elem="top_stile")
    ctx.check(
        "drop down door swings forward and down",
        closed_top is not None
        and opened_top is not None
        and opened_top[1][2] < closed_top[1][2] - 0.10
        and opened_top[0][1] < closed_top[0][1] - 0.20,
        details=f"closed_top={closed_top}, opened_top={opened_top}",
    )

    for i in range(4):
        knob = object_model.get_part(f"knob_{i}")
        joint = object_model.get_articulation(f"knob_{i}_turn")
        ctx.allow_overlap(
            body,
            knob,
            elem_a=f"knob_recess_{i}",
            elem_b="shaft_collar",
            reason="The knob shaft collar is seated slightly into the fascia recess like a retained rotary control.",
        )
        ctx.expect_overlap(
            body,
            knob,
            axes="yz",
            elem_a=f"knob_recess_{i}",
            elem_b="shaft_collar",
            min_overlap=0.0003,
            name=f"knob {i} collar seated in recess",
        )
        ctx.expect_gap(
            body,
            knob,
            axis="y",
            positive_elem=f"knob_recess_{i}",
            negative_elem="knob_cap",
            min_gap=0.0,
            max_gap=0.002,
            name=f"knob {i} is proud of the fascia",
        )
        ctx.check(
            f"knob {i} has realistic rotary travel",
            joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.lower < -2.0
            and joint.motion_limits.upper > 2.0,
            details=f"limits={joint.motion_limits}",
        )

    return ctx.report()


object_model = build_object_model()

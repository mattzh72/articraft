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
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_oven")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_metal = model.material("black_enamel", rgba=(0.025, 0.025, 0.028, 1.0))
    interior = model.material("dark_cavity_liner", rgba=(0.08, 0.085, 0.085, 1.0))
    glass = model.material("smoky_glass", rgba=(0.22, 0.34, 0.42, 0.36))
    chrome = model.material("polished_chrome", rgba=(0.86, 0.86, 0.82, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    basket_metal = model.material("perforated_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    ink = model.material("control_markings", rgba=(0.0, 0.0, 0.0, 1.0))

    body = model.part("body")

    # Countertop scale: about 50 cm wide, 39 cm deep, and 33 cm tall.
    # The oven box is built from real panels so the cooking cavity is visibly hollow.
    body.visual(Box((0.50, 0.39, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0525)), material=stainless, name="bottom_shell")
    body.visual(Box((0.50, 0.39, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.3175)), material=stainless, name="top_shell")
    body.visual(Box((0.018, 0.39, 0.30)), origin=Origin(xyz=(-0.241, 0.0, 0.185)), material=stainless, name="side_wall")
    body.visual(Box((0.018, 0.39, 0.30)), origin=Origin(xyz=(0.241, 0.0, 0.185)), material=stainless, name="control_side_wall")
    body.visual(Box((0.018, 0.39, 0.30)), origin=Origin(xyz=(0.145, 0.0, 0.185)), material=stainless, name="cavity_divider")
    body.visual(Box((0.50, 0.018, 0.30)), origin=Origin(xyz=(0.0, 0.186, 0.185)), material=stainless, name="rear_wall")

    # Front bezel around the oven opening and the raised control face.
    body.visual(Box((0.404, 0.018, 0.030)), origin=Origin(xyz=(-0.048, -0.186, 0.085)), material=dark_metal, name="front_bezel_bottom")
    body.visual(Box((0.404, 0.018, 0.030)), origin=Origin(xyz=(-0.048, -0.186, 0.285)), material=dark_metal, name="front_bezel_top")
    body.visual(Box((0.026, 0.018, 0.230)), origin=Origin(xyz=(-0.237, -0.186, 0.185)), material=dark_metal, name="front_bezel_side")
    body.visual(Box((0.026, 0.018, 0.230)), origin=Origin(xyz=(0.145, -0.186, 0.185)), material=dark_metal, name="front_bezel_control")
    body.visual(Box((0.096, 0.014, 0.265)), origin=Origin(xyz=(0.202, -0.202, 0.185)), material=stainless, name="control_panel")

    # Dark inner liner surfaces make the glass-front cavity read as open and hollow.
    body.visual(Box((0.372, 0.330, 0.006)), origin=Origin(xyz=(-0.048, -0.010, 0.073)), material=interior, name="cavity_floor")
    body.visual(Box((0.372, 0.330, 0.006)), origin=Origin(xyz=(-0.048, -0.010, 0.297)), material=interior, name="cavity_ceiling")
    body.visual(Box((0.006, 0.330, 0.220)), origin=Origin(xyz=(-0.229, -0.010, 0.185)), material=interior, name="cavity_liner_side")
    body.visual(Box((0.006, 0.330, 0.220)), origin=Origin(xyz=(0.136, -0.010, 0.185)), material=interior, name="cavity_liner_divider")
    body.visual(Box((0.372, 0.006, 0.220)), origin=Origin(xyz=(-0.048, 0.158, 0.185)), material=interior, name="cavity_liner_rear")

    # Straight side runners support and retain the sliding air-fry basket.
    body.visual(Box((0.016, 0.310, 0.010)), origin=Origin(xyz=(-0.222, -0.005, 0.104)), material=chrome, name="runner_0")
    body.visual(Box((0.016, 0.310, 0.010)), origin=Origin(xyz=(0.132, -0.005, 0.104)), material=chrome, name="runner_1")

    vent_panel = SlotPatternPanelGeometry(
        (0.260, 0.120),
        0.003,
        slot_size=(0.035, 0.006),
        pitch=(0.050, 0.018),
        frame=0.012,
        corner_radius=0.008,
        stagger=True,
    )
    body.visual(mesh_from_geometry(vent_panel, "top_vent"), origin=Origin(xyz=(-0.055, 0.015, 0.334)), material=dark_metal, name="top_vent")

    # Four rubber feet touch the bottom shell at counter height.
    for i, (x, y) in enumerate(((-0.200, -0.145), (0.200, -0.145), (-0.200, 0.145), (0.200, 0.145))):
        body.visual(Cylinder(radius=0.026, length=0.035), origin=Origin(xyz=(x, y, 0.0175)), material=rubber, name=f"foot_{i}")

    # Simple printed ticks around each rotary control.
    for i, z in enumerate((0.255, 0.185, 0.115)):
        body.visual(Box((0.004, 0.002, 0.020)), origin=Origin(xyz=(0.202, -0.210, z + 0.040)), material=ink, name=f"knob_tick_{i}")
        body.visual(Box((0.020, 0.002, 0.004)), origin=Origin(xyz=(0.162, -0.210, z)), material=ink, name=f"knob_min_{i}")
        body.visual(Box((0.020, 0.002, 0.004)), origin=Origin(xyz=(0.242, -0.210, z)), material=ink, name=f"knob_max_{i}")

    # Soft front stops touch the closed door so the hinged assembly is visibly seated.
    body.visual(Box((0.018, 0.008, 0.026)), origin=Origin(xyz=(-0.218, -0.199, 0.185)), material=rubber, name="door_stop_0")
    body.visual(Box((0.018, 0.008, 0.026)), origin=Origin(xyz=(0.124, -0.199, 0.185)), material=rubber, name="door_stop_1")

    # Fixed hinge leaves and alternating body knuckles under the lower front lip.
    for i, (x, length) in enumerate(((-0.218, 0.040), (-0.052, 0.110), (0.121, 0.040))):
        body.visual(Box((length, 0.036, 0.012)), origin=Origin(xyz=(x, -0.211, 0.075)), material=dark_metal, name=f"hinge_leaf_{i}")
        body.visual(Cylinder(radius=0.006, length=length), origin=Origin(xyz=(x, -0.228, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_metal, name=f"hinge_barrel_{i}")

    door = model.part("door")
    door_width = 0.386
    door_height = 0.225
    door_thickness = 0.018
    # The door part frame is on the lower hinge line. In the closed pose the
    # glass panel rises along local +Z and sits just in front of the front bezel.
    door.visual(Box((door_width, door_thickness, 0.022)), origin=Origin(xyz=(0.0, -0.017, 0.011)), material=dark_metal, name="bottom_rail")
    door.visual(Box((door_width, door_thickness, 0.022)), origin=Origin(xyz=(0.0, -0.017, door_height - 0.011)), material=dark_metal, name="top_rail")
    door.visual(Box((0.022, door_thickness, door_height)), origin=Origin(xyz=(-door_width / 2 + 0.011, -0.017, door_height / 2)), material=dark_metal, name="side_rail_0")
    door.visual(Box((0.022, door_thickness, door_height)), origin=Origin(xyz=(door_width / 2 - 0.011, -0.017, door_height / 2)), material=dark_metal, name="side_rail_1")
    door.visual(Box((0.342, 0.006, 0.181)), origin=Origin(xyz=(0.0, -0.020, 0.1125)), material=glass, name="glass_pane")
    door.visual(Cylinder(radius=0.010, length=0.280), origin=Origin(xyz=(0.0, -0.055, 0.180), rpy=(0.0, math.pi / 2.0, 0.0)), material=chrome, name="pull_handle")
    door.visual(Box((0.018, 0.036, 0.018)), origin=Origin(xyz=(-0.120, -0.035, 0.180)), material=chrome, name="handle_standoff_0")
    door.visual(Box((0.018, 0.036, 0.018)), origin=Origin(xyz=(0.120, -0.035, 0.180)), material=chrome, name="handle_standoff_1")
    door.visual(Cylinder(radius=0.008, length=0.075), origin=Origin(xyz=(-0.105, -0.033, 0.008), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_metal, name="hinge_knuckle_0")
    door.visual(Cylinder(radius=0.008, length=0.075), origin=Origin(xyz=(0.105, -0.033, 0.008), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_metal, name="hinge_knuckle_1")

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.048, -0.195, 0.077)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.75),
    )

    basket = model.part("basket")
    basket_floor = SlotPatternPanelGeometry(
        (0.318, 0.258),
        0.004,
        slot_size=(0.030, 0.006),
        pitch=(0.045, 0.018),
        frame=0.010,
        corner_radius=0.006,
        stagger=True,
    )
    basket.visual(mesh_from_geometry(basket_floor, "basket_floor"), origin=Origin(xyz=(0.0, 0.0, -0.035)), material=basket_metal, name="perforated_floor")
    basket.visual(Box((0.008, 0.260, 0.070)), origin=Origin(xyz=(-0.160, 0.0, 0.000)), material=basket_metal, name="basket_side_0")
    basket.visual(Box((0.008, 0.260, 0.070)), origin=Origin(xyz=(0.160, 0.0, 0.000)), material=basket_metal, name="basket_side_1")
    basket.visual(Box((0.320, 0.008, 0.070)), origin=Origin(xyz=(0.0, 0.130, 0.000)), material=basket_metal, name="basket_back")
    basket.visual(Box((0.320, 0.008, 0.052)), origin=Origin(xyz=(0.0, -0.130, -0.009)), material=basket_metal, name="basket_front")
    basket.visual(Box((0.330, 0.012, 0.010)), origin=Origin(xyz=(0.0, -0.134, 0.039)), material=chrome, name="front_rim")
    basket.visual(Box((0.330, 0.012, 0.010)), origin=Origin(xyz=(0.0, 0.134, 0.039)), material=chrome, name="rear_rim")
    basket.visual(Box((0.012, 0.270, 0.010)), origin=Origin(xyz=(-0.166, 0.0, 0.039)), material=chrome, name="side_rim_0")
    basket.visual(Box((0.012, 0.270, 0.010)), origin=Origin(xyz=(0.166, 0.0, 0.039)), material=chrome, name="side_rim_1")
    basket.visual(Box((0.018, 0.230, 0.010)), origin=Origin(xyz=(-0.172, 0.0, -0.036)), material=chrome, name="slide_rail_0")
    basket.visual(Box((0.018, 0.230, 0.010)), origin=Origin(xyz=(0.172, 0.0, -0.036)), material=chrome, name="slide_rail_1")
    basket.visual(Cylinder(radius=0.009, length=0.180), origin=Origin(xyz=(0.0, -0.158, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)), material=chrome, name="basket_handle")
    basket.visual(Box((0.014, 0.035, 0.014)), origin=Origin(xyz=(-0.080, -0.144, 0.010)), material=chrome, name="basket_handle_mount_0")
    basket.visual(Box((0.014, 0.035, 0.014)), origin=Origin(xyz=(0.080, -0.144, 0.010)), material=chrome, name="basket_handle_mount_1")

    model.articulation(
        "body_to_basket",
        ArticulationType.PRISMATIC,
        parent=body,
        child=basket,
        origin=Origin(xyz=(-0.045, -0.015, 0.150)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.240),
    )

    knob_geometry = KnobGeometry(
        0.046,
        0.028,
        body_style="skirted",
        top_diameter=0.036,
        skirt=KnobSkirt(0.054, 0.005, flare=0.06, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=18, depth=0.0011),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
        center=False,
    )
    knob_mesh = mesh_from_geometry(knob_geometry, "control_knob")
    for index, z in enumerate((0.255, 0.185, 0.115)):
        knob = model.part(f"knob_{index}")
        knob.visual(Cylinder(radius=0.008, length=0.012), origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=chrome, name="shaft")
        knob.visual(knob_mesh, origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_metal, name="dial_cap")
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(0.202, -0.209, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    basket = object_model.get_part("basket")
    door_joint = object_model.get_articulation("body_to_door")
    basket_slide = object_model.get_articulation("body_to_basket")
    knob_joints = [object_model.get_articulation(f"body_to_knob_{i}") for i in range(3)]

    ctx.check(
        "three continuous knob joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in knob_joints),
        details=f"joint types={[j.articulation_type for j in knob_joints]}",
    )
    ctx.check(
        "knob axes are front shafts",
        all(tuple(j.axis) == (0.0, -1.0, 0.0) for j in knob_joints),
        details=f"axes={[j.axis for j in knob_joints]}",
    )

    with ctx.pose({door_joint: 0.0, basket_slide: 0.0}):
        ctx.expect_contact(
            body,
            door,
            elem_a="door_stop_0",
            elem_b="side_rail_0",
            contact_tol=0.001,
            name="closed door seats on left stop",
        )
        ctx.expect_contact(
            body,
            door,
            elem_a="door_stop_1",
            elem_b="side_rail_1",
            contact_tol=0.001,
            name="closed door seats on right stop",
        )
        ctx.expect_gap(
            basket,
            body,
            axis="z",
            positive_elem="slide_rail_0",
            negative_elem="runner_0",
            max_gap=0.001,
            max_penetration=0.00001,
            name="basket rests on left runner",
        )
        ctx.expect_gap(
            basket,
            body,
            axis="z",
            positive_elem="slide_rail_1",
            negative_elem="runner_1",
            max_gap=0.001,
            max_penetration=0.00001,
            name="basket rests on right runner",
        )
        ctx.expect_overlap(
            basket,
            body,
            axes="y",
            elem_a="slide_rail_0",
            elem_b="runner_0",
            min_overlap=0.20,
            name="closed basket has retained runner engagement",
        )

    closed_top = ctx.part_element_world_aabb(door, elem="top_rail")
    with ctx.pose({door_joint: 1.55}):
        open_top = ctx.part_element_world_aabb(door, elem="top_rail")
    ctx.check(
        "door rotates downward and outward",
        closed_top is not None
        and open_top is not None
        and open_top[1][2] < closed_top[0][2] - 0.10
        and open_top[0][1] < closed_top[0][1] - 0.10,
        details=f"closed_top={closed_top}, open_top={open_top}",
    )

    rest_pos = ctx.part_world_position(basket)
    with ctx.pose({basket_slide: 0.240}):
        extended_pos = ctx.part_world_position(basket)
        ctx.expect_overlap(
            basket,
            body,
            axes="y",
            elem_a="slide_rail_0",
            elem_b="runner_0",
            min_overlap=0.015,
            name="extended basket remains captured on runner",
        )
        ctx.expect_gap(
            basket,
            body,
            axis="z",
            positive_elem="slide_rail_0",
            negative_elem="runner_0",
            max_gap=0.001,
            max_penetration=0.00001,
            name="extended basket remains supported vertically",
        )
    ctx.check(
        "basket slides straight out of the cavity",
        rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({door_joint: 1.55, basket_slide: 0.240}):
        ctx.expect_gap(
            basket,
            door,
            axis="z",
            min_gap=0.004,
            name="open door clears the pulled basket below",
        )

    return ctx.report()


object_model = build_object_model()

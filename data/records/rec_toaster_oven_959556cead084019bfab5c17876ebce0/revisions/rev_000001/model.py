from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_double_door_toaster_oven")

    enamel = model.material("warm_enamel", rgba=(0.86, 0.76, 0.56, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.82, 0.78, 1.0))
    dark = model.material("dark_cavity", rgba=(0.025, 0.022, 0.020, 1.0))
    glass = model.material("smoky_glass", rgba=(0.08, 0.12, 0.13, 0.42))
    black = model.material("black_rubber", rgba=(0.01, 0.009, 0.008, 1.0))
    label = model.material("black_label", rgba=(0.02, 0.018, 0.014, 1.0))

    body = model.part("body")

    # Countertop-appliance proportions: wide, shallow, and about 30 cm tall
    # including the feet.  The front is open behind the two French-style doors.
    body.visual(Box((0.62, 0.34, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0425)), material=enamel, name="bottom_panel")
    body.visual(Box((0.62, 0.34, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.2875)), material=enamel, name="top_panel")
    body.visual(Box((0.035, 0.34, 0.245)), origin=Origin(xyz=(-0.2925, 0.0, 0.165)), material=enamel, name="side_panel_0")
    body.visual(Box((0.035, 0.34, 0.245)), origin=Origin(xyz=(0.2925, 0.0, 0.165)), material=enamel, name="side_panel_1")
    body.visual(Box((0.62, 0.035, 0.245)), origin=Origin(xyz=(0.0, 0.1525, 0.165)), material=enamel, name="back_panel")

    # Chrome front rails and side hinge stiles give the retro toaster-oven face
    # a bright outline while leaving the oven cavity visibly open.
    body.visual(Box((0.62, 0.022, 0.035)), origin=Origin(xyz=(0.0, -0.181, 0.077)), material=chrome, name="front_lower_rail")
    body.visual(Box((0.62, 0.022, 0.040)), origin=Origin(xyz=(0.0, -0.181, 0.248)), material=chrome, name="front_upper_rail")
    body.visual(Box((0.035, 0.022, 0.175)), origin=Origin(xyz=(-0.2925, -0.181, 0.161)), material=chrome, name="front_side_rail_0")
    body.visual(Box((0.035, 0.022, 0.175)), origin=Origin(xyz=(0.2925, -0.181, 0.161)), material=chrome, name="front_side_rail_1")
    body.visual(Box((0.084, 0.025, 0.205)), origin=Origin(xyz=(0.0, -0.184, 0.166)), material=enamel, name="control_mullion")

    # Dark oven lining and a simple wire rack inside the cavity.
    body.visual(Box((0.52, 0.006, 0.160)), origin=Origin(xyz=(0.0, 0.132, 0.160)), material=dark, name="inner_back")
    body.visual(Box((0.52, 0.260, 0.006)), origin=Origin(xyz=(0.0, -0.020, 0.063)), material=dark, name="inner_floor")
    for i, y in enumerate((-0.10, -0.05, 0.0, 0.05, 0.10)):
        body.visual(
            Cylinder(radius=0.0025, length=0.55),
            origin=Origin(xyz=(0.0, y, 0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name=f"rack_cross_{i}",
        )
    for i, x in enumerate((-0.23, 0.23)):
        body.visual(
            Cylinder(radius=0.003, length=0.22),
            origin=Origin(xyz=(x, 0.0, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"rack_side_{i}",
        )

    for i, x in enumerate((-0.24, 0.24)):
        for j, y in enumerate((-0.12, 0.12)):
            body.visual(
                Cylinder(radius=0.028, length=0.025),
                origin=Origin(xyz=(x, y, 0.0125)),
                material=black,
                name=f"foot_{i}_{j}",
            )

    hinge_x = 0.283
    hinge_y = -0.192
    hinge_z = 0.165
    door_w = 0.232
    door_h = 0.196
    door_t = 0.018
    rail = 0.025

    def add_door(name: str, side: int):
        door = model.part(name)
        s = float(side)
        # The child frame is on the vertical hinge line.  Geometry extends
        # toward the center of the oven: +X for door_0, -X for door_1.
        center_x = s * door_w / 2.0
        hinge_rail_x = s * rail / 2.0
        inner_rail_x = s * (door_w - rail / 2.0)
        top_bottom_x = s * door_w / 2.0
        handle_x = s * (door_w - 0.020)

        door.visual(Box((rail, door_t, door_h)), origin=Origin(xyz=(hinge_rail_x, -door_t / 2.0, 0.0)), material=chrome, name="hinge_stile")
        door.visual(Cylinder(radius=0.009, length=door_h), origin=Origin(xyz=(0.0, -door_t / 2.0, 0.0)), material=chrome, name="hinge_barrel")
        door.visual(Box((rail, door_t, door_h)), origin=Origin(xyz=(inner_rail_x, -door_t / 2.0, 0.0)), material=chrome, name="inner_stile")
        door.visual(Box((door_w, door_t, rail)), origin=Origin(xyz=(top_bottom_x, -door_t / 2.0, door_h / 2.0 - rail / 2.0)), material=chrome, name="top_rail")
        door.visual(Box((door_w, door_t, rail)), origin=Origin(xyz=(top_bottom_x, -door_t / 2.0, -door_h / 2.0 + rail / 2.0)), material=chrome, name="bottom_rail")
        door.visual(
            Box((door_w - 2.0 * rail + 0.008, 0.006, door_h - 2.0 * rail + 0.008)),
            origin=Origin(xyz=(center_x, -0.013, 0.0)),
            material=glass,
            name="window_glass",
        )
        # Simple vertical pull handle mounted on the inner stile.
        door.visual(Box((0.014, 0.014, 0.130)), origin=Origin(xyz=(handle_x, -0.042, 0.0)), material=chrome, name="pull_bar")
        door.visual(Box((0.018, 0.026, 0.015)), origin=Origin(xyz=(handle_x, -0.023, 0.045)), material=chrome, name="pull_post_0")
        door.visual(Box((0.018, 0.026, 0.015)), origin=Origin(xyz=(handle_x, -0.023, -0.045)), material=chrome, name="pull_post_1")
        return door

    door_0 = add_door("door_0", side=1)
    door_1 = add_door("door_1", side=-1)

    model.articulation(
        "body_to_door_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door_0,
        origin=Origin(xyz=(-hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=0.0, upper=1.55),
    )
    model.articulation(
        "body_to_door_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door_1,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=0.0, upper=1.55),
    )

    knob_shape = KnobGeometry(
        0.050,
        0.027,
        body_style="skirted",
        top_diameter=0.038,
        edge_radius=0.001,
        skirt=KnobSkirt(0.058, 0.006, flare=0.06, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=24, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
        bore=KnobBore(style="round", diameter=0.012),
        center=False,
    )

    for i, z in enumerate((0.223, 0.166, 0.109)):
        # Fixed chrome shaft on the central front mullion.
        body.visual(
            Cylinder(radius=0.006, length=0.019),
            origin=Origin(xyz=(0.0, -0.1975, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"knob_shaft_{i}",
        )
        body.visual(Box((0.004, 0.003, 0.018)), origin=Origin(xyz=(0.030, -0.198, z + 0.018)), material=label, name=f"knob_tick_{i}")

        knob = model.part(f"knob_{i}")
        knob.visual(
            mesh_from_geometry(knob_shape, f"chrome_knob_{i}"),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name="knob_cap",
        )
        model.articulation(
            f"body_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(0.0, -0.207, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    door_joint_0 = object_model.get_articulation("body_to_door_0")
    door_joint_1 = object_model.get_articulation("body_to_door_1")
    knobs = [object_model.get_part(f"knob_{i}") for i in range(3)]
    knob_joints = [object_model.get_articulation(f"body_to_knob_{i}") for i in range(3)]

    ctx.check(
        "three continuous rotary knobs",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in knob_joints),
        details=str([j.articulation_type for j in knob_joints]),
    )
    knob_positions = [ctx.part_world_position(knob) for knob in knobs]
    ctx.check(
        "knobs form a vertical stack on the front",
        all(pos is not None for pos in knob_positions)
        and max(abs(pos[0]) for pos in knob_positions) < 0.002
        and max(abs(pos[1] + 0.207) for pos in knob_positions) < 0.002
        and knob_positions[0][2] > knob_positions[1][2] > knob_positions[2][2],
        details=str(knob_positions),
    )

    with ctx.pose({door_joint_0: 0.0, door_joint_1: 0.0}):
        ctx.expect_gap(
            body,
            door_0,
            axis="y",
            positive_elem="front_side_rail_0",
            negative_elem="hinge_stile",
            max_gap=0.001,
            max_penetration=0.0,
            name="door_0 hinge stile sits on front side rail",
        )
        ctx.expect_gap(
            body,
            door_1,
            axis="y",
            positive_elem="front_side_rail_1",
            negative_elem="hinge_stile",
            max_gap=0.001,
            max_penetration=0.0,
            name="door_1 hinge stile sits on front side rail",
        )

    for i, knob in enumerate(knobs):
        ctx.expect_overlap(
            knob,
            body,
            axes="xz",
            min_overlap=0.008,
            elem_a="knob_cap",
            elem_b=f"knob_shaft_{i}",
            name=f"knob_{i} is centered on its shaft",
        )
        ctx.expect_gap(
            body,
            knob,
            axis="y",
            positive_elem=f"knob_shaft_{i}",
            negative_elem="knob_cap",
            max_gap=0.002,
            max_penetration=0.001,
            name=f"knob_{i} seats against its front shaft",
        )

    closed_0 = ctx.part_element_world_aabb(door_0, elem="inner_stile")
    closed_1 = ctx.part_element_world_aabb(door_1, elem="inner_stile")
    with ctx.pose({door_joint_0: 1.2, door_joint_1: 1.2}):
        open_0 = ctx.part_element_world_aabb(door_0, elem="inner_stile")
        open_1 = ctx.part_element_world_aabb(door_1, elem="inner_stile")

    ctx.check(
        "door_0 rotates outward from its side hinge",
        closed_0 is not None and open_0 is not None and open_0[0][1] < closed_0[0][1] - 0.05,
        details=f"closed={closed_0}, open={open_0}",
    )
    ctx.check(
        "door_1 rotates outward from its side hinge",
        closed_1 is not None and open_1 is not None and open_1[0][1] < closed_1[0][1] - 0.05,
        details=f"closed={closed_1}, open={open_1}",
    )

    return ctx.report()


object_model = build_object_model()

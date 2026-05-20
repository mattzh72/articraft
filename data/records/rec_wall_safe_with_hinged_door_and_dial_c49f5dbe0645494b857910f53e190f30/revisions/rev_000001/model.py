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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_safe")

    painted_steel = model.material("blue_black_steel", rgba=(0.06, 0.075, 0.08, 1.0))
    dark_steel = model.material("dark_interior_steel", rgba=(0.025, 0.03, 0.035, 1.0))
    worn_edge = model.material("worn_edge_highlight", rgba=(0.18, 0.20, 0.20, 1.0))
    black = model.material("matte_black", rgba=(0.004, 0.004, 0.005, 1.0))
    brass = model.material("aged_brass", rgba=(0.76, 0.55, 0.24, 1.0))
    drawer_paint = model.material("drawer_gray", rgba=(0.12, 0.13, 0.13, 1.0))

    body = model.part("body")
    # Rectangular wall-safe carcass: a shallow steel box with a proud front frame.
    body.visual(Box((0.620, 0.026, 0.460)), origin=Origin(xyz=(0.0, 0.117, 0.230)), material=dark_steel, name="back_wall")
    body.visual(Box((0.028, 0.260, 0.460)), origin=Origin(xyz=(-0.296, 0.0, 0.230)), material=painted_steel, name="side_wall_0")
    body.visual(Box((0.028, 0.260, 0.460)), origin=Origin(xyz=(0.296, 0.0, 0.230)), material=painted_steel, name="side_wall_1")
    body.visual(Box((0.620, 0.260, 0.028)), origin=Origin(xyz=(0.0, 0.0, 0.446)), material=painted_steel, name="top_wall")
    body.visual(Box((0.620, 0.260, 0.028)), origin=Origin(xyz=(0.0, 0.0, 0.014)), material=painted_steel, name="bottom_wall")

    # Front frame pieces overlap the shell by a few millimeters, as welded steel.
    body.visual(Box((0.062, 0.036, 0.460)), origin=Origin(xyz=(-0.279, -0.132, 0.230)), material=painted_steel, name="front_jamb_0")
    body.visual(Box((0.062, 0.036, 0.460)), origin=Origin(xyz=(0.279, -0.132, 0.230)), material=painted_steel, name="front_jamb_1")
    body.visual(Box((0.620, 0.036, 0.052)), origin=Origin(xyz=(0.0, -0.132, 0.434)), material=painted_steel, name="front_rail_0")
    body.visual(Box((0.620, 0.036, 0.052)), origin=Origin(xyz=(0.0, -0.132, 0.026)), material=painted_steel, name="front_rail_1")
    body.visual(Box((0.500, 0.010, 0.018)), origin=Origin(xyz=(0.0, -0.111, 0.205)), material=worn_edge, name="shelf_lip")
    body.visual(Box((0.510, 0.220, 0.018)), origin=Origin(xyz=(0.0, 0.000, 0.205)), material=dark_steel, name="interior_shelf")

    # Fixed short runners for the lower service drawer, tied into the side walls.
    body.visual(Box((0.086, 0.174, 0.012)), origin=Origin(xyz=(-0.239, -0.020, 0.046)), material=worn_edge, name="runner_0")
    body.visual(Box((0.086, 0.174, 0.012)), origin=Origin(xyz=(0.239, -0.020, 0.046)), material=worn_edge, name="runner_1")

    # Alternating fixed hinge knuckles and leaves on the right frame edge.
    body.visual(Cylinder(radius=0.004, length=0.384), origin=Origin(xyz=(0.284, -0.174, 0.230)), material=worn_edge, name="hinge_pin")
    body.visual(Cylinder(radius=0.011, length=0.105), origin=Origin(xyz=(0.284, -0.174, 0.097)), material=worn_edge, name="hinge_knuckle_0")
    body.visual(Cylinder(radius=0.011, length=0.105), origin=Origin(xyz=(0.284, -0.174, 0.363)), material=worn_edge, name="hinge_knuckle_1")
    body.visual(Box((0.034, 0.048, 0.112)), origin=Origin(xyz=(0.300, -0.158, 0.097)), material=worn_edge, name="hinge_leaf_0")
    body.visual(Box((0.034, 0.048, 0.112)), origin=Origin(xyz=(0.300, -0.158, 0.363)), material=worn_edge, name="hinge_leaf_1")

    door = model.part("door")
    # Door frame is at the hinge pin.  Closed geometry extends along local -X.
    door.visual(Box((0.505, 0.040, 0.372)), origin=Origin(xyz=(-0.268, 0.0, 0.0)), material=painted_steel, name="door_slab")
    door.visual(Box((0.448, 0.008, 0.306)), origin=Origin(xyz=(-0.277, -0.024, 0.0)), material=dark_steel, name="recessed_panel")
    door.visual(Box((0.470, 0.010, 0.020)), origin=Origin(xyz=(-0.276, -0.030, 0.165)), material=worn_edge, name="door_trim_0")
    door.visual(Box((0.470, 0.010, 0.020)), origin=Origin(xyz=(-0.276, -0.030, -0.165)), material=worn_edge, name="door_trim_1")
    door.visual(Box((0.020, 0.010, 0.330)), origin=Origin(xyz=(-0.505, -0.030, 0.0)), material=worn_edge, name="door_trim_2")
    door.visual(Box((0.020, 0.010, 0.330)), origin=Origin(xyz=(-0.048, -0.030, 0.0)), material=worn_edge, name="door_trim_3")

    # Moving middle hinge knuckle and leaf: centered on the same hinge axis.
    door.visual(Cylinder(radius=0.010, length=0.132), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=worn_edge, name="hinge_knuckle")
    door.visual(Box((0.030, 0.012, 0.140)), origin=Origin(xyz=(-0.015, 0.0, 0.0)), material=worn_edge, name="hinge_leaf")

    # Short fixed handle below the dial, with two standoffs that reach the door.
    door.visual(Box((0.018, 0.040, 0.018)), origin=Origin(xyz=(-0.342, -0.040, -0.085)), material=brass, name="handle_post_0")
    door.visual(Box((0.018, 0.040, 0.018)), origin=Origin(xyz=(-0.225, -0.040, -0.085)), material=brass, name="handle_post_1")
    door.visual(Box((0.146, 0.018, 0.024)), origin=Origin(xyz=(-0.283, -0.068, -0.085)), material=brass, name="handle_grip")

    # A small bushing plate stays with the door; the dial itself is a child part.
    door.visual(Cylinder(radius=0.057, length=0.006), origin=Origin(xyz=(-0.283, -0.031, 0.082), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brass, name="dial_bushing")
    door.visual(Box((0.022, 0.006, 0.010)), origin=Origin(xyz=(-0.283, -0.032, 0.152)), material=brass, name="dial_index")

    dial = model.part("dial")
    dial_geom = KnobGeometry(
        0.092,
        0.020,
        body_style="cylindrical",
        edge_radius=0.0015,
        grip=KnobGrip(style="ribbed", count=48, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
    )
    dial.visual(mesh_from_geometry(dial_geom, "combination_dial"), origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=black, name="dial_cap")
    dial.visual(Cylinder(radius=0.013, length=0.010), origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brass, name="center_hub")
    dial.visual(Cylinder(radius=0.008, length=0.016), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brass, name="dial_shaft")

    drawer = model.part("drawer")
    # A small tray-style service drawer under the shelf. It keeps enough length
    # inside the runners to remain captured when pulled out.
    drawer.visual(Box((0.382, 0.014, 0.082)), origin=Origin(xyz=(0.0, -0.071, 0.0)), material=drawer_paint, name="drawer_front")
    drawer.visual(Box((0.382, 0.132, 0.012)), origin=Origin(xyz=(0.0, -0.001, -0.041)), material=drawer_paint, name="drawer_bottom")
    drawer.visual(Box((0.014, 0.132, 0.075)), origin=Origin(xyz=(-0.191, -0.001, -0.003)), material=drawer_paint, name="drawer_side_0")
    drawer.visual(Box((0.014, 0.132, 0.075)), origin=Origin(xyz=(0.191, -0.001, -0.003)), material=drawer_paint, name="drawer_side_1")
    drawer.visual(Box((0.360, 0.014, 0.075)), origin=Origin(xyz=(0.0, 0.064, -0.003)), material=drawer_paint, name="drawer_back")
    drawer.visual(Box((0.040, 0.016, 0.012)), origin=Origin(xyz=(0.0, -0.085, 0.018)), material=brass, name="drawer_pull")
    drawer.visual(Box((0.020, 0.154, 0.010)), origin=Origin(xyz=(-0.203, -0.004, -0.038)), material=worn_edge, name="runner_shoe_0")
    drawer.visual(Box((0.020, 0.154, 0.010)), origin=Origin(xyz=(0.203, -0.004, -0.038)), material=worn_edge, name="runner_shoe_1")

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.284, -0.174, 0.230)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(-0.283, -0.038, 0.082)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.024, 0.095)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.20, lower=0.0, upper=0.105),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    drawer = object_model.get_part("drawer")
    door_joint = object_model.get_articulation("body_to_door")
    dial_joint = object_model.get_articulation("door_to_dial")
    drawer_joint = object_model.get_articulation("body_to_drawer")

    ctx.allow_overlap(
        body,
        door,
        elem_a="hinge_pin",
        elem_b="hinge_knuckle",
        reason="The steel hinge pin is intentionally captured inside the door knuckle proxy.",
    )
    ctx.expect_within(
        body,
        door,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="hinge_knuckle",
        margin=0.001,
        name="hinge pin is centered in the door knuckle",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="z",
        elem_a="hinge_pin",
        elem_b="hinge_knuckle",
        min_overlap=0.120,
        name="door knuckle is retained on the vertical pin",
    )

    ctx.allow_overlap(
        dial,
        door,
        elem_a="dial_shaft",
        elem_b="dial_bushing",
        reason="The combination dial shaft is intentionally seated in the door bushing.",
    )
    ctx.expect_within(
        dial,
        door,
        axes="xz",
        inner_elem="dial_shaft",
        outer_elem="dial_bushing",
        margin=0.002,
        name="dial shaft is centered in bushing",
    )
    ctx.expect_overlap(
        dial,
        door,
        axes="y",
        elem_a="dial_shaft",
        elem_b="dial_bushing",
        min_overlap=0.004,
        name="dial shaft has seated insertion",
    )

    ctx.check(
        "dial has continuous shaft rotation",
        getattr(dial_joint, "articulation_type", None) == ArticulationType.CONTINUOUS
        and tuple(getattr(dial_joint, "axis", ())) == (0.0, -1.0, 0.0),
        details=f"type={getattr(dial_joint, 'articulation_type', None)}, axis={getattr(dial_joint, 'axis', None)}",
    )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="front_jamb_1",
            negative_elem="door_slab",
            min_gap=0.002,
            max_gap=0.012,
            name="closed door sits just proud of front frame",
        )
        closed_edge = ctx.part_element_world_aabb(door, elem="door_trim_2")

    with ctx.pose({door_joint: 1.20}):
        open_edge = ctx.part_element_world_aabb(door, elem="door_trim_2")

    ctx.check(
        "right-hinged door swings outward",
        closed_edge is not None
        and open_edge is not None
        and open_edge[0][1] < closed_edge[0][1] - 0.12,
        details=f"closed_edge={closed_edge}, open_edge={open_edge}",
    )

    with ctx.pose({drawer_joint: 0.0}):
        ctx.expect_gap(
            drawer,
            body,
            axis="z",
            positive_elem="runner_shoe_0",
            negative_elem="runner_0",
            max_gap=0.002,
            max_penetration=0.0,
            name="service drawer shoe rides on left runner",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="runner_shoe_0",
            elem_b="runner_0",
            min_overlap=0.120,
            name="closed drawer is fully engaged on runner",
        )
        rest_pos = ctx.part_world_position(drawer)

    with ctx.pose({drawer_joint: 0.105}):
        ctx.expect_gap(
            drawer,
            body,
            axis="z",
            positive_elem="runner_shoe_0",
            negative_elem="runner_0",
            max_gap=0.002,
            max_penetration=0.0,
            name="extended drawer remains supported by runner",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="runner_shoe_0",
            elem_b="runner_0",
            min_overlap=0.045,
            name="extended drawer keeps retained runner insertion",
        )
        extended_pos = ctx.part_world_position(drawer)

    ctx.check(
        "service drawer translates outward on short runners",
        rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.09,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()

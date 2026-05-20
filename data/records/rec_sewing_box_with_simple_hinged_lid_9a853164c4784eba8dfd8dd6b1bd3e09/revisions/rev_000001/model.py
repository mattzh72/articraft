from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_sewing_box")

    body_paint = model.material("hammered_olive_paint", color=(0.18, 0.24, 0.18, 1.0))
    lid_paint = model.material("dark_olive_lid", color=(0.14, 0.20, 0.15, 1.0))
    interior_paint = model.material("light_interior_enamel", color=(0.70, 0.72, 0.62, 1.0))
    rubber = model.material("black_rubber", color=(0.015, 0.014, 0.012, 1.0))
    steel = model.material("brushed_steel", color=(0.62, 0.60, 0.54, 1.0))
    dark_steel = model.material("dark_phosphate_steel", color=(0.16, 0.16, 0.15, 1.0))

    body = model.part("body")
    lid = model.part("lid")

    # Overall utility-box envelope: compact enough for sewing notions, but with
    # thick molded/painted walls and a real open storage cavity.
    w = 0.380
    d = 0.240
    wall = 0.020
    bottom_t = 0.020
    wall_h = 0.150
    rim_t = 0.018
    rim_w = 0.032
    body_top = bottom_t + wall_h + rim_t

    # Hollow base shell: bottom, four thick walls, and a raised top frame that
    # leaves the sewing-box opening unobstructed.
    body.visual(
        Box((w, d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=body_paint,
        name="bottom_pan",
    )
    body.visual(
        Box((wall, d, wall_h)),
        origin=Origin(xyz=(-w / 2.0 + wall / 2.0, 0.0, bottom_t + wall_h / 2.0)),
        material=body_paint,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, d, wall_h)),
        origin=Origin(xyz=(w / 2.0 - wall / 2.0, 0.0, bottom_t + wall_h / 2.0)),
        material=body_paint,
        name="side_wall_1",
    )
    body.visual(
        Box((w, wall, wall_h)),
        origin=Origin(xyz=(0.0, -d / 2.0 + wall / 2.0, bottom_t + wall_h / 2.0)),
        material=body_paint,
        name="front_wall",
    )
    body.visual(
        Box((w, wall, wall_h)),
        origin=Origin(xyz=(0.0, d / 2.0 - wall / 2.0, bottom_t + wall_h / 2.0)),
        material=body_paint,
        name="rear_wall",
    )
    body.visual(
        Box((rim_w, d, rim_t)),
        origin=Origin(xyz=(-w / 2.0 + rim_w / 2.0, 0.0, bottom_t + wall_h + rim_t / 2.0)),
        material=body_paint,
        name="side_rim_0",
    )
    body.visual(
        Box((rim_w, d, rim_t)),
        origin=Origin(xyz=(w / 2.0 - rim_w / 2.0, 0.0, bottom_t + wall_h + rim_t / 2.0)),
        material=body_paint,
        name="side_rim_1",
    )
    body.visual(
        Box((w, rim_w, rim_t)),
        origin=Origin(xyz=(0.0, -d / 2.0 + rim_w / 2.0, bottom_t + wall_h + rim_t / 2.0)),
        material=body_paint,
        name="front_rim",
    )
    body.visual(
        Box((w, rim_w, rim_t)),
        origin=Origin(xyz=(0.0, d / 2.0 - rim_w / 2.0, bottom_t + wall_h + rim_t / 2.0)),
        material=body_paint,
        name="rear_rim",
    )

    # Light enamel interior: dividers, a needle packet slot, and spool pegs,
    # all tied into the bottom pan instead of floating in the cavity.
    body.visual(
        Box((w - 2.0 * wall, 0.008, 0.070)),
        origin=Origin(xyz=(0.0, -0.018, bottom_t + 0.035)),
        material=interior_paint,
        name="long_divider",
    )
    body.visual(
        Box((0.008, d - 2.0 * wall, 0.058)),
        origin=Origin(xyz=(-0.070, 0.0, bottom_t + 0.029)),
        material=interior_paint,
        name="cross_divider_0",
    )
    body.visual(
        Box((0.008, d - 2.0 * wall, 0.058)),
        origin=Origin(xyz=(0.075, 0.0, bottom_t + 0.029)),
        material=interior_paint,
        name="cross_divider_1",
    )
    body.visual(
        Box((0.090, 0.010, 0.060)),
        origin=Origin(xyz=(0.0, -0.088, bottom_t + 0.030)),
        material=interior_paint,
        name="packet_slot",
    )
    for index, x in enumerate((-0.135, 0.135)):
        body.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, 0.065, bottom_t + 0.003)),
            material=interior_paint,
            name=f"spool_base_{index}",
        )
        body.visual(
            Cylinder(radius=0.006, length=0.062),
            origin=Origin(xyz=(x, 0.065, bottom_t + 0.031)),
            material=interior_paint,
            name=f"spool_peg_{index}",
        )

    # Rugged external protection: bottom feet, corner caps, and molded ribs.
    for index, x in enumerate((-0.150, 0.150)):
        body.visual(
            Box((0.060, 0.030, 0.012)),
            origin=Origin(xyz=(x, -0.094, -0.006)),
            material=rubber,
            name=f"front_foot_{index}",
        )
        body.visual(
            Box((0.060, 0.030, 0.012)),
            origin=Origin(xyz=(x, 0.094, -0.006)),
            material=rubber,
            name=f"rear_foot_{index}",
        )

    corner_positions = (
        (-w / 2.0 + 0.017, -d / 2.0 + 0.017),
        (w / 2.0 - 0.017, -d / 2.0 + 0.017),
        (-w / 2.0 + 0.017, d / 2.0 - 0.017),
        (w / 2.0 - 0.017, d / 2.0 - 0.017),
    )
    for index, (x, y) in enumerate(corner_positions):
        body.visual(
            Box((0.034, 0.034, 0.112)),
            origin=Origin(xyz=(x, y, bottom_t + 0.056)),
            material=rubber,
            name=f"corner_bumper_{index}",
        )

    for index, x in enumerate((-0.105, 0.0, 0.105)):
        body.visual(
            Box((0.014, 0.010, 0.085)),
            origin=Origin(xyz=(x, -d / 2.0 - 0.003, bottom_t + 0.065)),
            material=body_paint,
            name=f"front_rib_{index}",
        )

    # Rear hinge support structure fixed to the body: a vertical leaf, thick
    # saddle brackets, alternating knuckles, and visible pin-end caps.
    hinge_y = d / 2.0 + 0.025
    hinge_z = body_top + 0.018
    body.visual(
        Box((0.345, 0.006, 0.050)),
        origin=Origin(xyz=(0.0, d / 2.0 + 0.003, body_top - 0.027)),
        material=dark_steel,
        name="body_hinge_leaf",
    )
    for knuckle_name, saddle_name, x in (
        ("body_knuckle_0", "hinge_saddle_0", -0.145),
        ("body_knuckle_1", "hinge_saddle_1", 0.0),
        ("body_knuckle_2", "hinge_saddle_2", 0.145),
    ):
        body.visual(
            Box((0.064, 0.032, 0.020)),
            origin=Origin(xyz=(x, d / 2.0 + 0.016, hinge_z - 0.010)),
            material=dark_steel,
            name=saddle_name,
        )
        body.visual(
            Cylinder(radius=0.009, length=0.070),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=knuckle_name,
        )
    body.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(-0.183, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pin_head_0",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.183, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pin_head_1",
    )

    # Exposed screw heads seated into body panels and hinge leaves.
    for index, x in enumerate((-0.135, -0.045, 0.045, 0.135)):
        body.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, -d / 2.0 - 0.0015, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"front_screw_{index}",
        )
    for index, x in enumerate((-0.145, 0.0, 0.145)):
        body.visual(
            Cylinder(radius=0.005, length=0.004),
            origin=Origin(xyz=(x - 0.018, d / 2.0 + 0.0045, body_top - 0.033), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"hinge_screw_a_{index}",
        )
        body.visual(
            Cylinder(radius=0.005, length=0.004),
            origin=Origin(xyz=(x + 0.018, d / 2.0 + 0.0045, body_top - 0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"hinge_screw_b_{index}",
        )

    # Lid local frame is the rear hinge axis at q=0.  The closed panel extends
    # in local -Y from the hinge line; positive hinge motion opens upward.
    lid_w = 0.410
    lid_d = 0.275
    lid_t = 0.028
    lid.visual(
        Box((lid_w, lid_d, lid_t)),
        origin=Origin(xyz=(0.0, -0.1675, -0.002)),
        material=lid_paint,
        name="lid_panel",
    )
    lid.visual(
        Box((lid_w, 0.026, 0.014)),
        origin=Origin(xyz=(0.0, -0.262, 0.019)),
        material=lid_paint,
        name="front_frame",
    )
    lid.visual(
        Box((lid_w, 0.026, 0.014)),
        origin=Origin(xyz=(0.0, -0.043, 0.019)),
        material=lid_paint,
        name="rear_frame",
    )
    lid.visual(
        Box((0.026, 0.235, 0.014)),
        origin=Origin(xyz=(-0.187, -0.1675, 0.019)),
        material=lid_paint,
        name="side_frame_0",
    )
    lid.visual(
        Box((0.026, 0.235, 0.014)),
        origin=Origin(xyz=(0.187, -0.1675, 0.019)),
        material=lid_paint,
        name="side_frame_1",
    )
    lid.visual(
        Box((0.018, 0.188, 0.010)),
        origin=Origin(xyz=(0.0, -0.1675, 0.017)),
        material=lid_paint,
        name="center_rib",
    )
    lid.visual(
        Box((0.125, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, -lid_d - 0.006, -0.003)),
        material=rubber,
        name="front_pull",
    )

    # A compressed-looking rubber seal follows the body rim but is part of the
    # moving lid. It proves the closure line without filling the storage opening.
    lid.visual(
        Box((0.026, 0.210, 0.004)),
        origin=Origin(xyz=(-w / 2.0 + rim_w / 2.0, -0.145, -0.016)),
        material=rubber,
        name="side_seal_0",
    )
    lid.visual(
        Box((0.026, 0.210, 0.004)),
        origin=Origin(xyz=(w / 2.0 - rim_w / 2.0, -0.145, -0.016)),
        material=rubber,
        name="side_seal_1",
    )
    lid.visual(
        Box((0.330, 0.026, 0.004)),
        origin=Origin(xyz=(0.0, -0.249, -0.016)),
        material=rubber,
        name="front_seal",
    )
    lid.visual(
        Box((0.330, 0.026, 0.004)),
        origin=Origin(xyz=(0.0, -0.041, -0.016)),
        material=rubber,
        name="rear_seal",
    )

    # Lid-side hinge leaf and interleaved knuckles.
    lid.visual(
        Box((0.335, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, -0.040, -0.006)),
        material=dark_steel,
        name="lid_hinge_leaf",
    )
    for knuckle_name, saddle_name, x in (
        ("lid_knuckle_0", "lid_hinge_saddle_0", -0.0725),
        ("lid_knuckle_1", "lid_hinge_saddle_1", 0.0725),
    ):
        lid.visual(
            Box((0.058, 0.034, 0.014)),
            origin=Origin(xyz=(x, -0.014, -0.004)),
            material=dark_steel,
            name=saddle_name,
        )
        lid.visual(
            Cylinder(radius=0.0084, length=0.065),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=knuckle_name,
        )

    # Lid fasteners on the perimeter frame, pull, and hinge leaf.
    for index, (x, y) in enumerate(((-0.160, -0.262), (0.160, -0.262), (-0.160, -0.043), (0.160, -0.043))):
        lid.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(xyz=(x, y, 0.028)),
            material=steel,
            name=f"lid_screw_{index}",
        )
    for index, x in enumerate((-0.040, 0.040)):
        lid.visual(
            Cylinder(radius=0.005, length=0.004),
            origin=Origin(xyz=(x, -lid_d - 0.012, -0.003), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"pull_screw_{index}",
        )
    for index, x in enumerate((-0.110, 0.110)):
        lid.visual(
            Cylinder(radius=0.0048, length=0.004),
            origin=Origin(xyz=(x, -0.040, -0.001)),
            material=steel,
            name=f"lid_hinge_screw_{index}",
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.35),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    def _zmax(aabb):
        if aabb is None:
            return None
        v = aabb[1]
        try:
            return float(v[2])
        except TypeError:
            return float(v.z)

    # Closed state: the rubber seal lands on the raised body rim and the solid
    # lid panel remains just above the opening rather than filling it.
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="front_seal",
        negative_elem="front_rim",
        max_gap=0.001,
        max_penetration=0.0002,
        name="front seal sits on rim",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="front_rim",
        min_gap=0.001,
        max_gap=0.004,
        name="lid panel clears the framed opening",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="front_rim",
        min_overlap=0.025,
        name="lid covers the body rim",
    )

    # Hinge state: adjacent knuckles share the same pivot line but are separated
    # along X, making the simple utility hinge legible instead of decorative.
    ctx.expect_overlap(
        lid,
        body,
        axes="yz",
        elem_a="lid_knuckle_0",
        elem_b="body_knuckle_0",
        min_overlap=0.006,
        name="hinge knuckles are coaxial",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="x",
        positive_elem="lid_knuckle_0",
        negative_elem="body_knuckle_0",
        min_gap=0.003,
        max_gap=0.010,
        name="hinge knuckles have service clearance",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_frame")
    closed_front_z = _zmax(closed_front)
    with ctx.pose({hinge: 1.10}):
        opened_front = ctx.part_element_world_aabb(lid, elem="front_frame")
        opened_front_z = _zmax(opened_front)

    ctx.check(
        "lid opens upward about rear hinge",
        closed_front_z is not None
        and opened_front_z is not None
        and opened_front_z > closed_front_z + 0.10,
        details=f"closed_z={closed_front_z}, opened_z={opened_front_z}",
    )

    return ctx.report()


object_model = build_object_model()

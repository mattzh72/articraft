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
    model = ArticulatedObject(name="cost_optimized_sewing_box")

    warm_plastic = model.material("warm_molded_polypropylene", rgba=(0.86, 0.78, 0.62, 1.0))

    body = model.part("body")
    lid = model.part("lid")

    # Real-world desktop sewing box proportions: a shallow, open storage tray
    # with molded dividers and two integrated spool pegs.  All tray details are
    # one injection-molded body part to keep part count and assembly cost low.
    width = 0.34
    depth = 0.22
    height = 0.110
    wall = 0.008
    bottom = 0.012
    upper_wall_h = height - bottom
    wall_z = bottom + upper_wall_h / 2.0

    body.visual(
        Box((width, depth, bottom)),
        origin=Origin(xyz=(0.0, 0.0, bottom / 2.0)),
        material=warm_plastic,
        name="floor",
    )
    body.visual(
        Box((width, wall, upper_wall_h)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, wall_z)),
        material=warm_plastic,
        name="front_wall",
    )
    body.visual(
        Box((width, wall, upper_wall_h)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, wall_z)),
        material=warm_plastic,
        name="rear_wall",
    )
    body.visual(
        Box((wall, depth - 2.0 * wall, upper_wall_h)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, wall_z)),
        material=warm_plastic,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, depth - 2.0 * wall, upper_wall_h)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, wall_z)),
        material=warm_plastic,
        name="side_wall_1",
    )

    # Molded-in sewing storage features: low ribs create bobbin/needle pockets,
    # and short spool pegs rise from the floor.  The ribs contact the floor and
    # surrounding walls, so nothing is orphaned or glued on later.
    rib_h = 0.030
    body.visual(
        Box((0.006, depth - 2.0 * wall, rib_h)),
        origin=Origin(xyz=(-0.035, 0.0, bottom + rib_h / 2.0)),
        material=warm_plastic,
        name="long_divider",
    )
    body.visual(
        Box((0.105, 0.006, rib_h)),
        origin=Origin(xyz=(-0.087, -0.020, bottom + rib_h / 2.0)),
        material=warm_plastic,
        name="short_divider_0",
    )
    body.visual(
        Box((0.105, 0.006, rib_h)),
        origin=Origin(xyz=(-0.087, 0.048, bottom + rib_h / 2.0)),
        material=warm_plastic,
        name="short_divider_1",
    )
    for i, x in enumerate((0.070, 0.118)):
        body.visual(
            Cylinder(radius=0.010, length=0.050),
            origin=Origin(xyz=(x, -0.045, bottom + 0.025)),
            material=warm_plastic,
            name=f"spool_peg_{i}",
        )

    # A simple molded snap keeper on the front wall: cheap to mold, no separate
    # latch hardware, and visibly aligned with the flexible lid tab.
    body.visual(
        Box((0.070, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.002, 0.082)),
        material=warm_plastic,
        name="snap_keeper",
    )

    # Interleaved living-plastic barrel knuckles at the rear.  The joint axis
    # passes through these visible barrels; alternating body/lid knuckles give a
    # real hinge constraint instead of a decorative block.
    hinge_y = depth / 2.0 + 0.018
    hinge_z = height + 0.0125
    hinge_radius = 0.006
    knuckle_len = 0.052
    knuckle_centers = (-0.116, -0.058, 0.0, 0.058, 0.116)
    x = knuckle_centers[0]
    body.visual(
        Cylinder(radius=hinge_radius, length=knuckle_len),
        origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_plastic,
        name="body_knuckle_0",
    )
    body.visual(
        Box((knuckle_len, 0.008, 0.028)),
        origin=Origin(xyz=(x, depth / 2.0 + 0.003, height - 0.002)),
        material=warm_plastic,
        name="body_hinge_leaf_0",
    )
    body.visual(
        Box((knuckle_len, 0.014, 0.005)),
        origin=Origin(xyz=(x, depth / 2.0 + 0.0115, height + 0.007)),
        material=warm_plastic,
        name="body_hinge_bridge_0",
    )
    for idx in (2, 4):
        x = knuckle_centers[idx]
        body.visual(
            Cylinder(radius=hinge_radius, length=knuckle_len),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=warm_plastic,
            name=f"body_knuckle_{idx}",
        )
        body.visual(
            Box((knuckle_len, 0.008, 0.028)),
            origin=Origin(xyz=(x, depth / 2.0 + 0.003, height - 0.002)),
            material=warm_plastic,
            name=f"body_hinge_leaf_{idx}",
        )
        body.visual(
            Box((knuckle_len, 0.014, 0.005)),
            origin=Origin(xyz=(x, depth / 2.0 + 0.0115, height + 0.007)),
            material=warm_plastic,
            name=f"body_hinge_bridge_{idx}",
        )

    # The lid frame is the hinge axis.  At q=0 the shell lies closed over the
    # tray; positive joint motion swings the free front edge upward.
    lid_width = width + 0.018
    lid_depth = 0.239
    lid_thick = 0.012
    lid_rear_clearance = 0.015
    lid_front_y = -(lid_rear_clearance + lid_depth)
    lid.visual(
        Box((lid_width, lid_depth, lid_thick)),
        origin=Origin(xyz=(0.0, lid_front_y + lid_depth / 2.0, -lid_thick / 2.0)),
        material=warm_plastic,
        name="lid_panel",
    )

    skirt_h = 0.026
    skirt_t = 0.005
    skirt_z = -lid_thick - skirt_h / 2.0
    side_skirt_y = lid_front_y + lid_depth / 2.0
    lid.visual(
        Box((skirt_t, lid_depth - 0.012, skirt_h)),
        origin=Origin(xyz=(-lid_width / 2.0 + skirt_t / 2.0, side_skirt_y, skirt_z)),
        material=warm_plastic,
        name="skirt_0",
    )
    lid.visual(
        Box((skirt_t, lid_depth - 0.012, skirt_h)),
        origin=Origin(xyz=(lid_width / 2.0 - skirt_t / 2.0, side_skirt_y, skirt_z)),
        material=warm_plastic,
        name="skirt_1",
    )
    lid.visual(
        Box((0.096, skirt_t, 0.036)),
        origin=Origin(xyz=(0.0, lid_front_y - skirt_t / 2.0, -lid_thick - 0.018)),
        material=warm_plastic,
        name="snap_tab",
    )
    lid.visual(
        Box((0.060, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, lid_front_y + 0.002, -lid_thick - 0.030)),
        material=warm_plastic,
        name="snap_bead",
    )

    lid.visual(
        Cylinder(radius=hinge_radius, length=knuckle_len),
        origin=Origin(xyz=(knuckle_centers[1], 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_plastic,
        name="lid_knuckle_1",
    )
    lid.visual(
        Box((knuckle_len, 0.014, 0.005)),
        origin=Origin(xyz=(knuckle_centers[1], -0.010, -0.006)),
        material=warm_plastic,
        name="lid_hinge_bridge_1",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=knuckle_len),
        origin=Origin(xyz=(knuckle_centers[3], 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_plastic,
        name="lid_knuckle_3",
    )
    lid.visual(
        Box((knuckle_len, 0.014, 0.005)),
        origin=Origin(xyz=(knuckle_centers[3], -0.010, -0.006)),
        material=warm_plastic,
        name="lid_hinge_bridge_3",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="rear_wall",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed lid rests on rear rim without overlap",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="floor",
        min_overlap=0.18,
        name="lid covers the storage box footprint",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="x",
        positive_elem="lid_knuckle_1",
        negative_elem="body_knuckle_0",
        min_gap=0.002,
        max_gap=0.012,
        name="alternating hinge knuckles have assembly clearance",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="yz",
        elem_a="lid_knuckle_1",
        elem_b="body_knuckle_0",
        min_overlap=0.008,
        name="hinge knuckles share the same rotation axis",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.85}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "hinged lid swings upward at its limit",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

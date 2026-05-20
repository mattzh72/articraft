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
    model = ArticulatedObject(name="plain_lid_sewing_box")

    wood = model.material("warm_maple_wood", color=(0.74, 0.50, 0.30, 1.0))
    endgrain = model.material("slightly_darker_endgrain", color=(0.55, 0.34, 0.18, 1.0))
    brass = model.material("brushed_brass_hinge", color=(0.80, 0.62, 0.28, 1.0))
    lining = model.material("muted_red_felt_lining", color=(0.48, 0.06, 0.08, 1.0))

    # A compact real-world sewing box: roughly 34 cm wide, 24 cm deep, and
    # 13 cm tall including the plain top panel.
    width = 0.320
    depth = 0.220
    body_height = 0.115
    wall = 0.012
    floor = 0.010
    lid_thickness = 0.012
    lid_width = 0.338
    lid_depth = 0.232
    hinge_y = depth / 2.0 + 0.010
    hinge_z = body_height + lid_thickness / 2.0

    body = model.part("body")
    body.visual(
        Box((width, depth, floor)),
        origin=Origin(xyz=(0.0, 0.0, floor / 2.0)),
        material=wood,
        name="floor",
    )
    body.visual(
        Box((width, wall, body_height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, body_height / 2.0)),
        material=wood,
        name="front_wall",
    )
    body.visual(
        Box((width, wall, body_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, body_height / 2.0)),
        material=wood,
        name="rear_wall",
    )
    body.visual(
        Box((wall, depth - 2.0 * wall, body_height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, body_height / 2.0)),
        material=endgrain,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, depth - 2.0 * wall, body_height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, body_height / 2.0)),
        material=endgrain,
        name="side_wall_1",
    )
    # A visible felt bottom when the lid is opened; it sits on the wood floor
    # and does not create a separate moving mechanism.
    body.visual(
        Box((width - 2.0 * wall - 0.010, depth - 2.0 * wall - 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, floor + 0.002)),
        material=lining,
        name="felt_lining",
    )

    # Fixed hinge knuckles and small rear support pads are on the body.  They
    # are segmented to leave clearance for the moving lid knuckles.
    hinge_radius = 0.0045
    hinge_supports = [(-0.118, 0.045), (0.0, 0.039), (0.118, 0.045)]
    for i, (x, length) in enumerate(hinge_supports):
        body.visual(
            Box((length, 0.004, 0.020)),
            origin=Origin(xyz=(x, depth / 2.0 + 0.002, body_height - 0.011)),
            material=brass,
            name=f"hinge_leaf_{i}",
        )
        body.visual(
            Box((length, 0.003, 0.006)),
            origin=Origin(xyz=(x, depth / 2.0 + 0.0035, body_height + 0.0005)),
            material=brass,
            name=f"hinge_riser_{i}",
        )
        body.visual(
            Box((length, 0.008, 0.003)),
            origin=Origin(xyz=(x, hinge_y - 0.0015, body_height + 0.0005)),
            material=brass,
            name=f"hinge_saddle_{i}",
        )
        body.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(
                xyz=(x, hinge_y, hinge_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=f"body_knuckle_{i}",
        )

    lid = model.part("lid")
    # The child part frame lies exactly on the rear hinge axis.  At q=0 the
    # panel extends mostly along local -Y, forming one large uninterrupted top.
    lid.visual(
        Box((lid_width, lid_depth, lid_thickness)),
        origin=Origin(xyz=(0.0, -lid_depth / 2.0 - 0.008, 0.0)),
        material=wood,
        name="top_panel",
    )
    # Very small rear tabs tie the moving knuckles to the plain panel without
    # adding a decorative handle or breaking up the top face.
    lid_knuckles = [(-0.055, 0.063), (0.055, 0.063)]
    for i, (x, length) in enumerate(lid_knuckles):
        lid.visual(
            Box((length, 0.009, 0.006)),
            origin=Origin(xyz=(x, -0.0045, 0.0)),
            material=brass,
            name=f"lid_hinge_leaf_{i}",
        )
        lid.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(
                xyz=(x, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=f"lid_knuckle_{i}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        # The closed panel extends along local -Y from the rear hinge.  A
        # negative X axis makes positive q lift the front edge upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="top_panel",
            negative_elem="front_wall",
            max_gap=0.001,
            max_penetration=0.0,
            name="plain lid rests on the front rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="top_panel",
            elem_b="floor",
            min_overlap=0.18,
            name="single lid panel covers the box footprint",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: 1.25}):
        open_aabb = ctx.part_world_aabb(lid)
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="top_panel",
            negative_elem="front_wall",
            min_gap=0.008,
            name="opened lid lifts clear of the box rim",
        )
    ctx.check(
        "lid revolute joint lifts the front edge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.05,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

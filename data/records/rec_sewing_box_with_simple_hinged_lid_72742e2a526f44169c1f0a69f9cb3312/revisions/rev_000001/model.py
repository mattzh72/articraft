from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_sewing_box")

    warm_wood = Material("warm_oiled_wood", color=(0.62, 0.38, 0.20, 1.0))
    lid_wood = Material("plain_lid_wood", color=(0.70, 0.47, 0.26, 1.0))
    felt = Material("muted_red_felt", color=(0.55, 0.06, 0.08, 1.0))
    brass = Material("brushed_brass_hinge", color=(0.86, 0.62, 0.22, 1.0))

    body = model.part("body")
    # A small hollow rectangular sewing box: floor plus four wooden walls.
    body.visual(
        Box((0.300, 0.200, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=warm_wood,
        name="floor",
    )
    body.visual(
        Box((0.300, 0.014, 0.095)),
        origin=Origin(xyz=(0.0, -0.093, 0.0535)),
        material=warm_wood,
        name="front_wall",
    )
    body.visual(
        Box((0.300, 0.014, 0.095)),
        origin=Origin(xyz=(0.0, 0.093, 0.0535)),
        material=warm_wood,
        name="rear_wall",
    )
    body.visual(
        Box((0.014, 0.200, 0.095)),
        origin=Origin(xyz=(-0.143, 0.0, 0.0535)),
        material=warm_wood,
        name="side_wall_0",
    )
    body.visual(
        Box((0.014, 0.200, 0.095)),
        origin=Origin(xyz=(0.143, 0.0, 0.0535)),
        material=warm_wood,
        name="side_wall_1",
    )
    body.visual(
        Box((0.260, 0.160, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0134)),
        material=felt,
        name="felt_liner",
    )

    # Fixed hinge leaves are pulled behind the rear wall so the horizontal pin
    # axis reads clearly instead of being buried in the box corner.
    for index, x in enumerate((-0.104, 0.0, 0.104)):
        body.visual(
            Box((0.046, 0.003, 0.030)),
            origin=Origin(xyz=(x, 0.1015, 0.088)),
            material=brass,
            name=f"fixed_leaf_{index}",
        )
        body.visual(
            Box((0.042, 0.008, 0.005)),
            origin=Origin(xyz=(x, 0.1055, 0.104)),
            material=brass,
            name=f"fixed_leaf_bridge_{index}",
        )
        body.visual(
            Cylinder(radius=0.0055, length=0.042),
            origin=Origin(xyz=(x, 0.112, 0.108), rpy=(0.0, pi / 2.0, 0.0)),
            material=brass,
            name=f"fixed_knuckle_{index}",
        )

    lid = model.part("lid")
    # The top is deliberately a single plain slab; all visual complexity is kept
    # at the exposed rear hinge.
    lid.visual(
        Box((0.314, 0.195, 0.014)),
        origin=Origin(xyz=(0.0, -0.1175, 0.0)),
        material=lid_wood,
        name="lid_panel",
    )
    # A small inset underside lip helps the lid read as fitted over a hollow box.
    lid.visual(
        Box((0.250, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.190, -0.012)),
        material=lid_wood,
        name="front_lip",
    )
    lid.visual(
        Box((0.250, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.047, -0.012)),
        material=lid_wood,
        name="rear_lip",
    )
    lid.visual(
        Box((0.010, 0.150, 0.012)),
        origin=Origin(xyz=(-0.125, -0.1225, -0.012)),
        material=lid_wood,
        name="side_lip_0",
    )
    lid.visual(
        Box((0.010, 0.150, 0.012)),
        origin=Origin(xyz=(0.125, -0.1225, -0.012)),
        material=lid_wood,
        name="side_lip_1",
    )
    for index, x in enumerate((-0.052, 0.052)):
        lid.visual(
            Box((0.042, 0.026, 0.004)),
            origin=Origin(xyz=(x, -0.011, -0.002)),
            material=brass,
            name=f"moving_leaf_{index}",
        )
        lid.visual(
            Cylinder(radius=0.0055, length=0.042),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=brass,
            name=f"moving_knuckle_{index}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.112, 0.108)),
        # The closed panel extends in local -Y from the rear pin.  Around -X,
        # positive rotation lifts the front edge upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.85),
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
        negative_elem="front_wall",
        max_gap=0.001,
        max_penetration=0.0001,
        name="closed plain lid rests on the box body",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="x",
        elem_a="lid_panel",
        elem_b="front_wall",
        min_overlap=0.25,
        name="lid spans the box width",
    )

    rear_wall_aabb = ctx.part_element_world_aabb(body, elem="rear_wall")
    moving_knuckle_aabb = ctx.part_element_world_aabb(lid, elem="moving_knuckle_0")
    ctx.check(
        "hinge hardware is pulled behind the rear wall",
        rear_wall_aabb is not None
        and moving_knuckle_aabb is not None
        and moving_knuckle_aabb[0][1] > rear_wall_aabb[1][1] + 0.004,
        details=f"rear_wall={rear_wall_aabb}, moving_knuckle={moving_knuckle_aabb}",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.35}):
        open_panel_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "positive hinge motion opens the lid upward",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.10,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="small_sewing_box")

    wood = model.material("warm_wood", color=(0.72, 0.46, 0.25, 1.0))
    liner = model.material("muted_cloth_liner", color=(0.35, 0.18, 0.23, 1.0))
    brass = model.material("brushed_brass", color=(0.86, 0.64, 0.28, 1.0))
    dark = model.material("shadow_gap", color=(0.08, 0.055, 0.035, 1.0))

    # A shallow, deep, hand-sized sewing box in meters.
    width = 0.34
    depth = 0.22
    body_h = 0.105
    wall_t = 0.012
    floor_t = 0.012
    lid_t = 0.018
    overhang = 0.008
    hinge_r = 0.007
    hinge_y = depth / 2.0 + hinge_r
    hinge_z = body_h + lid_t / 2.0

    body = model.part("body")

    # Open rectangular body: a real floor and four wall panels, not a solid block.
    body.visual(
        Box((width, depth, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t / 2.0)),
        material=wood,
        name="floor_panel",
    )
    body.visual(
        Box((width, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall_t / 2.0, body_h / 2.0)),
        material=wood,
        name="front_wall",
    )
    body.visual(
        Box((width, wall_t, body_h)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall_t / 2.0, body_h / 2.0)),
        material=wood,
        name="rear_wall",
    )
    body.visual(
        Box((wall_t, depth, body_h)),
        origin=Origin(xyz=(-width / 2.0 + wall_t / 2.0, 0.0, body_h / 2.0)),
        material=wood,
        name="side_wall_0",
    )
    body.visual(
        Box((wall_t, depth, body_h)),
        origin=Origin(xyz=(width / 2.0 - wall_t / 2.0, 0.0, body_h / 2.0)),
        material=wood,
        name="side_wall_1",
    )

    # Dark cloth-like liner visible when the lid is opened.
    body.visual(
        Box((width - 2.0 * wall_t, depth - 2.0 * wall_t, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, floor_t + 0.0015)),
        material=liner,
        name="bottom_liner",
    )

    # A small front catch makes the object read as a sewing box without adding
    # another moving control.
    body.visual(
        Box((0.060, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.0015, body_h * 0.62)),
        material=brass,
        name="front_catch",
    )
    body.visual(
        Box((0.28, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.001, body_h - 0.024)),
        material=dark,
        name="front_shadow_line",
    )

    # Interleaved exposed hinge knuckles.  The child knuckles use the same hinge
    # axis but occupy the gaps, so the single revolute joint is mechanically
    # legible without broad part overlaps.
    body_knuckles = [
        (-0.138, -0.090),
        (-0.022, 0.022),
        (0.090, 0.138),
    ]
    lid_knuckles = [
        (-0.082, -0.030),
        (0.030, 0.082),
    ]
    for idx, (x0, x1) in enumerate(body_knuckles):
        seg_len = x1 - x0
        x_mid = (x0 + x1) / 2.0
        body.visual(
            Box((seg_len, 0.008, 0.028)),
            origin=Origin(xyz=(x_mid, depth / 2.0 + 0.004, body_h + 0.003)),
            material=brass,
            name=f"hinge_leaf_{idx}",
        )
        body.visual(
            Cylinder(radius=hinge_r, length=seg_len),
            origin=Origin(xyz=(x_mid, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=brass,
            name=f"hinge_knuckle_{idx}",
        )

    lid = model.part("lid")

    lid_depth = depth + 0.005
    panel_y = -(hinge_r + 0.004 + lid_depth / 2.0)
    panel_w = width + 2.0 * overhang

    lid.visual(
        Box((panel_w, lid_depth, lid_t)),
        origin=Origin(xyz=(0.0, panel_y, 0.0)),
        material=wood,
        name="lid_panel",
    )
    # A narrow inner lip overlaps the panel itself and hangs just inside the box
    # walls, showing the closed lid registers into the hollow body.
    lid.visual(
        Box((width - 0.028, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, panel_y - lid_depth / 2.0 + 0.032, -0.010)),
        material=wood,
        name="front_lip",
    )
    lid.visual(
        Box((0.050, 0.003, 0.014)),
        origin=Origin(xyz=(0.0, panel_y - lid_depth / 2.0 - 0.0015, -0.001)),
        material=brass,
        name="latch_leaf",
    )

    for idx, (x0, x1) in enumerate(lid_knuckles):
        seg_len = x1 - x0
        x_mid = (x0 + x1) / 2.0
        lid.visual(
            Box((seg_len, 0.014, 0.004)),
            origin=Origin(xyz=(x_mid, -0.010, 0.006)),
            material=brass,
            name=f"lid_hinge_leaf_{idx}",
        )
        lid.visual(
            Cylinder(radius=hinge_r, length=seg_len),
            origin=Origin(xyz=(x_mid, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=brass,
            name=f"lid_knuckle_{idx}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        # The closed lid panel extends forward along local -Y, so -X makes
        # positive joint motion lift the free/front edge upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.check(
        "single rear lid hinge",
        len(object_model.articulations) == 1
        and hinge.parent == body.name
        and hinge.child == lid.name
        and tuple(hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"articulations={object_model.articulations}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_wall",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed lid rests on body walls",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="front_wall",
            min_overlap=0.010,
            name="plain lid covers the box depth",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="y",
            positive_elem="front_lip",
            negative_elem="front_wall",
            min_gap=0.002,
            name="inner lid lip clears the front wall",
        )
        closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    with ctx.pose({hinge: 1.35}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_wall",
            min_gap=0.010,
            name="opened lid clears the front wall",
        )
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "revolute motion lifts the short deep lid",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.10,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

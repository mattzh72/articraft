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
    model = ArticulatedObject(name="nema_4x_stainless_cabinet")

    stainless = Material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    darker_steel = Material("shadowed_stainless", rgba=(0.48, 0.50, 0.49, 1.0))
    gasket = Material("black_neoprene_gasket", rgba=(0.015, 0.014, 0.013, 1.0))
    seal_shadow = Material("dark_recess", rgba=(0.08, 0.085, 0.085, 1.0))

    body = model.part("body")
    door = model.part("door")
    latch = model.part("latch")

    # Cabinet proportions: a wall-mounted NEMA 4X enclosure, 600 x 800 x 250 mm.
    width = 0.60
    height = 0.80
    depth = 0.25
    wall = 0.018

    # Hollow rectangular stainless body: five sheet-metal walls with an open front.
    body.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, depth - wall / 2.0, 0.0)),
        material=stainless,
        name="back_wall",
    )
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, depth / 2.0, 0.0)),
        material=stainless,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, depth / 2.0, 0.0)),
        material=stainless,
        name="side_wall_1",
    )
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, depth / 2.0, height / 2.0 - wall / 2.0)),
        material=stainless,
        name="top_wall",
    )
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, depth / 2.0, -height / 2.0 + wall / 2.0)),
        material=stainless,
        name="bottom_wall",
    )

    # Raised front gutter/flange that the door gasket compresses against.
    rim_depth = 0.020
    rim_w = 0.035
    body.visual(
        Box((rim_w, rim_depth, height + 0.070)),
        origin=Origin(xyz=(-width / 2.0 - rim_w / 2.0 + 0.002, -rim_depth / 2.0, 0.0)),
        material=stainless,
        name="front_rim_0",
    )
    body.visual(
        Box((rim_w, rim_depth, height + 0.070)),
        origin=Origin(xyz=(width / 2.0 + rim_w / 2.0 - 0.002, -rim_depth / 2.0, 0.0)),
        material=stainless,
        name="front_rim_1",
    )
    body.visual(
        Box((width + 0.070, rim_depth, rim_w)),
        origin=Origin(xyz=(0.0, -rim_depth / 2.0, height / 2.0 + rim_w / 2.0 - 0.002)),
        material=stainless,
        name="front_rim_2",
    )
    body.visual(
        Box((width + 0.070, rim_depth, rim_w)),
        origin=Origin(xyz=(0.0, -rim_depth / 2.0, -height / 2.0 - rim_w / 2.0 + 0.002)),
        material=stainless,
        name="front_rim_3",
    )
    body.visual(
        Box((width - 0.070, 0.004, height - 0.070)),
        origin=Origin(xyz=(0.0, depth - wall - 0.001, 0.0)),
        material=seal_shadow,
        name="interior_back_shadow",
    )

    # Door frame.  The door part frame is the hinge pin line; the panel extends in +X.
    hinge_x = -0.345
    hinge_y = -0.040
    door_w = 0.660
    door_h = 0.860
    door_t = 0.028
    door_center_x = 0.340

    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_center_x, 0.0, 0.0)),
        material=stainless,
        name="door_skin",
    )
    # Folded/raised perimeter lip on the door front.
    lip_depth = 0.007
    lip_w = 0.024
    door.visual(
        Box((door_w, lip_depth, lip_w)),
        origin=Origin(xyz=(door_center_x, -door_t / 2.0 - lip_depth / 2.0 + 0.001, door_h / 2.0 - lip_w / 2.0)),
        material=darker_steel,
        name="door_top_lip",
    )
    door.visual(
        Box((door_w, lip_depth, lip_w)),
        origin=Origin(xyz=(door_center_x, -door_t / 2.0 - lip_depth / 2.0 + 0.001, -door_h / 2.0 + lip_w / 2.0)),
        material=darker_steel,
        name="door_bottom_lip",
    )
    door.visual(
        Box((lip_w, lip_depth, door_h)),
        origin=Origin(xyz=(0.010 + lip_w / 2.0, -door_t / 2.0 - lip_depth / 2.0 + 0.001, 0.0)),
        material=darker_steel,
        name="door_hinge_lip",
    )
    door.visual(
        Box((lip_w, lip_depth, door_h)),
        origin=Origin(xyz=(0.010 + door_w - lip_w / 2.0, -door_t / 2.0 - lip_depth / 2.0 + 0.001, 0.0)),
        material=darker_steel,
        name="door_latch_lip",
    )

    # Continuous rectangular gasket on the back of the door.
    gasket_y = door_t / 2.0 + 0.0025
    gasket_t = 0.006
    gasket_w = 0.022
    gasket_h = 0.760
    gasket_x_left = 0.035
    gasket_x_right = 0.655
    door.visual(
        Box((gasket_w, gasket_t, gasket_h)),
        origin=Origin(xyz=(gasket_x_left, gasket_y, 0.0)),
        material=gasket,
        name="gasket_0",
    )
    door.visual(
        Box((gasket_w, gasket_t, gasket_h)),
        origin=Origin(xyz=(gasket_x_right, gasket_y, 0.0)),
        material=gasket,
        name="gasket_1",
    )
    door.visual(
        Box((gasket_x_right - gasket_x_left + gasket_w, gasket_t, gasket_w)),
        origin=Origin(xyz=((gasket_x_left + gasket_x_right) / 2.0, gasket_y, gasket_h / 2.0)),
        material=gasket,
        name="gasket_2",
    )
    door.visual(
        Box((gasket_x_right - gasket_x_left + gasket_w, gasket_t, gasket_w)),
        origin=Origin(xyz=((gasket_x_left + gasket_x_right) / 2.0, gasket_y, -gasket_h / 2.0)),
        material=gasket,
        name="gasket_3",
    )

    # Three exposed stainless revolute barrel hinges on the left edge.  Body
    # leaves/standoffs are in the body part; door knuckles are in the door part.
    hinge_radius = 0.010
    hinge_len = 0.130
    hinge_centers = (-0.265, 0.0, 0.265)
    for i, zc in enumerate(hinge_centers):
        # Cabinet-side strap and weld block, kept to the left/outside of the door skin.
        for suffix, zoff in (("top", 0.041), ("bottom", -0.041)):
            body.visual(
                Box((0.006, 0.066, 0.042)),
                origin=Origin(xyz=(hinge_x - 0.002, -0.020, zc + zoff)),
                material=stainless,
                name=f"hinge_side_leaf_{suffix}_{i}",
            )
        body.visual(
            Box((0.012, 0.024, hinge_len)),
            origin=Origin(xyz=(hinge_x + 0.006, -0.012, zc)),
            material=stainless,
            name=f"hinge_standoff_{i}",
        )
        # Alternating body knuckles: upper and lower segments of each hinge.
        body.visual(
            Cylinder(radius=hinge_radius, length=0.042),
            origin=Origin(xyz=(hinge_x, hinge_y, zc + 0.041), rpy=(0.0, 0.0, 0.0)),
            material=stainless,
            name=f"body_knuckle_top_{i}",
        )
        body.visual(
            Cylinder(radius=hinge_radius, length=0.042),
            origin=Origin(xyz=(hinge_x, hinge_y, zc - 0.041), rpy=(0.0, 0.0, 0.0)),
            material=stainless,
            name=f"body_knuckle_bottom_{i}",
        )
        body.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(hinge_x, hinge_y, zc + hinge_len / 2.0 - 0.001), rpy=(0.0, 0.0, 0.0)),
            material=darker_steel,
            name=f"hinge_pin_cap_{i}",
        )

        door.visual(
            Box((0.060, 0.006, hinge_len)),
            origin=Origin(xyz=(0.035, -door_t / 2.0 - 0.006, zc)),
            material=stainless,
            name=f"door_hinge_leaf_{i}",
        )
        door.visual(
            Cylinder(radius=hinge_radius, length=0.042),
            origin=Origin(xyz=(0.0, 0.0, zc), rpy=(0.0, 0.0, 0.0)),
            material=stainless,
            name=f"door_knuckle_{i}",
        )

    # Articulated quarter-turn latch on the right side of the door.
    latch_x = 0.575
    latch_z = 0.025
    latch_y = -door_t / 2.0 - 0.006
    latch.visual(
        Cylinder(radius=0.029, length=0.010),
        origin=Origin(xyz=(0.0, 0.001, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="escutcheon",
    )
    latch.visual(
        Box((0.078, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.008, 0.0)),
        material=darker_steel,
        name="turn_wing",
    )
    latch.visual(
        Cylinder(radius=0.008, length=0.038),
        origin=Origin(xyz=(0.0, 0.017, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=darker_steel,
        name="shaft",
    )
    latch.visual(
        Box((0.026, 0.008, 0.120)),
        origin=Origin(xyz=(0.0, 0.039, 0.0)),
        material=darker_steel,
        name="cam_bar",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=0.0, upper=2.1),
    )
    model.articulation(
        "latch_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(latch_x, latch_y, latch_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_turn = object_model.get_articulation("latch_turn")

    ctx.allow_overlap(
        door,
        latch,
        elem_a="door_skin",
        elem_b="shaft",
        reason="The quarter-turn latch shaft intentionally passes through the stainless door skin.",
    )
    ctx.expect_within(
        latch,
        door,
        axes="xz",
        inner_elem="shaft",
        outer_elem="door_skin",
        margin=0.002,
        name="latch shaft is centered within the door skin footprint",
    )
    ctx.expect_overlap(
        latch,
        door,
        axes="y",
        elem_a="shaft",
        elem_b="door_skin",
        min_overlap=0.020,
        name="latch shaft penetrates the door skin",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            min_gap=0.0002,
            max_gap=0.010,
            positive_elem="front_rim_1",
            negative_elem="gasket_1",
            name="closed gasket seats just in front of the rim",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="gasket_1",
            elem_b="front_rim_1",
            min_overlap=0.015,
            name="right gasket overlaps the cabinet rim footprint",
        )

    closed_latch_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({latch_turn: pi / 2.0}):
        turned_latch_aabb = ctx.part_world_aabb(latch)
    ctx.check(
        "quarter turn changes latch silhouette",
        closed_latch_aabb is not None
        and turned_latch_aabb is not None
        and abs((turned_latch_aabb[1][2] - turned_latch_aabb[0][2]) - (closed_latch_aabb[1][2] - closed_latch_aabb[0][2])) > 0.030,
        details=f"closed={closed_latch_aabb}, turned={turned_latch_aabb}",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.4}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens outward from the cabinet front",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.15,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

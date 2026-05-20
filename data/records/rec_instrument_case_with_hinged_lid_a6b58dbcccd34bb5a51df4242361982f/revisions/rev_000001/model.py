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
    model = ArticulatedObject(name="pedalboard_case")

    black_shell = Material("black_powder_coated_shell", rgba=(0.015, 0.014, 0.013, 1.0))
    carpet = Material("charcoal_loop_carpet", rgba=(0.045, 0.047, 0.045, 1.0))
    aluminum = Material("brushed_aluminum_hardware", rgba=(0.70, 0.68, 0.62, 1.0))
    dark_steel = Material("dark_steel_pivots", rgba=(0.12, 0.12, 0.12, 1.0))
    rubber = Material("matte_rubber_feet", rgba=(0.004, 0.004, 0.004, 1.0))

    # Real pedalboard cases are wide and shallow.  X is width, Y runs from the
    # front latch edge (-Y) to the rear hinge edge (+Y), and Z is vertical.
    width = 0.68
    depth = 0.38
    wall = 0.025
    bottom_t = 0.025
    wall_h = 0.065
    base_top = bottom_t + wall_h
    rim_t = 0.006
    rim_top = base_top + rim_t
    front_y = -depth / 2.0
    rear_y = depth / 2.0

    base = model.part("base")
    base.visual(
        Box((width, depth, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=black_shell,
        name="floor_pan",
    )
    base.visual(
        Box((width, wall, wall_h)),
        origin=Origin(xyz=(0.0, front_y + wall / 2.0, bottom_t + wall_h / 2.0)),
        material=black_shell,
        name="front_wall",
    )
    base.visual(
        Box((width, wall, wall_h)),
        origin=Origin(xyz=(0.0, rear_y - wall / 2.0, bottom_t + wall_h / 2.0)),
        material=black_shell,
        name="rear_wall",
    )
    for side, x in enumerate((-width / 2.0 + wall / 2.0, width / 2.0 - wall / 2.0)):
        base.visual(
            Box((wall, depth, wall_h)),
            origin=Origin(xyz=(x, 0.0, bottom_t + wall_h / 2.0)),
            material=black_shell,
            name=f"side_wall_{side}",
        )

    base.visual(
        Box((width - 2.0 * wall - 0.014, depth - 2.0 * wall - 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t + 0.003)),
        material=carpet,
        name="equipment_carpet",
    )
    base.visual(
        Box((width, wall, rim_t)),
        origin=Origin(xyz=(0.0, front_y + wall / 2.0, base_top + rim_t / 2.0)),
        material=aluminum,
        name="front_rim",
    )
    base.visual(
        Box((width, wall, rim_t)),
        origin=Origin(xyz=(0.0, rear_y - wall / 2.0, base_top + rim_t / 2.0)),
        material=aluminum,
        name="rear_rim",
    )
    for side, x in enumerate((-width / 2.0 + wall / 2.0, width / 2.0 - wall / 2.0)):
        base.visual(
            Box((wall, depth, rim_t)),
            origin=Origin(xyz=(x, 0.0, base_top + rim_t / 2.0)),
            material=aluminum,
            name=f"side_rim_{side}",
        )

    # Rubber feet are molded into the underside and touch the floor pan.
    for i, x in enumerate((-0.27, 0.27)):
        for j, y in enumerate((-0.145, 0.145)):
            base.visual(
                Cylinder(radius=0.020, length=0.010),
                origin=Origin(xyz=(x, y, -0.005)),
                material=rubber,
                name=f"foot_{i}_{j}",
            )

    # Rear hinge: alternating exposed knuckles clipped to a common horizontal
    # axis.  The physical revolute joint below uses this same line.
    hinge_y = rear_y + 0.012
    hinge_z = rim_top + 0.015
    base.visual(
        Box((width, 0.007, 0.026)),
        origin=Origin(xyz=(0.0, rear_y - 0.0035, rim_top + 0.004)),
        material=aluminum,
        name="rear_hinge_leaf",
    )
    for idx, (x, seg_len) in enumerate(((-0.265, 0.13), (0.0, 0.12), (0.265, 0.13))):
        base.visual(
            Cylinder(radius=0.012, length=seg_len),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=aluminum,
            name=f"base_hinge_knuckle_{idx}",
        )

    # Two latch backing plates, pivot pins, and keeper nubs on the lower tray.
    latch_xs = (-0.20, 0.20)
    latch_axis_y = front_y - 0.030
    latch_axis_z = rim_top + 0.010
    for idx, x in enumerate(latch_xs):
        base.visual(
            Box((0.120, 0.008, 0.052)),
            origin=Origin(xyz=(x, front_y - 0.004, latch_axis_z - 0.020)),
            material=aluminum,
            name=f"latch_plate_{idx}",
        )
        base.visual(
            Cylinder(radius=0.007, length=0.016),
            origin=Origin(xyz=(x, front_y - 0.016, latch_axis_z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"latch_pin_{idx}",
        )
        base.visual(
            Box((0.075, 0.006, 0.012)),
            origin=Origin(xyz=(x, front_y - 0.004, rim_top + 0.018)),
            material=aluminum,
            name=f"lower_keeper_{idx}",
        )

    lid = model.part("lid")
    lid_depth = 0.370
    lid_front_local = -0.390
    lid_rear_local = -0.020
    lid_center_y = (lid_front_local + lid_rear_local) / 2.0
    lid_t = 0.025
    # At q=0 the underside of this broad cover lies on the aluminum rim.
    lid_center_z = (rim_top - hinge_z) + lid_t / 2.0
    lid.visual(
        Box((width + 0.020, lid_depth, lid_t)),
        origin=Origin(xyz=(0.0, lid_center_y, lid_center_z)),
        material=black_shell,
        name="lid_panel",
    )
    lid.visual(
        Box((width + 0.035, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, lid_front_local - 0.006, lid_center_z + lid_t / 2.0 + 0.003)),
        material=aluminum,
        name="front_edge_band",
    )
    for side, x in enumerate((-(width + 0.020) / 2.0, (width + 0.020) / 2.0)):
        lid.visual(
            Box((0.012, lid_depth, 0.014)),
            origin=Origin(xyz=(x, lid_center_y, lid_center_z - 0.001)),
            material=aluminum,
            name=f"side_edge_band_{side}",
        )

    # Lid-side hinge knuckles occupy the spaces between the base knuckles.
    for idx, (x, seg_len) in enumerate(((-0.132, 0.115), (0.132, 0.115))):
        lid.visual(
            Cylinder(radius=0.012, length=seg_len),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=aluminum,
            name=f"lid_hinge_knuckle_{idx}",
        )
        lid.visual(
            Box((seg_len, 0.030, 0.010)),
            origin=Origin(xyz=(x, -0.015, 0.013)),
            material=aluminum,
            name=f"lid_hinge_tab_{idx}",
        )

    for idx, x in enumerate(latch_xs):
        lid.visual(
            Box((0.085, 0.012, 0.012)),
            origin=Origin(xyz=(x, lid_front_local + 0.010, lid_center_z + lid_t / 2.0 + 0.006)),
            material=aluminum,
            name=f"upper_keeper_{idx}",
        )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        # The closed lid extends along local -Y from the hinge line; -X makes
        # positive motion lift the front edge upward instead of into the tray.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.85),
    )

    for idx, x in enumerate(latch_xs):
        latch = model.part(f"latch_{idx}")
        latch.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="pivot_hub",
        )
        latch.visual(
            Box((0.050, 0.008, 0.022)),
            origin=Origin(xyz=(-0.033, 0.0, 0.0)),
            material=aluminum,
            name="wing_0",
        )
        latch.visual(
            Box((0.050, 0.008, 0.022)),
            origin=Origin(xyz=(0.033, 0.0, 0.0)),
            material=aluminum,
            name="wing_1",
        )
        latch.visual(
            Box((0.014, 0.010, 0.052)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=aluminum,
            name="center_spine",
        )
        model.articulation(
            f"latch_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=latch,
            origin=Origin(xyz=(x, latch_axis_y, latch_axis_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=5.0, lower=-1.5708, upper=1.5708),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    rear_hinge = object_model.get_articulation("rear_hinge")
    latch_0 = object_model.get_part("latch_0")
    latch_joint_0 = object_model.get_articulation("latch_pivot_0")

    with ctx.pose({rear_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_rim",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed lid seats on front rim",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="lid_panel",
            elem_b="equipment_carpet",
            min_overlap=0.25,
            name="lid spans equipment opening",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({rear_hinge: 1.20}):
        raised_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear hinge lifts front cover",
        closed_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > closed_aabb[1][2] + 0.12,
        details=f"closed={closed_aabb}, raised={raised_aabb}",
    )

    locked_aabb = ctx.part_world_aabb(latch_0)
    with ctx.pose({latch_joint_0: 1.5708}):
        turned_aabb = ctx.part_world_aabb(latch_0)
    ctx.check(
        "butterfly latch rotates on front pivot",
        locked_aabb is not None
        and turned_aabb is not None
        and (turned_aabb[1][2] - turned_aabb[0][2]) > (locked_aabb[1][2] - locked_aabb[0][2]) + 0.030,
        details=f"locked={locked_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

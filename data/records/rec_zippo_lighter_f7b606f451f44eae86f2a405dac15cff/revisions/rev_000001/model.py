from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brass_lift_out_lighter")

    # A small straight-sided petrol lighter, dimensioned close to a pocket
    # lighter: 38 mm wide, 14 mm deep, and about 74 mm tall when closed.
    width = 0.038
    depth = 0.014
    lower_h = 0.052
    lid_h = 0.022
    wall = 0.0012
    hinge_x = -width / 2.0 - 0.0030

    brass = model.material("worn_brass", rgba=(0.78, 0.56, 0.24, 1.0))
    dark_brass = model.material("darkened_brass", rgba=(0.38, 0.25, 0.10, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.12, 0.12, 1.0))
    wick_material = model.material("charred_wick", rgba=(0.05, 0.045, 0.035, 1.0))

    lower_case = model.part("lower_case")
    # Hollow lower shell: four straight brass walls and a bottom plate.
    lower_case.visual(
        Box((width, wall, lower_h)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, lower_h / 2.0)),
        material=brass,
        name="front_wall",
    )
    lower_case.visual(
        Box((width, wall, lower_h)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, lower_h / 2.0)),
        material=brass,
        name="back_wall",
    )
    lower_case.visual(
        Box((wall, depth, lower_h)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, lower_h / 2.0)),
        material=brass,
        name="hinge_side_wall",
    )
    lower_case.visual(
        Box((wall, depth, lower_h)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, lower_h / 2.0)),
        material=brass,
        name="free_side_wall",
    )
    lower_case.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=brass,
        name="bottom_plate",
    )
    lower_case.visual(
        Box((width + 0.0004, 0.0006, 0.0014)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.00015, lower_h - 0.0020)),
        material=dark_brass,
        name="front_seam_band",
    )
    lower_case.visual(
        Box((width + 0.0004, 0.0006, 0.0014)),
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.00015, lower_h - 0.0020)),
        material=dark_brass,
        name="back_seam_band",
    )

    # Side hinge hardware fixed to the lower case.  The knuckles are interleaved
    # with the lid knuckles, so they are close without occupying the same height.
    lower_case.visual(
        Box((0.0010, 0.0010, 0.031)),
        origin=Origin(xyz=(hinge_x, -0.0030, lower_h + lid_h / 2.0 - 0.0005)),
        material=brass,
        name="hinge_spine",
    )
    lower_case.visual(
        Box((0.0032, 0.0010, 0.0080)),
        origin=Origin(xyz=(hinge_x + 0.0016, -0.0030, lower_h - 0.0040)),
        material=brass,
        name="hinge_mount",
    )
    for idx, zc in enumerate((lower_h + 0.0036, lower_h + 0.0184)):
        lower_case.visual(
            Cylinder(radius=0.00125, length=0.0062),
            origin=Origin(xyz=(hinge_x, 0.0, zc)),
            material=brass,
            name=f"case_hinge_knuckle_{idx}",
        )
        lower_case.visual(
            Box((0.0010, 0.0042, 0.0062)),
            origin=Origin(xyz=(hinge_x, -0.0018, zc)),
            material=brass,
            name=f"case_hinge_leaf_{idx}",
        )

    lid = model.part("lid")
    lid_center_x = -hinge_x
    lid.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(lid_center_x, 0.0, lid_h - wall / 2.0)),
        material=brass,
        name="lid_top",
    )
    lid.visual(
        Box((width, wall, lid_h)),
        origin=Origin(xyz=(lid_center_x, -depth / 2.0 + wall / 2.0, lid_h / 2.0)),
        material=brass,
        name="lid_front_wall",
    )
    lid.visual(
        Box((width, wall, lid_h)),
        origin=Origin(xyz=(lid_center_x, depth / 2.0 - wall / 2.0, lid_h / 2.0)),
        material=brass,
        name="lid_back_wall",
    )
    lid.visual(
        Box((wall, depth, lid_h)),
        origin=Origin(xyz=(lid_center_x + width / 2.0 - wall / 2.0, 0.0, lid_h / 2.0)),
        material=brass,
        name="lid_free_wall",
    )
    lid.visual(
        Box((wall, depth, lid_h)),
        origin=Origin(xyz=(lid_center_x - width / 2.0 + wall / 2.0, 0.0, lid_h / 2.0)),
        material=brass,
        name="lid_hinge_wall",
    )
    lid.visual(
        Box((0.0032, 0.0010, 0.0064)),
        origin=Origin(xyz=(0.0016, 0.00175, 0.0110)),
        material=brass,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.00125, length=0.0064),
        origin=Origin(xyz=(0.0, 0.0, 0.0110)),
        material=brass,
        name="lid_hinge_knuckle",
    )

    insert = model.part("insert")
    insert.visual(
        Box((0.0320, 0.0100, 0.0420)),
        origin=Origin(xyz=(0.0, 0.0, 0.0210)),
        material=brushed_steel,
        name="lower_body",
    )
    insert.visual(
        Box((0.0340, 0.0110, 0.0030)),
        origin=Origin(xyz=(0.0, 0.0, 0.0435)),
        material=brushed_steel,
        name="top_collar",
    )

    chimney_panel = PerforatedPanelGeometry(
        (0.020, 0.020),
        0.00055,
        hole_diameter=0.0024,
        pitch=(0.0050, 0.0050),
        frame=0.0018,
        corner_radius=0.0008,
        stagger=True,
    )
    chimney_mesh = mesh_from_geometry(chimney_panel, "chimney_vent_panel")
    chimney_x = -0.0040
    chimney_z = 0.0550
    for name, y in (("chimney_front", -0.0049), ("chimney_back", 0.0049)):
        insert.visual(
            chimney_mesh,
            origin=Origin(xyz=(chimney_x, y, chimney_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=name,
        )
    insert.visual(
        Box((0.0008, 0.0098, 0.0210)),
        origin=Origin(xyz=(chimney_x - 0.0104, 0.0, chimney_z)),
        material=brushed_steel,
        name="chimney_side_0",
    )
    insert.visual(
        Box((0.0008, 0.0098, 0.0210)),
        origin=Origin(xyz=(chimney_x + 0.0104, 0.0, chimney_z)),
        material=brushed_steel,
        name="chimney_side_1",
    )
    insert.visual(
        Box((0.0206, 0.0100, 0.0012)),
        origin=Origin(xyz=(chimney_x, 0.0, 0.0450)),
        material=brushed_steel,
        name="chimney_seat_band",
    )
    insert.visual(
        Cylinder(radius=0.0011, length=0.0100),
        origin=Origin(xyz=(chimney_x - 0.0020, 0.0, 0.0495)),
        material=wick_material,
        name="wick",
    )

    wheel_x = 0.0115
    wheel_z = 0.0580
    for idx, y in enumerate((-0.00225, 0.00225)):
        insert.visual(
            Box((0.0065, 0.00065, 0.0150)),
            origin=Origin(xyz=(wheel_x, y, 0.0525)),
            material=brushed_steel,
            name=f"wheel_fork_{idx}",
        )
    insert.visual(
        Cylinder(radius=0.0012, length=0.0170),
        origin=Origin(xyz=(0.0165, 0.0, 0.0500)),
        material=dark_steel,
        name="flint_tube",
    )

    striker_wheel = model.part("striker_wheel")
    striker_wheel.visual(
        Cylinder(radius=0.0038, length=0.0030),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_disk",
    )
    for i in range(14):
        angle = 2.0 * math.pi * i / 14.0
        striker_wheel.visual(
            Box((0.00065, 0.0033, 0.00135)),
            origin=Origin(
                xyz=(0.00405 * math.cos(angle), 0.0, 0.00405 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=dark_steel,
            name=f"wheel_tooth_{i}",
        )

    model.articulation(
        "case_to_insert",
        ArticulationType.PRISMATIC,
        parent=lower_case,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, wall)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.10, lower=0.0, upper=0.035),
    )
    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_case,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, lower_h)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=0.0, upper=2.05),
    )
    model.articulation(
        "insert_to_striker",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=striker_wheel,
        origin=Origin(xyz=(wheel_x, 0.0, wheel_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=40.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_case = object_model.get_part("lower_case")
    lid = object_model.get_part("lid")
    insert = object_model.get_part("insert")
    striker_wheel = object_model.get_part("striker_wheel")
    lid_hinge = object_model.get_articulation("case_to_lid")
    insert_slide = object_model.get_articulation("case_to_insert")
    wheel_spin = object_model.get_articulation("insert_to_striker")

    ctx.expect_within(
        insert,
        lower_case,
        axes="xy",
        inner_elem="lower_body",
        margin=0.0,
        name="insert lower body fits within case width",
    )
    ctx.expect_overlap(
        insert,
        lower_case,
        axes="z",
        elem_a="lower_body",
        elem_b="front_wall",
        min_overlap=0.035,
        name="insert remains seated in lower shell",
    )
    ctx.expect_within(
        insert,
        lower_case,
        axes="xy",
        inner_elem="chimney_front",
        margin=0.0,
        name="vented chimney is within case opening footprint",
    )
    ctx.expect_overlap(
        insert,
        lower_case,
        axes="z",
        elem_a="chimney_front",
        elem_b="front_wall",
        min_overlap=0.004,
        name="chimney panel dips below upper case rim",
    )
    ctx.expect_gap(
        striker_wheel,
        insert,
        axis="y",
        positive_elem="wheel_disk",
        negative_elem="wheel_fork_0",
        min_gap=0.0001,
        max_gap=0.0010,
        name="striker wheel clears one fork cheek",
    )
    ctx.check(
        "striker axle is transverse",
        tuple(round(v, 4) for v in wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={wheel_spin.axis}",
    )

    closed_free_wall = ctx.part_element_world_aabb(lid, elem="lid_free_wall")
    with ctx.pose({lid_hinge: 2.0}):
        open_free_wall = ctx.part_element_world_aabb(lid, elem="lid_free_wall")
    ctx.check(
        "side hinge swings lid away from case",
        closed_free_wall is not None
        and open_free_wall is not None
        and open_free_wall[1][1] > closed_free_wall[1][1] + 0.020,
        details=f"closed={closed_free_wall}, open={open_free_wall}",
    )

    rest_insert_pos = ctx.part_world_position(insert)
    with ctx.pose({lid_hinge: 2.0, insert_slide: 0.035}):
        raised_insert_pos = ctx.part_world_position(insert)
        ctx.expect_overlap(
            insert,
            lower_case,
            axes="z",
            elem_a="lower_body",
            elem_b="front_wall",
            min_overlap=0.006,
            name="raised insert still has retained insertion",
        )
    ctx.check(
        "insert slides upward out of lower case",
        rest_insert_pos is not None
        and raised_insert_pos is not None
        and raised_insert_pos[2] > rest_insert_pos[2] + 0.030,
        details=f"rest={rest_insert_pos}, raised={raised_insert_pos}",
    )

    return ctx.report()


object_model = build_object_model()

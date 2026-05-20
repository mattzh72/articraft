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
    model = ArticulatedObject(name="clinical_foot_pedal_waste_bin")

    shell_mat = model.material("warm_clinical_white", rgba=(0.92, 0.94, 0.93, 1.0))
    rim_mat = model.material("pale_grey_rim", rgba=(0.74, 0.77, 0.76, 1.0))
    yellow_mat = model.material("clinical_yellow_lid", rgba=(0.98, 0.79, 0.12, 1.0))
    pedal_mat = model.material("textured_dark_pedal", rgba=(0.07, 0.08, 0.08, 1.0))
    rubber_mat = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    metal_mat = model.material("brushed_pin_metal", rgba=(0.58, 0.60, 0.60, 1.0))
    red_mat = model.material("clinical_warning_red", rgba=(0.82, 0.05, 0.04, 1.0))
    white_label_mat = model.material("label_white", rgba=(1.0, 1.0, 0.96, 1.0))
    damper_mat = model.material("soft_close_grey", rgba=(0.52, 0.57, 0.59, 1.0))

    width = 0.44
    depth = 0.36
    height = 0.72
    wall = 0.025
    rim_h = 0.025
    rim_z = height + rim_h / 2.0

    body = model.part("body")
    # The bin reads as a hollow rectangular receptacle: four tall walls, a base,
    # and a proud rim around the open top rather than a solid block.
    body.visual(
        Box((wall, width, height)),
        origin=Origin(xyz=(depth / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=shell_mat,
        name="front_wall",
    )
    body.visual(
        Box((wall, width, height)),
        origin=Origin(xyz=(-depth / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=shell_mat,
        name="rear_wall",
    )
    body.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall / 2.0, height / 2.0)),
        material=shell_mat,
        name="side_wall_0",
    )
    body.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall / 2.0, height / 2.0)),
        material=shell_mat,
        name="side_wall_1",
    )
    body.visual(
        Box((depth, width, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=shell_mat,
        name="floor_panel",
    )
    body.visual(
        Box((depth + 0.030, 0.035, rim_h)),
        origin=Origin(xyz=(0.0, width / 2.0 + 0.005, rim_z)),
        material=rim_mat,
        name="top_rim_0",
    )
    body.visual(
        Box((depth + 0.030, 0.035, rim_h)),
        origin=Origin(xyz=(0.0, -width / 2.0 - 0.005, rim_z)),
        material=rim_mat,
        name="top_rim_1",
    )
    body.visual(
        Box((0.035, width + 0.070, rim_h)),
        origin=Origin(xyz=(depth / 2.0 + 0.005, 0.0, rim_z)),
        material=rim_mat,
        name="top_rim_front",
    )
    body.visual(
        Box((0.035, width + 0.070, rim_h)),
        origin=Origin(xyz=(-depth / 2.0 - 0.005, 0.0, rim_z)),
        material=rim_mat,
        name="top_rim_rear",
    )

    # Clinical label panel and simple cross mark on the front face.
    body.visual(
        Box((0.006, 0.190, 0.150)),
        origin=Origin(xyz=(depth / 2.0 + 0.003, 0.0, 0.440)),
        material=red_mat,
        name="hazard_label",
    )
    body.visual(
        Box((0.008, 0.112, 0.020)),
        origin=Origin(xyz=(depth / 2.0 + 0.008, 0.0, 0.465)),
        material=white_label_mat,
        name="label_cross_bar",
    )
    body.visual(
        Box((0.008, 0.026, 0.110)),
        origin=Origin(xyz=(depth / 2.0 + 0.008, 0.0, 0.465)),
        material=white_label_mat,
        name="label_cross_stem",
    )

    # Rear hinge rail and soft-close damper hardware on the hinge side.
    body.visual(
        Cylinder(radius=0.012, length=0.400),
        origin=Origin(
            xyz=(-depth / 2.0 - 0.020, 0.0, height + 0.006),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal_mat,
        name="rear_hinge_rail",
    )
    body.visual(
        Box((0.055, 0.040, 0.040)),
        origin=Origin(xyz=(-depth / 2.0 - 0.014, -0.090, 0.640)),
        material=metal_mat,
        name="damper_mount",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.085),
        origin=Origin(
            xyz=(-depth / 2.0 - 0.055, -0.090, 0.640),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal_mat,
        name="damper_body",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.070),
        origin=Origin(
            xyz=(-depth / 2.0 - 0.020, -0.090, 0.640),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal_mat,
        name="damper_rod",
    )
    for idx, y in enumerate((0.066,)):
        body.visual(
            Box((0.060, 0.012, 0.015)),
            origin=Origin(xyz=(-depth / 2.0 - 0.030, y, 0.675)),
            material=metal_mat,
            name=f"cover_hinge_saddle_{idx}",
        )

    # Front pivot clevis for the independent foot pedal lever.
    pivot_x = depth / 2.0 + 0.020
    pivot_z = 0.105
    for idx, y in enumerate((-0.205, 0.205)):
        body.visual(
            Box((0.040, 0.030, 0.078)),
            origin=Origin(xyz=(pivot_x - 0.008, y, pivot_z)),
            material=rim_mat,
            name=f"pivot_tab_{idx}",
        )
    body.visual(
        Box((0.065, 0.030, 0.028)),
        origin=Origin(xyz=(pivot_x - 0.022, -0.205, pivot_z + 0.052)),
        material=rim_mat,
        name="pivot_tab_bridge_0",
    )
    body.visual(
        Box((0.065, 0.030, 0.028)),
        origin=Origin(xyz=(pivot_x - 0.022, 0.205, pivot_z + 0.052)),
        material=rim_mat,
        name="pivot_tab_bridge_1",
    )

    # Small non-moving rear feet keep the clinical bin stable on the floor.
    for idx, y in enumerate((-0.155, 0.155)):
        body.visual(
            Box((0.060, 0.070, 0.020)),
            origin=Origin(xyz=(-depth / 2.0 + 0.045, y, -0.010)),
            material=rubber_mat,
            name=f"rear_foot_{idx}",
        )

    lid = model.part("lid")
    # Child frame lies on the rear hinge line.  The flap panel extends along
    # local +X so positive rotation about -Y opens upward.
    lid.visual(
        Box((0.410, 0.480, 0.030)),
        origin=Origin(xyz=(0.205, 0.0, 0.015)),
        material=yellow_mat,
        name="lid_panel",
    )
    lid.visual(
        Box((0.036, 0.455, 0.048)),
        origin=Origin(xyz=(0.423, 0.0, -0.016)),
        material=yellow_mat,
        name="front_lip",
    )
    lid.visual(
        Box((0.290, 0.026, 0.040)),
        origin=Origin(xyz=(0.170, 0.252, -0.012)),
        material=yellow_mat,
        name="side_lip_0",
    )
    lid.visual(
        Box((0.290, 0.026, 0.040)),
        origin=Origin(xyz=(0.170, -0.252, -0.012)),
        material=yellow_mat,
        name="side_lip_1",
    )
    lid.visual(
        Box((0.235, 0.300, 0.010)),
        origin=Origin(xyz=(0.225, 0.0, 0.035)),
        material=model.material("subtle_lid_recess", rgba=(0.92, 0.70, 0.08, 1.0)),
        name="recessed_grip_panel",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yellow_mat,
        name="lid_hinge_barrel",
    )

    pedal = model.part("foot_pedal")
    pedal.visual(
        Cylinder(radius=0.010, length=0.440),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="pivot_axle",
    )
    pedal.visual(
        Box((0.260, 0.360, 0.024)),
        origin=Origin(xyz=(0.130, 0.0, -0.043)),
        material=pedal_mat,
        name="pedal_plate",
    )
    pedal.visual(
        Box((0.110, 0.085, 0.030)),
        origin=Origin(xyz=(0.055, -0.105, -0.020)),
        material=pedal_mat,
        name="lever_web_0",
    )
    pedal.visual(
        Box((0.110, 0.085, 0.030)),
        origin=Origin(xyz=(0.055, 0.105, -0.020)),
        material=pedal_mat,
        name="lever_web_1",
    )
    for idx, x in enumerate((0.035, 0.070, 0.105, 0.140, 0.175, 0.210)):
        pedal.visual(
            Box((0.012, 0.325, 0.007)),
            origin=Origin(xyz=(x, 0.0, -0.029)),
            material=rubber_mat,
            name=f"pedal_tread_{idx}",
        )

    damper_cover = model.part("damper_cover")
    # A separate U-shaped cap over the rear soft-close mechanism.  It is open
    # toward the bin, with side cheeks and a rear/top skin around the damper.
    damper_cover.visual(
        Cylinder(radius=0.0075, length=0.330),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=damper_mat,
        name="cover_hinge_barrel",
    )
    damper_cover.visual(
        Box((0.018, 0.250, 0.110)),
        origin=Origin(xyz=(-0.083, 0.0, -0.060)),
        material=damper_mat,
        name="cover_back",
    )
    damper_cover.visual(
        Box((0.098, 0.250, 0.018)),
        origin=Origin(xyz=(-0.045, 0.0, -0.008)),
        material=damper_mat,
        name="cover_top",
    )
    damper_cover.visual(
        Box((0.098, 0.018, 0.104)),
        origin=Origin(xyz=(-0.045, 0.134, -0.060)),
        material=damper_mat,
        name="cover_cheek_0",
    )
    damper_cover.visual(
        Box((0.098, 0.018, 0.104)),
        origin=Origin(xyz=(-0.045, -0.134, -0.060)),
        material=damper_mat,
        name="cover_cheek_1",
    )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-depth / 2.0 - 0.010, 0.0, height + rim_h)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "pedal_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=0.0, upper=0.45),
    )
    model.articulation(
        "damper_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=damper_cover,
        origin=Origin(xyz=(-depth / 2.0 - 0.030, -0.090, 0.690)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=0.0, upper=1.10),
    )

    lid_hinge.meta["description"] = "Rear horizontal hinge for the flap-style top lid."
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("foot_pedal")
    cover = object_model.get_part("damper_cover")
    lid_hinge = object_model.get_articulation("lid_hinge")
    pedal_pivot = object_model.get_articulation("pedal_pivot")
    cover_hinge = object_model.get_articulation("damper_cover_hinge")

    # The pedal axle is intentionally captured by the two front clevis tabs.
    for tab in ("pivot_tab_0", "pivot_tab_1"):
        ctx.allow_overlap(
            body,
            pedal,
            elem_a=tab,
            elem_b="pivot_axle",
            reason="The foot-pedal pivot axle is intentionally captured through the body clevis tab.",
        )
        ctx.expect_overlap(
            body,
            pedal,
            axes="yz",
            elem_a=tab,
            elem_b="pivot_axle",
            min_overlap=0.010,
            name=f"{tab} captures pedal axle",
        )

    with ctx.pose({lid_hinge: 0.0, pedal_pivot: 0.0, cover_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="top_rim_front",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed lid sits on front rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="top_rim_front",
            min_overlap=0.030,
            name="lid covers the top opening footprint",
        )
        ctx.expect_gap(
            pedal,
            body,
            axis="x",
            positive_elem="pedal_plate",
            negative_elem="front_wall",
            min_gap=0.010,
            name="pedal remains distinct under front wall",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="yz",
            elem_a="cover_back",
            elem_b="damper_body",
            min_overlap=0.030,
            name="damper cover sits over rear soft-close hardware",
        )

        lid_closed_aabb = ctx.part_world_aabb(lid)
        pedal_closed_aabb = ctx.part_world_aabb(pedal)
        cover_closed_aabb = ctx.part_world_aabb(cover)

    with ctx.pose({lid_hinge: 0.90}):
        lid_open_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({pedal_pivot: 0.35}):
        pedal_pressed_aabb = ctx.part_world_aabb(pedal)
    with ctx.pose({cover_hinge: 0.75}):
        cover_open_aabb = ctx.part_world_aabb(cover)

    ctx.check(
        "lid hinge opens upward",
        lid_closed_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_closed_aabb[1][2] + 0.090,
        details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
    )
    ctx.check(
        "foot pedal depresses downward",
        pedal_closed_aabb is not None
        and pedal_pressed_aabb is not None
        and pedal_pressed_aabb[0][2] < pedal_closed_aabb[0][2] - 0.035,
        details=f"closed={pedal_closed_aabb}, pressed={pedal_pressed_aabb}",
    )
    ctx.check(
        "damper cover swings up at back",
        cover_closed_aabb is not None
        and cover_open_aabb is not None
        and cover_open_aabb[1][2] > cover_closed_aabb[1][2] + 0.020,
        details=f"closed={cover_closed_aabb}, open={cover_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

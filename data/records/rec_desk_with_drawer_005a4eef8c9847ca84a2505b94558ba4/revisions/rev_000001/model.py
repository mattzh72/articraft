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
    model = ArticulatedObject(name="console_desk")

    walnut = model.material("walnut", rgba=(0.47, 0.31, 0.18, 1.0))
    charcoal = model.material("charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.28, 0.19, 0.12, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.65, 0.67, 0.70, 1.0))

    desk_width = 1.18
    desk_depth = 0.34
    desk_height = 0.78
    top_thickness = 0.03

    front_edge = -desk_depth / 2.0
    back_edge = desk_depth / 2.0
    flap_front_edge = 0.108
    flap_back_edge = 0.168
    flap_depth = flap_back_edge - flap_front_edge
    main_top_back_edge = 0.108
    main_top_depth = main_top_back_edge - front_edge

    top_z = desk_height - top_thickness / 2.0

    frame = model.part("frame")
    frame.visual(
        Box((desk_width, main_top_depth, top_thickness)),
        origin=Origin(xyz=(0.0, (front_edge + main_top_back_edge) / 2.0, top_z)),
        material=walnut,
        name="top_main",
    )

    leg_size = 0.036
    leg_height = 0.752
    leg_x = 0.556
    leg_y = 0.152
    for index, x_sign in enumerate((-1.0, 1.0)):
        for jndex, y_sign in enumerate((-1.0, 1.0)):
            frame.visual(
                Box((leg_size, leg_size, leg_height)),
                origin=Origin(
                    xyz=(
                        x_sign * leg_x,
                        y_sign * leg_y,
                        leg_height / 2.0,
                    )
                ),
                material=charcoal,
                name=f"leg_{index}_{jndex}",
            )

    apron_height = 0.072
    apron_z = 0.716
    frame.visual(
        Box((0.030, 0.272, apron_height)),
        origin=Origin(xyz=(-leg_x, 0.0, apron_z)),
        material=charcoal,
        name="side_apron_0",
    )
    frame.visual(
        Box((0.030, 0.272, apron_height)),
        origin=Origin(xyz=(leg_x, 0.0, apron_z)),
        material=charcoal,
        name="side_apron_1",
    )
    frame.visual(
        Box((1.080, 0.028, apron_height)),
        origin=Origin(xyz=(0.0, 0.148, apron_z)),
        material=charcoal,
        name="rear_apron",
    )
    frame.visual(
        Box((0.292, 0.028, apron_height)),
        origin=Origin(xyz=(-0.394, -0.148, apron_z)),
        material=charcoal,
        name="front_apron_0",
    )
    frame.visual(
        Box((0.292, 0.028, apron_height)),
        origin=Origin(xyz=(0.394, -0.148, apron_z)),
        material=charcoal,
        name="front_apron_1",
    )

    frame.visual(
        Box((0.016, 0.248, 0.094)),
        origin=Origin(xyz=(-0.248, -0.008, 0.703)),
        material=charcoal,
        name="drawer_bay_side_0",
    )
    frame.visual(
        Box((0.016, 0.248, 0.094)),
        origin=Origin(xyz=(0.248, -0.008, 0.703)),
        material=charcoal,
        name="drawer_bay_side_1",
    )
    frame.visual(
        Box((0.512, 0.016, 0.094)),
        origin=Origin(xyz=(0.0, 0.108, 0.703)),
        material=charcoal,
        name="drawer_bay_back",
    )

    frame.visual(
        Box((0.014, 0.160, 0.012)),
        origin=Origin(xyz=(-0.233, 0.020, 0.660)),
        material=satin_steel,
        name="left_runner_track",
    )
    frame.visual(
        Box((0.014, 0.160, 0.012)),
        origin=Origin(xyz=(0.233, 0.020, 0.660)),
        material=satin_steel,
        name="right_runner_track",
    )

    frame.visual(
        Box((0.024, 0.272, 0.028)),
        origin=Origin(xyz=(-leg_x, 0.0, 0.150)),
        material=charcoal,
        name="lower_stretcher_0",
    )
    frame.visual(
        Box((0.024, 0.272, 0.028)),
        origin=Origin(xyz=(leg_x, 0.0, 0.150)),
        material=charcoal,
        name="lower_stretcher_1",
    )
    frame.visual(
        Box((1.080, 0.024, 0.028)),
        origin=Origin(xyz=(0.0, 0.148, 0.150)),
        material=charcoal,
        name="rear_stretcher",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.494, 0.018, 0.094)),
        origin=Origin(xyz=(0.0, -0.136, 0.0)),
        material=walnut,
        name="front_panel",
    )
    drawer.visual(
        Box((0.012, 0.230, 0.075)),
        origin=Origin(xyz=(-0.222, -0.021, -0.010)),
        material=dark_wood,
        name="side_0",
    )
    drawer.visual(
        Box((0.012, 0.230, 0.075)),
        origin=Origin(xyz=(0.222, -0.021, -0.010)),
        material=dark_wood,
        name="side_1",
    )
    drawer.visual(
        Box((0.432, 0.012, 0.070)),
        origin=Origin(xyz=(0.0, 0.088, -0.010)),
        material=dark_wood,
        name="back_panel",
    )
    drawer.visual(
        Box((0.432, 0.228, 0.010)),
        origin=Origin(xyz=(0.0, -0.021, -0.0425)),
        material=dark_wood,
        name="bottom_panel",
    )
    drawer.visual(
        Box((0.010, 0.150, 0.010)),
        origin=Origin(xyz=(-0.233, 0.040, -0.019)),
        material=satin_steel,
        name="left_runner",
    )
    drawer.visual(
        Box((0.010, 0.150, 0.010)),
        origin=Origin(xyz=(0.233, 0.040, -0.019)),
        material=satin_steel,
        name="right_runner",
    )

    model.articulation(
        "frame_to_drawer",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.025, 0.690)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=0.120,
        ),
    )

    cable_flap = model.part("cable_flap")
    cable_flap.visual(
        Box((1.140, flap_depth, 0.022)),
        origin=Origin(xyz=(0.0, -flap_depth / 2.0, -0.003)),
        material=walnut,
        name="flap_panel",
    )
    cable_flap.visual(
        Cylinder(radius=0.006, length=1.120),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="hinge_barrel",
    )

    model.articulation(
        "frame_to_cable_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cable_flap,
        origin=Origin(xyz=(0.0, flap_back_edge, 0.772)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    drawer = object_model.get_part("drawer")
    cable_flap = object_model.get_part("cable_flap")

    drawer_slide = object_model.get_articulation("frame_to_drawer")
    flap_hinge = object_model.get_articulation("frame_to_cable_flap")

    drawer_lower = 0.0
    drawer_upper = (
        drawer_slide.motion_limits.upper if drawer_slide.motion_limits is not None else 0.0
    )
    flap_upper = flap_hinge.motion_limits.upper if flap_hinge.motion_limits is not None else 0.0

    ctx.expect_gap(
        frame,
        drawer,
        axis="z",
        positive_elem="top_main",
        negative_elem="front_panel",
        min_gap=0.010,
        max_gap=0.015,
        name="drawer front sits neatly below the slim top",
    )
    ctx.expect_gap(
        cable_flap,
        frame,
        axis="y",
        positive_elem="flap_panel",
        negative_elem="top_main",
        min_gap=0.0,
        max_gap=0.001,
        name="rear cable flap closes with a narrow seam",
    )
    ctx.expect_overlap(
        frame,
        drawer,
        axes="x",
        elem_a="top_main",
        elem_b="front_panel",
        min_overlap=0.490,
        name="drawer front stays centered beneath the desk top",
    )

    rest_pos = None
    extended_pos = None
    with ctx.pose({drawer_slide: drawer_lower}):
        rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: drawer_upper}):
        ctx.expect_overlap(
            drawer,
            frame,
            axes="y",
            elem_a="left_runner",
            elem_b="left_runner_track",
            min_overlap=0.020,
            name="left runner remains engaged at full extension",
        )
        ctx.expect_overlap(
            drawer,
            frame,
            axes="y",
            elem_a="right_runner",
            elem_b="right_runner_track",
            min_overlap=0.020,
            name="right runner remains engaged at full extension",
        )
        ctx.expect_gap(
            drawer,
            frame,
            axis="z",
            positive_elem="left_runner",
            negative_elem="left_runner_track",
            max_gap=0.001,
            max_penetration=1e-6,
            name="left runner rides on its internal track",
        )
        ctx.expect_gap(
            drawer,
            frame,
            axis="z",
            positive_elem="right_runner",
            negative_elem="right_runner_track",
            max_gap=0.001,
            max_penetration=1e-6,
            name="right runner rides on its internal track",
        )
        extended_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends forward from the desk",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[1] < rest_pos[1] - 0.10,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({flap_hinge: flap_upper}):
        flap_aabb = ctx.part_world_aabb(cable_flap)
        ctx.check(
            "cable flap lifts up above the work surface",
            flap_aabb is not None and flap_aabb[1][2] > 0.82,
            details=f"aabb={flap_aabb}",
        )
        ctx.check(
            "cable flap pivots from the rear edge",
            flap_aabb is not None and flap_aabb[0][1] > 0.135,
            details=f"aabb={flap_aabb}",
        )

    return ctx.report()


object_model = build_object_model()

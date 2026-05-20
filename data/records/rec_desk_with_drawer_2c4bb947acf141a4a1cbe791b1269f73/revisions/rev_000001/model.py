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
    model = ArticulatedObject(name="study_desk_with_drawer")

    walnut = model.material("warm_walnut", rgba=(0.48, 0.27, 0.13, 1.0))
    walnut_dark = model.material("dark_endgrain", rgba=(0.28, 0.14, 0.07, 1.0))
    inner_shadow = model.material("drawer_shadow", rgba=(0.035, 0.030, 0.026, 1.0))
    satin_black = model.material("satin_black", rgba=(0.015, 0.015, 0.017, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.68, 0.62, 1.0))

    frame = model.part("desk_frame")

    # +X is the front of the desk, +Z is up.  The fixed frame is authored as
    # one connected manufactured carcass: top, side panel supports, modesty
    # panel, drawer bay, slide rails, edge banding, feet, and subtle grain.
    frame.visual(
        Box((0.70, 1.35, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
        material=walnut,
        name="tabletop_slab",
    )
    frame.visual(
        Box((0.025, 1.37, 0.064)),
        origin=Origin(xyz=(0.3625, 0.0, 0.720)),
        material=walnut_dark,
        name="front_edge_band",
    )
    frame.visual(
        Box((0.025, 1.37, 0.064)),
        origin=Origin(xyz=(-0.3625, 0.0, 0.720)),
        material=walnut_dark,
        name="rear_edge_band",
    )
    frame.visual(
        Box((0.70, 0.025, 0.064)),
        origin=Origin(xyz=(0.0, 0.6875, 0.720)),
        material=walnut_dark,
        name="side_edge_0",
    )
    frame.visual(
        Box((0.70, 0.025, 0.064)),
        origin=Origin(xyz=(0.0, -0.6875, 0.720)),
        material=walnut_dark,
        name="side_edge_1",
    )

    for y, name in ((0.620, "side_panel_0"), (-0.620, "side_panel_1")):
        frame.visual(
            Box((0.62, 0.055, 0.692)),
            origin=Origin(xyz=(0.0, y, 0.346)),
            material=walnut,
            name=name,
        )
        frame.visual(
            Box((0.56, 0.075, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.013)),
            material=satin_black,
            name=f"leveling_foot_{name[-1]}",
        )

    frame.visual(
        Box((0.040, 1.185, 0.420)),
        origin=Origin(xyz=(-0.310, 0.0, 0.450)),
        material=walnut_dark,
        name="modesty_panel",
    )

    # Drawer pocket: a real-looking wooden case with a top, cheeks, lower rail,
    # and black reveals around the front opening rather than a solid block.
    frame.visual(
        Box((0.480, 0.800, 0.025)),
        origin=Origin(xyz=(0.110, 0.0, 0.6775)),
        material=walnut,
        name="drawer_case_top",
    )
    frame.visual(
        Box((0.480, 0.800, 0.025)),
        origin=Origin(xyz=(0.110, 0.0, 0.5000)),
        material=walnut,
        name="drawer_case_bottom",
    )
    frame.visual(
        Box((0.480, 0.030, 0.175)),
        origin=Origin(xyz=(0.110, 0.360, 0.588)),
        material=walnut,
        name="drawer_cheek_0",
    )
    frame.visual(
        Box((0.480, 0.030, 0.175)),
        origin=Origin(xyz=(0.110, -0.360, 0.588)),
        material=walnut,
        name="drawer_cheek_1",
    )
    frame.visual(
        Box((0.040, 0.800, 0.040)),
        origin=Origin(xyz=(0.330, 0.0, 0.670)),
        material=walnut_dark,
        name="front_top_rail",
    )
    frame.visual(
        Box((0.040, 0.800, 0.040)),
        origin=Origin(xyz=(0.330, 0.0, 0.490)),
        material=walnut_dark,
        name="front_bottom_rail",
    )
    frame.visual(
        Box((0.040, 0.046, 0.210)),
        origin=Origin(xyz=(0.330, 0.385, 0.590)),
        material=walnut_dark,
        name="front_stile_0",
    )
    frame.visual(
        Box((0.040, 0.046, 0.210)),
        origin=Origin(xyz=(0.330, -0.385, 0.590)),
        material=walnut_dark,
        name="front_stile_1",
    )
    frame.visual(
        Box((0.010, 0.705, 0.010)),
        origin=Origin(xyz=(0.352, 0.0, 0.670)),
        material=inner_shadow,
        name="top_shadow_reveal",
    )
    frame.visual(
        Box((0.010, 0.705, 0.010)),
        origin=Origin(xyz=(0.352, 0.0, 0.510)),
        material=inner_shadow,
        name="bottom_shadow_reveal",
    )
    frame.visual(
        Box((0.010, 0.010, 0.155)),
        origin=Origin(xyz=(0.352, 0.340, 0.590)),
        material=inner_shadow,
        name="side_shadow_0",
    )
    frame.visual(
        Box((0.010, 0.010, 0.155)),
        origin=Origin(xyz=(0.352, -0.340, 0.590)),
        material=inner_shadow,
        name="side_shadow_1",
    )
    frame.visual(
        Box((0.440, 0.018, 0.025)),
        origin=Origin(xyz=(0.100, 0.336, 0.575)),
        material=brushed_steel,
        name="fixed_slide_0",
    )
    frame.visual(
        Box((0.440, 0.018, 0.025)),
        origin=Origin(xyz=(0.100, -0.336, 0.575)),
        material=brushed_steel,
        name="fixed_slide_1",
    )

    # Fine, low-relief grain lines and a darker front apron give the otherwise
    # rectilinear desk the finish of a home-office furniture piece.
    for i, y in enumerate((-0.48, -0.31, -0.14, 0.03, 0.21, 0.39, 0.55)):
        frame.visual(
            Box((0.580, 0.006, 0.0015)),
            origin=Origin(xyz=(-0.015, y, 0.7507)),
            material=walnut_dark,
            name=f"grain_line_{i}",
        )
    frame.visual(
        Box((0.025, 1.19, 0.070)),
        origin=Origin(xyz=(0.321, 0.0, 0.355)),
        material=walnut_dark,
        name="front_lower_apron",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.035, 0.670, 0.150)),
        origin=Origin(xyz=(0.3675, 0.0, 0.590)),
        material=walnut_dark,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.440, 0.540, 0.025)),
        origin=Origin(xyz=(0.120, 0.0, 0.5275)),
        material=walnut,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.440, 0.020, 0.110)),
        origin=Origin(xyz=(0.120, 0.280, 0.585)),
        material=walnut,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.440, 0.020, 0.110)),
        origin=Origin(xyz=(0.120, -0.280, 0.585)),
        material=walnut,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((0.020, 0.540, 0.110)),
        origin=Origin(xyz=(-0.110, 0.0, 0.585)),
        material=walnut,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.015, 0.540, 0.110)),
        origin=Origin(xyz=(0.3425, 0.0, 0.585)),
        material=walnut,
        name="inner_front_wall",
    )
    drawer.visual(
        Box((0.420, 0.014, 0.018)),
        origin=Origin(xyz=(0.115, 0.297, 0.575)),
        material=brushed_steel,
        name="moving_slide_0",
    )
    drawer.visual(
        Box((0.420, 0.014, 0.018)),
        origin=Origin(xyz=(0.115, -0.297, 0.575)),
        material=brushed_steel,
        name="moving_slide_1",
    )
    drawer.visual(
        Cylinder(radius=0.007, length=0.032),
        origin=Origin(xyz=(0.376, 0.110, 0.600), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="handle_post_0",
    )
    drawer.visual(
        Cylinder(radius=0.007, length=0.032),
        origin=Origin(xyz=(0.376, -0.110, 0.600), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="handle_post_1",
    )
    drawer.visual(
        Cylinder(radius=0.012, length=0.285),
        origin=Origin(xyz=(0.392, 0.0, 0.600), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="handle_bar",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=drawer,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.32),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("desk_frame")
    drawer = object_model.get_part("drawer")
    slide = object_model.get_articulation("drawer_slide")

    ctx.expect_gap(
        drawer,
        frame,
        axis="x",
        positive_elem="drawer_face",
        negative_elem="front_top_rail",
        max_gap=0.001,
        max_penetration=0.00001,
        name="closed drawer face seats at front rail",
    )
    ctx.expect_within(
        drawer,
        frame,
        axes="y",
        inner_elem="drawer_face",
        outer_elem="drawer_case_top",
        margin=0.0,
        name="drawer face fits within the bay width",
    )
    ctx.expect_gap(
        frame,
        drawer,
        axis="y",
        positive_elem="fixed_slide_0",
        negative_elem="moving_slide_0",
        min_gap=0.002,
        max_gap=0.035,
        name="right slide has a realistic side clearance",
    )
    ctx.expect_gap(
        drawer,
        frame,
        axis="y",
        positive_elem="moving_slide_1",
        negative_elem="fixed_slide_1",
        min_gap=0.002,
        max_gap=0.035,
        name="left slide has a realistic side clearance",
    )

    closed_pos = ctx.part_world_position(drawer)
    with ctx.pose({slide: 0.32}):
        opened_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            frame,
            axes="x",
            elem_a="moving_slide_0",
            elem_b="fixed_slide_0",
            min_overlap=0.060,
            name="extended drawer retains right slide engagement",
        )
        ctx.expect_overlap(
            drawer,
            frame,
            axes="x",
            elem_a="moving_slide_1",
            elem_b="fixed_slide_1",
            min_overlap=0.060,
            name="extended drawer retains left slide engagement",
        )

    ctx.check(
        "drawer opens toward the front",
        closed_pos is not None and opened_pos is not None and opened_pos[0] > closed_pos[0] + 0.25,
        details=f"closed={closed_pos}, opened={opened_pos}",
    )

    return ctx.report()


object_model = build_object_model()

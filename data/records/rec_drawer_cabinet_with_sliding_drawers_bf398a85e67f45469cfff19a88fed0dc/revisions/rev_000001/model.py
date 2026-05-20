from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_drawer_dresser")

    warm_oak = model.material("warm_oak", rgba=(0.63, 0.38, 0.17, 1.0))
    dark_oak = model.material("dark_oak", rgba=(0.33, 0.18, 0.08, 1.0))
    drawer_oak = model.material("drawer_oak", rgba=(0.72, 0.45, 0.22, 1.0))

    width = 1.15
    depth = 0.46
    height = 0.95
    side_t = 0.035
    back_t = 0.025
    front_frame_t = 0.026
    front_frame_depth = 0.024

    chest = model.part("chest")

    # Wide, rectangular cabinet carcass: side panels, top/bottom, and a back board.
    chest.visual(
        Box((depth, side_t, height)),
        origin=Origin(xyz=(0.0, width / 2.0 - side_t / 2.0, height / 2.0)),
        material=warm_oak,
        name="side_panel_pos",
    )
    chest.visual(
        Box((depth, side_t, height)),
        origin=Origin(xyz=(0.0, -width / 2.0 + side_t / 2.0, height / 2.0)),
        material=warm_oak,
        name="side_panel_neg",
    )
    chest.visual(
        Box((depth, width, side_t)),
        origin=Origin(xyz=(0.0, 0.0, height - side_t / 2.0)),
        material=warm_oak,
        name="top_panel",
    )
    chest.visual(
        Box((depth, width, side_t)),
        origin=Origin(xyz=(0.0, 0.0, side_t / 2.0)),
        material=warm_oak,
        name="bottom_panel",
    )
    chest.visual(
        Box((back_t, width, height)),
        origin=Origin(xyz=(-depth / 2.0 + back_t / 2.0, 0.0, height / 2.0)),
        material=dark_oak,
        name="back_panel",
    )

    # Proud front frame rails divide the four openings and make the body read as a chest.
    front_x = depth / 2.0 + front_frame_depth / 2.0
    stile_w = 0.060
    chest.visual(
        Box((front_frame_depth, stile_w, height)),
        origin=Origin(xyz=(front_x, width / 2.0 - stile_w / 2.0, height / 2.0)),
        material=dark_oak,
        name="front_stile_pos",
    )
    chest.visual(
        Box((front_frame_depth, stile_w, height)),
        origin=Origin(xyz=(front_x, -width / 2.0 + stile_w / 2.0, height / 2.0)),
        material=dark_oak,
        name="front_stile_neg",
    )

    drawer_centers_z = (0.165, 0.380, 0.595, 0.810)
    rail_centers_z = (0.0575, 0.2725, 0.4875, 0.7025, 0.9175)
    rail_h = 0.035
    for rail_i, z in enumerate(rail_centers_z):
        chest.visual(
            Box((front_frame_depth, width, rail_h)),
            origin=Origin(xyz=(front_x, 0.0, z)),
            material=dark_oak,
            name=f"front_rail_{rail_i}",
        )

    drawer_depth = 0.390
    drawer_width = 0.860
    drawer_box_h = 0.130
    drawer_front_h = 0.170
    drawer_front_w = 0.960
    drawer_front_t = 0.026
    board_t = 0.018
    runner_h = 0.014
    runner_bottom_z = -0.088
    guide_h = 0.022
    guide_y_span = 0.160
    guide_clearance = 0.0
    slide_travel = 0.280

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.062,
            0.048,
            body_style="mushroom",
            base_diameter=0.030,
            top_diameter=0.055,
            crown_radius=0.004,
            edge_radius=0.0015,
            center=False,
        ),
        "turned_wood_knob",
    )

    for i, z in enumerate(drawer_centers_z):
        # Fixed wooden runner guides attached to the two side walls.
        guide_center_z = z + runner_bottom_z - guide_clearance - guide_h / 2.0
        chest.visual(
            Box((drawer_depth, guide_y_span, guide_h)),
            origin=Origin(xyz=(0.030, 0.540 - guide_y_span / 2.0, guide_center_z)),
            material=dark_oak,
            name=f"guide_{i}_pos",
        )
        chest.visual(
            Box((drawer_depth, guide_y_span, guide_h)),
            origin=Origin(xyz=(0.030, -0.540 + guide_y_span / 2.0, guide_center_z)),
            material=dark_oak,
            name=f"guide_{i}_neg",
        )

        drawer = model.part(f"drawer_{i}")

        # One equal-depth wooden drawer box per moving part.
        drawer.visual(
            Box((drawer_front_t, drawer_front_w, drawer_front_h)),
            origin=Origin(xyz=(0.258, 0.0, 0.0)),
            material=drawer_oak,
            name="front_panel",
        )
        drawer.visual(
            Box((drawer_depth, drawer_width, board_t)),
            origin=Origin(xyz=(0.055, 0.0, -0.065)),
            material=drawer_oak,
            name="bottom_board",
        )
        drawer.visual(
            Box((drawer_depth, board_t, drawer_box_h)),
            origin=Origin(xyz=(0.055, drawer_width / 2.0 - board_t / 2.0, -0.010)),
            material=drawer_oak,
            name="side_wall_pos",
        )
        drawer.visual(
            Box((drawer_depth, board_t, drawer_box_h)),
            origin=Origin(xyz=(0.055, -drawer_width / 2.0 + board_t / 2.0, -0.010)),
            material=drawer_oak,
            name="side_wall_neg",
        )
        drawer.visual(
            Box((board_t, drawer_width, drawer_box_h)),
            origin=Origin(xyz=(-0.131, 0.0, -0.010)),
            material=drawer_oak,
            name="back_wall",
        )
        drawer.visual(
            Box((0.360, 0.035, runner_h)),
            origin=Origin(xyz=(0.055, 0.390, runner_bottom_z + runner_h / 2.0)),
            material=dark_oak,
            name="runner_pos",
        )
        drawer.visual(
            Box((0.360, 0.035, runner_h)),
            origin=Origin(xyz=(0.055, -0.390, runner_bottom_z + runner_h / 2.0)),
            material=dark_oak,
            name="runner_neg",
        )

        # Simple turned wood knob: a collar and stem connect a mushroom-shaped pull.
        knob_front_x = 0.258 + drawer_front_t / 2.0
        drawer.visual(
            Cylinder(radius=0.026, length=0.012),
            origin=Origin(xyz=(knob_front_x + 0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_oak,
            name="knob_collar",
        )
        drawer.visual(
            Cylinder(radius=0.014, length=0.034),
            origin=Origin(xyz=(knob_front_x + 0.017, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_oak,
            name="knob_stem",
        )
        drawer.visual(
            knob_mesh,
            origin=Origin(xyz=(knob_front_x + 0.032, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_oak,
            name="knob_head",
        )

        model.articulation(
            f"chest_to_drawer_{i}",
            ArticulationType.PRISMATIC,
            parent=chest,
            child=drawer,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=slide_travel),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chest = object_model.get_part("chest")
    for i in range(4):
        drawer = object_model.get_part(f"drawer_{i}")
        joint = object_model.get_articulation(f"chest_to_drawer_{i}")

        ctx.expect_gap(
            drawer,
            chest,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="runner_pos",
            negative_elem=f"guide_{i}_pos",
            name=f"drawer_{i} rests just above positive runner guide",
        )
        ctx.expect_gap(
            drawer,
            chest,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="runner_neg",
            negative_elem=f"guide_{i}_neg",
            name=f"drawer_{i} rests just above negative runner guide",
        )

        ctx.expect_overlap(
            drawer,
            chest,
            axes="x",
            min_overlap=0.30,
            elem_a="runner_pos",
            elem_b=f"guide_{i}_pos",
            name=f"drawer_{i} runner is carried by guide when closed",
        )

        rest_position = ctx.part_world_position(drawer)
        with ctx.pose({joint: 0.280}):
            extended_position = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                chest,
                axes="x",
                min_overlap=0.060,
                elem_a="runner_pos",
                elem_b=f"guide_{i}_pos",
                name=f"drawer_{i} keeps retained runner insertion when pulled",
            )

        ctx.check(
            f"drawer_{i} slides outward along the dresser front",
            rest_position is not None
            and extended_position is not None
            and extended_position[0] > rest_position[0] + 0.25,
            details=f"rest={rest_position}, extended={extended_position}",
        )

    return ctx.report()


object_model = build_object_model()

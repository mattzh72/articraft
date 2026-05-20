from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_air_purifier")

    white = model.material("warm_white_plastic", rgba=(0.86, 0.88, 0.86, 1.0))
    light = model.material("front_satin_white", rgba=(0.94, 0.95, 0.93, 1.0))
    grey = model.material("soft_grey_plastic", rgba=(0.58, 0.61, 0.62, 1.0))
    dark = model.material("dark_filter_shadow", rgba=(0.10, 0.12, 0.13, 1.0))
    filter_blue = model.material("pale_filter_media", rgba=(0.72, 0.82, 0.86, 1.0))
    rail_mat = model.material("low_friction_rail", rgba=(0.35, 0.38, 0.39, 1.0))

    # Indoor wall-appliance proportions: broad, shallow, and tall.
    width = 0.44
    height = 0.62
    depth = 0.16
    wall = 0.016
    back_thickness = 0.012

    body = model.part("body")
    body.visual(
        Box((back_thickness, width, height)),
        origin=Origin(xyz=(-depth / 2 + back_thickness / 2, 0.0, 0.0)),
        material=white,
        name="back_pan",
    )
    body.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(0.0, -width / 2 + wall / 2, 0.0)),
        material=white,
        name="side_wall_0",
    )
    body.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(0.0, width / 2 - wall / 2, 0.0)),
        material=white,
        name="side_wall_1",
    )
    body.visual(
        Box((depth, width, wall)),
        origin=Origin(xyz=(0.0, 0.0, height / 2 - wall / 2)),
        material=white,
        name="top_wall",
    )
    body.visual(
        Box((depth, width, wall)),
        origin=Origin(xyz=(0.0, 0.0, -height / 2 + wall / 2)),
        material=white,
        name="bottom_wall",
    )
    # A raised front rim gives the housing a clear hollow service opening.
    rim_depth = 0.012
    body.visual(
        Box((rim_depth, wall * 1.2, height)),
        origin=Origin(xyz=(depth / 2 + rim_depth / 2, -width / 2 + wall * 0.6, 0.0)),
        material=light,
        name="front_rim_0",
    )
    body.visual(
        Box((rim_depth, wall * 1.2, height)),
        origin=Origin(xyz=(depth / 2 + rim_depth / 2, width / 2 - wall * 0.6, 0.0)),
        material=light,
        name="front_rim_1",
    )
    body.visual(
        Box((rim_depth, width, wall * 1.2)),
        origin=Origin(xyz=(depth / 2 + rim_depth / 2, 0.0, height / 2 - wall * 0.6)),
        material=light,
        name="front_rim_top",
    )
    body.visual(
        Box((rim_depth, width, wall * 1.2)),
        origin=Origin(xyz=(depth / 2 + rim_depth / 2, 0.0, -height / 2 + wall * 0.6)),
        material=light,
        name="front_rim_bottom",
    )

    # Wall-mount details on the rear pan.
    body.visual(
        Box((0.010, 0.13, 0.028)),
        origin=Origin(xyz=(-depth / 2 - 0.005, 0.0, height * 0.33)),
        material=grey,
        name="upper_wall_cleat",
    )
    body.visual(
        Box((0.010, 0.10, 0.024)),
        origin=Origin(xyz=(-depth / 2 - 0.005, 0.0, -height * 0.34)),
        material=grey,
        name="lower_wall_spacer",
    )

    # Bottom guide rails that carry the filter drawer.
    rail_length = 0.14
    rail_width = 0.018
    rail_height = 0.012
    rail_z = -height / 2 + wall + rail_height / 2
    for rail_name, stop_name, y in (
        ("bottom_rail_0", "rail_stop_0", -0.145),
        ("bottom_rail_1", "rail_stop_1", 0.145),
    ):
        body.visual(
            Box((rail_length, rail_width, rail_height)),
            origin=Origin(xyz=(-0.005, y, rail_z)),
            material=rail_mat,
            name=rail_name,
        )
        body.visual(
            Box((0.014, rail_width + 0.008, 0.028)),
            origin=Origin(xyz=(-0.076, y, rail_z + 0.008)),
            material=rail_mat,
            name=stop_name,
        )

    # Exposed alternating hinge barrels fixed to the housing side.
    hinge_x = depth / 2 + rim_depth + 0.005
    hinge_y = -width / 2 - 0.010
    hinge_radius = 0.008
    for index, z in enumerate((-0.215, 0.215)):
        body.visual(
            Cylinder(radius=hinge_radius, length=0.145),
            origin=Origin(xyz=(hinge_x, hinge_y, z)),
            material=grey,
            name=f"body_hinge_knuckle_{index}",
        )
        body.visual(
            Box((0.010, 0.020, 0.120)),
            origin=Origin(xyz=(hinge_x - 0.004, hinge_y + 0.010, z)),
            material=grey,
            name=f"body_hinge_leaf_{index}",
        )

    cover_width = 0.422
    cover_height = 0.588
    cover_thickness = 0.010
    cover_hinge_clearance = 0.016
    cover = model.part("front_cover")
    grille = SlotPatternPanelGeometry(
        (cover_width, cover_height),
        cover_thickness,
        slot_size=(0.040, 0.006),
        pitch=(0.055, 0.020),
        frame=0.024,
        corner_radius=0.010,
        slot_angle_deg=0.0,
        stagger=True,
    )
    cover.visual(
        mesh_from_geometry(grille, "front_cover_slotted_grille"),
        # SlotPatternPanelGeometry is local XY with thickness along local Z.
        # Rotate it so width follows child +Y, height follows +Z, and thickness
        # follows front/back +X.
        origin=Origin(
            xyz=(0.003, cover_hinge_clearance + cover_width / 2, 0.0),
            rpy=(math.pi / 2, 0.0, math.pi / 2),
        ),
        material=light,
        name="slotted_grille",
    )
    cover.visual(
        Box((0.018, 0.030, 0.210)),
        origin=Origin(xyz=(0.016, cover_hinge_clearance + cover_width - 0.026, -0.035)),
        material=grey,
        name="pull_recess",
    )
    cover.visual(
        Cylinder(radius=hinge_radius * 0.94, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=grey,
        name="cover_hinge_knuckle",
    )
    cover.visual(
        Box((0.010, 0.025, 0.170)),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=grey,
        name="cover_hinge_leaf",
    )

    model.articulation(
        "body_to_front_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        # The closed cover extends from the hinge along child +Y.  A negative
        # vertical axis makes positive q swing the free edge outward along +X.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=0.0, upper=1.85),
    )

    drawer = model.part("filter_drawer")
    runner_length = 0.13
    runner_height = 0.006
    runner_width = 0.012
    for runner_name, y in (
        ("drawer_runner_0", -0.145),
        ("drawer_runner_1", 0.145),
    ):
        drawer.visual(
            Box((runner_length, runner_width, runner_height)),
            origin=Origin(xyz=(0.002, y, 0.0)),
            material=rail_mat,
            name=runner_name,
        )
    drawer.visual(
        Box((0.070, 0.345, 0.022)),
        origin=Origin(xyz=(-0.010, 0.0, 0.013)),
        material=grey,
        name="drawer_tray",
    )
    drawer.visual(
        Box((0.050, 0.325, 0.230)),
        origin=Origin(xyz=(-0.015, 0.0, 0.145)),
        material=filter_blue,
        name="filter_media",
    )
    drawer.visual(
        Box((0.058, 0.345, 0.020)),
        origin=Origin(xyz=(-0.015, 0.0, 0.270)),
        material=grey,
        name="filter_top_frame",
    )
    drawer.visual(
        Box((0.058, 0.020, 0.250)),
        origin=Origin(xyz=(-0.015, -0.172, 0.145)),
        material=grey,
        name="filter_side_frame_0",
    )
    drawer.visual(
        Box((0.058, 0.020, 0.250)),
        origin=Origin(xyz=(-0.015, 0.172, 0.145)),
        material=grey,
        name="filter_side_frame_1",
    )
    # Subtle pleat ribs across the media face.
    for index, y in enumerate((-0.120, -0.080, -0.040, 0.0, 0.040, 0.080, 0.120)):
        drawer.visual(
            Box((0.010, 0.004, 0.218)),
            origin=Origin(xyz=(0.014, y, 0.145)),
            material=dark,
            name=f"pleat_rib_{index}",
        )

    drawer_origin_z = rail_z + rail_height / 2 + runner_height / 2
    model.articulation(
        "body_to_filter_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(-0.005, 0.0, drawer_origin_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.20, lower=0.0, upper=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("front_cover")
    drawer = object_model.get_part("filter_drawer")
    cover_hinge = object_model.get_articulation("body_to_front_cover")
    drawer_slide = object_model.get_articulation("body_to_filter_drawer")

    with ctx.pose({cover_hinge: 0.0, drawer_slide: 0.0}):
        ctx.expect_gap(
            cover,
            body,
            axis="x",
            min_gap=0.001,
            max_gap=0.020,
            positive_elem="slotted_grille",
            negative_elem="front_rim_top",
            name="closed cover sits just proud of the housing rim",
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a="drawer_runner_0",
            elem_b="bottom_rail_0",
            contact_tol=0.001,
            name="filter drawer rides on first bottom rail",
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a="drawer_runner_1",
            elem_b="bottom_rail_1",
            contact_tol=0.001,
            name="filter drawer rides on second bottom rail",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="drawer_runner_0",
            elem_b="bottom_rail_0",
            min_overlap=0.10,
            name="closed drawer runner is retained on rail",
        )

    rest_cover_aabb = ctx.part_world_aabb(cover)
    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({cover_hinge: 1.2, drawer_slide: 0.10}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="drawer_runner_0",
            elem_b="bottom_rail_0",
            min_overlap=0.030,
            name="extended drawer still has rail engagement",
        )
        opened_cover_aabb = ctx.part_world_aabb(cover)
        extended_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "front cover opens outward about vertical side hinge",
        rest_cover_aabb is not None
        and opened_cover_aabb is not None
        and opened_cover_aabb[1][0] > rest_cover_aabb[1][0] + 0.12,
        details=f"rest={rest_cover_aabb}, opened={opened_cover_aabb}",
    )
    ctx.check(
        "filter drawer slides forward",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.09,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()

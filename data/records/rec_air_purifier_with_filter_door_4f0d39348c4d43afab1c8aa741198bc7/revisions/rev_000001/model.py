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
    PerforatedPanelGeometry,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bathroom_wall_air_purifier")

    white = model.material("warm_white_plastic", rgba=(0.92, 0.94, 0.91, 1.0))
    off_white = model.material("slightly_darker_cover", rgba=(0.86, 0.89, 0.86, 1.0))
    graphite = model.material("charcoal_filter_media", rgba=(0.08, 0.09, 0.09, 1.0))
    pale_filter = model.material("washable_blue_filter", rgba=(0.45, 0.70, 0.82, 1.0))
    bracket_grey = model.material("powder_coated_bracket", rgba=(0.56, 0.58, 0.58, 1.0))
    hinge_metal = model.material("brushed_hinge_metal", rgba=(0.65, 0.66, 0.62, 1.0))
    screw_dark = model.material("dark_screw_heads", rgba=(0.08, 0.08, 0.075, 1.0))

    width = 0.320
    height = 0.520
    depth = 0.120
    wall = 0.012

    # The root is the fixed wall mounting bracket.  Its frame coincides with the
    # rear-bottom-center of the purifier housing, so the housing can be fixed to
    # it with a clean, contact-supported joint.
    bracket = model.part("wall_bracket")
    bracket.visual(
        Box((0.250, 0.006, 0.410)),
        origin=Origin(xyz=(0.0, -0.012, height * 0.52)),
        material=bracket_grey,
        name="wall_plate",
    )
    for idx, z in enumerate((0.112, 0.438)):
        bracket.visual(
            Box((0.226, 0.012, 0.018)),
            origin=Origin(xyz=(0.0, -0.006, z)),
            material=bracket_grey,
            name=f"hook_rail_{idx}",
        )
    for idx, z in enumerate((0.126, 0.424)):
        bracket.visual(
            Cylinder(radius=0.013, length=0.004),
            origin=Origin(xyz=(0.0, -0.0165, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=screw_dark,
            name=f"screw_head_{idx}",
        )
        bracket.visual(
            Box((0.018, 0.0015, 0.003)),
            origin=Origin(xyz=(0.0, -0.019, z), rpy=(0.0, 0.0, 0.0)),
            material=bracket_grey,
            name=f"screw_slot_{idx}",
        )

    housing = model.part("housing")
    housing.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, wall / 2.0, height / 2.0)),
        material=white,
        name="rear_panel",
    )
    housing.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, depth / 2.0, height / 2.0)),
        material=white,
        name="side_wall_0",
    )
    housing.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, depth / 2.0, height / 2.0)),
        material=white,
        name="side_wall_1",
    )
    housing.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, depth / 2.0, height - wall / 2.0)),
        material=white,
        name="top_wall",
    )
    housing.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, depth / 2.0, wall / 2.0)),
        material=white,
        name="bottom_wall",
    )
    # Internal tray slides and a small front base fascia make the tray read as a
    # guided washable-filter cassette rather than a loose floating insert.
    for idx, x in enumerate((-0.129, 0.129)):
        housing.visual(
            Box((0.014, depth - 0.010, 0.006)),
            origin=Origin(xyz=(x, depth / 2.0, 0.030)),
            material=off_white,
            name=f"tray_slide_{idx}",
        )
    housing.visual(
        Box((0.285, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, depth - 0.003, 0.014)),
        material=white,
        name="base_fascia",
    )

    # Two visible stationary hinge knuckle mounts at the right edge.  The moving
    # knuckles live on the cover part and share the revolute axis.
    pivot_x = width / 2.0 + 0.014
    pivot_y = depth + 0.008
    for idx, zc in enumerate((0.146, 0.420)):
        housing.visual(
            Box((0.030, 0.006, 0.078)),
            origin=Origin(xyz=(width / 2.0 + 0.006, depth + 0.003, zc)),
            material=hinge_metal,
            name=f"hinge_leaf_{idx}",
        )
        housing.visual(
            Cylinder(radius=0.0055, length=0.022),
            origin=Origin(xyz=(pivot_x, pivot_y, zc - 0.026)),
            material=hinge_metal,
            name=f"fixed_knuckle_{idx}_0",
        )
        housing.visual(
            Cylinder(radius=0.0055, length=0.022),
            origin=Origin(xyz=(pivot_x, pivot_y, zc + 0.026)),
            material=hinge_metal,
            name=f"fixed_knuckle_{idx}_1",
        )

    model.articulation(
        "bracket_to_housing",
        ArticulationType.FIXED,
        parent=bracket,
        child=housing,
        origin=Origin(),
    )

    cover = model.part("front_cover")
    cover_width = 0.306
    cover_height = 0.455
    cover_bottom = 0.055
    cover_center_z = cover_bottom + cover_height / 2.0
    cover_x_right = -0.012
    cover_center_x = cover_x_right - cover_width / 2.0
    cover_t = 0.012
    stile = 0.022
    top_rail = 0.024

    cover.visual(
        Box((stile, cover_t, cover_height)),
        origin=Origin(xyz=(cover_x_right - cover_width + stile / 2.0, 0.0, cover_center_z)),
        material=off_white,
        name="cover_side_stile",
    )
    cover.visual(
        Box((0.030, cover_t, cover_height)),
        origin=Origin(xyz=(cover_x_right - 0.015, 0.0, cover_center_z)),
        material=off_white,
        name="cover_hinge_stile",
    )
    cover.visual(
        Box((cover_width, cover_t, top_rail)),
        origin=Origin(xyz=(cover_center_x, 0.0, cover_bottom + top_rail / 2.0)),
        material=off_white,
        name="cover_bottom_rail",
    )
    cover.visual(
        Box((cover_width, cover_t, top_rail)),
        origin=Origin(xyz=(cover_center_x, 0.0, cover_bottom + cover_height - top_rail / 2.0)),
        material=off_white,
        name="cover_top_rail",
    )

    grille = SlotPatternPanelGeometry(
        (0.265, 0.425),
        0.003,
        slot_size=(0.044, 0.006),
        pitch=(0.059, 0.018),
        frame=0.010,
        corner_radius=0.006,
        slot_angle_deg=0.0,
        stagger=True,
    )
    cover.visual(
        mesh_from_geometry(grille, "front_cover_grille"),
        origin=Origin(xyz=(cover_center_x, 0.0045, cover_center_z + 0.008), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="front_grille",
    )
    cover.visual(
        Box((0.010, 0.005, 0.190)),
        origin=Origin(xyz=(cover_x_right - cover_width + 0.022, 0.0075, cover_center_z - 0.010)),
        material=graphite,
        name="finger_pull",
    )

    for idx, zc in enumerate((0.146, 0.420)):
        cover.visual(
            Cylinder(radius=0.0052, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=hinge_metal,
            name=f"moving_knuckle_{idx}",
        )
        cover.visual(
            Box((0.010, 0.005, 0.030)),
            origin=Origin(xyz=(-0.009, 0.0, zc)),
            material=hinge_metal,
            name=f"moving_leaf_{idx}",
        )

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cover,
        origin=Origin(xyz=(pivot_x, pivot_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    tray = model.part("filter_tray")
    tray_w = 0.252
    tray_l = 0.120
    tray_h = 0.018
    tray_y_center = 0.045
    tray_z = 0.030
    rail_w = 0.014
    lip_y = tray_y_center + tray_l / 2.0 + 0.007

    tray.visual(
        Box((rail_w, tray_l, tray_h)),
        origin=Origin(xyz=(-tray_w / 2.0 + rail_w / 2.0, tray_y_center, tray_z)),
        material=off_white,
        name="tray_side_0",
    )
    tray.visual(
        Box((rail_w, tray_l, tray_h)),
        origin=Origin(xyz=(tray_w / 2.0 - rail_w / 2.0, tray_y_center, tray_z)),
        material=off_white,
        name="tray_side_1",
    )
    tray.visual(
        Box((tray_w, 0.014, tray_h)),
        origin=Origin(xyz=(0.0, tray_y_center - tray_l / 2.0 + 0.007, tray_z)),
        material=off_white,
        name="tray_rear_rail",
    )
    tray.visual(
        Box((tray_w, 0.014, tray_h)),
        origin=Origin(xyz=(0.0, tray_y_center + tray_l / 2.0 - 0.007, tray_z)),
        material=off_white,
        name="tray_front_rail",
    )
    filter_mesh = PerforatedPanelGeometry(
        (0.232, 0.104),
        0.003,
        hole_diameter=0.0045,
        pitch=(0.011, 0.011),
        frame=0.006,
        corner_radius=0.004,
        stagger=True,
    )
    tray.visual(
        mesh_from_geometry(filter_mesh, "washable_filter_mesh"),
        origin=Origin(xyz=(0.0, tray_y_center, tray_z + 0.007)),
        material=pale_filter,
        name="filter_mesh",
    )
    tray.visual(
        Box((0.120, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, lip_y, tray_z - 0.002)),
        material=graphite,
        name="tray_pull_lip",
    )

    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=tray,
        origin=Origin(xyz=(0.0, 0.025, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.075),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    cover = object_model.get_part("front_cover")
    tray = object_model.get_part("filter_tray")
    cover_hinge = object_model.get_articulation("cover_hinge")
    tray_slide = object_model.get_articulation("tray_slide")

    ctx.expect_overlap(
        cover,
        housing,
        axes="xz",
        min_overlap=0.25,
        name="closed cover spans the front opening",
    )
    ctx.expect_gap(
        cover,
        housing,
        axis="y",
        positive_elem="front_grille",
        negative_elem="top_wall",
        min_gap=0.0,
        max_gap=0.020,
        name="front cover sits just proud of housing",
    )

    closed_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 1.35}):
        open_cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "cover hinge swings the free edge outward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][1] > closed_cover_aabb[1][1] + 0.12,
        details=f"closed={closed_cover_aabb}, opened={open_cover_aabb}",
    )

    ctx.expect_within(
        tray,
        housing,
        axes="xz",
        margin=0.003,
        name="filter tray rides within housing width and base height",
    )
    ctx.expect_overlap(
        tray,
        housing,
        axes="y",
        min_overlap=0.10,
        name="closed tray is fully inserted in base",
    )

    closed_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.075}):
        ctx.expect_overlap(
            tray,
            housing,
            axes="y",
            min_overlap=0.040,
            name="extended tray retains slide engagement",
        )
        extended_tray_pos = ctx.part_world_position(tray)
    ctx.check(
        "filter tray slides outward from the housing base",
        closed_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] > closed_tray_pos[1] + 0.070,
        details=f"closed={closed_tray_pos}, extended={extended_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()

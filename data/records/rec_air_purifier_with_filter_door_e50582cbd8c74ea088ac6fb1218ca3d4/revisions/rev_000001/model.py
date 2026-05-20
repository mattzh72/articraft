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
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_desk_air_purifier")

    body_mat = model.material("satin_warm_white", rgba=(0.86, 0.87, 0.84, 1.0))
    dark_mat = model.material("charcoal_black", rgba=(0.035, 0.038, 0.040, 1.0))
    shadow_mat = model.material("deep_internal_shadow", rgba=(0.010, 0.011, 0.012, 1.0))
    metal_mat = model.material("brushed_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    filter_mat = model.material("pleated_filter_fiber", rgba=(0.90, 0.86, 0.72, 1.0))
    seal_mat = model.material("soft_grey_gasket", rgba=(0.20, 0.22, 0.23, 1.0))

    depth = 0.52
    width = 0.36
    height = 0.12
    wall = 0.016

    housing = model.part("housing")

    # Low-profile shell: top cover, side walls, front face frame, and a raised rear
    # service opening so the filter cassette can slide out after the bottom panel drops.
    housing.visual(
        Box((depth, width, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, height / 2 - 0.009)),
        material=body_mat,
        name="top_cover",
    )
    housing.visual(
        Box((depth, wall, height - 0.010)),
        origin=Origin(xyz=(0.0, width / 2 - wall / 2, -0.004)),
        material=body_mat,
        name="side_wall_0",
    )
    housing.visual(
        Box((depth, wall, height - 0.010)),
        origin=Origin(xyz=(0.0, -width / 2 + wall / 2, -0.004)),
        material=body_mat,
        name="side_wall_1",
    )
    housing.visual(
        Box((0.018, width, 0.017)),
        origin=Origin(xyz=(depth / 2 - 0.009, 0.0, height / 2 - 0.028)),
        material=body_mat,
        name="front_top_lip",
    )
    housing.visual(
        Box((0.018, width, 0.014)),
        origin=Origin(xyz=(depth / 2 - 0.009, 0.0, -height / 2 + 0.006)),
        material=body_mat,
        name="front_bottom_lip",
    )
    housing.visual(
        Box((0.018, 0.022, height - 0.018)),
        origin=Origin(xyz=(depth / 2 - 0.009, width / 2 - 0.011, -0.002)),
        material=body_mat,
        name="front_side_lip_0",
    )
    housing.visual(
        Box((0.018, 0.022, height - 0.018)),
        origin=Origin(xyz=(depth / 2 - 0.009, -width / 2 + 0.011, -0.002)),
        material=body_mat,
        name="front_side_lip_1",
    )
    housing.visual(
        Box((0.018, width, 0.052)),
        origin=Origin(xyz=(-depth / 2 + 0.009, 0.0, 0.025)),
        material=body_mat,
        name="rear_upper_band",
    )

    front_grille = VentGrilleGeometry(
        (0.082, 0.305),
        frame=0.009,
        face_thickness=0.004,
        duct_depth=0.018,
        duct_wall=0.0025,
        slat_pitch=0.012,
        slat_width=0.006,
        slat_angle_deg=28.0,
        corner_radius=0.004,
        slats=VentGrilleSlats(profile="airfoil", direction="down", divider_count=2, divider_width=0.003),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.0012),
        sleeve=VentGrilleSleeve(style="short", depth=0.012, wall=0.0025),
    )
    housing.visual(
        mesh_from_geometry(front_grille, "front_grille"),
        origin=Origin(xyz=(depth / 2 + 0.002, 0.0, -0.001), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark_mat,
        name="front_grille",
    )

    # Fixed portions of the exposed front hinge and wall-mounted slide rails.
    hinge_rpy = (-math.pi / 2, 0.0, 0.0)
    hinge_x = depth / 2 - 0.010
    hinge_z = -height / 2 - 0.006
    housing.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(hinge_x, -0.110, hinge_z), rpy=hinge_rpy),
        material=metal_mat,
        name="fixed_hinge_knuckle_0",
    )
    housing.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(hinge_x, 0.110, hinge_z), rpy=hinge_rpy),
        material=metal_mat,
        name="fixed_hinge_knuckle_1",
    )
    housing.visual(
        Box((0.390, 0.010, 0.010)),
        origin=Origin(xyz=(0.000, width / 2 - wall - 0.005, -0.043)),
        material=metal_mat,
        name="rail_track_0",
    )
    housing.visual(
        Box((0.390, 0.010, 0.010)),
        origin=Origin(xyz=(0.000, -width / 2 + wall + 0.005, -0.043)),
        material=metal_mat,
        name="rail_track_1",
    )
    housing.visual(
        Box((0.050, 0.012, 0.028)),
        origin=Origin(xyz=(-0.160, width / 2 - wall - 0.006, -0.039)),
        material=body_mat,
        name="rail_bracket_0",
    )
    housing.visual(
        Box((0.050, 0.012, 0.028)),
        origin=Origin(xyz=(-0.160, -width / 2 + wall + 0.006, -0.039)),
        material=body_mat,
        name="rail_bracket_1",
    )

    # A small, real pushbutton control on the top surface.
    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_mat,
        name="button_cap",
    )

    bottom_panel = model.part("bottom_panel")
    panel_leaf = SlotPatternPanelGeometry(
        (0.445, 0.285),
        0.004,
        slot_size=(0.036, 0.006),
        pitch=(0.054, 0.018),
        frame=0.018,
        corner_radius=0.004,
        stagger=True,
    )
    bottom_panel.visual(
        Box((0.478, 0.320, 0.010)),
        origin=Origin(xyz=(-0.251, 0.0, -0.002)),
        material=body_mat,
        name="panel_leaf",
    )
    bottom_panel.visual(
        mesh_from_geometry(panel_leaf, "bottom_panel_slots"),
        origin=Origin(xyz=(-0.251, 0.0, -0.008)),
        material=dark_mat,
        name="slotted_inset",
    )
    bottom_panel.visual(
        Box((0.020, 0.095, 0.008)),
        origin=Origin(xyz=(-0.012, 0.0, -0.002)),
        material=body_mat,
        name="hinge_leaf",
    )
    bottom_panel.visual(
        Cylinder(radius=0.006, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=hinge_rpy),
        material=metal_mat,
        name="moving_hinge_knuckle",
    )
    bottom_panel.visual(
        Box((0.045, 0.090, 0.006)),
        origin=Origin(xyz=(-0.448, 0.0, -0.010)),
        material=seal_mat,
        name="rear_pull_latch",
    )

    filter_drawer = model.part("filter_drawer")
    filter_drawer.visual(
        Box((0.360, 0.285, 0.008)),
        origin=Origin(xyz=(0.020, 0.0, -0.024)),
        material=dark_mat,
        name="drawer_tray",
    )
    filter_drawer.visual(
        Box((0.345, 0.012, 0.014)),
        origin=Origin(xyz=(0.020, 0.144, -0.015)),
        material=metal_mat,
        name="runner_0",
    )
    filter_drawer.visual(
        Box((0.345, 0.012, 0.014)),
        origin=Origin(xyz=(0.020, -0.144, -0.015)),
        material=metal_mat,
        name="runner_1",
    )
    filter_drawer.visual(
        Box((0.300, 0.235, 0.020)),
        origin=Origin(xyz=(0.035, 0.0, -0.010)),
        material=filter_mat,
        name="filter_media",
    )
    for index, x in enumerate((-0.095, -0.055, -0.015, 0.025, 0.065, 0.105, 0.145)):
        filter_drawer.visual(
            Box((0.006, 0.235, 0.024)),
            origin=Origin(xyz=(x, 0.0, -0.008)),
            material=filter_mat,
            name=f"pleat_{index}",
        )
    filter_drawer.visual(
        Box((0.012, 0.275, 0.030)),
        origin=Origin(xyz=(-0.158, 0.0, -0.014)),
        material=dark_mat,
        name="rear_frame",
    )
    filter_drawer.visual(
        Box((0.012, 0.275, 0.030)),
        origin=Origin(xyz=(0.205, 0.0, -0.014)),
        material=dark_mat,
        name="front_frame",
    )
    filter_drawer.visual(
        Box((0.050, 0.135, 0.020)),
        origin=Origin(xyz=(-0.184, 0.0, -0.028)),
        material=seal_mat,
        name="drawer_pull",
    )

    model.articulation(
        "housing_to_power_button",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=power_button,
        origin=Origin(xyz=(-0.180, 0.105, height / 2)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.04, lower=0.0, upper=0.004),
    )
    model.articulation(
        "housing_to_bottom_panel",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=bottom_panel,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "housing_to_filter_drawer",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_drawer,
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.240),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    bottom_panel = object_model.get_part("bottom_panel")
    filter_drawer = object_model.get_part("filter_drawer")
    power_button = object_model.get_part("power_button")
    bottom_hinge = object_model.get_articulation("housing_to_bottom_panel")
    drawer_slide = object_model.get_articulation("housing_to_filter_drawer")

    ctx.expect_gap(
        housing,
        bottom_panel,
        axis="z",
        positive_elem="front_bottom_lip",
        negative_elem="panel_leaf",
        min_gap=0.001,
        max_gap=0.006,
        name="closed bottom panel sits just below the housing lip",
    )
    ctx.expect_overlap(
        bottom_panel,
        housing,
        axes="xy",
        elem_a="panel_leaf",
        elem_b="top_cover",
        min_overlap=0.300,
        name="bottom panel covers the purifier footprint",
    )
    ctx.expect_within(
        filter_drawer,
        housing,
        axes="y",
        inner_elem="filter_media",
        outer_elem="top_cover",
        margin=0.0,
        name="filter cassette is laterally inside the low-profile shell",
    )
    ctx.expect_overlap(
        filter_drawer,
        housing,
        axes="x",
        elem_a="filter_media",
        elem_b="top_cover",
        min_overlap=0.250,
        name="closed filter drawer remains inserted under the grille housing",
    )
    ctx.expect_contact(
        power_button,
        housing,
        elem_a="button_cap",
        elem_b="top_cover",
        contact_tol=0.001,
        name="power button cap is seated on the top cover",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(bottom_panel, elem="panel_leaf")
    rest_drawer_pos = ctx.part_world_position(filter_drawer)
    with ctx.pose({bottom_hinge: 1.20}):
        open_panel_aabb = ctx.part_element_world_aabb(bottom_panel, elem="panel_leaf")
    with ctx.pose({drawer_slide: 0.240}):
        extended_drawer_pos = ctx.part_world_position(filter_drawer)
        ctx.expect_overlap(
            filter_drawer,
            housing,
            axes="x",
            elem_a="runner_0",
            elem_b="rail_track_0",
            min_overlap=0.080,
            name="extended drawer still rides on the side rail",
        )
    with ctx.pose({bottom_hinge: 1.20, drawer_slide: 0.240}):
        ctx.expect_gap(
            bottom_panel,
            filter_drawer,
            axis="x",
            positive_elem="panel_leaf",
            negative_elem="front_frame",
            min_gap=0.050,
            name="open panel swings clear of the rearward sliding drawer",
        )

    ctx.check(
        "bottom panel drops downward from the front hinge",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[0][2] < closed_panel_aabb[0][2] - 0.160,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )
    ctx.check(
        "filter drawer slides out toward the rear service edge",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] < rest_drawer_pos[0] - 0.180,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()

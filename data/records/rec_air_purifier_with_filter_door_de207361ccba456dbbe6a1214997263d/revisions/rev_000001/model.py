from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WIDTH = 0.62
HEIGHT = 0.90
DEPTH = 0.075
PANEL_W = 0.56
PANEL_H = 0.78
PANEL_BOTTOM = (HEIGHT - PANEL_H) / 2.0
HINGE_X = -PANEL_W / 2.0
HINGE_Y = DEPTH + 0.007
HINGE_Z = PANEL_BOTTOM
CARTRIDGE_TRAVEL = 0.095


def _housing_shell() -> cq.Workplane:
    """Shallow open-front tray that reads as a thin wall-panel appliance."""
    wall = 0.018
    back = 0.012
    outer = cq.Workplane("XY").box(WIDTH, DEPTH, HEIGHT).translate((0.0, DEPTH / 2.0, HEIGHT / 2.0))
    front_cut = (
        cq.Workplane("XY")
        .box(WIDTH - 2.0 * wall, DEPTH + 0.030, HEIGHT - 2.0 * wall)
        .translate((0.0, back + (DEPTH + 0.030) / 2.0, HEIGHT / 2.0))
    )
    shell = outer.cut(front_cut)
    # Soften the large appliance perimeter without changing the mounting faces.
    return shell.edges("|Z").fillet(0.006)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_panel_air_purifier")

    wall_mat = model.material("painted_wall", rgba=(0.86, 0.86, 0.82, 1.0))
    shell_mat = model.material("warm_white_plastic", rgba=(0.92, 0.93, 0.90, 1.0))
    trim_mat = model.material("soft_gray_trim", rgba=(0.58, 0.60, 0.59, 1.0))
    dark_mat = model.material("dark_shadow_plastic", rgba=(0.08, 0.09, 0.09, 1.0))
    filter_mat = model.material("filter_media", rgba=(0.72, 0.67, 0.55, 1.0))
    rail_mat = model.material("guide_rail_nylon", rgba=(0.18, 0.19, 0.19, 1.0))
    metal_mat = model.material("brushed_pin_metal", rgba=(0.62, 0.64, 0.64, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.96, 0.026, 1.16)),
        origin=Origin(xyz=(0.0, -0.012, HEIGHT / 2.0)),
        material=wall_mat,
        name="wall_slab",
    )
    housing.visual(
        mesh_from_cadquery(_housing_shell(), "housing_shell", tolerance=0.001),
        material=shell_mat,
        name="housing_shell",
    )
    # A dark recessed back panel makes the open service cavity visible behind the hinged front.
    housing.visual(
        Box((WIDTH - 0.075, 0.004, HEIGHT - 0.12)),
        origin=Origin(xyz=(0.0, 0.012, HEIGHT / 2.0)),
        material=dark_mat,
        name="rear_cavity",
    )
    # Bottom guide rails, slightly proud of the floor of the cavity.
    housing.visual(
        Box((0.030, 0.060, 0.014)),
        origin=Origin(xyz=(-0.17, 0.043, 0.114)),
        material=rail_mat,
        name="guide_rail_0",
    )
    housing.visual(
        Box((0.030, 0.060, 0.014)),
        origin=Origin(xyz=(0.17, 0.043, 0.114)),
        material=rail_mat,
        name="guide_rail_1",
    )
    # Fixed hinge leaf and two stationary knuckles on the housing side.
    housing.visual(
        Box((0.026, 0.006, PANEL_H + 0.018)),
        origin=Origin(xyz=(HINGE_X - 0.018, 0.075, HINGE_Z + PANEL_H / 2.0)),
        material=metal_mat,
        name="fixed_hinge_leaf",
    )
    housing.visual(
        Cylinder(radius=0.0075, length=0.19),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z + 0.115)),
        material=metal_mat,
        name="lower_hinge_knuckle",
    )
    housing.visual(
        Cylinder(radius=0.0075, length=0.19),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z + PANEL_H - 0.115)),
        material=metal_mat,
        name="upper_hinge_knuckle",
    )

    filter_panel = model.part("filter_panel")
    panel_face = SlotPatternPanelGeometry(
        (PANEL_W, PANEL_H),
        0.006,
        slot_size=(0.040, 0.008),
        pitch=(0.060, 0.025),
        frame=0.035,
        corner_radius=0.010,
        slot_angle_deg=0.0,
        stagger=True,
    )
    filter_panel.visual(
        mesh_from_geometry(panel_face, "slotted_filter_panel"),
        origin=Origin(xyz=(PANEL_W / 2.0 + 0.014, 0.003, PANEL_H / 2.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=shell_mat,
        name="slotted_face",
    )
    # Raised trim frame and slim pull lip are separate materials but overlap the face locally.
    filter_panel.visual(
        Box((PANEL_W + 0.020, 0.010, 0.024)),
        origin=Origin(xyz=(PANEL_W / 2.0, 0.006, PANEL_H + 0.012)),
        material=trim_mat,
        name="top_trim",
    )
    filter_panel.visual(
        Box((PANEL_W + 0.020, 0.010, 0.024)),
        origin=Origin(xyz=(PANEL_W / 2.0, 0.006, -0.012)),
        material=trim_mat,
        name="bottom_trim",
    )
    filter_panel.visual(
        Box((0.024, 0.010, PANEL_H + 0.020)),
        origin=Origin(xyz=(0.021, 0.006, PANEL_H / 2.0)),
        material=trim_mat,
        name="hinge_trim",
    )
    filter_panel.visual(
        Box((0.024, 0.010, PANEL_H + 0.020)),
        origin=Origin(xyz=(PANEL_W + 0.012, 0.006, PANEL_H / 2.0)),
        material=trim_mat,
        name="latch_trim",
    )
    filter_panel.visual(
        Box((0.018, 0.018, 0.090)),
        origin=Origin(xyz=(PANEL_W + 0.003, 0.014, PANEL_H / 2.0)),
        material=dark_mat,
        name="finger_pull",
    )
    filter_panel.visual(
        Box((0.014, 0.004, 0.340)),
        origin=Origin(xyz=(0.004, 0.001, PANEL_H / 2.0)),
        material=metal_mat,
        name="moving_hinge_leaf",
    )
    filter_panel.visual(
        Cylinder(radius=0.0065, length=0.33),
        origin=Origin(xyz=(0.0, 0.0, PANEL_H / 2.0)),
        material=metal_mat,
        name="middle_hinge_knuckle",
    )

    cartridge = model.part("filter_cartridge")
    cartridge.visual(
        Box((0.430, 0.035, 0.510)),
        origin=Origin(xyz=(0.0, 0.0, 0.410)),
        material=filter_mat,
        name="pleated_media",
    )
    cartridge.visual(
        Box((0.480, 0.042, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.680)),
        material=dark_mat,
        name="top_frame",
    )
    cartridge.visual(
        Box((0.480, 0.042, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=dark_mat,
        name="bottom_frame",
    )
    cartridge.visual(
        Box((0.030, 0.042, 0.570)),
        origin=Origin(xyz=(-0.240, 0.0, 0.410)),
        material=dark_mat,
        name="side_frame_0",
    )
    cartridge.visual(
        Box((0.030, 0.042, 0.570)),
        origin=Origin(xyz=(0.240, 0.0, 0.410)),
        material=dark_mat,
        name="side_frame_1",
    )
    cartridge.visual(
        Box((0.022, 0.050, 0.012)),
        origin=Origin(xyz=(-0.17, 0.0, 0.127)),
        material=rail_mat,
        name="runner_0",
    )
    cartridge.visual(
        Box((0.022, 0.050, 0.012)),
        origin=Origin(xyz=(0.17, 0.0, 0.127)),
        material=rail_mat,
        name="runner_1",
    )

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=filter_panel,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.75),
    )
    model.articulation(
        "cartridge_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=cartridge,
        origin=Origin(xyz=(0.0, 0.043, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=CARTRIDGE_TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    panel = object_model.get_part("filter_panel")
    cartridge = object_model.get_part("filter_cartridge")
    hinge = object_model.get_articulation("panel_hinge")
    slide = object_model.get_articulation("cartridge_slide")

    with ctx.pose({hinge: 0.0, slide: 0.0}):
        ctx.expect_gap(
            panel,
            housing,
            axis="y",
            min_gap=0.001,
            max_gap=0.012,
            positive_elem="slotted_face",
            negative_elem="housing_shell",
            name="closed panel sits just proud of housing",
        )
        ctx.expect_overlap(
            panel,
            housing,
            axes="xz",
            min_overlap=0.45,
            elem_a="slotted_face",
            elem_b="housing_shell",
            name="front panel covers the service opening",
        )
        ctx.expect_contact(
            cartridge,
            housing,
            elem_a="runner_0",
            elem_b="guide_rail_0",
            contact_tol=0.001,
            name="cartridge runner rests on lower guide rail",
        )

    closed_panel_aabb = ctx.part_element_world_aabb(panel, elem="slotted_face")
    closed_cart_pos = ctx.part_world_position(cartridge)
    with ctx.pose({hinge: 1.35, slide: CARTRIDGE_TRAVEL}):
        opened_panel_aabb = ctx.part_element_world_aabb(panel, elem="slotted_face")
        extended_cart_pos = ctx.part_world_position(cartridge)

    ctx.check(
        "hinged panel swings outward",
        closed_panel_aabb is not None
        and opened_panel_aabb is not None
        and opened_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.25,
        details=f"closed={closed_panel_aabb}, opened={opened_panel_aabb}",
    )
    ctx.check(
        "filter cartridge slides out",
        closed_cart_pos is not None
        and extended_cart_pos is not None
        and extended_cart_pos[1] > closed_cart_pos[1] + 0.080,
        details=f"closed={closed_cart_pos}, extended={extended_cart_pos}",
    )

    return ctx.report()


object_model = build_object_model()

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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_specimen_chest_freezer")

    medical_white = Material("powder_coated_medical_white", color=(0.92, 0.96, 0.98, 1.0))
    liner_gray = Material("brushed_stainless_liner", color=(0.68, 0.72, 0.73, 1.0))
    dark_rubber = Material("dark_rubber_gasket", color=(0.015, 0.018, 0.020, 1.0))
    hinge_steel = Material("satin_hinge_steel", color=(0.55, 0.57, 0.58, 1.0))
    panel_black = Material("black_control_glass", color=(0.02, 0.025, 0.03, 1.0))
    cold_blue = Material("cold_blue_display", color=(0.05, 0.38, 0.85, 1.0))
    tray_blue = Material("translucent_polycarbonate_tray", color=(0.55, 0.82, 0.93, 0.74))

    # Realistic chest-freezer proportions: long and narrow in X/Y, waist-height in Z.
    length = 1.20
    width = 0.42
    height = 0.72
    wall_t = 0.045
    floor_t = 0.090
    rim_t = 0.025

    body = model.part("body")

    wall_h = height - rim_t
    # Insulated outer tub, deliberately open at the top.
    body.visual(
        Box((length, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall_t / 2.0, wall_h / 2.0)),
        material=medical_white,
        name="front_wall",
    )
    body.visual(
        Box((length, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall_t / 2.0, wall_h / 2.0)),
        material=medical_white,
        name="rear_wall",
    )
    body.visual(
        Box((wall_t, width, wall_h)),
        origin=Origin(xyz=(-length / 2.0 + wall_t / 2.0, 0.0, wall_h / 2.0)),
        material=medical_white,
        name="end_wall_0",
    )
    body.visual(
        Box((wall_t, width, wall_h)),
        origin=Origin(xyz=(length / 2.0 - wall_t / 2.0, 0.0, wall_h / 2.0)),
        material=medical_white,
        name="end_wall_1",
    )
    body.visual(
        Box((length, width, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t / 2.0)),
        material=medical_white,
        name="insulated_floor",
    )

    rim_z = height - rim_t / 2.0
    body.visual(
        Box((length + 0.035, 0.070, rim_t)),
        origin=Origin(xyz=(0.0, -width / 2.0 + 0.020, rim_z)),
        material=medical_white,
        name="front_rim",
    )
    body.visual(
        Box((length + 0.035, 0.070, rim_t)),
        origin=Origin(xyz=(0.0, width / 2.0 - 0.020, rim_z)),
        material=medical_white,
        name="rear_rim",
    )
    body.visual(
        Box((0.070, width + 0.035, rim_t)),
        origin=Origin(xyz=(-length / 2.0 + 0.020, 0.0, rim_z)),
        material=medical_white,
        name="end_rim_0",
    )
    body.visual(
        Box((0.070, width + 0.035, rim_t)),
        origin=Origin(xyz=(length / 2.0 - 0.020, 0.0, rim_z)),
        material=medical_white,
        name="end_rim_1",
    )

    inner_l = length - 2.0 * wall_t - 0.020
    inner_w = width - 2.0 * wall_t - 0.020
    liner_h = height - floor_t - rim_t
    body.visual(
        Box((inner_l, inner_w, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, floor_t + 0.003)),
        material=liner_gray,
        name="liner_floor",
    )
    body.visual(
        Box((inner_l, 0.006, liner_h)),
        origin=Origin(xyz=(0.0, -inner_w / 2.0, floor_t + liner_h / 2.0)),
        material=liner_gray,
        name="front_liner",
    )
    body.visual(
        Box((inner_l, 0.006, liner_h)),
        origin=Origin(xyz=(0.0, inner_w / 2.0, floor_t + liner_h / 2.0)),
        material=liner_gray,
        name="rear_liner",
    )
    body.visual(
        Box((0.006, inner_w, liner_h)),
        origin=Origin(xyz=(-inner_l / 2.0, 0.0, floor_t + liner_h / 2.0)),
        material=liner_gray,
        name="end_liner_0",
    )
    body.visual(
        Box((0.006, inner_w, liner_h)),
        origin=Origin(xyz=(inner_l / 2.0, 0.0, floor_t + liner_h / 2.0)),
        material=liner_gray,
        name="end_liner_1",
    )

    # Interior full-length guide rails for the adjustable divider tray.
    rail_z = floor_t + 0.015
    for y, rail_name in [
        (-inner_w / 2.0 + 0.030, "divider_rail_0"),
        (inner_w / 2.0 - 0.030, "divider_rail_1"),
    ]:
        body.visual(
            Box((inner_l - 0.070, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, y, rail_z)),
            material=hinge_steel,
            name=rail_name,
        )

    # Front service/control area: a dark digital panel and a lower condenser grille.
    body.visual(
        Box((0.260, 0.010, 0.105)),
        origin=Origin(xyz=(-0.300, -width / 2.0 - 0.004, 0.525)),
        material=panel_black,
        name="control_panel",
    )
    body.visual(
        Box((0.135, 0.012, 0.040)),
        origin=Origin(xyz=(-0.335, -width / 2.0 - 0.010, 0.540)),
        material=cold_blue,
        name="temperature_display",
    )
    for i, x in enumerate([-0.240, -0.205, -0.170]):
        body.visual(
            Box((0.018, 0.012, 0.018)),
            origin=Origin(xyz=(x, -width / 2.0 - 0.011, 0.505)),
            material=liner_gray,
            name=f"membrane_key_{i}",
        )
    body.visual(
        Box((0.320, 0.010, 0.115)),
        origin=Origin(xyz=(0.330, -width / 2.0 - 0.004, 0.190)),
        material=panel_black,
        name="condenser_grille",
    )
    for i in range(6):
        body.visual(
            Box((0.280, 0.012, 0.007)),
            origin=Origin(xyz=(0.330, -width / 2.0 - 0.011, 0.150 + i * 0.016)),
            material=medical_white,
            name=f"grille_louver_{i}",
        )

    # Low feet keep the freezer visibly supported off the floor.
    for i, x in enumerate([-0.480, 0.480]):
        for j, y in enumerate([-0.150, 0.150]):
            body.visual(
                Box((0.100, 0.070, 0.045)),
                origin=Origin(xyz=(x, y, -0.0225)),
                material=dark_rubber,
                name=f"foot_{i}_{j}",
            )

    hinge_y = width / 2.0 + 0.040
    hinge_z = height + 0.051
    hinge_xs = (-0.380, 0.380)
    hinge_axis_rpy = (0.0, math.pi / 2.0, 0.0)
    for h, hx in enumerate(hinge_xs):
        body.visual(
            Box((0.180, 0.008, 0.095)),
            origin=Origin(xyz=(hx, width / 2.0 + 0.004, height + 0.010)),
            material=hinge_steel,
            name=f"hinge_leaf_{h}",
        )
        for k, dx in enumerate([-0.055, 0.055]):
            body.visual(
                Box((0.045, 0.040, 0.010)),
                origin=Origin(xyz=(hx + dx, width / 2.0 + 0.025, hinge_z)),
                material=hinge_steel,
                name=f"hinge_bridge_{h}_{k}",
            )
            body.visual(
                Cylinder(radius=0.012, length=0.045),
                origin=Origin(xyz=(hx + dx, hinge_y, hinge_z), rpy=hinge_axis_rpy),
                material=hinge_steel,
                name=f"hinge_knuckle_{h}_{k}",
            )

    lid = model.part("lid")
    lid_length = length + 0.080
    lid_width = width + 0.100
    lid_thick = 0.075
    lid_rear_clearance = 0.045
    lid_panel = (
        cq.Workplane("XY")
        .box(lid_length, lid_width, lid_thick)
        .edges("|Z")
        .fillet(0.022)
        .edges(">Z or <Z")
        .fillet(0.006)
    )
    lid.visual(
        mesh_from_cadquery(lid_panel, "lid_panel", tolerance=0.0015, angular_tolerance=0.12),
        origin=Origin(xyz=(0.0, -lid_width / 2.0 - lid_rear_clearance, 0.0)),
        material=medical_white,
        name="lid_panel",
    )
    # A black compression gasket under the lid tracks the top rim.
    gasket_z = -0.044
    lid.visual(
        Box((lid_length - 0.120, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, -width / 2.0 + 0.020 - hinge_y, gasket_z)),
        material=dark_rubber,
        name="front_gasket",
    )
    lid.visual(
        Box((lid_length - 0.120, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, width / 2.0 - 0.020 - hinge_y, gasket_z)),
        material=dark_rubber,
        name="rear_gasket",
    )
    lid.visual(
        Box((0.030, lid_width - 0.140, 0.014)),
        origin=Origin(xyz=(-lid_length / 2.0 + 0.060, -hinge_y, gasket_z)),
        material=dark_rubber,
        name="end_gasket_0",
    )
    lid.visual(
        Box((0.030, lid_width - 0.140, 0.014)),
        origin=Origin(xyz=(lid_length / 2.0 - 0.060, -hinge_y, gasket_z)),
        material=dark_rubber,
        name="end_gasket_1",
    )
    lid.visual(
        Box((0.560, 0.042, 0.045)),
        origin=Origin(xyz=(0.0, -lid_width - 0.014 - lid_rear_clearance, -0.006)),
        material=dark_rubber,
        name="front_handle",
    )
    for h, hx in enumerate(hinge_xs):
        lid.visual(
            Box((0.180, 0.065, 0.006)),
            origin=Origin(xyz=(hx, -0.040, 0.040)),
            material=hinge_steel,
            name=f"hinge_strap_{h}",
        )
        lid.visual(
            Box((0.055, 0.012, 0.040)),
            origin=Origin(xyz=(hx, -0.010, 0.020)),
            material=hinge_steel,
            name=f"hinge_tang_{h}",
        )
        lid.visual(
            Cylinder(radius=0.0105, length=0.055),
            origin=Origin(xyz=(hx, 0.0, 0.0), rpy=hinge_axis_rpy),
            material=hinge_steel,
            name=f"hinge_knuckle_{h}",
        )

    divider = model.part("divider")
    divider_width = inner_w - 0.055
    divider.visual(
        Box((0.090, divider_width, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=tray_blue,
        name="slide_sled",
    )
    divider.visual(
        Box((0.034, divider_width - 0.020, 0.430)),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=tray_blue,
        name="partition_panel",
    )
    divider.visual(
        Box((0.060, 0.150, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        material=tray_blue,
        name="thumb_tab",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=1.25),
    )

    divider_start_x = -inner_l / 2.0 + 0.085
    divider_z = rail_z + 0.021
    divider_travel = inner_l - 0.170
    model.articulation(
        "body_to_divider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=divider,
        origin=Origin(xyz=(divider_start_x, 0.0, divider_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.18, lower=0.0, upper=divider_travel),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    divider = object_model.get_part("divider")
    lid_hinge = object_model.get_articulation("body_to_lid")
    divider_slide = object_model.get_articulation("body_to_divider")

    body_visuals = {visual.name for visual in body.visuals}
    lid_visuals = {visual.name for visual in lid.visuals}
    ctx.check(
        "two exposed barrel hinges",
        all(
            name in body_visuals
            for name in (
                "hinge_knuckle_0_0",
                "hinge_knuckle_0_1",
                "hinge_knuckle_1_0",
                "hinge_knuckle_1_1",
            )
        )
        and all(name in lid_visuals for name in ("hinge_knuckle_0", "hinge_knuckle_1")),
        details=f"body={sorted(body_visuals)}, lid={sorted(lid_visuals)}",
    )
    ctx.check(
        "lid hinge is rear long edge revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in lid_hinge.axis) == (-1.0, 0.0, 0.0)
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper >= 1.2,
        details=f"type={lid_hinge.articulation_type}, axis={lid_hinge.axis}, limits={lid_hinge.motion_limits}",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a="front_gasket",
        elem_b="front_rim",
        contact_tol=0.002,
        name="closed lid gasket seats on rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="front_rim",
        min_overlap=0.050,
        name="lid covers the narrow freezer mouth",
    )

    rest_handle = ctx.part_element_world_aabb(lid, elem="front_handle")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_handle = ctx.part_element_world_aabb(lid, elem="front_handle")
    ctx.check(
        "lid opens upward from rear hinges",
        rest_handle is not None
        and open_handle is not None
        and open_handle[1][2] > rest_handle[1][2] + 0.20,
        details=f"rest_handle={rest_handle}, open_handle={open_handle}",
    )

    ctx.check(
        "divider slide is full-length prismatic",
        divider_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 3) for v in divider_slide.axis) == (1.0, 0.0, 0.0)
        and divider_slide.motion_limits.lower == 0.0
        and divider_slide.motion_limits.upper > 0.85,
        details=f"type={divider_slide.articulation_type}, axis={divider_slide.axis}, limits={divider_slide.motion_limits}",
    )
    ctx.expect_contact(
        divider,
        body,
        elem_a="slide_sled",
        elem_b="divider_rail_0",
        contact_tol=0.003,
        name="divider sled rides on guide rail",
    )
    ctx.expect_gap(
        divider,
        body,
        axis="y",
        positive_elem="partition_panel",
        negative_elem="front_liner",
        min_gap=0.010,
        name="divider clears front liner",
    )
    ctx.expect_gap(
        body,
        divider,
        axis="y",
        positive_elem="rear_liner",
        negative_elem="partition_panel",
        min_gap=0.010,
        name="divider clears rear liner",
    )
    ctx.expect_gap(
        divider,
        body,
        axis="x",
        positive_elem="slide_sled",
        negative_elem="end_liner_0",
        min_gap=0.020,
        name="divider starts inside first end wall",
    )

    rest_divider = ctx.part_world_position(divider)
    with ctx.pose({divider_slide: divider_slide.motion_limits.upper}):
        extended_divider = ctx.part_world_position(divider)
        ctx.expect_gap(
            body,
            divider,
            axis="x",
            positive_elem="end_liner_1",
            negative_elem="slide_sled",
            min_gap=0.020,
            name="divider reaches inside far end wall",
        )
        ctx.expect_gap(
            divider,
            body,
            axis="y",
            positive_elem="partition_panel",
            negative_elem="front_liner",
            min_gap=0.010,
            name="extended divider clears front liner",
        )
        ctx.expect_gap(
            body,
            divider,
            axis="y",
            positive_elem="rear_liner",
            negative_elem="partition_panel",
            min_gap=0.010,
            name="extended divider clears rear liner",
        )
    ctx.check(
        "divider traverses the interior length",
        rest_divider is not None
        and extended_divider is not None
        and extended_divider[0] > rest_divider[0] + 0.85,
        details=f"rest={rest_divider}, extended={extended_divider}",
    )

    return ctx.report()


object_model = build_object_model()

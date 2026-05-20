from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A simple ring whose local bottom is z=0."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _rounded_slot_plate(length: float, width: float, height: float) -> cq.Workplane:
    """Capsule-shaped plate with local bottom at z=0."""
    return cq.Workplane("XY").slot2D(length, width).extrude(height)


def _body_shell() -> cq.Workplane:
    body_r = 0.245
    inner_r = 0.204
    body_h = 0.162

    outer = cq.Workplane("XY").circle(body_r).extrude(body_h)

    cavity = (
        cq.Workplane("XY")
        .workplane(offset=0.035)
        .circle(inner_r)
        .extrude(body_h + 0.030)
    )
    shell = outer.cut(cavity)

    right_outer = (
        cq.Workplane("YZ")
        .center(0.0, 0.095)
        .slot2D(0.125, 0.054)
        .extrude(0.056)
    )
    right_inner = (
        cq.Workplane("YZ")
        .center(0.0, 0.095)
        .slot2D(0.072, 0.022)
        .extrude(0.070)
    )
    right_handle = right_outer.cut(right_inner).translate((body_r - 0.010, 0.0, 0.0))

    left_outer = (
        cq.Workplane("YZ")
        .center(0.0, 0.095)
        .slot2D(0.125, 0.054)
        .extrude(-0.056)
    )
    left_inner = (
        cq.Workplane("YZ")
        .center(0.0, 0.095)
        .slot2D(0.072, 0.022)
        .extrude(-0.070)
    )
    left_handle = left_outer.cut(left_inner).translate((-body_r + 0.010, 0.0, 0.0))

    return shell.union(right_handle).union(left_handle)


def _lid_dome() -> cq.Workplane:
    """Hollow spherical-cap dome with a small flat top pad for the vent slider."""
    base_radius = 0.235
    dome_h = 0.130
    wall = 0.010
    sphere_r = (base_radius * base_radius + dome_h * dome_h) / (2.0 * dome_h)
    sphere_z = dome_h - sphere_r

    upper_clip = cq.Workplane("XY").box(0.700, 0.700, 0.500).translate((0.0, 0.0, 0.250))
    outer = cq.Workplane("XY").sphere(sphere_r).translate((0.0, 0.0, sphere_z)).intersect(upper_clip)
    inner = cq.Workplane("XY").sphere(sphere_r - wall).translate((0.0, 0.0, sphere_z))
    dome = outer.cut(inner)

    top_pad = (
        cq.Workplane("XY")
        .workplane(offset=dome_h - 0.004)
        .circle(0.078)
        .extrude(0.014)
    )
    return dome.union(top_pad)


def _vent_cover() -> cq.Workplane:
    plate = _rounded_slot_plate(0.070, 0.038, 0.007)
    thumb_tab = (
        cq.Workplane("XY")
        .ellipse(0.018, 0.010)
        .extrude(0.014)
        .translate((-0.012, 0.0, 0.007))
    )
    return plate.union(thumb_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dutch_oven_tabletop_combination_oven")

    ceramic = model.material("warm_white_ceramic", rgba=(0.86, 0.79, 0.66, 1.0))
    dark_ceramic = model.material("charcoal_ceramic", rgba=(0.05, 0.047, 0.042, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.61, 0.57, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    body_r = 0.245
    body_h = 0.162
    hinge_y = body_r + 0.005
    hinge_z = body_h + 0.018
    lid_center_y = -hinge_y
    lid_base_z = -0.018
    top_track_z = 0.122

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "body_shell", tolerance=0.0012, angular_tolerance=0.08),
        material=ceramic,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.235, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=black_rubber,
        name="base_collar",
    )
    body.visual(
        Cylinder(radius=0.181, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=dark_ceramic,
        name="heating_plate",
    )
    body.visual(
        mesh_from_cadquery(_annular_cylinder(0.246, 0.203, 0.008), "top_rim"),
        origin=Origin(xyz=(0.0, 0.0, body_h - 0.008)),
        material=brushed_steel,
        name="top_rim",
    )

    for index, x in enumerate((-0.066, 0.066)):
        body.visual(
            Box((0.060, 0.050, 0.012)),
            origin=Origin(xyz=(x, hinge_y + 0.006, body_h - 0.008)),
            material=brushed_steel,
            name=f"hinge_foot_{index}",
        )
        body.visual(
            Box((0.060, 0.030, 0.036)),
            origin=Origin(xyz=(x, hinge_y + 0.017, body_h + 0.004)),
            material=brushed_steel,
            name=f"hinge_post_{index}",
        )
        body.visual(
            Cylinder(radius=0.011, length=0.055),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"body_knuckle_{index}",
        )
        body.visual(
            Box((0.056, 0.040, 0.006)),
            origin=Origin(xyz=(x, hinge_y + 0.017, hinge_z - 0.012)),
            material=brushed_steel,
            name=f"body_hinge_leaf_{index}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_dome(), "lid_dome", tolerance=0.0012, angular_tolerance=0.08),
        origin=Origin(xyz=(0.0, lid_center_y, lid_base_z)),
        material=ceramic,
        name="lid_dome",
    )
    lid.visual(
        mesh_from_cadquery(_annular_cylinder(0.238, 0.210, 0.010), "lid_rim"),
        origin=Origin(xyz=(0.0, lid_center_y, lid_base_z)),
        material=brushed_steel,
        name="lid_rim",
    )
    lid.visual(
        Cylinder(radius=0.011, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="lid_knuckle",
    )
    lid.visual(
        Box((0.060, 0.048, 0.006)),
        origin=Origin(xyz=(0.0, -0.024, -0.012)),
        material=brushed_steel,
        name="lid_hinge_leaf",
    )
    lid.visual(
        mesh_from_cadquery(_rounded_slot_plate(0.180, 0.054, 0.004), "vent_track"),
        origin=Origin(xyz=(0.0, lid_center_y, top_track_z)),
        material=brushed_steel,
        name="vent_track",
    )
    lid.visual(
        mesh_from_cadquery(_rounded_slot_plate(0.040, 0.011, 0.0008), "vent_slot"),
        origin=Origin(xyz=(0.0, lid_center_y, top_track_z + 0.004)),
        material=dark_ceramic,
        name="vent_slot",
    )

    vent_cover = model.part("vent_cover")
    vent_cover.visual(
        mesh_from_cadquery(_vent_cover(), "vent_cover_plate"),
        material=dark_ceramic,
        name="cover_plate",
    )

    lid_hinge = model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.18),
    )
    vent_slide = model.articulation(
        "lid_to_vent_cover",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=vent_cover,
        origin=Origin(xyz=(0.0, lid_center_y, top_track_z + 0.0048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.060),
    )

    # Keep local variables referenced in tests via model lookup; assignments document intent.
    _ = (lid_hinge, vent_slide)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    vent_cover = object_model.get_part("vent_cover")
    lid_hinge = object_model.get_articulation("body_to_lid")
    vent_slide = object_model.get_articulation("lid_to_vent_cover")

    with ctx.pose({lid_hinge: 0.0, vent_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_dome",
            negative_elem="top_rim",
            max_gap=0.0015,
            max_penetration=0.0,
            name="closed lid seats on metal rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_dome",
            elem_b="body_shell",
            min_overlap=0.35,
            name="round lid footprint covers the ceramic body",
        )
        ctx.expect_contact(
            vent_cover,
            lid,
            elem_a="cover_plate",
            elem_b="vent_track",
            contact_tol=0.001,
            name="sliding vent cover rides on the top track",
        )
        ctx.expect_overlap(
            vent_cover,
            lid,
            axes="xy",
            elem_a="cover_plate",
            elem_b="vent_slot",
            min_overlap=0.008,
            name="vent cover rests over the steam slot",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    closed_cover_pos = ctx.part_world_position(vent_cover)
    with ctx.pose({lid_hinge: 0.95, vent_slide: 0.060}):
        opened_aabb = ctx.part_world_aabb(lid)
        open_cover_pos = ctx.part_world_position(vent_cover)
        ctx.expect_gap(
            vent_cover,
            lid,
            axis="x",
            positive_elem="cover_plate",
            negative_elem="vent_slot",
            min_gap=0.002,
            name="slider exposes the steam vent at full travel",
        )

    ctx.check(
        "domed lid hinges upward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.08,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )
    ctx.check(
        "vent cover translates along the lid top",
        closed_cover_pos is not None
        and open_cover_pos is not None
        and open_cover_pos[0] > closed_cover_pos[0] + 0.045,
        details=f"closed={closed_cover_pos}, open={open_cover_pos}",
    )

    return ctx.report()


object_model = build_object_model()

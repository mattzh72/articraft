from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """Simple open cylindrical shell/ring in meters, extruded upward from z=0."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="simple_rice_cooker")

    porcelain = Material("warm_white_enamel", rgba=(0.92, 0.88, 0.78, 1.0))
    dark_plastic = Material("charcoal_plastic", rgba=(0.035, 0.038, 0.040, 1.0))
    black = Material("black_marking", rgba=(0.005, 0.005, 0.006, 1.0))
    steel = Material("brushed_steel", rgba=(0.68, 0.70, 0.68, 1.0))
    amber = Material("amber_cook_mark", rgba=(1.0, 0.52, 0.06, 1.0))
    green = Material("green_warm_mark", rgba=(0.10, 0.75, 0.24, 1.0))

    body_radius = 0.165
    body_bottom = 0.020
    body_height = 0.235
    body_top = body_bottom + body_height
    hinge_y = body_radius + 0.015
    hinge_z = body_top

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(
            _annular_cylinder(body_radius, 0.137, body_height),
            "body_wall",
            tolerance=0.001,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.0, body_bottom)),
        material=porcelain,
        name="body_wall",
    )
    body.visual(
        mesh_from_cadquery(
            _annular_cylinder(body_radius + 0.006, 0.127, 0.020),
            "top_rim",
            tolerance=0.001,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.0, body_top - 0.018)),
        material=porcelain,
        name="top_rim",
    )
    body.visual(
        Cylinder(radius=body_radius * 0.95, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_plastic,
        name="bottom_foot",
    )
    body.visual(
        Box((0.105, 0.014, 0.085)),
        origin=Origin(xyz=(0.0, -body_radius - 0.006, 0.122)),
        material=steel,
        name="front_panel",
    )
    body.visual(
        Box((0.018, 0.002, 0.008)),
        origin=Origin(xyz=(-0.038, -body_radius - 0.014, 0.155)),
        material=green,
        name="warm_mark",
    )
    body.visual(
        Box((0.018, 0.002, 0.008)),
        origin=Origin(xyz=(0.038, -body_radius - 0.014, 0.155)),
        material=amber,
        name="cook_mark",
    )
    # Rear hinge clevis knuckles mounted to the cylindrical rim.  They leave a
    # central gap for the moving lid barrel.
    for i, x in enumerate((-0.092, 0.092)):
        body.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_plastic,
            name=f"rear_knuckle_{i}",
        )
        body.visual(
            Box((0.046, 0.040, 0.010)),
            origin=Origin(xyz=(x, body_radius - 0.012, hinge_z - 0.001)),
            material=dark_plastic,
            name=f"rear_leaf_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.168, length=0.032),
        # The child frame is on the rear hinge axis; the closed lid disk extends
        # toward the front of the cooker along local -Y.
        origin=Origin(xyz=(0.0, -hinge_y, 0.020)),
        material=porcelain,
        name="lid_disc",
    )
    lid.visual(
        Box((0.105, 0.075, 0.026)),
        origin=Origin(xyz=(0.0, -hinge_y, 0.047)),
        material=dark_plastic,
        name="lid_handle",
    )
    lid.visual(
        Cylinder(radius=0.0105, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.100, 0.026, 0.008)),
        origin=Origin(xyz=(0.0, -0.012, 0.006)),
        material=dark_plastic,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.062, -hinge_y + 0.040, 0.038)),
        material=steel,
        name="steam_vent",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        # Closed lid extends along local -Y, so the negative X hinge axis makes
        # positive motion lift the front edge upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    dial = model.part("dial")
    dial_shape = KnobGeometry(
        0.066,
        0.026,
        body_style="skirted",
        base_diameter=0.073,
        top_diameter=0.058,
        edge_radius=0.0015,
        grip=KnobGrip(style="fluted", count=18, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    dial.visual(
        mesh_from_geometry(dial_shape, "rotary_dial"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="dial_cap",
    )
    dial.visual(
        Box((0.004, 0.002, 0.026)),
        origin=Origin(xyz=(0.0, -0.027, 0.010)),
        material=porcelain,
        name="pointer_line",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, -body_radius - 0.013, 0.122)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("dial")
    lid_hinge = object_model.get_articulation("body_to_lid")
    dial_axle = object_model.get_articulation("body_to_dial")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.010,
            positive_elem="lid_disc",
            negative_elem="top_rim",
            name="closed lid sits just above the upper rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_disc",
            elem_b="top_rim",
            min_overlap=0.24,
            name="closed lid covers the cylindrical cooker body",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_disc")
    with ctx.pose({lid_hinge: 1.0}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_disc")
    ctx.check(
        "lid hinge opens upward from rear rim",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.10,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    ctx.expect_contact(
        dial,
        body,
        elem_a="dial_cap",
        elem_b="front_panel",
        contact_tol=0.002,
        name="rotary dial seats on the front panel",
    )
    rest_pos = ctx.part_world_position(dial)
    with ctx.pose({dial_axle: math.pi * 1.25}):
        turned_pos = ctx.part_world_position(dial)
    ctx.check(
        "dial rotates continuously on fixed central axle",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()

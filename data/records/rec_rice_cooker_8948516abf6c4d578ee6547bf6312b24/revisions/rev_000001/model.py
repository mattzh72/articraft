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


def _tube(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """Open cylindrical appliance shell/ring, modeled in meters."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _domed_lid_mesh(radius: float, dome_height: float, skirt_height: float, center_x: float) -> cq.Workplane:
    """Low rice-cooker dome: a shallow spherical cap fused to a short rim skirt."""
    sphere_radius = (radius * radius + dome_height * dome_height) / (2.0 * dome_height)
    sphere_center_z = skirt_height + dome_height - sphere_radius
    cap = (
        cq.Workplane("XY")
        .sphere(sphere_radius)
        .translate((center_x, 0.0, sphere_center_z))
        .intersect(
            cq.Workplane("XY")
            .box(2.4 * radius, 2.4 * radius, dome_height)
            .translate((center_x, 0.0, skirt_height + dome_height / 2.0))
        )
    )
    skirt = cq.Workplane("XY").circle(radius).extrude(skirt_height).translate((center_x, 0.0, 0.0))
    return skirt.union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="simple_rice_cooker")

    warm_white = model.material("warm_white", rgba=(0.92, 0.90, 0.84, 1.0))
    lower_plastic = model.material("inset_lower_band", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.08, 0.085, 0.09, 1.0))
    panel_gray = model.material("control_panel_gray", rgba=(0.58, 0.60, 0.60, 1.0))
    latch_gray = model.material("latch_gray", rgba=(0.82, 0.82, 0.78, 1.0))
    metal = model.material("brushed_metal", rgba=(0.70, 0.70, 0.68, 1.0))
    red = model.material("cook_red", rgba=(0.82, 0.08, 0.04, 1.0))
    green = model.material("warm_green", rgba=(0.05, 0.55, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_tube(0.140, 0.112, 0.170), "main_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=warm_white,
        name="main_body_shell",
    )
    body.visual(
        mesh_from_cadquery(_tube(0.128, 0.104, 0.044), "lower_band"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=lower_plastic,
        name="lower_band",
    )
    body.visual(
        mesh_from_cadquery(_tube(0.146, 0.134, 0.010), "top_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=metal,
        name="top_rim",
    )
    # A flat molded front control pod is deliberately embedded into the curved shell,
    # so the selector is visibly carried by the panel rather than floating in the body.
    body.visual(
        Box((0.032, 0.108, 0.112)),
        origin=Origin(xyz=(0.146, 0.0, 0.116)),
        material=panel_gray,
        name="front_control_panel",
    )
    body.visual(
        Box((0.016, 0.086, 0.032)),
        origin=Origin(xyz=(0.166, 0.0, 0.176)),
        material=latch_gray,
        name="front_latch_zone",
    )
    body.visual(
        Box((0.010, 0.088, 0.005)),
        origin=Origin(xyz=(0.164, 0.0, 0.157)),
        material=dark_gray,
        name="latch_shadow_gap",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.164, 0.0, 0.104), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="dial_bushing",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.164, -0.026, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="cook_indicator",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.164, 0.026, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=green,
        name="warm_indicator",
    )
    # Rear hinge supports fixed to the body rim.
    for idx, y in enumerate((-0.041, 0.041)):
        body.visual(
            Box((0.022, 0.034, 0.018)),
            origin=Origin(xyz=(-0.147, y, 0.211)),
            material=dark_gray,
            name=f"hinge_bracket_{idx}",
        )
        body.visual(
            Cylinder(radius=0.0055, length=0.034),
            origin=Origin(xyz=(-0.153, y, 0.216), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_gray,
            name=f"fixed_hinge_knuckle_{idx}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_domed_lid_mesh(0.132, 0.056, 0.012, 0.153), "domed_lid"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=warm_white,
        name="domed_lid",
    )
    lid.visual(
        Box((0.034, 0.048, 0.010)),
        origin=Origin(xyz=(0.017, 0.0, 0.008)),
        material=dark_gray,
        name="hinge_strap",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="lid_hinge_knuckle",
    )

    selector_dial = model.part("selector_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.048,
            0.018,
            body_style="skirted",
            top_diameter=0.039,
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0010, width=0.0015),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0008, angle_deg=0.0),
            center=False,
        ),
        "selector_dial_cap",
    )
    selector_dial.visual(
        dial_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="dial_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.153, 0.0, 0.216)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "panel_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_dial,
        origin=Origin(xyz=(0.168, 0.0, 0.104)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("selector_dial")
    lid_hinge = object_model.get_articulation("body_to_lid")
    dial_joint = object_model.get_articulation("panel_to_dial")

    ctx.check(
        "dial joint is continuous selector",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )
    ctx.check(
        "selector dial rotates about front axle",
        tuple(round(v, 3) for v in dial_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={dial_joint.axis}",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="x",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem="dial_cap",
        negative_elem="dial_bushing",
        name="dial cap is seated on front bushing",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="yz",
        min_overlap=0.020,
        elem_a="dial_cap",
        elem_b="front_control_panel",
        name="dial is centered on control panel below latch",
    )
    ctx.expect_gap(
        body,
        dial,
        axis="z",
        min_gap=0.020,
        positive_elem="front_latch_zone",
        negative_elem="dial_cap",
        name="latch zone is above selector dial",
    )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.05}):
        opened_aabb = ctx.part_world_aabb(lid)
        ctx.expect_origin_gap(
            lid,
            body,
            axis="z",
            min_gap=0.03,
            name="lid opens upward on rear hinge",
        )
    ctx.check(
        "positive lid motion raises lid shell",
        closed_aabb is not None and opened_aabb is not None and opened_aabb[1][2] > closed_aabb[1][2] + 0.02,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

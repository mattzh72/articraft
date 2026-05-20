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


def _rounded_body() -> cq.Workplane:
    """Compact rounded plastic flash-drive body, authored in meters."""
    return (
        cq.Workplane("XY")
        .box(0.046, 0.018, 0.008)
        .edges("|X")
        .fillet(0.0032)
        .edges(">X or <X")
        .fillet(0.0012)
        .translate((-0.005, 0.0, 0.0))
    )


def _usb_sleeve() -> cq.Workplane:
    """Thin hollow USB-A metal sleeve, open through the length."""
    outer = cq.Workplane("XY").box(0.016, 0.0122, 0.0046).translate((0.025, 0.0, 0.0))
    bore = cq.Workplane("XY").box(0.018, 0.0097, 0.0028).translate((0.025, 0.0, 0.0))
    return outer.cut(bore).edges("|X").fillet(0.00045)


def _swivel_cover() -> cq.Workplane:
    """One clean U-shaped stamped cover plate in the cover joint frame.

    The local origin is the side pivot axis.  At q=0 the cover surrounds the
    body/connector footprint with clearance; the continuous joint then spins
    this whole yoke about the pin.
    """
    plate_t = 0.0015
    rail_w = 0.0040
    span_x = 0.056
    outer_y = 0.026
    rail_center_x = 0.026

    lower_rail = cq.Workplane("XY").box(span_x, rail_w, plate_t).translate((rail_center_x, 0.0, 0.0))
    upper_rail = cq.Workplane("XY").box(span_x, rail_w, plate_t).translate((rail_center_x, outer_y, 0.0))
    nose_bridge = cq.Workplane("XY").box(rail_w, outer_y + rail_w, plate_t).translate((0.052, outer_y / 2.0, 0.0))
    pivot_lobe = cq.Workplane("XY").circle(0.0053).extrude(plate_t).translate((0.0, 0.0, -plate_t / 2.0))
    pin_hole = cq.Workplane("XY").circle(0.00275).extrude(plate_t * 4.0).translate((0.0, 0.0, -plate_t * 2.0))

    cover = lower_rail.union(upper_rail).union(nose_bridge).union(pivot_lobe).cut(pin_hole)
    return cover.edges("|Z").fillet(0.0010).edges(">Z or <Z").fillet(0.00025)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_swivel_drive")

    plastic = Material("satin_black_plastic", rgba=(0.015, 0.017, 0.020, 1.0))
    dark_plastic = Material("dark_insert_plastic", rgba=(0.004, 0.004, 0.005, 1.0))
    steel = Material("brushed_stainless_steel", rgba=(0.68, 0.70, 0.69, 1.0))
    darker_steel = Material("shadowed_steel_edges", rgba=(0.42, 0.44, 0.43, 1.0))
    gold = Material("contact_gold", rgba=(1.0, 0.72, 0.18, 1.0))
    for material in (plastic, dark_plastic, steel, darker_steel, gold):
        model.materials.append(material)

    pivot_xyz = (-0.019, -0.0105, 0.0060)

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_body(), "body_shell", tolerance=0.00045, angular_tolerance=0.08),
        material=plastic,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_usb_sleeve(), "connector_sleeve", tolerance=0.00035, angular_tolerance=0.08),
        material=steel,
        name="connector_sleeve",
    )
    body.visual(
        Box((0.014, 0.0075, 0.0011)),
        origin=Origin(xyz=(0.025, 0.0, -0.00055)),
        material=dark_plastic,
        name="connector_tongue",
    )
    for index, y in enumerate((-0.0027, -0.0009, 0.0009, 0.0027)):
        body.visual(
            Box((0.0036, 0.0010, 0.00025)),
            origin=Origin(xyz=(0.0290, y, 0.000125)),
            material=gold,
            name=f"contact_{index}",
        )

    # Side pivot hardware fixed to the body: a shaft through the cover hole and
    # a small proud cap that visually retains the swivel cover.
    body.visual(
        Cylinder(radius=0.0037, length=0.0012),
        origin=Origin(xyz=(pivot_xyz[0], pivot_xyz[1], 0.00430)),
        material=darker_steel,
        name="pin_boss",
    )
    body.visual(
        Cylinder(radius=0.0021, length=0.0039),
        origin=Origin(xyz=(pivot_xyz[0], pivot_xyz[1], 0.00545)),
        material=darker_steel,
        name="pin_shaft",
    )
    body.visual(
        Cylinder(radius=0.00345, length=0.0009),
        origin=Origin(xyz=(pivot_xyz[0], pivot_xyz[1], 0.00720)),
        material=steel,
        name="pin_cap",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_swivel_cover(), "cover_shell", tolerance=0.00035, angular_tolerance=0.06),
        material=steel,
        name="cover_shell",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=pivot_xyz),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    swivel = object_model.get_articulation("body_to_cover")

    ctx.check(
        "cover joint is continuous",
        swivel.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={swivel.articulation_type}",
    )
    ctx.check(
        "cover spins about side pin axis",
        tuple(round(v, 6) for v in swivel.axis) == (0.0, 0.0, 1.0),
        details=f"axis={swivel.axis}",
    )

    body_box = ctx.part_element_world_aabb(body, elem="body_shell")
    connector_box = ctx.part_element_world_aabb(body, elem="connector_sleeve")
    ctx.check(
        "connector protrudes from compact body",
        body_box is not None
        and connector_box is not None
        and connector_box[1][0] > body_box[1][0] + 0.010,
        details=f"body={body_box}, connector={connector_box}",
    )

    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="cover_shell",
        negative_elem="body_shell",
        min_gap=0.0008,
        max_gap=0.0022,
        name="cover plate clears body top",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a="cover_shell",
        elem_b="body_shell",
        min_overlap=0.015,
        name="u cover wraps around body footprint",
    )
    ctx.expect_within(
        body,
        cover,
        axes="xy",
        inner_elem="pin_shaft",
        outer_elem="cover_shell",
        margin=0.001,
        name="body pin sits inside cover pivot collar",
    )

    rest_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "continuous swivel changes cover pose",
        rest_aabb is not None
        and turned_aabb is not None
        and abs(turned_aabb[0][0] - rest_aabb[0][0]) > 0.010
        and abs(turned_aabb[1][1] - rest_aabb[1][1]) > 0.010,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

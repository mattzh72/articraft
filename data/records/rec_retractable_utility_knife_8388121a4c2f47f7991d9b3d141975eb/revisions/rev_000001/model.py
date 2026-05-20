from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


HANDLE_LENGTH = 0.180
HANDLE_THICKNESS = 0.026
HANDLE_HEIGHT = 0.040
CARRIER_REST_X = -0.025
CARRIER_REST_Y = HANDLE_THICKNESS / 2 + 0.002
CARRIER_REST_Z = 0.000
CARRIER_TRAVEL = 0.035
WHEEL_X = 0.058
WHEEL_Y = 0.0215
WHEEL_Z = 0.021


def _utility_handle_body() -> cq.Workplane:
    """One continuous heavy-duty handle shell with side carriage pocket and front blade throat."""
    body = cq.Workplane("XY").box(HANDLE_LENGTH, HANDLE_THICKNESS, HANDLE_HEIGHT)
    body = body.edges("|Y").fillet(0.006)

    # Shallow side pocket on the +Y wall for the sliding carrier and thumb tab stem.
    pocket_depth = 0.0075
    pocket_extra = 0.004
    pocket = (
        cq.Workplane("XY")
        .box(0.136, pocket_depth + pocket_extra, 0.024)
        .translate((-0.010, HANDLE_THICKNESS / 2 - pocket_depth / 2 + pocket_extra / 2, 0.000))
    )
    body = body.cut(pocket)

    # A center throat reaches through the front nose so the blade can slide out without
    # intersecting the molded body.
    blade_throat = (
        cq.Workplane("XY")
        .box(0.105, 0.019, 0.018)
        .translate((0.054, 0.0055, 0.000))
    )
    body = body.cut(blade_throat)

    # Rear lanyard hole through the handle side plates.
    lanyard_hole = (
        cq.Workplane("XZ")
        .center(-0.073, 0.004)
        .circle(0.0065)
        .extrude(HANDLE_THICKNESS * 2.0, both=True)
    )
    body = body.cut(lanyard_hole)

    return body


def _utility_blade() -> cq.Workplane:
    """Flat trapezoid blade blank with a sharp utility-knife point, authored in carrier frame."""
    pts = [
        (0.034, -0.0055),
        (0.080, -0.0055),
        (0.098, -0.0005),
        (0.080, 0.0060),
        (0.034, 0.0060),
    ]
    return cq.Workplane("XZ").polyline(pts).close().extrude(0.003, both=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_utility_knife")

    black_nylon = model.material("black_glass_filled_nylon", rgba=(0.025, 0.027, 0.026, 1.0))
    dark_rubber = model.material("molded_dark_rubber", rgba=(0.010, 0.012, 0.012, 1.0))
    orange_rubber = model.material("orange_grip_inlay", rgba=(0.95, 0.34, 0.05, 1.0))
    satin_steel = model.material("satin_tool_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_steel = model.material("dark_phosphate_steel", rgba=(0.10, 0.11, 0.12, 1.0))

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_utility_handle_body(), "handle_body", tolerance=0.0007, angular_tolerance=0.08),
        material=black_nylon,
        name="handle_body",
    )
    # Rubber grip pads and steel screw heads are embedded into the handle side surfaces.
    handle.visual(
        Box((0.052, 0.0020, 0.014)),
        origin=Origin(xyz=(-0.044, -HANDLE_THICKNESS / 2 - 0.0002, -0.004)),
        material=orange_rubber,
        name="rear_grip_inlay",
    )
    handle.visual(
        Box((0.034, 0.0020, 0.012)),
        origin=Origin(xyz=(0.026, -HANDLE_THICKNESS / 2 - 0.0002, -0.005)),
        material=dark_rubber,
        name="front_grip_inlay",
    )
    for index, x in enumerate((-0.060, 0.004, 0.064)):
        handle.visual(
            Cylinder(radius=0.0042, length=0.0018),
            origin=Origin(xyz=(x, -HANDLE_THICKNESS / 2 - 0.0006, 0.011), rpy=(math.pi / 2, 0.0, 0.0)),
            material=satin_steel,
            name=f"screw_head_{index}",
        )
    handle.visual(
        Cylinder(radius=0.0038, length=0.0135),
        origin=Origin(xyz=(WHEEL_X, WHEEL_Y, WHEEL_Z), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_shaft",
    )
    handle.visual(
        Box((0.136, 0.0012, 0.012)),
        origin=Origin(xyz=(-0.010, HANDLE_THICKNESS / 2 - 0.0006, 0.000)),
        material=dark_steel,
        name="slide_bed",
    )
    handle.visual(
        Cylinder(radius=0.0085, length=0.0052),
        origin=Origin(xyz=(WHEEL_X, 0.0146, WHEEL_Z), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="lock_bushing",
    )

    carrier = model.part("blade_carrier")
    carrier.visual(
        Box((0.075, 0.004, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=dark_steel,
        name="carrier_rail",
    )
    carrier.visual(
        Box((0.018, 0.004, 0.006)),
        origin=Origin(xyz=(0.034, 0.000, 0.000)),
        material=dark_steel,
        name="blade_bridge",
    )
    carrier.visual(
        mesh_from_cadquery(_utility_blade(), "utility_blade", tolerance=0.00035, angular_tolerance=0.06),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=satin_steel,
        name="blade",
    )
    carrier.visual(
        Box((0.030, 0.006, 0.006)),
        origin=Origin(xyz=(-0.010, 0.003, 0.004)),
        material=dark_steel,
        name="thumb_stem",
    )
    carrier.visual(
        Box((0.032, 0.006, 0.014)),
        origin=Origin(xyz=(-0.010, 0.0065, 0.008)),
        material=orange_rubber,
        name="thumb_tab",
    )

    wheel = model.part("lock_wheel")
    wheel_geometry = KnobGeometry(
        0.026,
        0.007,
        body_style="cylindrical",
        edge_radius=0.0008,
        grip=KnobGrip(style="ribbed", count=28, depth=0.0010, width=0.0014),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=35.0),
        bore=KnobBore(style="round", diameter=0.0088),
    )
    wheel.visual(
        mesh_from_geometry(wheel_geometry, "lock_wheel"),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_disk",
    )

    model.articulation(
        "handle_to_carrier",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carrier,
        origin=Origin(xyz=(CARRIER_REST_X, CARRIER_REST_Y, CARRIER_REST_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=0.16, lower=0.0, upper=CARRIER_TRAVEL),
    )
    model.articulation(
        "handle_to_lock_wheel",
        ArticulationType.CONTINUOUS,
        parent=handle,
        child=wheel,
        origin=Origin(xyz=(WHEEL_X, WHEEL_Y, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    carrier = object_model.get_part("blade_carrier")
    wheel = object_model.get_part("lock_wheel")
    slide = object_model.get_articulation("handle_to_carrier")
    wheel_joint = object_model.get_articulation("handle_to_lock_wheel")

    ctx.expect_contact(
        carrier,
        handle,
        elem_a="carrier_rail",
        elem_b="slide_bed",
        contact_tol=0.0008,
        name="carrier rail rides on the side slide bed",
    )
    ctx.expect_overlap(
        carrier,
        handle,
        axes="x",
        elem_a="carrier_rail",
        elem_b="slide_bed",
        min_overlap=0.055,
        name="retracted carrier stays supported by the handle track",
    )
    ctx.expect_gap(
        wheel,
        handle,
        axis="y",
        positive_elem="wheel_disk",
        negative_elem="lock_bushing",
        max_gap=0.001,
        max_penetration=0.0001,
        name="lock wheel seats against its side bushing",
    )
    ctx.expect_within(
        handle,
        wheel,
        axes="xz",
        inner_elem="wheel_shaft",
        outer_elem="wheel_disk",
        margin=0.001,
        name="short shaft is centered in the lock wheel",
    )

    handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_body")
    blade_retracted_aabb = ctx.part_element_world_aabb(carrier, elem="blade")
    ctx.check(
        "blade is retracted inside the nose at zero travel",
        handle_aabb is not None
        and blade_retracted_aabb is not None
        and blade_retracted_aabb[1][0] < handle_aabb[1][0] - 0.010,
        details=f"handle_aabb={handle_aabb}, blade_aabb={blade_retracted_aabb}",
    )

    rest_pos = ctx.part_world_position(carrier)
    wheel_rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({slide: CARRIER_TRAVEL, wheel_joint: math.pi / 2}):
        ctx.expect_contact(
            carrier,
            handle,
            elem_a="carrier_rail",
            elem_b="slide_bed",
            contact_tol=0.0008,
            name="extended carrier remains on the slide bed",
        )
        ctx.expect_overlap(
            carrier,
            handle,
            axes="x",
            elem_a="carrier_rail",
            elem_b="slide_bed",
            min_overlap=0.065,
            name="extended carrier retains enough track engagement",
        )
        ctx.expect_gap(
            wheel,
            carrier,
            axis="z",
            positive_elem="wheel_disk",
            negative_elem="blade",
            min_gap=0.001,
            name="lock wheel clears the extended blade path",
        )
        blade_extended_aabb = ctx.part_element_world_aabb(carrier, elem="blade")
        extended_pos = ctx.part_world_position(carrier)
        wheel_turned_pos = ctx.part_world_position(wheel)

    ctx.check(
        "blade carrier translates forward along the handle axis",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.030,
        details=f"rest_pos={rest_pos}, extended_pos={extended_pos}",
    )
    ctx.check(
        "extended blade protrudes beyond the handle nose",
        handle_aabb is not None
        and blade_extended_aabb is not None
        and blade_extended_aabb[1][0] > handle_aabb[1][0] + 0.012,
        details=f"handle_aabb={handle_aabb}, blade_aabb={blade_extended_aabb}",
    )
    ctx.check(
        "lock wheel rotates in place on its shaft",
        wheel_rest_pos is not None
        and wheel_turned_pos is not None
        and abs(wheel_rest_pos[0] - wheel_turned_pos[0]) < 1e-6
        and abs(wheel_rest_pos[1] - wheel_turned_pos[1]) < 1e-6
        and abs(wheel_rest_pos[2] - wheel_turned_pos[2]) < 1e-6,
        details=f"rest_pos={wheel_rest_pos}, turned_pos={wheel_turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()

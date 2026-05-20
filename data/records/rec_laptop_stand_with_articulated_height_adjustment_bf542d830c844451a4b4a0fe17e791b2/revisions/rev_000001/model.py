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


BASE_WIDTH = 0.62
BASE_DEPTH = 0.36
BASE_THICKNESS = 0.030
POST_SPACING = 0.44
POST_X = POST_SPACING / 2.0
POST_Y = 0.125
POST_RADIUS = 0.018
POST_LENGTH = 0.66
CROSSHEAD_REST_Z = 0.30
CROSSHEAD_TRAVEL = 0.24
HINGE_Y = -0.075


def _translated_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _vertical_cylinder(radius: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XY").circle(radius).extrude(height).translate((cx, cy, cz - height / 2.0))


def _x_tube(
    *,
    x_start: float,
    length: float,
    outer_radius: float,
    inner_radius: float,
    y: float,
    z: float,
) -> cq.Workplane:
    # A hollow hinge knuckle whose cylinder axis is local +X.
    return (
        cq.Workplane("YZ")
        .workplane(offset=x_start)
        .center(y, z)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )


def _make_crosshead() -> cq.Workplane:
    sleeve_h = 0.085
    outer_r = 0.036
    # Slight running contact keeps the sliding bushings physically captured on
    # the guide posts instead of reading as floating loose collars.
    clearance_r = POST_RADIUS - 0.0006

    bar = _translated_box((0.54, 0.070, 0.055), (0.0, 0.0, 0.0))
    crosshead = bar
    for x in (-POST_X, POST_X):
        crosshead = crosshead.union(_vertical_cylinder(outer_r, sleeve_h, (x, 0.0, 0.0)))

    # Clearance bores leave the fixed rear posts visible inside the sliding collars.
    for x in (-POST_X, POST_X):
        crosshead = crosshead.cut(_vertical_cylinder(clearance_r, sleeve_h + 0.040, (x, 0.0, 0.0)))

    # Two fork cheeks connect the sliding crosshead to the hinge pin in the gaps
    # between the tray knuckles.
    crosshead = crosshead.union(_translated_box((0.040, 0.080, 0.046), (-0.095, -0.050, 0.0)))
    crosshead = crosshead.union(_translated_box((0.040, 0.080, 0.046), (0.095, -0.050, 0.0)))
    return crosshead


def _make_tray() -> cq.Workplane:
    width = 0.52
    depth = 0.36
    plate_t = 0.018
    plate_center_y = -depth / 2.0 - 0.020

    plate = _translated_box((width, depth, plate_t), (0.0, plate_center_y, -0.014))
    vent_cut = _translated_box((0.250, 0.145, 0.090), (0.0, plate_center_y, -0.014))
    tray = plate.cut(vent_cut)

    # Raised front stop and low side returns keep a laptop from sliding off the tray.
    tray = tray.union(_translated_box((0.50, 0.018, 0.055), (0.0, plate_center_y - depth / 2.0 + 0.009, 0.004)))
    tray = tray.union(_translated_box((0.018, depth - 0.040, 0.030), (-width / 2.0 + 0.009, plate_center_y + 0.012, -0.003)))
    tray = tray.union(_translated_box((0.018, depth - 0.040, 0.030), (width / 2.0 - 0.009, plate_center_y + 0.012, -0.003)))

    # Three tray-side hinge knuckles share one hinge axis with the crosshead pin.
    for x_start, length in ((-0.245, 0.095), (-0.0475, 0.095), (0.150, 0.095)):
        # A short bridge fuses each barrel to the plate so the tray is one rigid
        # manufactured shelf rather than three loose hinge collars.
        tray = tray.union(_translated_box((length, 0.024, 0.026), (x_start + length / 2.0, -0.028, -0.010)))
        tray = tray.union(
            _x_tube(
                x_start=x_start,
                length=length,
                outer_radius=0.0175,
                inner_radius=0.0082,
                y=0.0,
                z=0.0,
            )
        )

    return tray


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_hinge_laptop_stand")
    dark = model.material("dark_anodized_aluminum", rgba=(0.05, 0.055, 0.06, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=dark,
        name="base_plate",
    )
    base.visual(
        Box((0.50, 0.22, 0.006)),
        origin=Origin(xyz=(0.0, -0.035, BASE_THICKNESS + 0.003)),
        material=rubber,
        name="top_pad",
    )
    for name, x in (("guide_post_0", -POST_X), ("guide_post_1", POST_X)):
        base.visual(
            Cylinder(radius=POST_RADIUS, length=POST_LENGTH),
            origin=Origin(xyz=(x, POST_Y, BASE_THICKNESS + POST_LENGTH / 2.0)),
            material=steel,
            name=name,
        )
        base.visual(
            Cylinder(radius=0.030, length=0.018),
            origin=Origin(xyz=(x, POST_Y, BASE_THICKNESS + 0.009)),
            material=dark,
            name=f"{name}_foot",
        )
        base.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(x, POST_Y, BASE_THICKNESS + POST_LENGTH + 0.009)),
            material=dark,
            name=f"{name}_cap",
        )
    base.visual(
        Box((POST_SPACING + 0.11, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, POST_Y, BASE_THICKNESS + 0.020)),
        material=dark,
        name="rear_anchor",
    )

    crosshead = model.part("crosshead")
    crosshead.visual(
        mesh_from_cadquery(_make_crosshead(), "crosshead_carriage", tolerance=0.0008),
        origin=Origin(),
        material=dark,
        name="carriage",
    )
    crosshead.visual(
        Cylinder(radius=0.0085, length=0.52),
        origin=Origin(xyz=(0.0, HINGE_Y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_make_tray(), "vented_top_tray", tolerance=0.0008),
        origin=Origin(),
        material=dark,
        name="vented_tray",
    )

    model.articulation(
        "base_to_crosshead",
        ArticulationType.PRISMATIC,
        parent=base,
        child=crosshead,
        origin=Origin(xyz=(0.0, POST_Y, CROSSHEAD_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=CROSSHEAD_TRAVEL),
    )
    model.articulation(
        "crosshead_to_tray",
        ArticulationType.REVOLUTE,
        parent=crosshead,
        child=tray,
        origin=Origin(xyz=(0.0, HINGE_Y, 0.0)),
        # The tray extends forward along local -Y; positive rotation around +X
        # lowers the front edge into a normal laptop-stand working angle.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=0.0, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    crosshead = object_model.get_part("crosshead")
    tray = object_model.get_part("tray")
    lift = object_model.get_articulation("base_to_crosshead")
    tilt = object_model.get_articulation("crosshead_to_tray")

    for post_name in ("guide_post_0", "guide_post_1"):
        ctx.allow_overlap(
            crosshead,
            base,
            elem_a="carriage",
            elem_b=post_name,
            reason="The carriage bushings are intentionally modeled with slight sliding contact around the vertical guide posts.",
        )
        ctx.expect_contact(
            crosshead,
            base,
            elem_a="carriage",
            elem_b=post_name,
            contact_tol=0.0015,
            name=f"{post_name} supports the sliding carriage bushing",
        )

    ctx.allow_overlap(
        crosshead,
        tray,
        elem_a="hinge_pin",
        elem_b="vented_tray",
        reason="The steel hinge pin is intentionally captured inside the tray hinge barrels.",
    )
    ctx.expect_contact(
        crosshead,
        tray,
        elem_a="hinge_pin",
        elem_b="vented_tray",
        contact_tol=0.001,
        name="tray hinge barrels are captured on the shared crosshead pin",
    )

    ctx.expect_overlap(
        crosshead,
        base,
        axes="z",
        elem_a="carriage",
        elem_b="guide_post_0",
        min_overlap=0.050,
        name="crosshead is retained on one guide post",
    )
    ctx.expect_overlap(
        crosshead,
        base,
        axes="z",
        elem_a="carriage",
        elem_b="guide_post_1",
        min_overlap=0.050,
        name="crosshead is retained on the matching guide post",
    )
    ctx.expect_overlap(
        tray,
        crosshead,
        axes="x",
        elem_a="vented_tray",
        elem_b="hinge_pin",
        min_overlap=0.20,
        name="single tray shares the crosshead hinge pin",
    )

    low_pos = ctx.part_world_position(crosshead)
    with ctx.pose({lift: CROSSHEAD_TRAVEL}):
        high_pos = ctx.part_world_position(crosshead)
        ctx.expect_overlap(
            crosshead,
            base,
            axes="z",
            elem_a="carriage",
            elem_b="guide_post_0",
            min_overlap=0.050,
            name="raised crosshead remains on guide posts",
        )

    ctx.check(
        "crosshead slides vertically upward",
        low_pos is not None and high_pos is not None and high_pos[2] > low_pos[2] + 0.20,
        details=f"low={low_pos}, high={high_pos}",
    )

    flat_aabb = ctx.part_world_aabb(tray)
    with ctx.pose({tilt: 0.60}):
        tilted_aabb = ctx.part_world_aabb(tray)

    ctx.check(
        "tray rotates downward about horizontal hinge",
        flat_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[0][2] < flat_aabb[0][2] - 0.12,
        details=f"flat={flat_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
